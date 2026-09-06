"""Live MuJoCo viewer: G1 in the arena, PD hold, sensors on.

From ``docker/``::

    ./run.sh python examples/01_empty_arena.py

Green dots are Mid-360, cyan is D435i depth. Density is configurable
(``--overlay sparse|dense|full``, or ``--lidar-dots`` / ``--depth-stride``).
Press ``C`` to look through ``d435i_rgb`` / ``d435i_depth``.

``--empty`` uses the floor-only default XML. ``--scene`` loads a cached
RoboCasa dump (or any MJCF that includes ``g1_robot.xml``). ``--spawn`` /
``--yaw`` set spawn heading (live: numpad trolley, not WASD — those are MuJoCo's).
``--headless`` is a short smoke with no window. ``--no-gantry`` skips the
overhead crane (floating base will fall).
"""

from __future__ import annotations

import argparse
import faulthandler
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from numpy.typing import NDArray

from g1_simulacrum import G1Simulacrum
from g1_simulacrum.gantry import ElasticBand, quat_wxyz_from_yaw
from g1_simulacrum.sensors.data_types import DepthFrame, PointCloud

faulthandler.enable()

_GLFW_KEY_C = 67
_GLFW_KEY_KP_2 = 322
_GLFW_KEY_KP_4 = 324
_GLFW_KEY_KP_5 = 325
_GLFW_KEY_KP_6 = 326
_GLFW_KEY_KP_7 = 327
_GLFW_KEY_KP_8 = 328
_GLFW_KEY_KP_9 = 329
_GLFW_KEY_KP_SUBTRACT = 333
_GLFW_KEY_KP_ADD = 334
_GANTRY_RGBA = np.array([0.95, 0.85, 0.15, 0.9], dtype=np.float32)
_GANTRY_HEADING_RGBA = np.array([0.98, 0.45, 0.12, 0.95], dtype=np.float32)
_GANTRY_STEP_XY = 0.05
_GANTRY_STEP_YAW = np.deg2rad(5.0)

_PKG = Path(__file__).resolve().parents[1]
_INSPECT_XML = _PKG / "g1_simulacrum" / "model" / "mjcf" / "g1_inspect.xml"
_LIDAR_SITE = "mid360"
_IDENTITY_MAT = np.eye(3, dtype=np.float64).reshape(9)
_LIDAR_RGBA = np.array([0.15, 0.95, 0.25, 0.55], dtype=np.float32)
_DEPTH_RGBA = np.array([0.15, 0.75, 0.95, 0.7], dtype=np.float32)


@dataclass
class OverlayConfig:
    """Inspect-viewer overlay markers. Does not change sensor sample counts."""

    lidar_dots: int = 0  # 0 = every Mid-360 return (~24k)
    depth_stride: int = 4
    lidar_radius: float = 0.006
    depth_radius: float = 0.008


_OVERLAY_PRESETS = {
    "sparse": {"lidar_dots": 1800, "depth_stride": 16, "lidar_radius": 0.012, "depth_radius": 0.018},
    "dense": {"lidar_dots": 0, "depth_stride": 4, "lidar_radius": 0.006, "depth_radius": 0.008},
    "full": {"lidar_dots": 0, "depth_stride": 2, "lidar_radius": 0.004, "depth_radius": 0.005},
}


def _site_world(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if sid < 0:
        raise ValueError(f"missing site {name!r}")
    return data.site_xpos[sid], data.site_xmat[sid].reshape(3, 3)


def _lidar_world(
    model: mujoco.MjModel, data: mujoco.MjData, cloud: PointCloud
) -> NDArray[np.float64]:
    """PointCloud is in ``mid360_link``; draw in world."""
    origin, rot = _site_world(model, data, _LIDAR_SITE)
    pts = np.asarray(cloud.points, dtype=np.float64)
    if pts.size == 0:
        return pts.reshape(0, 3)
    return origin + pts @ rot.T


def _depth_world(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    frame: DepthFrame,
    *,
    stride: int,
) -> NDArray[np.float64]:
    """Unproject a strided D435i depth grid into world (camera looks along −Z)."""
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "d435i_depth")
    if cid < 0:
        return np.zeros((0, 3), dtype=np.float64)
    step = max(1, int(stride))
    depth = frame.depth[::step, ::step]
    z = depth.reshape(-1)
    valid = z > 0
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float64)
    h_s, w_s = depth.shape
    vv, uu = np.indices((h_s, w_s))
    u = (uu.reshape(-1)[valid].astype(np.float64) * step)
    v = (vv.reshape(-1)[valid].astype(np.float64) * step)
    z = z[valid].astype(np.float64)
    k = frame.intrinsics
    x = (u - k.cx) / k.fx * z
    y = (k.cy - v) / k.fy * z
    cam = np.stack([x, y, -z], axis=1)
    pos = data.cam_xpos[cid]
    rot = data.cam_xmat[cid].reshape(3, 3)
    return pos + cam @ rot.T


def _write_points(
    scn: mujoco.MjvScene,
    start: int,
    points: NDArray[np.float64],
    *,
    radius: float,
    rgba: NDArray[np.float32],
    inited: list[int],
    max_n: int,
) -> int:
    """Init BOX geoms once (discoverse-style); afterwards only write pos.

    Boxes are much cheaper than spheres: 12 triangles vs a tessellated sphere.
    MuJoCo has no point-sprite primitive (github.com/google-deepmind/mujoco/issues/3270).
    """
    if points.size == 0 or start >= scn.maxgeom:
        return 0
    n = min(len(points), max_n, scn.maxgeom - start)
    if n <= 0:
        return 0
    if len(points) > n:
        idx = np.linspace(0, len(points) - 1, n, dtype=int)
        points = points[idx]
    size = np.array([radius, radius, radius], dtype=np.float64)
    geoms = scn.geoms
    need = start + n
    while inited[0] < need:
        i = inited[0]
        mujoco.mjv_initGeom(
            geoms[i],
            mujoco.mjtGeom.mjGEOM_BOX,
            size,
            np.zeros(3),
            _IDENTITY_MAT,
            rgba,
        )
        inited[0] += 1
    for i, p in enumerate(points):
        g = geoms[start + i]
        g.pos[:] = p
        g.size[:] = size
        g.rgba[:] = rgba
    return n


def _attach_body_id(model: mujoco.MjModel) -> int:
    """Unitree G1 crane hangs from torso; pelvis if that body is missing."""
    for name in ("torso_link", "pelvis"):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid >= 0:
            return bid
    raise ValueError("no torso_link or pelvis to hang the crane from")


def _print_gantry(gantry: ElasticBand) -> None:
    t = gantry.target
    print(
        f"crane hook={t[0]:.3f} {t[1]:.3f} {t[2]:.3f}  "
        f"cable={gantry.length:.2f}m  yaw={np.rad2deg(gantry.yaw):.1f}°",
        flush=True,
    )


def _paint_gantry(
    scn: mujoco.MjvScene,
    body: NDArray[np.float64],
    gantry: ElasticBand,
) -> None:
    hook = gantry.target
    if scn.ngeom < scn.maxgeom:
        geom = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.zeros(3),
            np.zeros(3),
            _IDENTITY_MAT,
            _GANTRY_RGBA,
        )
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.008, body, hook)
        scn.ngeom += 1
    fwd = gantry.forward_xy
    tip = body + np.array([0.4 * fwd[0], 0.4 * fwd[1], 0.0])
    if scn.ngeom < scn.maxgeom:
        geom = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.zeros(3),
            np.zeros(3),
            _IDENTITY_MAT,
            _GANTRY_HEADING_RGBA,
        )
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.018, body, tip)
        scn.ngeom += 1


def _paint(
    viewer: mujoco.viewer.Handle,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    cloud: PointCloud | None,
    depth: DepthFrame | None,
    gantry: ElasticBand | None,
    attach_id: int,
    overlay: OverlayConfig,
    *,
    refresh_clouds: bool,
    inited: list[int],
    drawn: list[int],
) -> None:
    scn = viewer.user_scn
    if refresh_clouds:
        lidar_cap = overlay.lidar_dots if overlay.lidar_dots > 0 else scn.maxgeom
        n_l = 0
        n_d = 0
        if cloud is not None:
            n_l = _write_points(
                scn,
                0,
                _lidar_world(model, data, cloud),
                radius=overlay.lidar_radius,
                rgba=_LIDAR_RGBA,
                inited=inited,
                max_n=lidar_cap,
            )
        if depth is not None:
            n_d = _write_points(
                scn,
                n_l,
                _depth_world(model, data, depth, stride=overlay.depth_stride),
                radius=overlay.depth_radius,
                rgba=_DEPTH_RGBA,
                inited=inited,
                max_n=scn.maxgeom - n_l,
            )
        drawn[0], drawn[1] = n_l, n_d
    scn.ngeom = drawn[0] + drawn[1]
    if gantry is not None and gantry.enable:
        _paint_gantry(scn, data.xpos[attach_id], gantry)


def _configure_viewer(
    viewer: mujoco.viewer.Handle, *, lookat: NDArray[np.float64]
) -> None:
    viewer.cam.azimuth = 140.0
    viewer.cam.elevation = -18.0
    viewer.cam.distance = 3.6
    viewer.cam.lookat[:] = lookat
    for i in range(len(viewer.opt.sitegroup)):
        viewer.opt.sitegroup[i] = 1
    cam_flag = getattr(mujoco.mjtVisFlag, "mjVIS_CAMERA", None)
    if cam_flag is not None:
        viewer.opt.flags[cam_flag] = True
    # User overlay is thousands of tiny boxes; shadows/reflections dominate GPU time.
    shadow = getattr(mujoco.mjtRndFlag, "mjRND_SHADOW", None)
    refl = getattr(mujoco.mjtRndFlag, "mjRND_REFLECTION", None)
    if shadow is not None:
        viewer.user_scn.flags[shadow] = 0
    if refl is not None:
        viewer.user_scn.flags[refl] = 0


def _hud(
    obs,
    *,
    lidar_n: int,
    depth_valid: int,
    lidar_drawn: int,
    depth_drawn: int,
) -> str:
    imu = obs.sensors.imu_pelvis
    gyro = "—" if imu is None else f"{np.linalg.norm(imu.gyro):.3f}"
    acc = "—" if imu is None else f"{np.linalg.norm(imu.accel):.2f}"
    return (
        f"t={obs.timestamp:7.3f}s  z={obs.base_state.position[2]:.3f}m  "
        f"lidar={lidar_n:5d} draw={lidar_drawn:5d}  "
        f"depth_valid={depth_valid:6d} draw={depth_drawn:5d}  "
        f"|gyro|={gyro}  |acc|={acc}"
    )


def _run_viewer(
    sim: G1Simulacrum,
    q_hold: np.ndarray,
    *,
    gantry: ElasticBand | None,
    attach_id: int,
    overlay: OverlayConfig,
) -> None:
    control_hz = sim.config.controller.control_hz
    sync_every = max(1, int(round(control_hz / 60.0)))
    last_cloud: PointCloud | None = None
    last_depth: DepthFrame | None = None
    last_lidar_n = 0
    last_depth_valid = 0
    gantry_note = (
        "Yellow cable is the overhead crane (Unitree cable + GEAR heading lock, "
        "hook at z=2). Body PD stays on and holds the spawn pose, same as GEAR-SONIC.\n"
        "Numpad 8/2/4/6 trolley, 7/9 change the heading lock (±5°), +/− cable, "
        "5 toggles the crane (PD keeps holding). Letter keys stay with MuJoCo.\n"
        if gantry is not None
        else "No crane — PD hold will not keep a floating base standing.\n"
    )
    print(
        gantry_note
        + f"Overlay: lidar_dots={'all' if overlay.lidar_dots == 0 else overlay.lidar_dots}  "
        f"depth_stride={overlay.depth_stride}  "
        f"radii lidar={overlay.lidar_radius} depth={overlay.depth_radius}\n"
        "Viewer: green = Mid-360, cyan = D435i depth.\n"
        "Click the 3D view so it has focus, then press C to cycle\n"
        "free cam → d435i_rgb → d435i_depth (looks through the RealSense).\n"
        "Or: right-hand panel tab → Rendering → Camera dropdown.\n"
        "Close the window to exit."
    )
    cam_names = ("d435i_rgb", "d435i_depth")
    cam_ids = [
        mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, n) for n in cam_names
    ]
    cam_i = {"i": -1}

    def _set_crane(on: bool) -> None:
        if gantry is None:
            return
        gantry.enable = on
        if on:
            print("crane on — PD hold + cable", flush=True)
            return
        q_hold[:] = sim.controller.body_qpos()
        sim.controller.set_targets(q_hold)
        print("crane off — PD hold, no cable", flush=True)

    def _on_key(keycode: int) -> None:
        if gantry is not None:
            if keycode == _GLFW_KEY_KP_8:
                gantry.nudge_local(forward=_GANTRY_STEP_XY)
                _print_gantry(gantry)
                return
            if keycode == _GLFW_KEY_KP_2:
                gantry.nudge_local(forward=-_GANTRY_STEP_XY)
                _print_gantry(gantry)
                return
            if keycode == _GLFW_KEY_KP_4:
                gantry.nudge_local(left=_GANTRY_STEP_XY)
                _print_gantry(gantry)
                return
            if keycode == _GLFW_KEY_KP_6:
                gantry.nudge_local(left=-_GANTRY_STEP_XY)
                _print_gantry(gantry)
                return
            if keycode == _GLFW_KEY_KP_7:
                gantry.nudge_yaw(_GANTRY_STEP_YAW)
                _print_gantry(gantry)
                return
            if keycode == _GLFW_KEY_KP_9:
                gantry.nudge_yaw(-_GANTRY_STEP_YAW)
                _print_gantry(gantry)
                return
            if keycode == _GLFW_KEY_KP_SUBTRACT:
                gantry.length = max(0.0, gantry.length - 0.1)
                _print_gantry(gantry)
                return
            if keycode == _GLFW_KEY_KP_ADD:
                gantry.length += 0.1
                _print_gantry(gantry)
                return
            if keycode == _GLFW_KEY_KP_5:
                _set_crane(not gantry.enable)
                return
        if keycode != _GLFW_KEY_C:
            return
        cam_i["i"] = (cam_i["i"] + 1) % (len(cam_names) + 1)
        handle = viewer_holder[0]
        if handle is None:
            return
        if cam_i["i"] == len(cam_names):
            handle.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            print("camera: free", flush=True)
            return
        name = cam_names[cam_i["i"]]
        cid = cam_ids[cam_i["i"]]
        if cid < 0:
            return
        handle.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        handle.cam.fixedcamid = cid
        print(f"camera: {name}", flush=True)

    viewer_holder: list[mujoco.viewer.Handle | None] = [None]
    with mujoco.viewer.launch_passive(
        sim.model,
        sim.data,
        key_callback=_on_key,
        show_left_ui=True,
        show_right_ui=True,
    ) as viewer:
        viewer_holder[0] = viewer
        _configure_viewer(viewer, lookat=sim.data.xpos[attach_id].copy())
        step_i = 0
        last_print = 0.0
        inited = [0]
        drawn = [0, 0]
        clouds_dirty = True
        while viewer.is_running():
            if gantry is not None:
                gantry.apply(sim.model, sim.data, attach_id)
            obs = sim.step(q_hold)
            if obs.sensors.lidar is not None:
                last_cloud = obs.sensors.lidar
                last_lidar_n = last_cloud.num_points
                clouds_dirty = True
            if obs.sensors.depth is not None:
                last_depth = obs.sensors.depth
                last_depth_valid = int(np.count_nonzero(last_depth.depth > 0))
                clouds_dirty = True
            step_i += 1
            if step_i % sync_every == 0:
                _paint(
                    viewer,
                    sim.model,
                    sim.data,
                    last_cloud,
                    last_depth,
                    gantry,
                    attach_id,
                    overlay,
                    refresh_clouds=clouds_dirty,
                    inited=inited,
                    drawn=drawn,
                )
                clouds_dirty = False
                viewer.sync()
                now = time.perf_counter()
                if now - last_print >= 0.5:
                    print(
                        _hud(
                            obs,
                            lidar_n=last_lidar_n,
                            depth_valid=last_depth_valid,
                            lidar_drawn=drawn[0],
                            depth_drawn=drawn[1],
                        ),
                        flush=True,
                    )
                    last_print = now


def _run_headless(
    sim: G1Simulacrum,
    q_hold: np.ndarray,
    *,
    gantry: ElasticBand | None,
    attach_id: int,
) -> None:
    for i in range(50):
        if gantry is not None:
            gantry.apply(sim.model, sim.data, attach_id)
        obs = sim.step(q_hold)
        if i == 0 or (obs.sensors.imu_pelvis is not None and i % 10 == 0):
            print(
                f"t={obs.timestamp:.3f} height={obs.base_state.position[2]:.3f} "
                f"pelvis_imu={obs.sensors.imu_pelvis is not None} "
                f"lidar={None if obs.sensors.lidar is None else obs.sensors.lidar.num_points} "
                f"depth={obs.sensors.depth is not None}"
            )

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--empty",
        action="store_true",
        help="floor-only g1_sensorized.xml instead of the inspect boxes",
    )
    parser.add_argument(
        "--scene",
        default=None,
        help="MJCF that already includes g1_robot.xml (e.g. a robocasa dump)",
    )
    parser.add_argument("--config", default="configs/default.yaml")
    parser.add_argument(
        "--spawn",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=None,
        help="G1 freejoint spawn (metres). Crane hook XY follows this",
    )
    parser.add_argument(
        "--yaw",
        type=float,
        default=None,
        help="heading degrees about +Z (0 faces +X). Sets spawn and gantry yaw",
    )
    parser.add_argument("--headless", action="store_true", help="50 control steps, no GLFW window")
    parser.add_argument(
        "--no-gantry",
        "--free-base",
        action="store_true",
        dest="no_gantry",
        help="no overhead crane; PD only (floating base will fall)",
    )
    parser.add_argument(
        "--overlay",
        choices=tuple(_OVERLAY_PRESETS),
        default=None,
        help="density preset sparse|dense|full (default: dense)",
    )
    parser.add_argument(
        "--lidar-dots",
        type=int,
        default=None,
        metavar="N",
        help="max Mid-360 points to draw (0 = all ~24k). Overrides --overlay",
    )
    parser.add_argument(
        "--depth-stride",
        type=int,
        default=None,
        metavar="N",
        help="keep every Nth D435i pixel (1=finest). Overrides --overlay",
    )
    parser.add_argument(
        "--lidar-radius",
        type=float,
        default=None,
        help="green lidar overlay box half-size in metres",
    )
    parser.add_argument(
        "--depth-radius",
        type=float,
        default=None,
        help="cyan depth overlay box half-size in metres",
    )
    args = parser.parse_args()
    if args.empty and args.scene:
        parser.error("use only one of --empty and --scene")

    sim = G1Simulacrum.from_config(args.config)
    if args.spawn is not None:
        sim.config.robot.spawn_pos = (args.spawn[0], args.spawn[1], args.spawn[2])
    if args.yaw is not None:
        q = quat_wxyz_from_yaw(np.deg2rad(args.yaw))
        sim.config.robot.spawn_quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    overlay = OverlayConfig()
    if args.overlay is not None:
        for key, value in _OVERLAY_PRESETS[args.overlay].items():
            setattr(overlay, key, value)
    if args.lidar_dots is not None:
        overlay.lidar_dots = args.lidar_dots
    if args.depth_stride is not None:
        overlay.depth_stride = args.depth_stride
    if args.lidar_radius is not None:
        overlay.lidar_radius = args.lidar_radius
    if args.depth_radius is not None:
        overlay.depth_radius = args.depth_radius
    if args.scene is not None:
        scene = Path(args.scene)
        if not scene.is_file():
            scene = _PKG / args.scene
        if not scene.is_file():
            raise FileNotFoundError(args.scene)
    else:
        scene = None if args.empty else _INSPECT_XML
    sim.build(scene_xml=scene)
    obs = sim.reset()
    q_hold = obs.joint_state.position.copy()
    attach_id = _attach_body_id(sim.model)
    gantry: ElasticBand | None = None
    if not args.no_gantry:
        gantry = ElasticBand.overhead(
            sim.data.xpos[attach_id],
            quat_wxyz=sim.config.robot.spawn_quat,
        )
        _print_gantry(gantry)
    print(
        f"compiled {sim.compiled.xml_path.name}  "
        f"body={len(sim.compiled.body_joint_ids)} hand={len(sim.compiled.hand_joint_ids)}  "
        f"mid360={sim.config.sensors.mid360.enabled} d435i={sim.config.sensors.d435i.enabled}  "
        f"gantry={gantry is not None}  overlay={args.overlay or 'dense'}"
    )
    if args.headless:
        _run_headless(sim, q_hold, gantry=gantry, attach_id=attach_id)
        return
    _run_viewer(sim, q_hold, gantry=gantry, attach_id=attach_id, overlay=overlay)


if __name__ == "__main__":
    main()
