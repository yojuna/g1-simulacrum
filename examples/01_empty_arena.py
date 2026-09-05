"""Live MuJoCo viewer: G1 in the arena, PD hold, sensors on.

From ``docker/``::

    ./run.sh python examples/01_empty_arena.py

Green dots are Mid-360, cyan is D435i depth. Density is configurable
(``--overlay sparse|dense|full``, or ``--lidar-dots`` / ``--depth-stride``).
Press ``C`` to look through ``d435i_rgb`` / ``d435i_depth``.

``--empty`` uses the floor-only default XML. ``--headless`` is a short smoke
with no window. ``--no-gantry`` drops the elastic band (robot will fall).
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from numpy.typing import NDArray

from g1_simulacrum import G1Simulacrum
from g1_simulacrum.config import ViewerOverlayConfig
from g1_simulacrum.gantry import ElasticBand
from g1_simulacrum.sensors.data_types import DepthFrame, PointCloud

_GLFW_KEY_7 = 55
_GLFW_KEY_8 = 56
_GLFW_KEY_9 = 57
_GLFW_KEY_C = 67
_GANTRY_RGBA = np.array([0.95, 0.85, 0.15, 0.9], dtype=np.float32)

_PKG = Path(__file__).resolve().parents[1]
_INSPECT_XML = _PKG / "g1_simulacrum" / "model" / "mjcf" / "g1_inspect.xml"
_LIDAR_SITE = "mid360"
_IDENTITY_MAT = np.eye(3, dtype=np.float64).reshape(9)
_LIDAR_RGBA = np.array([0.15, 0.95, 0.25, 0.55], dtype=np.float32)
_DEPTH_RGBA = np.array([0.15, 0.75, 0.95, 0.7], dtype=np.float32)

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


def _add_spheres(
    scn: mujoco.MjvScene,
    points: NDArray[np.float64],
    *,
    radius: float,
    rgba: NDArray[np.float32],
    max_n: int,
) -> int:
    if points.size == 0 or scn.ngeom >= scn.maxgeom:
        return 0
    n = min(len(points), max_n, scn.maxgeom - scn.ngeom)
    if n <= 0:
        return 0
    if len(points) > n:
        idx = np.linspace(0, len(points) - 1, n, dtype=int)
        points = points[idx]
    size = np.array([radius, 0.0, 0.0], dtype=np.float64)
    added = 0
    for p in points:
        if scn.ngeom >= scn.maxgeom:
            break
        mujoco.mjv_initGeom(
            scn.geoms[scn.ngeom],
            mujoco.mjtGeom.mjGEOM_SPHERE,
            size,
            p,
            _IDENTITY_MAT,
            rgba,
        )
        scn.ngeom += 1
        added += 1
    return added


def _paint_gantry(
    scn: mujoco.MjvScene,
    pelvis: NDArray[np.float64],
    anchor: NDArray[np.float64],
) -> None:
    if scn.ngeom >= scn.maxgeom:
        return
    geom = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        np.zeros(3),
        np.zeros(3),
        _IDENTITY_MAT,
        _GANTRY_RGBA,
    )
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.012, pelvis, anchor)
    scn.ngeom += 1


def _paint(
    viewer: mujoco.viewer.Handle,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    cloud: PointCloud | None,
    depth: DepthFrame | None,
    gantry: ElasticBand | None,
    pelvis_id: int,
    overlay: ViewerOverlayConfig,
) -> tuple[int, int]:
    scn = viewer.user_scn
    scn.ngeom = 0
    lidar_drawn = 0
    depth_drawn = 0
    lidar_cap = overlay.lidar_dots if overlay.lidar_dots > 0 else scn.maxgeom
    if cloud is not None:
        lidar_drawn = _add_spheres(
            scn,
            _lidar_world(model, data, cloud),
            radius=overlay.lidar_radius,
            rgba=_LIDAR_RGBA,
            max_n=lidar_cap,
        )
    if depth is not None:
        depth_drawn = _add_spheres(
            scn,
            _depth_world(model, data, depth, stride=overlay.depth_stride),
            radius=overlay.depth_radius,
            rgba=_DEPTH_RGBA,
            max_n=scn.maxgeom - scn.ngeom,
        )
    if gantry is not None and gantry.enable:
        _paint_gantry(scn, data.xpos[pelvis_id], gantry.target)
    return lidar_drawn, depth_drawn


def _configure_viewer(viewer: mujoco.viewer.Handle) -> None:
    viewer.cam.azimuth = 140.0
    viewer.cam.elevation = -18.0
    viewer.cam.distance = 3.6
    viewer.cam.lookat[:] = [0.5, 0.0, 0.75]
    for i in range(len(viewer.opt.sitegroup)):
        viewer.opt.sitegroup[i] = 1
    cam_flag = getattr(mujoco.mjtVisFlag, "mjVIS_CAMERA", None)
    if cam_flag is not None:
        viewer.opt.flags[cam_flag] = True


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
    pelvis_id: int,
    overlay: ViewerOverlayConfig,
) -> None:
    control_hz = sim.config.controller.control_hz
    sync_every = max(1, int(round(control_hz / 60.0)))
    last_cloud: PointCloud | None = None
    last_depth: DepthFrame | None = None
    last_lidar_n = 0
    last_depth_valid = 0
    gantry_note = (
        "Yellow tether is the SONIC-style elastic gantry (not a weld).\n"
        "7/8 lower/raise, 9 toggle. --no-gantry lets it fall.\n"
        if gantry is not None
        else "No gantry — PD hold will not keep a floating base standing.\n"
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

    def _on_key(keycode: int) -> None:
        if gantry is not None:
            if keycode == _GLFW_KEY_7:
                gantry.length -= 0.1
                print(f"gantry length={gantry.length:.2f} target_z={gantry.target[2]:.2f}", flush=True)
                return
            if keycode == _GLFW_KEY_8:
                gantry.length += 0.1
                print(f"gantry length={gantry.length:.2f} target_z={gantry.target[2]:.2f}", flush=True)
                return
            if keycode == _GLFW_KEY_9:
                gantry.enable = not gantry.enable
                print(f"gantry {'on' if gantry.enable else 'off'}", flush=True)
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
        _configure_viewer(viewer)
        step_i = 0
        last_print = 0.0
        while viewer.is_running():
            if gantry is not None:
                gantry.apply(sim.model, sim.data, pelvis_id)
            obs = sim.step(q_hold)
            if obs.sensors.lidar is not None:
                last_cloud = obs.sensors.lidar
                last_lidar_n = last_cloud.num_points
            if obs.sensors.depth is not None:
                last_depth = obs.sensors.depth
                last_depth_valid = int(np.count_nonzero(last_depth.depth > 0))
            step_i += 1
            if step_i % sync_every == 0:
                lidar_drawn, depth_drawn = _paint(
                    viewer,
                    sim.model,
                    sim.data,
                    last_cloud,
                    last_depth,
                    gantry,
                    pelvis_id,
                    overlay,
                )
                viewer.sync()
                now = time.perf_counter()
                if now - last_print >= 0.5:
                    print(
                        _hud(
                            obs,
                            lidar_n=last_lidar_n,
                            depth_valid=last_depth_valid,
                            lidar_drawn=lidar_drawn,
                            depth_drawn=depth_drawn,
                        ),
                        flush=True,
                    )
                    last_print = now


def _run_headless(
    sim: G1Simulacrum,
    q_hold: np.ndarray,
    *,
    gantry: ElasticBand | None,
    pelvis_id: int,
) -> None:
    for i in range(50):
        if gantry is not None:
            gantry.apply(sim.model, sim.data, pelvis_id)
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
    parser.add_argument("--headless", action="store_true", help="50 control steps, no GLFW window")
    parser.add_argument(
        "--no-gantry",
        "--free-base",
        action="store_true",
        dest="no_gantry",
        help="disable the elastic gantry (robot will fall; PD is joints-only)",
    )
    parser.add_argument(
        "--overlay",
        choices=tuple(_OVERLAY_PRESETS),
        default=None,
        help="density preset sparse|dense|full (overrides YAML viewer:). Default is YAML.",
    )
    parser.add_argument(
        "--lidar-dots",
        type=int,
        default=None,
        metavar="N",
        help="max Mid-360 points to draw (0 = all ~24k). Overrides --overlay / YAML",
    )
    parser.add_argument(
        "--depth-stride",
        type=int,
        default=None,
        metavar="N",
        help="keep every Nth D435i pixel (1=finest). Overrides --overlay / YAML",
    )
    parser.add_argument(
        "--lidar-radius",
        type=float,
        default=None,
        help="green lidar sphere radius in metres",
    )
    parser.add_argument(
        "--depth-radius",
        type=float,
        default=None,
        help="cyan depth sphere radius in metres",
    )
    args = parser.parse_args()

    sim = G1Simulacrum.from_config("configs/default.yaml")
    overlay = sim.config.viewer.model_copy()
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
    scene = None if args.empty else _INSPECT_XML
    sim.build(scene_xml=scene)
    obs = sim.reset()
    q_hold = obs.joint_state.position.copy()
    pelvis_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    gantry: ElasticBand | None = None
    if not args.no_gantry and pelvis_id >= 0:
        px, py, pz = sim.data.xpos[pelvis_id]
        gantry = ElasticBand(point=np.array([px, py, 1.0]))
    print(
        f"compiled {sim.compiled.xml_path.name}  "
        f"body={len(sim.compiled.body_joint_ids)} hand={len(sim.compiled.hand_joint_ids)}  "
        f"mid360={sim.config.sensors.mid360.enabled} d435i={sim.config.sensors.d435i.enabled}  "
        f"gantry={gantry is not None}  overlay={args.overlay or 'yaml'}"
    )
    if args.headless:
        _run_headless(sim, q_hold, gantry=gantry, pelvis_id=pelvis_id)
        return
    _run_viewer(sim, q_hold, gantry=gantry, pelvis_id=pelvis_id, overlay=overlay)


if __name__ == "__main__":
    main()
