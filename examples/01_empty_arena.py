"""Live MuJoCo viewer: G1 in the arena, PD hold, sensors on.

From ``docker/``::

    ./run.sh python examples/01_empty_arena.py

Green dots are a downsampled Mid-360 cloud. Press ``C`` to look through
``d435i_rgb`` / ``d435i_depth`` (or the right-hand Rendering → Camera list).
IMU / lidar / depth stats print in the terminal.

``--empty`` uses the floor-only default XML. ``--headless`` is a short smoke
with no window.
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
from g1_simulacrum.sensors.data_types import DepthFrame, PointCloud

_PKG = Path(__file__).resolve().parents[1]
_INSPECT_XML = _PKG / "g1_simulacrum" / "model" / "mjcf" / "g1_inspect.xml"
_LIDAR_SITE = "mid360"
_IDENTITY_MAT = np.eye(3, dtype=np.float64).reshape(9)
_LIDAR_RGBA = np.array([0.15, 0.95, 0.25, 0.55], dtype=np.float32)
_DEPTH_RGBA = np.array([0.15, 0.75, 0.95, 0.7], dtype=np.float32)
_LIDAR_RADIUS = 0.012
_DEPTH_RADIUS = 0.018
_MAX_LIDAR_DOTS = 1800
_DEPTH_GRID = (36, 48)  # (rows, cols) subsample of the depth image


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
    model: mujoco.MjModel, data: mujoco.MjData, frame: DepthFrame
) -> NDArray[np.float64]:
    """Unproject a sparse D435i depth grid into world (camera looks along −Z)."""
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "d435i_depth")
    if cid < 0:
        return np.zeros((0, 3), dtype=np.float64)
    depth = frame.depth
    h, w = depth.shape
    nr, nc = _DEPTH_GRID
    rows = np.linspace(0, h - 1, nr, dtype=int)
    cols = np.linspace(0, w - 1, nc, dtype=int)
    vv, uu = np.meshgrid(rows, cols, indexing="ij")
    z = depth[vv, uu]
    valid = z > 0
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float64)
    z = z[valid].astype(np.float64)
    u = uu[valid].astype(np.float64)
    v = vv[valid].astype(np.float64)
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


def _paint(
    viewer: mujoco.viewer.Handle,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    cloud: PointCloud | None,
    depth: DepthFrame | None,
) -> None:
    scn = viewer.user_scn
    scn.ngeom = 0
    if cloud is not None:
        _add_spheres(
            scn,
            _lidar_world(model, data, cloud),
            radius=_LIDAR_RADIUS,
            rgba=_LIDAR_RGBA,
            max_n=_MAX_LIDAR_DOTS,
        )
    if depth is not None:
        _add_spheres(
            scn,
            _depth_world(model, data, depth),
            radius=_DEPTH_RADIUS,
            rgba=_DEPTH_RGBA,
            max_n=_DEPTH_GRID[0] * _DEPTH_GRID[1],
        )


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
) -> str:
    imu = obs.sensors.imu_pelvis
    gyro = "—" if imu is None else f"{np.linalg.norm(imu.gyro):.3f}"
    acc = "—" if imu is None else f"{np.linalg.norm(imu.accel):.2f}"
    return (
        f"t={obs.timestamp:7.3f}s  z={obs.base_state.position[2]:.3f}m  "
        f"lidar={lidar_n:5d}  depth_valid={depth_valid:6d}  "
        f"|gyro|={gyro}  |acc|={acc}"
    )


def _run_viewer(sim: G1Simulacrum, q_hold: np.ndarray, *, pin_base: bool) -> None:
    control_hz = sim.config.controller.control_hz
    sync_every = max(1, int(round(control_hz / 60.0)))
    last_cloud: PointCloud | None = None
    last_depth: DepthFrame | None = None
    last_lidar_n = 0
    last_depth_valid = 0
    pin_note = (
        "Pelvis welded for inspection (not a balance controller). --free-base lets it fall.\n"
        if pin_base
        else "Floating base is free — PD hold will not keep it standing.\n"
    )
    print(
        pin_note
        + "Viewer: green = Mid-360, cyan = D435i depth subsample.\n"
        "Press C to cycle free cam → d435i_rgb → d435i_depth\n"
        "(or open the right-hand panel and pick Camera under Rendering).\n"
        "Close the window to exit."
    )
    cam_names = ("d435i_rgb", "d435i_depth")
    cam_ids = [
        mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, n) for n in cam_names
    ]
    cam_i = {"i": -1}

    def _on_key(keycode: int) -> None:
        # GLFW_KEY_C = 67; cycle free → rgb → depth.
        if keycode != 67:
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
            obs = sim.step(q_hold)
            if obs.sensors.lidar is not None:
                last_cloud = obs.sensors.lidar
                last_lidar_n = last_cloud.num_points
            if obs.sensors.depth is not None:
                last_depth = obs.sensors.depth
                last_depth_valid = int(np.count_nonzero(last_depth.depth > 0))
            step_i += 1
            if step_i % sync_every == 0:
                _paint(viewer, sim.model, sim.data, last_cloud, last_depth)
                viewer.sync()
                now = time.perf_counter()
                if now - last_print >= 0.5:
                    print(_hud(obs, lidar_n=last_lidar_n, depth_valid=last_depth_valid), flush=True)
                    last_print = now


def _run_headless(sim: G1Simulacrum, q_hold: np.ndarray) -> None:
    for i in range(50):
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
        "--free-base",
        action="store_true",
        help="do not pin the floating base (robot will fall; PD is joints-only)",
    )
    args = parser.parse_args()

    sim = G1Simulacrum.from_config("configs/default.yaml")
    scene = None if args.empty else _INSPECT_XML
    pin_base = not args.free_base
    if scene is not None and pin_base:
        # Match g1_29dof pelvis pos; the inspect weld holds this pose.
        sim.config.robot.spawn_pos = [0.0, 0.0, 0.793]
    sim.build(scene_xml=scene)
    obs = sim.reset()
    if args.free_base and sim.model.neq:
        sim.data.eq_active[:] = 0
        mujoco.mj_forward(sim.model, sim.data)
    q_hold = obs.joint_state.position.copy()
    print(
        f"compiled {sim.compiled.xml_path.name}  "
        f"body={len(sim.compiled.body_joint_ids)} hand={len(sim.compiled.hand_joint_ids)}  "
        f"mid360={sim.config.sensors.mid360.enabled} d435i={sim.config.sensors.d435i.enabled}"
    )
    if args.headless:
        _run_headless(sim, q_hold)
        return
    _run_viewer(sim, q_hold, pin_base=pin_base)


if __name__ == "__main__":
    main()
