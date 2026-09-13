"""Inspect-style lidar/depth overlays for the SONIC GLFW viewer.

Copied from ``examples/01_empty_arena.py`` so the DDS loop can show the same
green Mid-360 / cyan depth / orange FOV / depth PiP without blocking LowState.
Sparse by default: overlays are debug, not the sensor sample count.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
from numpy.typing import NDArray

from g1_simulacrum.sensors.data_types import DepthFrame, PointCloud

_LIDAR_SITE = "mid360"
_IDENTITY_MAT = np.eye(3, dtype=np.float64).reshape(9)
_LIDAR_RGBA = np.array([0.15, 0.95, 0.25, 0.55], dtype=np.float32)
_DEPTH_RGBA = np.array([0.15, 0.75, 0.95, 0.7], dtype=np.float32)
_FRUSTUM_RGBA = np.array([0.95, 0.55, 0.15, 0.85], dtype=np.float32)
_FRUSTUM_LEN = 0.18
_PIP_W = 320
_PIP_H = 240
_PIP_NEAR = 0.3
_PIP_FAR = 3.0


@dataclass
class OverlayConfig:
    lidar_dots: int = 1800  # 0 = every Mid-360 return (~24k)
    depth_stride: int = 16
    lidar_radius: float = 0.012
    depth_radius: float = 0.018


# Inspect-viewer density only. Does not change lidar/depth sample counts.
OVERLAY_PRESETS: dict[str, dict[str, float | int]] = {
    "sparse": {
        "lidar_dots": 1800,
        "depth_stride": 16,
        "lidar_radius": 0.012,
        "depth_radius": 0.018,
    },
    "dense": {
        "lidar_dots": 0,
        "depth_stride": 4,
        "lidar_radius": 0.006,
        "depth_radius": 0.008,
    },
    "full": {
        "lidar_dots": 0,
        "depth_stride": 2,
        "lidar_radius": 0.004,
        "depth_radius": 0.005,
    },
}


def overlay_from_preset(name: str) -> OverlayConfig:
    if name not in OVERLAY_PRESETS:
        known = ", ".join(OVERLAY_PRESETS)
        raise ValueError(f"unknown overlay preset {name!r} (known: {known})")
    cfg = OverlayConfig()
    for key, value in OVERLAY_PRESETS[name].items():
        setattr(cfg, key, value)
    return cfg


def _site_world(
    model: mujoco.MjModel, data: mujoco.MjData, name: str
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if sid < 0:
        raise ValueError(f"missing site {name!r}")
    return data.site_xpos[sid], data.site_xmat[sid].reshape(3, 3)


def _lidar_world(
    model: mujoco.MjModel, data: mujoco.MjData, cloud: PointCloud
) -> NDArray[np.float64]:
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


def _add_capsule(
    scn: mujoco.MjvScene,
    a: NDArray[np.float64],
    b: NDArray[np.float64],
    *,
    radius: float,
    rgba: NDArray[np.float32],
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
        rgba,
    )
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, a, b)
    scn.ngeom += 1


def _paint_depth_frustum(scn: mujoco.MjvScene, model: mujoco.MjModel, data: mujoco.MjData) -> None:
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "d435i_depth")
    if cid < 0:
        return
    fovy = float(model.cam_fovy[cid])
    res = model.cam_resolution[cid]
    aspect = float(res[0]) / float(res[1]) if res[1] else 4.0 / 3.0
    z = _FRUSTUM_LEN
    half_h = z * np.tan(np.deg2rad(fovy) * 0.5)
    half_w = half_h * aspect
    corners_cam = np.array(
        [
            [half_w, half_h, -z],
            [-half_w, half_h, -z],
            [-half_w, -half_h, -z],
            [half_w, -half_h, -z],
        ],
        dtype=np.float64,
    )
    pos = data.cam_xpos[cid]
    rot = data.cam_xmat[cid].reshape(3, 3)
    corners = pos + corners_cam @ rot.T
    for p in corners:
        _add_capsule(scn, pos, p, radius=0.003, rgba=_FRUSTUM_RGBA)
    for i in range(4):
        _add_capsule(scn, corners[i], corners[(i + 1) % 4], radius=0.003, rgba=_FRUSTUM_RGBA)


def _colorize_depth(depth: NDArray[np.float32]) -> NDArray[np.uint8]:
    ys = np.linspace(0, depth.shape[0] - 1, _PIP_H).astype(np.int32)
    xs = np.linspace(0, depth.shape[1] - 1, _PIP_W).astype(np.int32)
    d = np.asarray(depth[np.ix_(ys, xs)], dtype=np.float32)
    t = np.clip((d - _PIP_NEAR) / (_PIP_FAR - _PIP_NEAR), 0.0, 1.0)
    u = 1.0 - t
    r = np.clip(1.5 - np.abs(4.0 * u - 3.0), 0.0, 1.0)
    g = np.clip(1.5 - np.abs(4.0 * u - 2.0), 0.0, 1.0)
    b = np.clip(1.5 - np.abs(4.0 * u - 1.0), 0.0, 1.0)
    rgb = np.stack(
        [(255.0 * r).astype(np.uint8), (255.0 * g).astype(np.uint8), (255.0 * b).astype(np.uint8)],
        axis=-1,
    )
    rgb[d <= 0.0] = 0
    return rgb


def paint_depth_pip(viewer: mujoco.viewer.Handle, frame: DepthFrame | None) -> None:
    if frame is None:
        viewer.clear_images()
        return
    vp = viewer.viewport
    if vp is None or int(vp.width) < _PIP_W + 16 or int(vp.height) < _PIP_H + 16:
        return
    rect = mujoco.MjrRect(int(vp.width) - _PIP_W - 12, 12, _PIP_W, _PIP_H)
    image = np.ascontiguousarray(_colorize_depth(frame.depth))
    viewer.set_images((rect, image))


def configure_overlay_viewer(viewer: mujoco.viewer.Handle) -> None:
    cam_flag = getattr(mujoco.mjtVisFlag, "mjVIS_CAMERA", None)
    if cam_flag is not None:
        viewer.opt.flags[cam_flag] = False
    shadow = getattr(mujoco.mjtRndFlag, "mjRND_SHADOW", None)
    refl = getattr(mujoco.mjtRndFlag, "mjRND_REFLECTION", None)
    if shadow is not None:
        viewer.user_scn.flags[shadow] = 0
    if refl is not None:
        viewer.user_scn.flags[refl] = 0


def paint_sensor_overlay(
    viewer: mujoco.viewer.Handle,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    cloud: PointCloud | None,
    depth: DepthFrame | None,
    overlay: OverlayConfig,
    *,
    inited: list[int],
    drawn: list[int],
    refresh_clouds: bool,
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
    _paint_depth_frustum(scn, model, data)
