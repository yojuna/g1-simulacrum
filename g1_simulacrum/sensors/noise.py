"""Sensor noise models.

Datasheet 1σ starting points and labeled placeholders (Architecture).
Not calibrated from robot logs.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
from scipy import ndimage


# ---------------------------------------------------------------------------
# LiDAR noise (Mid-360)
# ---------------------------------------------------------------------------

def apply_lidar_noise(
    points: NDArray[np.float32],
    *,
    range_sigma: float = 0.02,
    dropout_rate: float = 0.02,
    near_field_rate: float = 0.01,
    near_field_max: float = 0.3,
    rng: np.random.Generator | None = None,
) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
    """Apply datasheet-1σ range noise plus labeled dropout/clutter placeholders.

    Not calibrated from robot logs (wiki/sim-fidelity.md).

    1. Range noise — Gaussian on radial distance (Livox ≤2 cm @ 10 m start).
    2. Near-field clutter — placeholder internal-reflection stand-in.
    3. Random dropout — placeholder missed returns.

    Returns:
        points: (M, 3) noised point cloud (M <= N after dropout).
        intensities: (M,) synthetic intensity values.
    """
    if rng is None:
        rng = np.random.default_rng()

    n = len(points)
    if n == 0:
        return points, np.empty(0, dtype=np.float32)

    points = points.copy()

    # 1. Range noise
    if range_sigma > 0:
        ranges = np.linalg.norm(points, axis=1, keepdims=True)
        ranges = np.maximum(ranges, 1e-6)
        directions = points / ranges
        noise = rng.normal(0, range_sigma, size=(n, 1)).astype(np.float32)
        points = directions * (ranges + noise)

    # 2. Near-field clutter
    if near_field_rate > 0:
        clutter_mask = rng.random(n) < near_field_rate
        n_clutter = clutter_mask.sum()
        if n_clutter > 0:
            clutter_ranges = rng.uniform(0, near_field_max, size=(n_clutter, 1)).astype(np.float32)
            directions_c = points[clutter_mask] / np.maximum(
                np.linalg.norm(points[clutter_mask], axis=1, keepdims=True), 1e-6
            )
            points[clutter_mask] = directions_c * clutter_ranges

    # 3. Dropout
    if dropout_rate > 0:
        keep_mask = rng.random(n) >= dropout_rate
        points = points[keep_mask]
    else:
        keep_mask = np.ones(n, dtype=bool)

    # Synthetic intensities (uniform baseline with distance falloff)
    ranges_final = np.linalg.norm(points, axis=1)
    intensities = np.clip(200.0 / (ranges_final + 1.0), 10, 255).astype(np.float32)

    return points, intensities


# ---------------------------------------------------------------------------
# Depth camera noise (D435i stereo-matching artifacts)
# ---------------------------------------------------------------------------

def apply_depth_noise(
    depth: NDArray[np.float32],
    *,
    edge_erosion: bool = True,
    sigma: float = 0.005,
    hole_rate: float = 0.01,
    min_range: float = 0.3,
    max_range: float = 3.0,
    rng: np.random.Generator | None = None,
) -> NDArray[np.float32]:
    """Apply a pinhole stereo stand-in (wiki). Not a RealSense.

    Callers should pass config ranges (default YAML: 0.3–3 m). Do not treat
    0.105–10 m as equal quality.

    1. Range clipping — outside [min_range, max_range] set to 0 (invalid).
    2. Edge erosion — depth discontinuities get eroded (stereo-matching artifact).
    3. Distance-dependent Gaussian noise — precision degrades with range.
    4. Random holes — fraction of pixels set to 0 (failed stereo match).

    Returns:
        depth: (H, W) noised depth map. 0 = invalid.
    """
    if rng is None:
        rng = np.random.default_rng()

    depth = depth.copy()

    # 1. Range clipping
    valid = (depth > min_range) & (depth < max_range)
    depth[~valid] = 0.0

    # 2. Edge erosion
    if edge_erosion:
        depth = _erode_depth_edges(depth)

    # 3. Distance-dependent noise
    if sigma > 0:
        # D435i depth precision degrades quadratically with distance
        # σ ≈ sigma * (depth² / baseline_focal)
        noise_scale = sigma * (depth ** 2)
        noise = rng.normal(0, 1, size=depth.shape).astype(np.float32) * noise_scale
        depth = depth + noise
        depth[depth < 0] = 0.0

    # 4. Random holes
    if hole_rate > 0:
        holes = rng.random(depth.shape) < hole_rate
        depth[holes] = 0.0

    return depth


def _erode_depth_edges(depth: NDArray[np.float32], threshold: float = 0.05) -> NDArray[np.float32]:
    """Remove depth at object boundaries where stereo matching fails.

    Detects edges via Sobel gradient on the depth map, then zeros out pixels
    with high depth gradient (simulating the D435i's inability to resolve
    stereo matches at occlusion boundaries).
    """
    # Compute depth gradient magnitude
    gx = ndimage.sobel(depth, axis=1)
    gy = ndimage.sobel(depth, axis=0)
    gradient_mag = np.sqrt(gx ** 2 + gy ** 2)

    # Threshold relative to local depth (edges are where gradient > threshold * depth)
    edge_mask = gradient_mag > (threshold * np.maximum(depth, 0.1))

    # Dilate the edge mask slightly (stereo artifacts are a few pixels wide)
    edge_mask = ndimage.binary_dilation(edge_mask, iterations=2)

    depth[edge_mask] = 0.0
    return depth
