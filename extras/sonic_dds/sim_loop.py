"""SONIC sim2sim loop: G1Simulacrum + vendored UnitreeSdk2Bridge.

Mirrors gear_sonic BaseSimulator: one process owns MuJoCo, the GLFW viewer,
and DDS PublishLowState / LowCmd actuation. There is no separate sim process.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Callable

import mujoco
import mujoco.viewer
import numpy as np

from g1_simulacrum import G1Simulacrum
from g1_simulacrum.config import ControllerConfig, G1SimulacrumConfig
from g1_simulacrum.gantry import ElasticBand, quat_wxyz_from_yaw
from g1_simulacrum.model.joints import BODY_JOINT_NAMES
from g1_simulacrum.sensors.data_types import DepthFrame, PointCloud

from .channel import init_channel
from .lowcmd_actuation import (
    apply_torques,
    compute_body_torques,
    compute_hand_torques,
)
from .observation_adapter import prepare_obs_dict
from .overlay import (
    OverlayConfig,
    configure_overlay_viewer,
    paint_depth_pip,
    paint_sensor_overlay,
)
from .unitree_sdk2py_bridge import UnitreeSdk2Bridge
from .wbc_config import DEFAULT_STANDING_Q, SonicDdsConfig, bridge_config_dict


def pick_budgeted_sensor(lidar_due: bool, depth_due: bool, last: str) -> str | None:
    """At most one leftover-budget sensor per tick. Alternate when both are due.

    Always preferring lidar starves D435i: skipped scans stay due, so every
    leftover slot is claimed by Mid-360 and depth stays at 0 Hz.
    """
    if lidar_due and depth_due:
        return "depth" if last == "lidar" else "lidar"
    if lidar_due:
        return "lidar"
    if depth_due:
        return "depth"
    return None

# GLFW keycodes — GEAR ElasticBand uses number-row 7/8/9; inspect viewer uses numpad.
_GLFW_KEY_7 = 55
_GLFW_KEY_8 = 56
_GLFW_KEY_9 = 57
_GLFW_KEY_KP_2 = 322
_GLFW_KEY_KP_4 = 324
_GLFW_KEY_KP_5 = 325
_GLFW_KEY_KP_6 = 326
_GLFW_KEY_KP_7 = 327
_GLFW_KEY_KP_8 = 328
_GLFW_KEY_KP_9 = 329
_GLFW_KEY_KP_SUBTRACT = 333
_GLFW_KEY_KP_ADD = 334
_GANTRY_STEP_XY = 0.05
_GANTRY_STEP_YAW = float(np.deg2rad(5.0))


def _attach_body_id(model: mujoco.MjModel, name: str) -> int:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid >= 0:
        return bid
    for fallback in ("torso_link", "pelvis"):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, fallback)
        if bid >= 0:
            return bid
    raise ValueError(f"missing gantry attach body {name!r}")


def apply_gear_parity(sim_config: G1SimulacrumConfig, sonic_config: SonicDdsConfig) -> None:
    """Match GEAR's 200 Hz LowCmd loop without widening our 1 ms MJCF timestep.

    GEAR uses one ``mj_step`` at 5 ms on *their* XML. That dt NaNs on
    g1-sensorized. Keep ``model.opt.timestep`` (1 ms) and take 5 substeps
    per 200 Hz control tick so wall time and policy rate match GEAR.
    """
    sim_config.controller.physics_hz = 1000.0
    sim_config.controller.control_hz = 200.0
    sonic_config.loop.control_hz = 200.0


def apply_gear_dof_dissipation(model: mujoco.MjModel) -> None:
    """Copy gear_sonic joint defaults (damping/armature/frictionloss) onto hinges."""
    for i in range(6, model.nv):
        model.dof_damping[i] = 0.05
        model.dof_armature[i] = 0.01
        model.dof_frictionloss[i] = 0.2


def disable_dds_cameras(sim_config: G1SimulacrumConfig) -> None:
    """Do not construct Mid-360 / D435i when the DDS loop is motion-only."""
    sim_config.sensors.mid360.enabled = False
    sim_config.sensors.d435i.enabled = False
    sim_config.sensors.imu.mid360.enabled = False
    sim_config.sensors.imu.d435i.enabled = False


def _print_gantry(gantry: ElasticBand) -> None:
    print(
        f"gantry mode={gantry.mode} enable={gantry.enable} length={gantry.length:.2f} "
        f"point=({gantry.point[0]:.2f}, {gantry.point[1]:.2f}, {gantry.point[2]:.2f})",
        flush=True,
    )


class SonicDdsSimLoop:
    """Drive G1Simulacrum from SONIC deploy over Unitree DDS (gear_sonic pattern)."""

    def __init__(
        self,
        sim_config: G1SimulacrumConfig,
        sonic_config: SonicDdsConfig,
        *,
        scene_xml: str | Path | None = None,
        onscreen: bool | None = None,
        overlay: OverlayConfig | None = None,
    ) -> None:
        self._sonic = sonic_config
        self._wbc = bridge_config_dict(sonic_config)
        self._onscreen = sonic_config.loop.onscreen if onscreen is None else onscreen

        if sonic_config.hands != sim_config.robot.hands:
            raise ValueError(
                f"sonic hands={sonic_config.hands!r} != sim robot.hands={sim_config.robot.hands!r}"
            )

        if not sonic_config.loop.cameras:
            disable_dds_cameras(sim_config)

        sim_config.controller = ControllerConfig(
            type="passthrough",
            physics_hz=1000.0,
            control_hz=sonic_config.loop.control_hz,
        )
        if sonic_config.loop.gear_parity:
            apply_gear_parity(sim_config, sonic_config)
            sim_config.controller.control_hz = sonic_config.loop.control_hz

        print("Building MuJoCo model…", flush=True)
        self._sim = G1Simulacrum(config=sim_config)
        self._sim.build(scene_xml=scene_xml)
        apply_gear_dof_dissipation(self._sim.model)

        physics_hz = sim_config.controller.physics_hz
        control_hz = sim_config.controller.control_hz
        substeps = max(1, int(round(physics_hz / control_hz)))
        cam = "on" if sonic_config.loop.cameras else "off"
        budget = sonic_config.loop.sensor_budget
        sensors_how = (
            f"budgeted leftover (slot {budget:.2f} reserved for LowState+mj_step; "
            "not in step_physics)"
            if sonic_config.loop.cameras
            else "off"
        )
        print(
            f"physics: dt={self._sim.model.opt.timestep}  "
            f"physics_hz={physics_hz}  control_hz={control_hz}  "
            f"substeps={substeps}  loop_hz={sonic_config.loop.control_hz}  "
            f"gantry={sonic_config.gantry.mode}@{sonic_config.gantry.attach_body}  "
            f"cameras={cam}  sensors={sensors_how}",
            flush=True,
        )

        iface = self._wbc.get("INTERFACE") or "(auto)"
        print(
            f"DDS: ChannelFactoryInitialize domain={self._wbc['DOMAIN_ID']} interface={iface}",
            flush=True,
        )
        init_channel(self._wbc)
        self._bridge = UnitreeSdk2Bridge(self._wbc)
        if self._wbc.get("USE_JOYSTICK"):
            self._bridge.SetupJoystick(
                device_id=self._wbc["JOYSTICK_DEVICE"],
                js_type=self._wbc["JOYSTICK_TYPE"],
            )

        self._gantry: ElasticBand | None = None
        self._attach_id = _attach_body_id(
            self._sim.model, sonic_config.gantry.attach_body
        )
        self._control_dt = 1.0 / sonic_config.loop.control_hz
        self._sync_every = max(
            1, int(round(sonic_config.loop.control_hz * sonic_config.loop.viewer_dt))
        )
        self._running = False
        self._viewer: mujoco.viewer.Handle | None = None
        self._lowstate_count = 0
        self._lowstate_t0 = 0.0
        self._lowstate_n0 = 0
        self._lowstate_sim_t0 = 0.0
        self._lowstate_stats_interval_s = float(sonic_config.loop.stats_log_interval_s)
        self._recover_count = 0
        self._overlay = overlay if overlay is not None else OverlayConfig()
        self._overlay_inited = [0]
        self._overlay_drawn = [0, 0]
        self._overlay_refresh = False
        self._last_cloud: PointCloud | None = None
        self._last_depth: DepthFrame | None = None
        self._lidar_ok = 0
        self._depth_ok = 0
        self._sensor_skip = 0
        self._sensor_turn = ""
        self._bringup_done = not (
            sonic_config.gantry.enabled and sonic_config.gantry.bringup.enabled
        )
        self._bringup_released = False
        self._bringup_lower_ticks = 0
        self._bringup_lower_wait_ticks = 0
        self._bringup_settle_t0: float | None = None
        self._bringup_cmd_t0: float | None = None
        self._bringup_waiting_cmd_announced = False
        self._bringup_standing_announced = False
        self._plant_state_path = Path(
            "/workspace/docker/ws_sonic_redux/logs/.plant_state.json"
        )
        self._plant_state_tick = 0

    def _tick_gantry_bringup(self) -> None:
        cfg = self._sonic.gantry.bringup
        if self._bringup_done or not cfg.enabled or self._gantry is None:
            return

        cmd = self._bridge.cmd_received()
        if cfg.wait_for_lowcmd_to_lower and not cmd:
            return

        if self._bringup_lower_ticks < cfg.lower_steps:
            interval = max(1, int(cfg.lower_interval_ticks))
            self._bringup_lower_wait_ticks += 1
            if self._bringup_lower_wait_ticks % interval != 0:
                return
            step = float(cfg.lower_step_m)
            if self._gantry.mode == "gear":
                self._gantry.length -= step
            else:
                self._gantry.length = max(0.0, self._gantry.length - step)
            self._bringup_lower_ticks += 1
            if self._bringup_lower_ticks == 1:
                print("gantry bringup: lowering (auto)", flush=True)
            if self._bringup_lower_ticks == cfg.lower_steps:
                _print_gantry(self._gantry)
            return

        if self._bringup_settle_t0 is None:
            self._bringup_settle_t0 = time.monotonic()
            pelvis_z = float(self._sim.data.qpos[2])
            print(
                f"gantry bringup: lowered, settling {cfg.settle_s:.1f}s "
                f"(pelvis_z={pelvis_z:.3f})",
                flush=True,
            )
            return

        if time.monotonic() - self._bringup_settle_t0 < cfg.settle_s:
            return

        if cfg.release and cfg.wait_for_lowcmd_to_release and not cmd:
            self._bringup_cmd_t0 = None
            if not self._bringup_waiting_cmd_announced:
                print(
                    "gantry bringup: waiting for deploy ] (LowCmd) before release",
                    flush=True,
                )
                self._bringup_waiting_cmd_announced = True
            return

        post_settle = float(cfg.post_cmd_settle_s)
        if cfg.release and post_settle > 0.0:
            if self._bringup_cmd_t0 is None:
                self._bringup_cmd_t0 = time.monotonic()
                pelvis_z = float(self._sim.data.qpos[2])
                if cfg.release_harness_on_lowcmd and self._gantry is not None:
                    self._gantry.enable = False
                    self._bringup_released = True
                    print(
                        f"gantry bringup: LowCmd active — harness OFF, "
                        f"balancing {post_settle:.1f}s on policy (pelvis_z={pelvis_z:.3f})",
                        flush=True,
                    )
                else:
                    print(
                        f"gantry bringup: LowCmd active — balancing {post_settle:.1f}s "
                        f"with harness (pelvis_z={pelvis_z:.3f})",
                        flush=True,
                    )
                return
            if time.monotonic() - self._bringup_cmd_t0 < post_settle:
                return

        pelvis_z = float(self._sim.data.qpos[2])
        if cfg.release and cfg.require_standing_to_release:
            if pelvis_z < cfg.min_pelvis_z or pelvis_z > cfg.max_pelvis_z:
                if not self._bringup_standing_announced:
                    print(
                        f"gantry bringup: waiting for stable stand "
                        f"(pelvis_z={pelvis_z:.3f}, want {cfg.min_pelvis_z:.2f}–{cfg.max_pelvis_z:.2f})",
                        flush=True,
                    )
                    self._bringup_standing_announced = True
                return
            self._bringup_standing_announced = False

        if cfg.release:
            if not self._bringup_released:
                if self._gantry is not None:
                    self._gantry.enable = False
                self._bringup_released = True
                print(
                    f"gantry bringup: RELEASED (pelvis_z={pelvis_z:.3f}) — balancing on ground",
                    flush=True,
                )
            else:
                print(
                    f"gantry bringup: done (pelvis_z={pelvis_z:.3f}) — policy on ground",
                    flush=True,
                )
        else:
            print(f"gantry bringup: done (pelvis_z={pelvis_z:.3f}, crane still on)", flush=True)
        self._bringup_done = True

    def _write_plant_state(self) -> None:
        self._plant_state_tick += 1
        if self._plant_state_tick % 10 != 0:
            return
        pelvis_z = float(self._sim.data.qpos[2])
        gantry_on = bool(self._gantry.enable) if self._gantry is not None else False
        fallen = pelvis_z < self._sonic.loop.fall_height
        standing = (
            self._sonic.gantry.bringup.min_pelvis_z
            <= pelvis_z
            <= self._sonic.gantry.bringup.max_pelvis_z
        )
        payload = {
            "pelvis_z": pelvis_z,
            "gantry_enabled": gantry_on,
            "bringup_done": self._bringup_done,
            "bringup_released": self._bringup_released,
            "lowcmd": self._bridge.cmd_received(),
            "sim_time": float(self._sim.data.time),
            "recover_count": self._recover_count,
            "standing": standing and not fallen,
            "fallen": fallen,
        }
        tmp = self._plant_state_path.with_suffix(".tmp")
        self._plant_state_path.parent.mkdir(parents=True, exist_ok=True)
        tmp.write_text(json.dumps(payload))
        tmp.replace(self._plant_state_path)

    @property
    def sim(self) -> G1Simulacrum:
        return self._sim

    @property
    def bridge(self) -> UnitreeSdk2Bridge:
        return self._bridge

    def setup_gantry(self) -> ElasticBand:
        self._gantry = self._make_gantry()
        return self._gantry

    def _make_gantry(self) -> ElasticBand:
        pos = self._sim.data.qpos[0:3].copy()
        quat = quat_wxyz_from_yaw(0.0)
        if self._sonic.gantry.mode == "gear":
            return ElasticBand.gear_hold(pos, quat_wxyz=quat)
        return ElasticBand.overhead(pos, quat_wxyz=quat)

    def _on_key(self, keycode: int) -> None:
        gantry = self._gantry
        if gantry is None:
            return
        if keycode in (_GLFW_KEY_7, _GLFW_KEY_KP_SUBTRACT):
            # GEAR length is a signed vertical offset; cable rest length is ≥ 0.
            if gantry.mode == "gear":
                gantry.length -= 0.1
            else:
                gantry.length = max(0.0, gantry.length - 0.1)
            _print_gantry(gantry)
            return
        if keycode in (_GLFW_KEY_8, _GLFW_KEY_KP_ADD):
            gantry.length += 0.1
            _print_gantry(gantry)
            return
        if keycode in (_GLFW_KEY_9, _GLFW_KEY_KP_5):
            gantry.enable = not gantry.enable
            print(f"ElasticBand enable: {gantry.enable}", flush=True)
            return
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

    def _open_viewer(self) -> None:
        if not self._onscreen:
            print("MuJoCo viewer: off (--headless)", flush=True)
            return
        try:
            self._viewer = mujoco.viewer.launch_passive(
                self._sim.model,
                self._sim.data,
                key_callback=self._on_key,
                show_left_ui=False,
                show_right_ui=False,
            )
        except Exception as exc:
            print(f"MuJoCo viewer failed ({exc}); continuing headless", flush=True)
            self._viewer = None
            return
        pelvis = mujoco.mj_name2id(self._sim.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self._viewer.cam.azimuth = 120
        self._viewer.cam.elevation = -30
        self._viewer.cam.distance = 2.0
        self._viewer.cam.lookat[:] = np.array([0.0, 0.0, 0.5])
        if pelvis >= 0:
            self._viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self._viewer.cam.trackbodyid = pelvis
        if self._sonic.loop.cameras:
            configure_overlay_viewer(self._viewer)
            dots = "all" if self._overlay.lidar_dots <= 0 else str(self._overlay.lidar_dots)
            print(
                "Overlay: green Mid-360, cyan depth, orange FOV, depth PiP "
                f"(lidar_dots={dots}, "
                f"depth_stride={self._overlay.depth_stride})",
                flush=True,
            )
        print(
            "MuJoCo viewer: on (GEAR keys 7/8 hold height, 9 toggle; numpad trolley like inspect)",
            flush=True,
        )

    def _close_viewer(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None

    def reset(self) -> None:
        data = self._sim.data
        data.xfrc_applied[:] = 0.0
        data.qacc[:] = 0.0
        self._sim.reset()
        self._apply_standing_pose()
        mujoco.mj_forward(self._sim.model, self._sim.data)
        self._bridge.reset()
        self._last_cloud = None
        self._last_depth = None
        self._overlay_refresh = True
        if self._sonic.gantry.enabled:
            if self._bringup_released and self._gantry is not None:
                # Fall recovery after crane release: keep harness off.
                self._gantry.enable = False
            else:
                self._gantry = self._make_gantry()

    def _apply_standing_pose(self) -> None:
        compiled = self._sim.compiled
        data = self._sim.data
        for i, name in enumerate(BODY_JOINT_NAMES):
            data.qpos[compiled.body_qposadr[name]] = DEFAULT_STANDING_Q[i]

    def _state_exploded(self) -> bool:
        data = self._sim.data
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qacc).all():
            return True
        z = float(data.qpos[2])
        return not np.isfinite(z) or z < self._sonic.loop.fall_height

    def _recover_if_exploded(self, where: str) -> bool:
        if not self._state_exploded():
            return False
        data = self._sim.data
        z = float(data.qpos[2]) if np.isfinite(data.qpos[2]) else float("nan")
        self._recover_count += 1
        if self._recover_count <= 3 or self._recover_count % 50 == 0:
            print(
                f"Warning: sim reset ({where}) time={data.time:.4f} z={z:.3f} "
                f"count={self._recover_count}",
                flush=True,
            )
        self.reset()
        return True

    def sim_step(self) -> None:
        """One control tick: publish state, apply LowCmd torques, integrate physics."""
        data = self._sim.data
        compiled = self._sim.compiled
        loop = self._sonic.loop

        self._recover_if_exploded("pre-step")

        if loop.publish_before_step:
            obs_dict = prepare_obs_dict(compiled, data)
            self._bridge.PublishLowState(obs_dict)
            self._lowstate_count += 1
            if self._lowstate_count == 1:
                print(
                    "Publishing rt/lowstate (deploy should stop waiting for LowState)",
                    flush=True,
                )
                self._lowstate_t0 = time.monotonic()
                self._lowstate_n0 = 1
                self._lowstate_sim_t0 = float(data.time)
            elif (
                self._lowstate_stats_interval_s > 0.0
                and time.monotonic() - self._lowstate_t0 >= self._lowstate_stats_interval_s
            ):
                wall = time.monotonic() - self._lowstate_t0
                n = self._lowstate_count - self._lowstate_n0
                sim_dt = float(data.time) - self._lowstate_sim_t0
                realtime = sim_dt / wall if wall > 0 else 0.0
                extra = ""
                if loop.cameras:
                    extra = (
                        f"  lidar={self._lidar_ok / wall:.1f} Hz "
                        f"depth={self._depth_ok / wall:.1f} Hz "
                        f"skip={self._sensor_skip}"
                    )
                    self._lidar_ok = 0
                    self._depth_ok = 0
                    self._sensor_skip = 0
                print(
                    f"rt/lowstate published {self._lowstate_count} times  "
                    f"~{n / wall:.0f} Hz  sim/wall={realtime:.2f}{extra}",
                    flush=True,
                )
                self._lowstate_t0 = time.monotonic()
                self._lowstate_n0 = self._lowstate_count
                self._lowstate_sim_t0 = float(data.time)
            if self._bridge.joystick:
                self._bridge.PublishWirelessController()

        if self._gantry is not None and self._sonic.gantry.enabled:
            self._tick_gantry_bringup()
            if self._gantry.enable:
                self._gantry.apply(self._sim.model, data, self._attach_id)

        if loop.wait_for_cmd and not self._bridge.cmd_received():
            mujoco.mj_forward(self._sim.model, data)
            return

        body_tau = compute_body_torques(compiled, data, self._bridge)
        left_tau, right_tau = compute_hand_torques(compiled, data, self._bridge)
        apply_torques(
            compiled,
            data,
            body_tau,
            left_tau,
            right_tau,
            torque_limits=tuple(self._wbc["motor_effort_limit_list"]),
        )

        self._sim.step_physics(sensors=False)
        self._recover_if_exploded("post-step")
        self._write_plant_state()

    def _sensors_due(self) -> bool:
        t = float(self._sim.data.time)
        mgr = self._sim.sensor_manager
        return mgr.lidar_due(t) or mgr.depth_due(t)

    def _maybe_step_sensors(self, deadline: float) -> None:
        """Leftover-budget lidar/depth. Stopgap — see wiki/sim-process-model.md.

        Only start a scan if this control slot's wall deadline is still in the
        future. A scan may still overrun; ``run`` then skips sleep until the
        absolute schedule catches up so RTF can return to 1.
        """
        if not self._sonic.loop.cameras:
            return
        due = self._sensors_due()
        now = time.monotonic()
        slot_start = deadline - self._control_dt
        budget_s = self._sonic.loop.sensor_budget * self._control_dt
        if now >= deadline or (now - slot_start) >= budget_s:
            if due:
                self._sensor_skip += 1
            return
        t = float(self._sim.data.time)
        mgr = self._sim.sensor_manager
        which = pick_budgeted_sensor(
            mgr.lidar_due(t), mgr.depth_due(t), self._sensor_turn
        )
        if which == "lidar":
            cloud = mgr.step_lidar(t)
            if cloud is not None:
                self._last_cloud = cloud
                self._lidar_ok += 1
                self._overlay_refresh = True
            self._sensor_turn = "lidar"
            return
        if which == "depth":
            depth = mgr.step_depth(t)
            if depth is not None:
                self._last_depth = depth
                self._depth_ok += 1
                self._overlay_refresh = True
            self._sensor_turn = "depth"

    def _paint_overlays(self) -> None:
        if self._viewer is None or not self._sonic.loop.cameras:
            return
        paint_sensor_overlay(
            self._viewer,
            self._sim.model,
            self._sim.data,
            self._last_cloud,
            self._last_depth,
            self._overlay,
            inited=self._overlay_inited,
            drawn=self._overlay_drawn,
            refresh_clouds=self._overlay_refresh,
        )
        paint_depth_pip(self._viewer, self._last_depth)
        self._overlay_refresh = False

    def run(
        self,
        *,
        duration_s: float | None = None,
        on_step: Callable[[], bool] | None = None,
    ) -> None:
        """Blocking loop at control_hz until duration, viewer close, or KeyboardInterrupt."""
        self._running = True
        if self._sonic.gantry.enabled and self._gantry is None:
            self.setup_gantry()
        self._open_viewer()

        t0 = time.monotonic()
        sim_cnt = 0
        try:
            while self._running:
                if self._viewer is not None and not self._viewer.is_running():
                    break
                deadline = t0 + (sim_cnt + 1) * self._control_dt
                if on_step is not None and not on_step():
                    break
                self.sim_step()
                self._maybe_step_sensors(deadline)
                if (
                    sim_cnt % self._sync_every == 0
                    and self._viewer is not None
                    and time.monotonic() < deadline
                ):
                    self._paint_overlays()
                    self._viewer.sync()
                sim_cnt += 1
                if duration_s is not None and (time.monotonic() - t0) >= duration_s:
                    break
                now = time.monotonic()
                if now < deadline:
                    time.sleep(deadline - now)
        except KeyboardInterrupt:
            print("SONIC DDS bridge interrupted.")
        finally:
            self._close_viewer()
            self._running = False

    def stop(self) -> None:
        self._running = False
