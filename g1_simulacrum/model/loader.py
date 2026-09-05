"""MJCF model loading and composition.

Assembles the sensorized G1 by:
1. Loading the base G1 MJCF from Menagerie
2. Injecting sensor mount bodies (Mid-360, D435i) into the kinematic tree
3. Adding MuJoCo sensor definitions (accelerometer, gyro, cameras)
4. Optionally merging with an external scene MJCF
"""

from __future__ import annotations

import importlib.resources
import logging
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

from ..config import G1SimulacrumConfig

logger = logging.getLogger(__name__)

# Known Menagerie paths — resolved at import time if available
_MENAGERIE_G1_PATH: Path | None = None
_MENAGERIE_D435I_PATH: Path | None = None

try:
    import mujoco_menagerie
    _menagerie_root = Path(mujoco_menagerie.__path__[0]) if hasattr(mujoco_menagerie, "__path__") else None
    if _menagerie_root and (_menagerie_root / "unitree_g1").exists():
        _MENAGERIE_G1_PATH = _menagerie_root / "unitree_g1"
    if _menagerie_root and (_menagerie_root / "realsense_d435i").exists():
        _MENAGERIE_D435I_PATH = _menagerie_root / "realsense_d435i"
except ImportError:
    pass


class ModelLoader:
    """Composes the sensorized G1 MJCF model."""

    def __init__(self, config: G1SimulacrumConfig) -> None:
        self._config = config

    def build(self, *, scene_xml: str | None = None) -> mujoco.MjModel:
        """Build and compile the full model.

        Args:
            scene_xml: Path to an external scene MJCF. If provided, the G1
                       is injected into it. Otherwise, a minimal ground-plane
                       scene is created.

        Returns:
            Compiled MjModel.
        """
        # Step 1: Parse the base G1 model
        g1_tree = self._load_g1_base()

        # Step 2: Inject sensor mounts into the G1's body tree
        self._inject_sensor_mounts(g1_tree)

        # Step 3: Add MuJoCo sensor definitions
        self._add_sensor_definitions(g1_tree)

        # Step 4: Compose with scene
        if scene_xml is not None:
            final_tree = self._merge_with_scene(g1_tree, scene_xml)
        else:
            final_tree = self._create_default_scene(g1_tree)

        # Step 5: Write to temp file and compile
        return self._compile(final_tree)

    # ------------------------------------------------------------------
    # Step 1: Load base G1
    # ------------------------------------------------------------------

    def _load_g1_base(self) -> ET.ElementTree:
        """Load the G1 MJCF from Menagerie or bundled assets."""
        g1_path = self._resolve_g1_path()
        logger.info("Loading G1 model from %s", g1_path)
        return ET.parse(g1_path)

    def _resolve_g1_path(self) -> Path:
        """Find the G1 MJCF file, checking Menagerie first."""
        # Check Menagerie installation
        if _MENAGERIE_G1_PATH is not None:
            xml_name = "g1.xml"
            candidate = _MENAGERIE_G1_PATH / xml_name
            if candidate.exists():
                return candidate

        # Fall back to bundled model
        bundled = Path(__file__).parent / "mjcf" / "g1_29dof.xml"
        if bundled.exists():
            return bundled

        raise FileNotFoundError(
            "Cannot find G1 MJCF model. Install mujoco_menagerie or place "
            "the model at g1_simulacrum/model/mjcf/g1_29dof.xml"
        )

    # ------------------------------------------------------------------
    # Step 2: Inject sensor mounts
    # ------------------------------------------------------------------

    def _inject_sensor_mounts(self, tree: ET.ElementTree) -> None:
        """Add sensor bodies to the G1's kinematic tree."""
        root = tree.getroot()
        worldbody = root.find("worldbody")
        if worldbody is None:
            raise ValueError("G1 MJCF has no <worldbody>")

        cfg = self._config.sensors

        # Find the torso body for LiDAR mount
        if cfg.mid360.enabled:
            mount_body = self._find_body(worldbody, cfg.mid360.mount_body)
            if mount_body is not None:
                lidar_xml = self._mid360_body_xml(cfg.mid360)
                lidar_elem = ET.fromstring(f"<root>{lidar_xml}</root>")
                for child in lidar_elem:
                    mount_body.append(child)
                logger.info("Injected Mid-360 mount on '%s'", cfg.mid360.mount_body)

        # Find the head body for D435i mount
        if cfg.d435i.enabled:
            mount_body = self._find_body(worldbody, cfg.d435i.mount_body)
            if mount_body is not None:
                cam_xml = self._d435i_body_xml(cfg.d435i)
                cam_elem = ET.fromstring(f"<root>{cam_xml}</root>")
                for child in cam_elem:
                    mount_body.append(child)
                logger.info("Injected D435i mount on '%s'", cfg.d435i.mount_body)

    def _find_body(self, elem: ET.Element, name: str) -> ET.Element | None:
        """Recursively find a <body name="..."> in the tree."""
        if elem.tag == "body" and elem.get("name") == name:
            return elem
        for child in elem:
            result = self._find_body(child, name)
            if result is not None:
                return result
        return None

    def _mid360_body_xml(self, cfg) -> str:
        px, py, pz = cfg.mount_pos
        qw, qx, qy, qz = cfg.mount_quat
        return f"""
        <body name="mid360_link" pos="{px} {py} {pz}" quat="{qw} {qx} {qy} {qz}">
            <inertial pos="0 0 0" mass="0.265" diaginertia="0.0002 0.0002 0.0002"/>
            <geom type="cylinder" size="0.0325 0.03"
                  rgba="0.15 0.15 0.15 1" contype="0" conaffinity="0" group="1"/>
            <site name="mid360_imu_site" pos="0 0 0"/>
        </body>
        """

    def _d435i_body_xml(self, cfg) -> str:
        px, py, pz = cfg.mount_pos
        qw, qx, qy, qz = cfg.mount_quat
        w, h = cfg.resolution
        return f"""
        <body name="d435i_link" pos="{px} {py} {pz}" quat="{qw} {qx} {qy} {qz}">
            <inertial pos="0 0 0" mass="0.072" diaginertia="0.00005 0.00005 0.00002"/>
            <geom type="box" size="0.0445 0.0125 0.0125"
                  rgba="0.3 0.3 0.3 1" contype="0" conaffinity="0" group="1"/>
            <camera name="d435i_depth" pos="0 0 0" fovy="58" resolution="{w} {h}"/>
            <camera name="d435i_rgb" pos="0 0.015 0" fovy="58" resolution="{w} {h}"/>
            <site name="d435i_imu_site" pos="0 0 0"/>
        </body>
        """

    # ------------------------------------------------------------------
    # Step 3: Sensor definitions
    # ------------------------------------------------------------------

    def _add_sensor_definitions(self, tree: ET.ElementTree) -> None:
        """Add <sensor> block with accelerometers and gyros."""
        root = tree.getroot()
        sensor_elem = root.find("sensor")
        if sensor_elem is None:
            sensor_elem = ET.SubElement(root, "sensor")

        cfg = self._config.sensors

        if cfg.mid360.enabled and cfg.imu.enabled:
            ET.SubElement(sensor_elem, "accelerometer", {
                "name": "mid360_accel",
                "site": "mid360_imu_site",
                "noise": str(cfg.imu.noise.accel_sigma),
            })
            ET.SubElement(sensor_elem, "gyro", {
                "name": "mid360_gyro",
                "site": "mid360_imu_site",
                "noise": str(cfg.imu.noise.gyro_sigma),
            })

        if cfg.d435i.enabled and cfg.imu.enabled:
            ET.SubElement(sensor_elem, "accelerometer", {
                "name": "d435i_accel",
                "site": "d435i_imu_site",
                "noise": str(cfg.imu.noise.accel_sigma),
            })
            ET.SubElement(sensor_elem, "gyro", {
                "name": "d435i_gyro",
                "site": "d435i_imu_site",
                "noise": str(cfg.imu.noise.gyro_sigma),
            })

    # ------------------------------------------------------------------
    # Step 4: Scene composition
    # ------------------------------------------------------------------

    def _create_default_scene(self, g1_tree: ET.ElementTree) -> ET.ElementTree:
        """Wrap the G1 model in a minimal scene with ground plane."""
        scene = ET.Element("mujoco", {"model": "g1_sensorized_scene"})

        # Include common defaults
        ET.SubElement(scene, "option", {
            "timestep": str(1.0 / self._config.controller.physics_hz),
            "gravity": "0 0 -9.81",
        })

        visual = ET.SubElement(scene, "visual")
        ET.SubElement(visual, "headlight", {
            "diffuse": "0.6 0.6 0.6",
            "ambient": "0.3 0.3 0.3",
        })

        # Ground plane
        worldbody = ET.SubElement(scene, "worldbody")
        ET.SubElement(worldbody, "light", {
            "name": "top_light", "pos": "0 0 3",
            "dir": "0 0 -1", "directional": "true",
        })
        ET.SubElement(worldbody, "geom", {
            "name": "floor", "type": "plane",
            "size": "10 10 0.1", "rgba": "0.8 0.8 0.8 1",
            "friction": " ".join(str(f) for f in self._config.environment.ground_friction),
        })

        # Insert the G1 body tree
        g1_root = g1_tree.getroot()
        g1_worldbody = g1_root.find("worldbody")
        if g1_worldbody is not None:
            for child in g1_worldbody:
                worldbody.append(child)

        # Copy over non-worldbody elements (actuators, sensors, defaults, etc.)
        for elem in g1_root:
            if elem.tag != "worldbody":
                scene.append(elem)

        return ET.ElementTree(scene)

    def _merge_with_scene(
        self, g1_tree: ET.ElementTree, scene_path: str
    ) -> ET.ElementTree:
        """Merge the sensorized G1 into an external scene MJCF.

        The G1 body subtree is injected into the scene's <worldbody>.
        Actuators and sensors are appended to existing blocks or created.
        """
        scene_tree = ET.parse(scene_path)
        scene_root = scene_tree.getroot()

        scene_worldbody = scene_root.find("worldbody")
        if scene_worldbody is None:
            scene_worldbody = ET.SubElement(scene_root, "worldbody")

        g1_root = g1_tree.getroot()

        # Insert G1 body tree
        g1_worldbody = g1_root.find("worldbody")
        if g1_worldbody is not None:
            for child in g1_worldbody:
                scene_worldbody.append(child)

        # Merge actuators, sensors, etc.
        for tag in ("actuator", "sensor", "default", "asset"):
            g1_block = g1_root.find(tag)
            if g1_block is not None:
                scene_block = scene_root.find(tag)
                if scene_block is None:
                    scene_root.append(g1_block)
                else:
                    for child in g1_block:
                        scene_block.append(child)

        # Set timestep
        option = scene_root.find("option")
        if option is None:
            option = ET.SubElement(scene_root, "option")
        option.set("timestep", str(1.0 / self._config.controller.physics_hz))

        return scene_tree

    # ------------------------------------------------------------------
    # Step 5: Compile
    # ------------------------------------------------------------------

    def _compile(self, tree: ET.ElementTree) -> mujoco.MjModel:
        """Write the composed MJCF to a temp file and compile."""
        with tempfile.NamedTemporaryFile(
            suffix=".xml", mode="wb", delete=False
        ) as f:
            tree.write(f, encoding="utf-8", xml_declaration=True)
            tmp_path = f.name

        logger.info("Compiling model from %s", tmp_path)
        model = mujoco.MjModel.from_xml_path(tmp_path)
        logger.info(
            "Model compiled: %d bodies, %d joints, %d actuators, %d sensors",
            model.nbody, model.njnt, model.nu, model.nsensor,
        )
        return model
