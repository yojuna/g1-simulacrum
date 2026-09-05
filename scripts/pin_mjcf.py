#!/usr/bin/env python3
"""Pin Unitree G1 MJCF into this package. Authoring only — not used at runtime.

Pristine Unitree files live in-tree:

    g1_simulacrum/model/mjcf/upstream/   body MJCF, with-hand MJCF, URDF
    g1_simulacrum/model/mjcf/assets/     STLs named by those MJCFs

This script applies *named* ElementTree edits to produce the owned robot XML.
It does not clone unitree_ros or download a tarball.

    cd docker && ./run.sh python scripts/pin_mjcf.py           # offline
    cd docker && ./run.sh python scripts/pin_mjcf.py --fetch   # bump pin

``--fetch`` GETs only those named files from GitHub raw at PIN_SHA.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import shutil
import urllib.request
from datetime import date
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MJCF = ROOT / "g1_simulacrum" / "model" / "mjcf"
ASSETS = MJCF / "assets"
UPSTREAM = MJCF / "upstream"
WIKI_SOURCES = ROOT / "wiki" / "sources.md"

# Architecture / wiki pin (2026-06-16, "fix g1 mid360_joint transform").
PIN_SHA = "7c40519e02d7dd16c06b25fe3fa3b67fdeb7cd74"
URDF_PIN_SHA = PIN_SHA
UNITREE_RAW = (
    "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
    f"{PIN_SHA}/robots/g1_description"
)
UPSTREAM_NAMES = (
    "g1_29dof_rev_1_0.xml",
    "g1_29dof_with_hand_rev_1_0.xml",
    "g1_29dof_rev_1_0.urdf",
)

# Wiki g1-sensors.md — assert parsed URDF matches these, do not invent.
EXPECTED_URDF = {
    "mid360_joint": {
        "xyz": (0.0002835, 0.00003, 0.428434),
        "rpy": (3.141592653589793, 0.05112069379091391, 0.0),
    },
    "d435_joint": {
        "xyz": (0.0576235, 0.01753, 0.42987),
        "rpy": (0.0, 0.8307767239493009, 0.0),
    },
    "imu_in_torso_joint": {
        "xyz": (-0.03959, -0.00224, 0.14792),
        "rpy": (0.0, 0.0, 0.0),
    },
    "imu_in_pelvis_joint": {
        "xyz": (0.04525, 0.0, -0.08339),
        "rpy": (0.0, 0.0, 0.0),
    },
}

# Livox IMU in the Mid-360 optical frame (wiki/g1-sensors.md), not a URDF joint.
MID360_IMU_POS = "0.011 0.02329 -0.04412"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    h.update(path.read_bytes())
    return h.hexdigest()


def mesh_files_from(root: ET.Element) -> list[str]:
    return [m.get("file") for m in root.iter("mesh") if m.get("file")]


def raw_url(sha: str, rel: str) -> str:
    return (
        "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        f"{sha}/robots/g1_description/{rel}"
    )


def http_get(url: str) -> bytes:
    with urllib.request.urlopen(url) as resp:
        return resp.read()


def fetch_exact_files(sha: str) -> None:
    """Download only the Unitree files we vendor. No repo clone, no tarball."""
    UPSTREAM.mkdir(parents=True, exist_ok=True)
    ASSETS.mkdir(parents=True, exist_ok=True)
    for name in UPSTREAM_NAMES:
        url = raw_url(sha, name)
        (UPSTREAM / name).write_bytes(http_get(url))
        print(f"  fetched {name}")
    mesh_names: set[str] = set()
    for xml_name in UPSTREAM_NAMES:
        if xml_name.endswith(".xml"):
            mesh_names.update(mesh_files_from(ET.parse(UPSTREAM / xml_name).getroot()))
    for name in sorted(mesh_names):
        url = raw_url(sha, f"meshes/{name}")
        (ASSETS / name).write_bytes(http_get(url))
    (UPSTREAM / "PIN_SHA").write_text(sha + "\n")
    print(f"  fetched {len(mesh_names)} meshes")


def fmt_vec(values: tuple[float, ...]) -> str:
    return " ".join(f"{v:.15g}" for v in values)


def parse_vec(text: str) -> tuple[float, ...]:
    return tuple(float(x) for x in text.split())


def vecs_close(a: tuple[float, ...], b: tuple[float, ...], tol: float = 1e-9) -> bool:
    return len(a) == len(b) and all(abs(x - y) <= tol for x, y in zip(a, b))


def find_named(root: ET.Element, tag: str, name: str) -> ET.Element:
    for elem in root.iter(tag):
        if elem.get("name") == name:
            return elem
    raise LookupError(f"<{tag} name='{name}'> not found")


def indent_xml(elem: ET.Element) -> None:
    ET.indent(elem, space="  ")


def write_xml(path: Path, root: ET.Element) -> None:
    indent_xml(root)
    tree = ET.ElementTree(root)
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding="utf-8", xml_declaration=True)


def wrap_include(children: list[ET.Element], dest: Path, comment: str) -> None:
    """Write a MuJoCo include fragment. Root must be <mujocoinclude>."""
    root = ET.Element("mujocoinclude")
    root.append(ET.Comment(f" {comment} "))
    for child in children:
        root.append(child)
    write_xml(dest, root)


def strip_unitree_floor_scene(root: ET.Element) -> None:
    """Drop the demo floor/skybox that follows the robot tree in Unitree MJCF."""
    for child in list(root):
        if child.tag in ("statistic", "visual"):
            root.remove(child)
    worldbodies = [c for c in root if c.tag == "worldbody"]
    for extra in worldbodies[1:]:
        root.remove(extra)
    assets = [c for c in root if c.tag == "asset"]
    for extra in assets[1:]:
        root.remove(extra)


def is_hand_child(elem: ET.Element, wrist_mesh: str) -> bool:
    if elem.tag == "body":
        name = elem.get("name") or ""
        return "_hand_" in name
    if elem.tag == "geom":
        mesh = elem.get("mesh") or ""
        return mesh != wrist_mesh and ("hand" in mesh or "rubber" in mesh)
    return False


def take_hand_children(wrist: ET.Element, wrist_mesh: str) -> list[ET.Element]:
    taken: list[ET.Element] = []
    for child in list(wrist):
        if is_hand_child(child, wrist_mesh):
            wrist.remove(child)
            taken.append(child)
    return taken


def copy_inertial(dst_body: ET.Element, src_body: ET.Element) -> None:
    src = src_body.find("inertial")
    dst = dst_body.find("inertial")
    if src is None:
        raise LookupError(f"no inertial on {src_body.get('name')}")
    if dst is None:
        dst_body.insert(0, copy.deepcopy(src))
        return
    dst.attrib.clear()
    dst.attrib.update(src.attrib)


def parse_urdf_origins(urdf_path: Path) -> dict[str, dict[str, tuple[float, ...]]]:
    root = ET.parse(urdf_path).getroot()
    found: dict[str, dict[str, tuple[float, ...]]] = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        if name not in EXPECTED_URDF:
            continue
        origin = joint.find("origin")
        if origin is None:
            raise LookupError(f"URDF joint {name} has no <origin>")
        found[name] = {
            "xyz": parse_vec(origin.get("xyz", "0 0 0")),
            "rpy": parse_vec(origin.get("rpy", "0 0 0")),
        }
    missing = set(EXPECTED_URDF) - set(found)
    if missing:
        raise LookupError(f"URDF missing joints: {sorted(missing)}")
    return found


def assert_urdf_matches_wiki(origins: dict[str, dict[str, tuple[float, ...]]]) -> None:
    mismatches: list[str] = []
    for name, expected in EXPECTED_URDF.items():
        got = origins[name]
        if not vecs_close(got["xyz"], expected["xyz"]) or not vecs_close(
            got["rpy"], expected["rpy"]
        ):
            mismatches.append(
                f"{name}: got xyz={got['xyz']} rpy={got['rpy']}; "
                f"wiki expects xyz={expected['xyz']} rpy={expected['rpy']}"
            )
    if mismatches:
        raise SystemExit(
            "URDF mount poses do not match wiki/g1-sensors.md "
            f"(pin {URDF_PIN_SHA[:12]}). Local clones often still have the "
            "pre-fix Mid-360 (z=0.41618, no roll π).\n  "
            + "\n  ".join(mismatches)
        )


def add_missing_meshes(dst_asset: ET.Element, src_asset: ET.Element) -> None:
    have = {m.get("name") for m in dst_asset.findall("mesh")}
    for mesh in src_asset.findall("mesh"):
        if mesh.get("name") not in have:
            dst_asset.append(copy.deepcopy(mesh))


def require_assets(filenames: list[str]) -> list[tuple[str, str, str]]:
    missing = [n for n in filenames if not (ASSETS / n).exists()]
    if missing:
        raise SystemExit(
            "missing STLs in g1_simulacrum/model/mjcf/assets/:\n  "
            + "\n  ".join(missing)
            + "\nRun: docker/run.sh python scripts/pin_mjcf.py --fetch"
        )
    records: list[tuple[str, str, str]] = []
    for name in filenames:
        dest = ASSETS / name
        records.append(
            (str(dest.relative_to(ROOT)), f"assets/{name}", sha256_file(dest))
        )
    return records


def write_mounts(origins: dict[str, dict[str, tuple[float, ...]]]) -> None:
    mid = origins["mid360_joint"]
    cam = origins["d435_joint"]
    mounts = MJCF / "mounts"
    mounts.mkdir(parents=True, exist_ok=True)

    (mounts / "mid360.xml").write_text(
        f"""\
<mujocoinclude>
  <!-- Mid-360 on torso_link.
       URDF mid360_joint unitree_ros@{URDF_PIN_SHA[:12]}
       xyz={fmt_vec(mid["xyz"])} rpy={fmt_vec(mid["rpy"])}
       (roll π, pitch ~2.93°). Do not use discoverse quat="0 0 1 0". -->
  <body name="mid360_link" pos="{fmt_vec(mid["xyz"])}" euler="{fmt_vec(mid["rpy"])}">
    <inertial pos="0 0 0" mass="0.265" diaginertia="0.0002 0.0002 0.0002"/>
    <geom name="mid360_visual" type="cylinder" size="0.0325 0.03"
          rgba="0.15 0.15 0.15 1" contype="0" conaffinity="0" group="4"/>
    <site name="mid360" pos="0 0 0" size="0.005" rgba="0 1 0 1" group="5"/>
    <site name="mid360_imu_site" pos="{MID360_IMU_POS}" size="0.004"/>
  </body>
</mujocoinclude>
"""
    )
    (mounts / "d435i.xml").write_text(
        f"""\
<mujocoinclude>
  <!-- D435i on torso_link.
       URDF d435_joint unitree_ros@{URDF_PIN_SHA[:12]} (URDF link d435_link).
       xyz={fmt_vec(cam["xyz"])} rpy={fmt_vec(cam["rpy"])} (~47.60° pitch).
       Depth fovy 58° (wiki, 87° H at 4:3); RGB fovy 42° (wiki, 69° H). -->
  <body name="d435i_link" pos="{fmt_vec(cam["xyz"])}" euler="{fmt_vec(cam["rpy"])}">
    <inertial pos="0 0 0" mass="0.072" diaginertia="0.00005 0.00005 0.00002"/>
    <geom type="box" size="0.045 0.013 0.013"
          rgba="0.3 0.3 0.3 1" contype="0" conaffinity="0" group="4"/>
    <!-- URDF d435_link is camera_link (X forward, Z up). Look along +X. -->
    <camera name="d435i_depth" pos="0 0 0" fovy="58" resolution="640 480"
            xyaxes="0 -1 0 0 0 1"/>
    <camera name="d435i_rgb" pos="0 0 0" fovy="42" resolution="640 480"
            xyaxes="0 -1 0 0 0 1"/>
    <site name="d435i_imu_site" pos="0 0 0" size="0.004"/>
  </body>
</mujocoinclude>
"""
    )
    (mounts / "imus.xml").write_text(
        """\
<!-- Device IMUs only. Pelvis/torso gyro+accel stay in the body snapshot. -->
<mujocoinclude>
  <accelerometer name="mid360_accel" site="mid360_imu_site"/>
  <gyro name="mid360_gyro" site="mid360_imu_site"/>
  <accelerometer name="d435i_accel" site="d435i_imu_site"/>
  <gyro name="d435i_gyro" site="d435i_imu_site"/>
</mujocoinclude>
"""
    )


def _set_model_name(path: Path, name: str) -> None:
    tree = ET.parse(path)
    tree.getroot().set("model", name)
    write_xml(path, tree.getroot())


def write_wrappers() -> None:
    """One include hop only. Nested full-document includes merge the body twice."""
    shutil.copyfile(MJCF / "g1_29dof.xml", MJCF / "g1_robot.xml")
    shutil.copyfile(MJCF / "g1_29dof_none.xml", MJCF / "g1_robot_none.xml")
    _set_model_name(MJCF / "g1_robot.xml", "g1_robot")
    _set_model_name(MJCF / "g1_robot_none.xml", "g1_robot_none")
    scene = """\
<mujoco model="{model}">
  <include file="{robot}"/>
  <option timestep="0.001" gravity="0 0 -9.81"/>
  <statistic center="0 0 0.8" extent="1.2"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3"/>
  </visual>
  <worldbody>
    <light name="top_light" pos="0 0 3.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="10 10 0.1" rgba="0.8 0.8 0.8 1"/>
  </worldbody>
</mujoco>
"""
    (MJCF / "g1_sensorized.xml").write_text(
        scene.format(model="g1_sensorized", robot="g1_robot.xml")
    )
    (MJCF / "g1_sensorized_none.xml").write_text(
        scene.format(model="g1_sensorized_none", robot="g1_robot_none.xml")
    )


def extract_hand_motors(hands_root: ET.Element, dest: Path) -> None:
    actuator = hands_root.find("actuator")
    if actuator is None:
        raise LookupError("with-hand MJCF has no <actuator>")
    motors = [
        copy.deepcopy(m)
        for m in actuator.findall("motor")
        if "hand" in (m.get("joint") or "")
    ]
    if len(motors) != 14:
        names = [m.get("name") for m in motors]
        raise LookupError(f"expected 14 Dex3 motors, got {len(motors)}: {names}")
    wrap_include(motors, dest, "Dex3 actuators from g1_29dof_with_hand_rev_1_0")


def compose_body(
    body_src: Path,
    hands_src: Path,
    *,
    kit: str,
    dest: Path,
    model_name: str,
) -> None:
    tree = ET.parse(body_src)
    root = tree.getroot()
    root.set("model", model_name)

    compiler = root.find("compiler")
    if compiler is None:
        raise LookupError("no <compiler>")
    compiler.set("meshdir", "assets")
    compiler.set("eulerseq", "xyz")

    strip_unitree_floor_scene(root)
    asset = root.find("asset")
    if asset is None:
        raise LookupError("no <asset>")

    hands_root = ET.parse(hands_src).getroot()
    hands_asset = hands_root.find("asset")
    if hands_asset is None:
        raise LookupError("with-hand MJCF has no <asset>")
    add_missing_meshes(asset, hands_asset)

    torso = find_named(root, "body", "torso_link")
    insert_at = len(list(torso))
    for i, child in enumerate(list(torso)):
        if child.tag == "site" and child.get("name") == "imu_in_torso":
            insert_at = i + 1
            break
    torso.insert(insert_at, ET.Element("include", {"file": "mounts/mid360.xml"}))
    torso.insert(insert_at + 1, ET.Element("include", {"file": "mounts/d435i.xml"}))

    for side, wrist_mesh in (
        ("left", "left_wrist_yaw_link"),
        ("right", "right_wrist_yaw_link"),
    ):
        wrist = find_named(root, "body", f"{side}_wrist_yaw_link")
        include_rel = f"end_effectors/{kit}/{side}.xml"
        if kit == "dex3":
            src_wrist = find_named(hands_root, "body", f"{side}_wrist_yaw_link")
            copy_inertial(wrist, src_wrist)
            take_hand_children(wrist, wrist_mesh)  # drop rubber
            hand_bits = take_hand_children(copy.deepcopy(src_wrist), wrist_mesh)
            wrap_include(
                hand_bits,
                MJCF / "end_effectors" / "dex3" / f"{side}.xml",
                f"{side} Dex3 from g1_29dof_with_hand_rev_1_0, {side}_wrist_yaw_link children",
            )
        else:
            rubber = take_hand_children(wrist, wrist_mesh)
            wrap_include(
                rubber,
                MJCF / "end_effectors" / "none" / f"{side}.xml",
                f"{side} rubber hand from g1_29dof_rev_1_0",
            )
        ET.SubElement(wrist, "include", {"file": include_rel})

    if kit == "dex3":
        extract_hand_motors(hands_root, MJCF / "end_effectors" / "dex3" / "actuators.xml")
        actuator = root.find("actuator")
        if actuator is None:
            raise LookupError("no <actuator>")
        ET.SubElement(
            actuator, "include", {"file": "end_effectors/dex3/actuators.xml"}
        )

    sensor = root.find("sensor")
    if sensor is None:
        raise LookupError("no <sensor>")
    ET.SubElement(sensor, "include", {"file": "mounts/imus.xml"})

    write_xml(dest, root)


def write_pin_record(*, copies: list[tuple[str, str, str]]) -> None:
    lines = [
        "# MJCF pin",
        "",
        f"Generated {date.today().isoformat()} by `scripts/pin_mjcf.py` "
        f"(via `docker/run.sh python scripts/pin_mjcf.py`).",
        "Runtime does not run this script.",
        "",
        "## Vendored Unitree sources",
        "",
        f"- GitHub pin: `unitree_ros@{PIN_SHA}`",
        f"- Raw base: `{UNITREE_RAW}/`",
        "- In-tree: `g1_simulacrum/model/mjcf/upstream/` (XML + URDF) and `assets/` (STLs)",
        "- Re-fetch named files only: `docker/run.sh python scripts/pin_mjcf.py --fetch`",
        "",
        "## Files (sha256)",
        "",
    ]
    for dest, source, digest in copies:
        lines.append(f"- `{dest}` ← `{source}` `{digest}`")
    lines += [
        "",
        "## Named edits (not string slices)",
        "",
        "- `compiler`: `meshdir=assets`, `eulerseq=xyz`",
        "- Strip Unitree demo floor/skybox (`statistic` / extra `worldbody` / extra `asset`)",
        "- `torso_link`: include `mounts/mid360.xml`, `mounts/d435i.xml`",
        "- `left_wrist_yaw_link` / `right_wrist_yaw_link`: replace hand children with includes",
        "- Dex3: copy wrist `inertial` from with-hand; extract `_hand_` bodies and palm geoms by name",
        "- Dex3 actuators: motors whose `joint` contains `hand` (14)",
        "- `sensor`: include device IMUs only (`mounts/imus.xml`)",
        "- Mount `pos`/`euler` from URDF `<joint><origin>` of `mid360_joint` and `d435_joint`",
        "",
    ]
    (MJCF / "PIN.md").write_text("\n".join(lines))


def update_wiki_sources() -> None:
    marker = "## This package snapshot"
    block = (
        f"{marker}\n\n"
        f"Pinned {date.today().isoformat()} via "
        f"`docker/run.sh python scripts/pin_mjcf.py`. "
        f"Pristine Unitree files are vendored under "
        f"`g1_simulacrum/model/mjcf/upstream/` and `assets/`. "
        f"Bump with `--fetch` (named GitHub raw files only). "
        f"See `g1_simulacrum/model/mjcf/PIN.md`.\n\n"
        f"| What | Pin |\n"
        f"|------|-----|\n"
        f"| Body MJCF, Dex3 MJCF, URDF, STLs "
        f"| `unitree_ros@{PIN_SHA[:12]}` |\n"
        f"| Mid-360 IMU site in lidar frame | wiki Livox offset `{MID360_IMU_POS}` |\n"
    )
    text = WIKI_SOURCES.read_text()
    if marker in text:
        pre, rest = text.split(marker, 1)
        nxt = rest.find("\n## ")
        if nxt == -1:
            text = pre.rstrip() + "\n\n" + block
        else:
            after = rest[nxt:]
            text = pre.rstrip() + "\n\n" + block + after
    else:
        text = text.rstrip() + "\n\n" + block
    WIKI_SOURCES.write_text(text if text.endswith("\n") else text + "\n")


def require_upstream() -> None:
    missing = [n for n in UPSTREAM_NAMES if not (UPSTREAM / n).exists()]
    if missing:
        raise SystemExit(
            "missing vendored Unitree files in "
            "g1_simulacrum/model/mjcf/upstream/:\n  "
            + "\n  ".join(missing)
            + "\nRun: docker/run.sh python scripts/pin_mjcf.py --fetch"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--fetch",
        action="store_true",
        help="download only the named XML/URDF/STLs at PIN_SHA into upstream/ and assets/",
    )
    parser.add_argument(
        "--pin-sha",
        default=PIN_SHA,
        help="unitree_ros git SHA used by --fetch",
    )
    args = parser.parse_args()
    copies: list[tuple[str, str, str]] = []
    if args.fetch:
        print(f"fetching named files from unitree_ros@{args.pin_sha[:12]}")
        fetch_exact_files(args.pin_sha)

    require_upstream()
    for name in UPSTREAM_NAMES:
        path = UPSTREAM / name
        copies.append(
            (str(path.relative_to(ROOT)), f"upstream/{name}", sha256_file(path))
        )

    urdf_path = UPSTREAM / "g1_29dof_rev_1_0.urdf"
    origins = parse_urdf_origins(urdf_path)
    assert_urdf_matches_wiki(origins)

    write_mounts(origins)
    compose_body(
        UPSTREAM / "g1_29dof_rev_1_0.xml",
        UPSTREAM / "g1_29dof_with_hand_rev_1_0.xml",
        kit="dex3",
        dest=MJCF / "g1_29dof.xml",
        model_name="g1_29dof",
    )
    compose_body(
        UPSTREAM / "g1_29dof_rev_1_0.xml",
        UPSTREAM / "g1_29dof_with_hand_rev_1_0.xml",
        kit="none",
        dest=MJCF / "g1_29dof_none.xml",
        model_name="g1_29dof_none",
    )
    write_wrappers()

    mesh_names = sorted(
        set(mesh_files_from(ET.parse(MJCF / "g1_29dof.xml").getroot()))
        | set(mesh_files_from(ET.parse(MJCF / "g1_29dof_none.xml").getroot()))
    )
    copies.extend(require_assets(mesh_names))
    write_pin_record(copies=copies)
    update_wiki_sources()
    print(f"pinned MJCF → {MJCF}")
    print(f"  pin {PIN_SHA[:12]}")
    print(f"  meshes {len(mesh_names)}")


if __name__ == "__main__":
    main()
