#!/usr/bin/env python3
"""Dump a RoboCasa / RoboSuite scene to pinned MJCF for g1-simulacrum.

Authoring only — not imported at runtime. Needs a Python that can
``import robocasa`` and ``import robosuite`` (the ws_robocasa venv).
This package's Docker image does not install those.

Any registered env works. Scene knobs (``--layout``, ``--style``, and
``--set``) are passed only if that env class accepts them. Kitchen tasks
do; a plain RoboSuite ``Lift`` does not.

    # list env / layout / style names
    /path/to/.venv_robocasa/bin/python scripts/export_robocasa_scene.py --list

    # fixtures-only kitchen (layout 0, Scandinavian)
    /path/to/.venv_robocasa/bin/python scripts/export_robocasa_scene.py \
        --env Kitchen --layout one_wall_small --style scandanavian --seed 0

    # a task env (same kitchen knobs, plus sampled objects)
    /path/to/.venv_robocasa/bin/python scripts/export_robocasa_scene.py \
        --env PnPCounterToCab --layout 0 --style 1 --seed 0

Runtime (this image)::

    ./run.sh python examples/01_empty_arena.py \\
        --scene g1_simulacrum/model/mjcf/robocasa_<slug>.xml \\
        --config configs/robocasa_<slug>.yaml
"""

from __future__ import annotations

import argparse
import hashlib
import inspect
import os
import shutil
import sys
from datetime import date
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MJCF = ROOT / "g1_simulacrum" / "model" / "mjcf"
CONFIGS = ROOT / "configs"
G1_INCLUDE = "g1_robot.xml"
G1_SPAWN_Z = 0.82

# Sections copied from the dump. compiler/size stay under our control
# (meshdir must point at mjcf/assets; timestep is this package's 1 ms).
_KEEP_SECTIONS = (
    "asset",
    "worldbody",
    "actuator",
    "sensor",
    "tendon",
    "equality",
    "contact",
    "visual",
    "statistic",
    "keyframe",
    "custom",
    "size",
)

# Default layout/style when the env accepts those kwargs and the user omitted them.
_DEFAULT_LAYOUT = "0"
_DEFAULT_STYLE = "1"


def _require_robocasa() -> None:
    os.environ.setdefault("MUJOCO_GL", "egl")
    try:
        import robocasa  # noqa: F401
        import robosuite  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            "this script needs robocasa + robosuite (not the g1-simulacrum image).\n"
            "Use ws_robocasa/.venv_robocasa, then:\n"
            "  python robocasa/scripts/download_kitchen_assets.py\n"
            f"Import error: {exc}"
        ) from exc


def _slugify(text: str) -> str:
    out = []
    prev_dash = False
    for ch in text.lower().replace(" ", "_"):
        if ch.isalnum() or ch in "._":
            out.append(ch)
            prev_dash = False
        elif not prev_dash:
            out.append("-")
            prev_dash = True
    return "".join(out).strip("-._") or "scene"


def _env_cls(env_name: str):
    from robosuite.environments.base import REGISTERED_ENVS

    if env_name not in REGISTERED_ENVS:
        known = ", ".join(sorted(REGISTERED_ENVS))
        raise SystemExit(f"unknown env {env_name!r}. Registered: {known}")
    return REGISTERED_ENVS[env_name]


def _init_params(cls) -> dict:
    """Named ``__init__`` params across the MRO (Kitchen tasks wrap ``**kwargs``)."""
    params: dict = {}
    for klass in reversed(cls.__mro__):
        if klass is object:
            continue
        init = getattr(klass, "__init__", None)
        if not callable(init):
            continue
        try:
            sig = inspect.signature(init)
        except (TypeError, ValueError):
            continue
        for name, param in sig.parameters.items():
            if name == "self" or param.kind == inspect.Parameter.VAR_KEYWORD:
                continue
            params[name] = param
    return params


def _has_param(cls, name: str) -> bool:
    return name in _init_params(cls)


def _select_kwargs(cls, kwargs: dict) -> dict:
    allowed = _init_params(cls)
    return {k: v for k, v in kwargs.items() if k in allowed}


def _coerce(value: str):
    lowered = value.lower()
    if lowered in ("true", "false"):
        return lowered == "true"
    if lowered in ("none", "null"):
        return None
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def _parse_set(items: list[str]) -> dict:
    extra: dict = {}
    for item in items:
        if "=" not in item:
            raise SystemExit(f"--set expects KEY=VALUE, got {item!r}")
        key, raw = item.split("=", 1)
        key = key.strip()
        if not key:
            raise SystemExit(f"empty key in --set {item!r}")
        extra[key] = _coerce(raw.strip())
    return extra


def _parse_layout_style(value: str, kind: str) -> int:
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

    enum = LayoutType if kind == "layout" else StyleType
    if value.lstrip("-").isdigit():
        parsed = int(value)
    else:
        key = value.strip().upper().replace("-", "_").replace(" ", "_")
        try:
            parsed = int(enum[key])
        except KeyError as exc:
            names = ", ".join(m.name.lower() for m in enum if int(m) >= 0)
            raise SystemExit(
                f"unknown {kind} {value!r}. Try an int or one of: {names}"
            ) from exc
    if parsed < 0:
        raise SystemExit(
            f"{kind} {value!r} is a group id; dump one concrete {kind}, not ALL"
        )
    return parsed


def _layout_style_names(layout_id: int | None, style_id: int | None) -> tuple[str, str]:
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

    layout = LayoutType(layout_id).name.lower() if layout_id is not None else "na"
    style = StyleType(style_id).name.lower() if style_id is not None else "na"
    return layout, style


def _robocasa_env_names() -> set[str]:
    import robocasa

    names: set[str] = set()
    for attr in dir(robocasa):
        if attr.startswith("ALL_") and attr.endswith("_ENVIRONMENTS") and attr != "ALL_ENVIRONMENTS":
            names.update(getattr(robocasa, attr))
    return names


def _list_and_exit() -> None:
    _require_robocasa()
    import robosuite
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType
    from robosuite.environments.base import REGISTERED_ENVS

    robocasa_names = _robocasa_env_names()
    print("RoboCasa envs ([layout/style] if that env accepts those kwargs):")
    for name in sorted(robocasa_names):
        cls = REGISTERED_ENVS.get(name)
        mark = "  [layout/style]" if cls is not None and _has_param(cls, "layout_ids") else ""
        print(f"  {name}{mark}")
    other = sorted(set(robosuite.ALL_ENVIRONMENTS) - robocasa_names)
    print("\nOther RoboSuite envs (valid --env; no kitchen layout/style):")
    for name in other:
        print(f"  {name}")
    print("\nLayouts:")
    for item in LayoutType:
        if int(item) >= 0:
            print(f"  {int(item):2d}  {item.name.lower()}")
    print("\nStyles:")
    for item in StyleType:
        if int(item) >= 0:
            print(f"  {int(item):2d}  {item.name.lower()}")


def _make_env(
    *,
    env_name: str,
    robot: str,
    layout_id: int | None,
    style_id: int | None,
    seed: int,
    extra: dict,
):
    import robosuite
    from robosuite.controllers import load_composite_controller_config

    cls = _env_cls(env_name)
    os.environ.setdefault("MUJOCO_GL", "egl")
    kwargs: dict = {
        "env_name": env_name,
        "robots": robot,
        "controller_configs": load_composite_controller_config(robot=robot),
        "has_renderer": False,
        "has_offscreen_renderer": False,
        "use_camera_obs": False,
        "ignore_done": True,
        "seed": seed,
        "control_freq": 20,
    }
    if _has_param(cls, "renderer"):
        kwargs["renderer"] = "mujoco"
    if _has_param(cls, "layout_ids"):
        if layout_id is None or style_id is None:
            raise SystemExit(f"{env_name} needs --layout and --style")
        kwargs["layout_ids"] = [layout_id]
        kwargs["style_ids"] = [style_id]
    elif layout_id is not None or style_id is not None:
        raise SystemExit(f"{env_name} does not take --layout / --style")
    if _has_param(cls, "translucent_robot"):
        kwargs["translucent_robot"] = False
    for key, value in extra.items():
        if not _has_param(cls, key):
            raise SystemExit(f"{env_name} does not accept --set {key}=")
        kwargs[key] = value
    kwargs = _select_kwargs(cls, {k: v for k, v in kwargs.items() if k != "env_name"})
    env = robosuite.make(env_name, **kwargs)
    env.reset()
    return env


def _dumped_xml(env) -> str:
    sim = getattr(env, "sim", None)
    model = getattr(sim, "model", None) if sim is not None else None
    if model is not None and hasattr(model, "get_xml"):
        return model.get_xml()
    return env.model.get_xml()


def _add_prefix(prefixes: list[str], value) -> None:
    if not value:
        return
    text = str(value)
    if text not in prefixes:
        prefixes.append(text)


def _robot_prefixes(env) -> tuple[str, ...]:
    prefixes: list[str] = []
    for robot in getattr(env, "robots", []) or []:
        model = getattr(robot, "robot_model", None)
        if model is not None:
            _add_prefix(prefixes, getattr(model, "naming_prefix", None))
            _add_prefix(prefixes, getattr(getattr(model, "base", None), "naming_prefix", None))
        grippers = getattr(robot, "gripper", None)
        if isinstance(grippers, dict):
            for gripper in grippers.values():
                _add_prefix(prefixes, getattr(gripper, "naming_prefix", None))
        elif grippers is not None:
            _add_prefix(prefixes, getattr(grippers, "naming_prefix", None))
    if not prefixes:
        prefixes.append("robot0_")
    return tuple(prefixes)


def _is_robot_name(name: str | None, prefixes: tuple[str, ...]) -> bool:
    if not name:
        return False
    return any(name.startswith(p) for p in prefixes)


def _elem_is_robot(elem: ET.Element, prefixes: tuple[str, ...]) -> bool:
    for key in (
        "name",
        "joint",
        "site",
        "body",
        "mesh",
        "class",
        "childclass",
        "target",
        "geom1",
        "geom2",
        "material",
        "texture",
    ):
        if _is_robot_name(elem.get(key), prefixes):
            return True
    return False


def _strip_robot(root: ET.Element, prefixes: tuple[str, ...]) -> None:
    for parent in list(root.iter()):
        for child in list(parent):
            if _elem_is_robot(child, prefixes):
                parent.remove(child)


def _spawn_xy(root: ET.Element, prefixes: tuple[str, ...], env) -> tuple[float, float]:
    try:
        model = env.robots[0].robot_model
        body = getattr(model, "root_body", None) or f"{model.naming_prefix}base"
        bid = env.sim.model.body_name2id(body)
        xpos = env.sim.data.body_xpos[bid]
        return float(xpos[0]), float(xpos[1])
    except Exception:
        pass
    for suffix in ("base", "base_link", "root", "pelvis"):
        for body in root.iter("body"):
            name = body.get("name") or ""
            if any(name == f"{p}{suffix}" for p in prefixes):
                pos = tuple(float(x) for x in (body.get("pos") or "0 0 0").split())
                return pos[0], pos[1]
    return 0.0, 0.0


def _asset_search_roots() -> list[Path]:
    roots: list[Path] = []
    try:
        import robocasa.models as rc_models

        roots.append(Path(rc_models.assets_root))
    except Exception:
        pass
    try:
        import robosuite.models as rs_models

        roots.append(Path(rs_models.assets_root))
    except Exception:
        pass
    return roots


def _resolve_asset(raw: str, roots: list[Path]) -> Path | None:
    src = Path(raw)
    if src.is_file():
        return src
    for root in roots:
        candidate = root / raw
        if candidate.is_file():
            return candidate
        candidate = root / src.name
        if candidate.is_file():
            return candidate
    return None


def _asset_relkey(src: Path) -> str:
    parts = src.resolve().parts
    for i, part in enumerate(parts):
        if part == "assets" and i + 1 < len(parts):
            return "/".join(parts[i + 1 :])
    return src.name


def _shell_dumped_meshes(root: ET.Element) -> None:
    # RoboCasa visual OBJs are often open surfaces; MuJoCo volume inertia fails.
    for mesh in root.iter("mesh"):
        if mesh.get("inertia") is None:
            mesh.set("inertia", "shell")


def _copy_assets(root: ET.Element, *, slug: str, dest_root: Path) -> int:
    roots = _asset_search_roots()
    copied: set[Path] = set()
    missing: list[str] = []
    for elem in root.iter():
        raw = elem.get("file")
        if not raw:
            continue
        src = _resolve_asset(raw, roots)
        if src is None:
            missing.append(raw)
            continue
        rel = _asset_relkey(src)
        dest = dest_root / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        if dest not in copied:
            shutil.copy2(src, dest)
            copied.add(dest)
        elem.set("file", f"robocasa/{slug}/{rel}")
    if missing:
        preview = "\n  ".join(missing[:12])
        more = f"\n  … {len(missing) - 12} more" if len(missing) > 12 else ""
        raise SystemExit(f"dumped MJCF references missing files:\n  {preview}{more}")
    return len(copied)


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    h.update(path.read_bytes())
    return h.hexdigest()


def _write_xml(path: Path, root: ET.Element) -> None:
    ET.indent(root, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def _compiler_attrib(dumped: ET.Element) -> dict[str, str]:
    attrib = {"angle": "radian", "meshdir": "assets", "texturedir": "assets", "eulerseq": "xyz"}
    src = dumped.find("compiler")
    if src is not None:
        for key, value in src.attrib.items():
            if key in ("meshdir", "texturedir"):
                continue
            attrib[key] = value
    attrib.setdefault("inertiagrouprange", "0 0")
    attrib.setdefault("autolimits", "true")
    return attrib


def _build_scene(dumped: ET.Element, *, slug: str) -> ET.Element:
    scene = ET.Element("mujoco", {"model": f"g1_{slug}"})
    scene.append(
        ET.Comment(
            " Cached RoboCasa/RoboSuite dump. Authoring: scripts/export_robocasa_scene.py. "
            "Sits next to g1_robot.xml so compiler meshdir=assets still resolves. "
        )
    )
    ET.SubElement(scene, "compiler", _compiler_attrib(dumped))
    ET.SubElement(scene, "include", {"file": G1_INCLUDE})
    option = ET.SubElement(scene, "option", {"timestep": "0.001", "gravity": "0 0 -9.81"})
    src_opt = dumped.find("option")
    if src_opt is not None:
        for key, value in src_opt.attrib.items():
            if key != "timestep":
                option.set(key, value)
    for tag in _KEEP_SECTIONS:
        src = dumped.find(tag)
        if src is not None and (len(src) > 0 or (src.text and src.text.strip())):
            scene.append(src)
    return scene


def _write_pin(
    path: Path,
    *,
    env_name: str,
    layout_id: int | None,
    style_id: int | None,
    seed: int,
    robot: str,
    slug: str,
    extra: dict,
    n_assets: int,
    spawn: tuple[float, float, float],
    scene_path: Path,
) -> None:
    layout, style = _layout_style_names(layout_id, style_id)
    import robocasa

    extra_txt = ", ".join(f"{k}={v!r}" for k, v in extra.items()) or "(none)"
    lines = [
        f"# RoboCasa scene dump `{slug}`",
        "",
        f"Generated {date.today().isoformat()} by `scripts/export_robocasa_scene.py`.",
        "Runtime does not import robocasa. Load `robocasa_<slug>.xml` via `G1Simulacrum.build(scene_xml=...)`.",
        "",
        "| Key | Value |",
        "|-----|-------|",
        f"| env | `{env_name}` |",
        f"| layout | `{layout_id}` `{layout}` |",
        f"| style | `{style_id}` `{style}` |",
        f"| seed | `{seed}` |",
        f"| dummy robot (stripped) | `{robot}` |",
        f"| extra kwargs | `{extra_txt}` |",
        f"| robocasa | `{Path(robocasa.__file__).resolve()}` |",
        f"| assets copied | `{n_assets}` |",
        f"| G1 spawn_pos | `{spawn[0]:.5f} {spawn[1]:.5f} {spawn[2]:.5f}` |",
        "",
        f"Scene sha256 `{_sha256(scene_path)}`.",
        "",
    ]
    path.write_text("\n".join(lines))


def _write_runtime_config(path: Path, spawn: tuple[float, float, float]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "# Spawn for a cached RoboCasa dump. Other keys use G1SimulacrumConfig defaults.\n"
        "robot:\n"
        f"  spawn_pos: [{spawn[0]:.6f}, {spawn[1]:.6f}, {spawn[2]:.6f}]\n"
        "  spawn_quat: [1.0, 0.0, 0.0, 0.0]\n"
    )


def _resolve_scene_knobs(env_name: str, args) -> tuple[int | None, int | None, dict]:
    cls = _env_cls(env_name)
    extra = _parse_set(args.set)
    layout_id = None
    style_id = None
    if _has_param(cls, "layout_ids"):
        layout_id = _parse_layout_style(args.layout or _DEFAULT_LAYOUT, "layout")
        style_id = _parse_layout_style(args.style or _DEFAULT_STYLE, "style")
    elif args.layout is not None or args.style is not None:
        raise SystemExit(f"{env_name} does not take --layout / --style")
    return layout_id, style_id, extra


def _default_slug(
    env_name: str,
    layout_id: int | None,
    style_id: int | None,
    seed: int,
    extra: dict,
) -> str:
    parts = [env_name]
    if layout_id is not None:
        layout, style = _layout_style_names(layout_id, style_id)
        parts.append(layout)
        parts.append(style)
    parts.append(f"seed{seed}")
    for key in sorted(extra):
        parts.append(f"{key}-{extra[key]}")
    return _slugify("_".join(str(p) for p in parts))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="print envs, layouts, styles")
    parser.add_argument("--env", default="Kitchen", help="registered robocasa/robosuite env")
    parser.add_argument("--layout", default=None, help="layout int or name (if the env accepts it)")
    parser.add_argument("--style", default=None, help="style int or name (if the env accepts it)")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--robot", default="PandaOmron", help="dummy robot, stripped from dump")
    parser.add_argument("--slug", default=None, help="cache folder name")
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="extra env __init__ kwargs (obj_instance_split=A, init_robot_base_pos=counter, …)",
    )
    args = parser.parse_args()
    if args.list:
        _list_and_exit()
        return

    _require_robocasa()
    layout_id, style_id, extra = _resolve_scene_knobs(args.env, args)
    slug = args.slug or _default_slug(args.env, layout_id, style_id, args.seed, extra)

    extra_txt = f" extra={extra}" if extra else ""
    print(
        f"building {args.env} (robot={args.robot}, seed={args.seed}, "
        f"layout={layout_id}, style={style_id}{extra_txt}) …",
        flush=True,
    )
    env = _make_env(
        env_name=args.env,
        robot=args.robot,
        layout_id=layout_id,
        style_id=style_id,
        seed=args.seed,
        extra=extra,
    )
    prefixes = _robot_prefixes(env)
    dumped = ET.fromstring(_dumped_xml(env))
    spawn_x, spawn_y = _spawn_xy(dumped, prefixes, env)
    spawn = (spawn_x, spawn_y, G1_SPAWN_Z)
    _strip_robot(dumped, prefixes)
    _shell_dumped_meshes(dumped)

    asset_dest = MJCF / "assets" / "robocasa" / slug
    if asset_dest.exists():
        shutil.rmtree(asset_dest)
    n_assets = _copy_assets(dumped, slug=slug, dest_root=asset_dest)

    scene = _build_scene(dumped, slug=slug)
    dump_dir = MJCF / "robocasa" / slug
    dump_dir.mkdir(parents=True, exist_ok=True)
    stale = dump_dir / "scene.xml"
    if stale.exists():
        stale.unlink()
    scene_path = MJCF / f"robocasa_{slug}.xml"
    _write_xml(scene_path, scene)
    _write_pin(
        dump_dir / "PIN.md",
        env_name=args.env,
        layout_id=layout_id,
        style_id=style_id,
        seed=args.seed,
        robot=args.robot,
        slug=slug,
        extra=extra,
        n_assets=n_assets,
        spawn=spawn,
        scene_path=scene_path,
    )
    cfg_path = CONFIGS / f"robocasa_{slug}.yaml"
    _write_runtime_config(cfg_path, spawn)

    env.close()
    print(f"cached {scene_path.relative_to(ROOT)}")
    print(f"  assets {n_assets} → {asset_dest.relative_to(ROOT)}")
    print(f"  spawn {spawn[0]:.4f} {spawn[1]:.4f} {spawn[2]:.4f}")
    print(f"  config {cfg_path.relative_to(ROOT)}")
    print(
        "runtime: docker/run.sh python examples/01_empty_arena.py "
        f"--scene {scene_path.relative_to(ROOT)} --config {cfg_path.relative_to(ROOT)}"
    )


if __name__ == "__main__":
    sys.stdout.reconfigure(line_buffering=True)
    main()
