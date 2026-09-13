#!/usr/bin/env python3
"""Show GCR16-2000 v1 in RViz. Prefer mode:=assembled after you export STLs."""

import os
import shutil
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _v1_dir():
    """Return the v1 package root (parent of launch/)."""
    return Path(__file__).resolve().parent.parent


def _register_ament_prefix(v1):
    """Build a real ament prefix so package://gcr16_2000_v1 resolves."""
    prefix = v1 / ".ament_prefix"
    share_pkg = prefix / "share" / "gcr16_2000_v1"
    share_pkg.mkdir(parents=True, exist_ok=True)
    meshes_link = share_pkg / "meshes"
    if meshes_link.is_symlink():
        meshes_link.unlink()
    elif meshes_link.is_dir():
        shutil.rmtree(meshes_link)
    elif meshes_link.exists():
        meshes_link.unlink()
    meshes_link.symlink_to((v1 / "meshes").resolve(), target_is_directory=True)
    index_dir = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index_dir.mkdir(parents=True, exist_ok=True)
    (index_dir / "gcr16_2000_v1").write_text("")
    return prefix


def _parse_cli_launch_args(argv):
    """Parse ros2-style name:=value tokens from argv."""
    parsed = []
    for argument in argv:
        if ":=" not in argument or argument.startswith(":="):
            continue
        name, value = argument.split(":=", maxsplit=1)
        parsed.append((name, value))
    return parsed


def _cli_launch_arg(name, default):
    """Read name:=value from process argv (python3 launch or ros2 launch)."""
    prefix = f"{name}:="
    for arg in sys.argv:
        if arg.startswith(prefix):
            return arg.split(":=", 1)[1]
    return default


def _fix_stls(mesh_dir):
    """Rewrite STLs as binary if the helper script is present."""
    fixer = _v1_dir() / "scripts" / "fix_stls.py"
    if not fixer.is_file():
        return
    import runpy

    saved_argv = sys.argv
    sys.argv = [str(fixer), str(mesh_dir)]
    try:
        runpy.run_path(str(fixer), run_name="__main__")
    except SystemExit as exc:
        if exc.code not in (0, None):
            print("[gcr16 v1] fix_stls exit", exc.code)
    finally:
        sys.argv = saved_argv


def _launch_setup(context, *args, **kwargs):
    """Build robot_description and start display nodes."""
    v1 = _v1_dir()
    prefix = _register_ament_prefix(v1)
    ament = str(prefix) + os.pathsep + os.environ.get("AMENT_PREFIX_PATH", "")
    os.environ["AMENT_PREFIX_PATH"] = ament

    # python3 launch/display.launch.py mode:=move does not go through ros2 launch,
    # so DeclareLaunchArgument stays at default unless we also read sys.argv.
    mode = _cli_launch_arg("mode", LaunchConfiguration("mode").perform(context))
    use_cad = _cli_launch_arg(
        "use_cad_meshes", LaunchConfiguration("use_cad_meshes").perform(context)
    )
    mesh_dir = _cli_launch_arg(
        "mesh_dir", LaunchConfiguration("mesh_dir").perform(context)
    )
    if not mesh_dir:
        mesh_dir = str(v1 / "meshes")
    print(f"[gcr16 v1] mode={mode} slider={'yes' if mode == 'move' else 'no'}")

    if use_cad.lower() in ("true", "1"):
        _fix_stls(mesh_dir)

    stl0 = Path(mesh_dir) / "GCR16-J0.stl"
    resolved = prefix / "share" / "gcr16_2000_v1" / "meshes" / "GCR16-J0.stl"
    print("[gcr16 v1] AMENT prefix", prefix)
    print("[gcr16 v1] resolved mesh", resolved, "exists", resolved.is_file())
    if stl0.is_file():
        print(f"[gcr16 v1] {stl0} size={stl0.stat().st_size} bytes")
    if mode == "move":
        for index in range(7):
            stl = Path(mesh_dir) / f"GCR16-J{index}.stl"
            if not stl.is_file():
                print(f"[gcr16 v1] WARNING missing {stl.name} — run scripts/export_meshes.sh")
                continue
            if stl.stat().st_size < 400000 and index in (1, 2):
                print(
                    f"[gcr16 v1] WARNING {stl.name} is tiny ({stl.stat().st_size} bytes). "
                    "Re-export from isaac_sim_test/3d/GCR16-Jn.step via scripts/export_meshes.sh"
                )
    if use_cad.lower() in ("true", "1") and (not stl0.is_file() or stl0.stat().st_size < 80):
        print(
            f"[gcr16 v1] Missing or empty {stl0} — using dummy boxes. "
            "Run ./scripts/export_meshes.sh then python3 scripts/fix_stls.py"
        )
        use_cad = "false"

    if mode == "move":
        xacro_path = v1 / "urdf" / "gcr16_2000_move.urdf.xacro"
    else:
        xacro_path = v1 / "urdf" / "gcr16_2000_assembled.urdf.xacro"

    import xacro

    use_combined = "true"
    all_stl = Path(mesh_dir) / "GCR16-all.stl"
    if mode == "assembled" and use_cad.lower() in ("true", "1"):
        if all_stl.is_file() and all_stl.stat().st_size > 80:
            print(f"[gcr16 v1] using combined mesh {all_stl} size={all_stl.stat().st_size}")
        else:
            use_combined = "false"
            print("[gcr16 v1] no GCR16-all.stl — per-link meshes")

    mappings = {"use_cad_meshes": use_cad, "mesh_dir": mesh_dir}
    if mode == "assembled":
        mappings["use_combined_mesh"] = use_combined

    robot_desc = xacro.process_file(
        str(xacro_path),
        mappings=mappings,
    ).toxml()

    rviz_cfg = v1 / "rviz" / "display.rviz"
    js_remap = [("joint_states", "/gcr16/joint_states")]
    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_desc, "use_sim_time": False}],
            remappings=js_remap + [("robot_description", "/gcr16/robot_description")],
            additional_env={"AMENT_PREFIX_PATH": ament},
        ),
    ]
    # Assembled URDF is all fixed joints. jsp_gui then emits empty/mismatched
    # JointState, and leftover Panda /joint_states also looks "invalid".
    if mode == "move":
        nodes.append(
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[{"robot_description": robot_desc}],
                remappings=js_remap
                + [("robot_description", "/gcr16/robot_description")],
            )
        )
        print("[gcr16 v1] starting joint_state_publisher_gui on /gcr16/joint_states")
    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_cfg)] if rviz_cfg.exists() else [],
            additional_env={"AMENT_PREFIX_PATH": ament},
        )
    )
    return nodes


def generate_launch_description():
    """RViz display: assembled CAD pose (default) or placeholder moving joints."""
    v1 = _v1_dir()
    prefix = _register_ament_prefix(v1)
    ament = str(prefix) + os.pathsep + os.environ.get("AMENT_PREFIX_PATH", "")
    os.environ["AMENT_PREFIX_PATH"] = ament
    return LaunchDescription(
        [
            SetEnvironmentVariable("AMENT_PREFIX_PATH", ament),
            DeclareLaunchArgument("mode", default_value="assembled"),
            DeclareLaunchArgument("use_cad_meshes", default_value="true"),
            DeclareLaunchArgument("mesh_dir", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )


def main(argv=None):
    """Allow `python3 launch/display.launch.py mode:=move` without colcon."""
    from launch import LaunchService

    if argv is None:
        argv = sys.argv[1:]
    extra = [
        SetLaunchConfiguration(name, value)
        for name, value in _parse_cli_launch_args(argv)
    ]
    ls = LaunchService(argv=argv)
    ld = generate_launch_description()
    ls.include_launch_description(LaunchDescription(extra + list(ld.entities)))
    return ls.run()


if __name__ == "__main__":
    sys.exit(main())
