#!/usr/bin/env python3
"""Show GCR16-2000 v1 in RViz. Prefer mode:=assembled after you export STLs."""

import os
import shutil
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
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


def _fix_stls(mesh_dir):
    """Rewrite STLs as binary if the helper script is present."""
    fixer = _v1_dir() / "scripts" / "fix_stls.py"
    if not fixer.is_file():
        return
    import runpy

    sys.argv = [str(fixer), str(mesh_dir)]
    try:
        runpy.run_path(str(fixer), run_name="__main__")
    except SystemExit as exc:
        if exc.code not in (0, None):
            print("[gcr16 v1] fix_stls exit", exc.code)


def _launch_setup(context, *args, **kwargs):
    """Build robot_description and start display nodes."""
    v1 = _v1_dir()
    prefix = _register_ament_prefix(v1)
    ament = str(prefix) + os.pathsep + os.environ.get("AMENT_PREFIX_PATH", "")
    os.environ["AMENT_PREFIX_PATH"] = ament

    mode = LaunchConfiguration("mode").perform(context)
    use_cad = LaunchConfiguration("use_cad_meshes").perform(context)
    mesh_dir = LaunchConfiguration("mesh_dir").perform(context)
    if not mesh_dir:
        mesh_dir = str(v1 / "meshes")

    if use_cad.lower() in ("true", "1"):
        _fix_stls(mesh_dir)

    stl0 = Path(mesh_dir) / "GCR16-J0.stl"
    resolved = prefix / "share" / "gcr16_2000_v1" / "meshes" / "GCR16-J0.stl"
    print("[gcr16 v1] AMENT prefix", prefix)
    print("[gcr16 v1] resolved mesh", resolved, "exists", resolved.is_file())
    if stl0.is_file():
        print(f"[gcr16 v1] {stl0} size={stl0.stat().st_size} bytes")
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

    robot_desc = xacro.process_file(
        str(xacro_path),
        mappings={"use_cad_meshes": use_cad, "mesh_dir": mesh_dir},
    ).toxml()

    rviz_cfg = v1 / "rviz" / "display.rviz"
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_desc, "use_sim_time": False}],
            additional_env={"AMENT_PREFIX_PATH": ament},
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_cfg)] if rviz_cfg.exists() else [],
            additional_env={"AMENT_PREFIX_PATH": ament},
        ),
    ]


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
    """Allow `python3 launch/display.launch.py` on the sim box without colcon."""
    from launch import LaunchService

    ls = LaunchService(argv=argv)
    ls.include_launch_description(generate_launch_description())
    return ls.run()


if __name__ == "__main__":
    sys.exit(main())
