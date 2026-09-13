#!/usr/bin/env python3
"""Show GCR16-2000 v1 in RViz. Prefer mode:=assembled after you export STLs."""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _v1_dir():
    """Return the v1 package root (parent of launch/)."""
    return Path(__file__).resolve().parent.parent


def _launch_setup(context, *args, **kwargs):
    """Build robot_description and start display nodes."""
    v1 = _v1_dir()
    mode = LaunchConfiguration("mode").perform(context)
    use_cad = LaunchConfiguration("use_cad_meshes").perform(context)
    mesh_dir = LaunchConfiguration("mesh_dir").perform(context)
    if not mesh_dir:
        mesh_dir = str(v1 / "meshes")

    stl0 = Path(mesh_dir) / "GCR16-J0.stl"
    if use_cad.lower() in ("true", "1") and not stl0.is_file():
        print(
            f"[gcr16 v1] No {stl0} — using dummy boxes. "
            "Run ./scripts/export_meshes.sh on the sim box."
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
    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_desc, "use_sim_time": False}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_cfg)] if rviz_cfg.exists() else [],
        ),
    ]
    return nodes


def generate_launch_description():
    """RViz display: assembled CAD pose (default) or placeholder moving joints."""
    return LaunchDescription(
        [
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
