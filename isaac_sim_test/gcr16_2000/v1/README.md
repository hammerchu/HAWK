# GCR16-2000 v1

First drop to **see** the SIASUN / DUCO GCR16-2000 in RViz and later swap it for the Panda in the Isaac + MoveIt scene.

STEP is 21 MB; CAD meshes are generated **on the sim box**, not in this repo.

## What is in v1

| File | Role |
|---|---|
| `urdf/gcr16_2000_assembled.urdf.xacro` | J0–J6 meshes (or dummy boxes) all in CAD world pose. **Best first check.** |
| `urdf/gcr16_2000_move.urdf.xacro` | 6 revolute joints with **placeholder** origins. Will look wrong until we tune axes. |
| `launch/display.launch.py` | `robot_state_publisher` + `joint_state_publisher_gui` + RViz |
| `scripts/export_meshes.sh` | FreeCAD export on Ubuntu |

RViz display is the right first tool. Full MoveIt config comes after the arm looks right and joints spin on the real axes.

## On the sim machine

```bash
# 1) FreeCAD (mesh export)
sudo apt update
sudo apt install -y freecad

# 2) ROS 2 display deps (Humble)
sudo apt install -y \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-rviz2

# 3) Export STLs next to this package (needs the STEP under isaac_sim_test/3d/)
cd /path/to/HAWK/isaac_sim_test/gcr16_2000/v1
./scripts/export_meshes.sh

# 4) See the CAD pose (no joint motion)
source /opt/ros/humble/setup.bash
python3 launch/display.launch.py   # also works: ros2 launch if you installed the package

# or:
ros2 launch $(pwd)/launch/display.launch.py use_cad_meshes:=true mode:=assembled
```

Movable (placeholder IK — expect a broken pose until we fix joint xyz):

```bash
ros2 launch $(pwd)/launch/display.launch.py use_cad_meshes:=true mode:=move
```

Drag sliders in `joint_state_publisher_gui`. If the mesh explodes, the joint origins are still guesses.

## After this looks good

1. Tune `gcr16_2000_move.urdf.xacro` joint `xyz` / `rpy` / `axis` against the CAD.
2. Run MoveIt Setup Assistant on that URDF (same flow as `panda_arm_example`).
3. Point `topic_based_ros2_control` + Isaac Articulation at the new joint names (`gcr16_joint1` …).
4. Swap the Panda USD / MoveIt package in the tested scene.

## Joint names (v1)

`gcr16_joint1` … `gcr16_joint6`  
Limits: ±360°. Speeds from the spec (J1 120°/s, J2 180°/s, J3–J6 225°/s).
