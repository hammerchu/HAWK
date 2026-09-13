# How to control the Isaac Sim arm from MoveIt

Follow this on the **sim machine** (Ubuntu 22.04 + ROS 2 Humble + Isaac Sim).  
Goal: Plan/Execute in MoveIt RViz, and have the Franka/Panda in Isaac Sim follow.

This is the path we already walked. ROS 1 (`rospy` / `hawkEnv`) is **not** part of this loop.

---

## What we set up (summary)


| Piece                                       | Role                                                         |
| ------------------------------------------- | ------------------------------------------------------------ |
| ROS 2 Humble                                | Common bus for MoveIt and Isaac                              |
| MoveIt 2 (`panda_arm_example`)              | Plan + Execute                                               |
| `ros2_control` + `topic_based_ros2_control` | Turns MoveIt trajectories into `sensor_msgs/JointState`      |
| Isaac ROS 2 Bridge + Action Graph           | Subscribes to joint **commands**, publishes joint **states** |
| Same `ROS_DOMAIN_ID` + Fast DDS             | So `ros2 topic list` can see Isaac topics                    |


Data flow:

```
MoveIt Execute
  -> panda_arm_controller (FollowJointTrajectory)
  -> ros2_control TopicBasedSystem
  -> /isaac_joint_commands   (JointState commands)
  -> Isaac ROS2 Subscribe Joint State
  -> Articulation Controller
  -> Franka in the scene

Isaac ROS2 Publish Joint State
  -> /isaac_joint_states  (JointState feedback)
  -> TopicBasedSystem
  -> /joint_states
  -> MoveIt current-state monitor
```

If `/controller_manager` is down, Execute fails with “latest received state has time 0.000000”.  
If Isaac is not **Playing**, those Isaac topics do not exist.

---



## 1. One-time: ROS 2 Humble

Use `setup_ros.sh` in this repo, or install desktop Humble yourself.

Then add MoveIt + control (Humble has **no** `ros-humble-mock-components` apt package):

```bash
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-resources-panda-description \
  ros-humble-moveit-resources-panda-moveit-config \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-hardware-interface \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-topic-based-ros2-control
```

Confirm the topic-based plugin class name:

```bash
ros2 pkg prefix topic_based_ros2_control
grep plugin /opt/ros/humble/share/topic_based_ros2_control/*.xml
# must show: topic_based_ros2_control/TopicBasedSystem
```

`ros2 pkg executables controller_manager` should list `ros2_control_node`.

---



## 2. One-time: MoveIt config package

We used Setup Assistant and built:

`~/ws_moveit2/panda_arm_example`

That workspace may have **no** `src/` — edits then live under `install/.../share/panda_arm_example/`. If you later `colcon build` from a real src tree, copy the same edits into source first.

### Hardware plugin (this was the crash)

File:

`install/panda_arm_example/share/panda_arm_example/config/panda.ros2_control.xacro`

Must be `TopicBasedSystem` (with a **d**). `TopicBaseSystem` makes `ros2_control_node` abort (`exit -6`).

```xml
<plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
<param name="joint_commands_topic">/isaac_joint_commands</param>
<param name="joint_states_topic">/isaac_joint_states</param>
```

Those two topic names must match the Isaac Action Graph **exactly**.

After editing a **source** copy:

```bash
cd ~/ws_moveit2/panda_arm_example
colcon build --packages-select panda_arm_example
source install/setup.bash
```

---



## 3. One-time: Isaac Sim scene

1. Start Isaac from a Humble terminal (env vars must be on the **Isaac process**, not only on `ros2 topic list`):

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
# then launch Isaac from THIS terminal, e.g. ./isaac-sim.sh
```

1. Enable extensions:
  - **ROS 2 Bridge** (`isaacsim.ros2.bridge`) — required
  - ROS 2 Core / Nodes as needed
  - Isaac Sim ROS 2 Control is optional for this OmniGraph path
2. Action Graph (already saved in the scene):
  - **ROS2 Context** — domain ID `0`
  - **On Playback Tick** — wired so nodes run while Playing
  - **ROS2 Subscribe Joint State**
    - topic: `isaac_joint_commands`
    - outputs → **Articulation Controller**
  - **ROS2 Publish Joint State**
    - topic: `isaac_joint_states`
    - `targetPrim`: the real articulation root (the saved scene used `/Frankla` — that prim must exist)
  - **ROS2 Publish Clock** + **Isaac Read Simulation Time** (optional, useful later)
3. **Play** the sim. Loading a USD is not enough. No Play → no Isaac topics.

---



## 4. Every run (order matters)

Use **two clean terminals**. Do not launch MoveIt from a shell whose `LD_LIBRARY_PATH` still has `isaacsim/_build` leftovers.

### Terminal A — Isaac

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
# start Isaac, load the saved scene, enable ROS 2 Bridge, press Play
```



### Terminal B — check topics, then MoveIt

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
ros2 daemon stop
ros2 topic list
```

You must see at least:

- `/isaac_joint_commands`
- `/isaac_joint_states`
- usually `/clock` if the clock graph is running

If those are missing: Isaac is not Playing, bridge failed, or domain / RMW / daemon is stale.

Optional poke (Isaac arm should move if names match the USD):

```bash
ros2 topic pub --once /isaac_joint_commands sensor_msgs/msg/JointState "{
  name: ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7'],
  position: [0.2, -0.4, 0.0, -2.0, 0.0, 1.6, 0.8]
}"
```

Then start MoveIt (**one** demo only):

```bash
source ~/ws_moveit2/panda_arm_example/install/setup.bash
ros2 launch panda_arm_example demo.launch.py
```

Healthy `ros2 node list` includes `/controller_manager`.  
Spawners (`spawner_panda_arm_controller`, …) should **finish and disappear**, not sit on `list_controllers`.

In RViz: Plan, then **Execute**. Isaac must stay Playing.

---



## 5. Checks when it breaks

```bash
ros2 node list
ros2 topic list
ros2 topic echo /joint_states --once
ros2 topic echo /isaac_joint_commands --once
```


| Symptom                                        | Meaning                         | Fix                                                        |
| ---------------------------------------------- | ------------------------------- | ---------------------------------------------------------- |
| Isaac topics missing                           | Graph not running               | Play; launch Isaac from sourced Humble; `ros2 daemon stop` |
| `Waiting for at least 1 matching subscription` | Isaac not on this DDS graph     | Same domain / RMW; Play; bridge on                         |
| `ros2_control_node` `exit -6`                  | Hardware plugin failed          | See latest `~/.ros/log/*/launch.log` `what():`             |
| `TopicBaseSystem` does not exist               | Typo in xacro                   | Use `TopicBasedSystem`                                     |
| Spawners wait on `list_controllers`            | Manager never stayed up         | Fix plugin / URDF; relaunch                                |
| Execute: state time `0.000000`                 | No `/joint_states`              | Manager + broadcaster not running                          |
| RViz plans, Isaac frozen                       | Topics or joint names mismatch  | Align names; Articulation `targetPrim`                     |
| Two `/robot_state_publisher`                   | Two demos stacked               | `pkill` leftovers, launch once                             |
| Clock sync warning on one PC                   | Usually no joint state, not NTP | Fix `/joint_states` first                                  |


Pull the real crash line:

```bash
LATEST=$(ls -td ~/.ros/log/*/ | head -1)
grep -n "what():\|process has died\|TopicBase" "$LATEST/launch.log"
```

---



## 6. Bugs we already hit (do not repeat)

1. **ROS 1** `hawkEnv` **/** `rospy` — cannot talk to Isaac ROS 2 Bridge. Leave it for old scripts only (`isaac_sim_test/publish_topic_to_arm.py`).
2. **Forgot Play** — Bridge ON is not enough.
3. **Env vars only on** `ros2 topic list` — Isaac must be started with the same `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` / `source /opt/ros/humble/setup.bash`.
4. `ros-humble-mock-components` — not a Humble package. Mock hardware is in `hardware-interface`. We use **topic-based** hardware for Isaac, not mock.
5. `moveit_simple_controller_manager` **≠** `controller_manager` — the first is MoveIt’s client. The server is `ros2_control_node` (`/controller_manager`).
6. **Plugin typo** — `TopicBaseSystem` vs `TopicBasedSystem`.
7. **Topic name drift** — the working pair is `/isaac_joint_commands` + `/isaac_joint_states`. Isaac Action Graph and the xacro must use that same pair.
8. `pandas_gripper_controller` — likely a Setup Assistant typo (`panda` vs `pandas`). Can fail gripper spawn after the manager is up. Rename to match `ros2_controllers.yaml` / MoveIt controllers if the gripper spawner errors.
9. `/recognize_objects` **in RViz** — ignore; perception is not installed.
10. **Isaac** `LD_LIBRARY_PATH` **in the MoveIt terminal** — can abort Humble binaries. New shell, source Humble + the MoveIt workspace only.

---



## 7. What we did *not* finish

After the `TopicBasedSystem` rename + matching Isaac topics, `/controller_manager` should stay up, spawners should complete, Execute should publish `/isaac_joint_commands`, and the Isaac arm should move if joint names match the USD.

If Execute works in RViz but Isaac does not move: joint names, `targetPrim`, or subscribe topic.  
If Execute still says state time `0.0`: manager / `/joint_states` still dead — read `launch.log` again.

Do not point Isaac’s **publish** topic at `/joint_states` while TopicBasedSystem also writes commands from that same name, or you can loop.

---

## 8. Follow a Python curve with the tip

Isaac / MoveIt already running and Execute works. Then, on the sim machine:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/panda_arm_example/install/setup.bash

python3 /path/to/HAWK/isaac_sim_test/follow_curve_moveit.py
```

The script reads the current tip pose from TF, samples a small XY circle (same Z, same orientation), calls `/compute_cartesian_path`, then `/execute_trajectory`. Isaac follows because Execute still goes out `/isaac_joint_commands`.

If `fraction` is below `0.99`, shrink `--ros-args -p radius:=0.04` or start from a more reachable RViz pose.

If TF fails, check frames (`panda_link0` / `panda_link8`) against `ros2 run tf2_tools view_frames`.

Replace `sample_circle_waypoints()` when you want a different curve.

---

## Quick restart cheat sheet

```bash
# kill leftover MoveIt
pkill -f demo.launch.py; pkill -f move_group; pkill -f rviz2
pkill -f ros2_control_node; pkill -f robot_state_publisher; pkill -f spawner

# Isaac: Humble env, scene, Play
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash

# other terminal
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
ros2 daemon stop
ros2 topic list   # expect isaac_joint_* 
source ~/ws_moveit2/panda_arm_example/install/setup.bash
ros2 launch panda_arm_example demo.launch.py
```

