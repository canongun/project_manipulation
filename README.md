# project_manipulation


## Usage of Packages

___mr_ee_coordinate_publisher.py___

**Note:** In order to use the script, first you need to execute moveit_planning_context launch file.

```
roslaunch multirobot_v1_moveit_config moveit_planning_context.launch

```

**Usage:** 'rosrun multirobot_state_publisher mr_ee_coordinate_publisher.py **orientation_x** **orientation_y** **orientation_z** **orientation_w** **pos_x** **pos_y** **pos_z**'
**Note:** orientation_x [rad], orientation_y [rad], orientation_z [rad], orientation_w [rad], pos_x [m], pos_y [m], pos_z [m] (All inputs must be float!)
**Important:** This node uses 'move_group' class from MoveIt! and calculates the position of 'arm1_wrist_3_link' relative to 'table_base_link'! So when users give a point that they want to reach, they must calculate that point according to 'table_base_link' frame.

```
rosrun multirobot_state_publisher mr_ee_coordinate_publisher.py 0.300 -0.070 1.703 -0.702 0.000 0.000 0.712

```

___mr_ee_cartesian_path_publisher.py___

Specify the direction and offset amount int the code, then just execute:

```
rosrun multirobot_state_publisher mr_ee_cartesian_path_publisher.py

```

---

## Multirobot Services

ROS Service packages for multirobot project.

__ground_truth_listener_server.py__

It creates a server that listens to the ground truth of the mobile arm's "tool0" link, and returns a Response that includes the position and orientation information of the link when there is a request.

Launching the services:

```
roslaunch multirobot_services multirobot_service.launch

```

## Multirobot Actions

ROS Action packages for multirobot project.

__send_ee_opposite_server.py__

It sends the fixed arm to the opposite side of the mobile arm, aligning the Z-axis.

__move_mobile_platform_server.py__

It executes commands to move the mobile platform.

__move_both_server.py__

It includes the algorithms to move both fixed and mobie arm synchronously.

Launching the Actions:

```
roslaunch multirobot_actions multirobot_action.launch

```

After, launching both Services and Actions, users should execute the following lines in order:

First, set both arms to their "operation_ready" pose by usinng their MoveIt! packages.

Second, execute:

```
rosrun multirobot_actions send_ee_opposite_client.py

```

Third, execute:

```
rosrun multirobot_actions move_both_client.py

```