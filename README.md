# project_manipulation


## Multirobot State Publisher

___mobile_platform_controller.py___

This script is responsible for the mobile platform's linear and rotational controller. Since the mobile platform is dragging in simulation (while not publishing and Twist() message), it is crucial to publish a linear x velocity to make platform steady. That's why users should execute this controller in the beginning of the simulation. After it asks for an input, if the input starts with 'linear' it publishes a constant velocity until the desired translation is done. If the input starts with 'tetha' it sends a request to the mobile platform rotation controller that uses a custom PID controller to rotate the platform.

**Important:** To be able to use rotation controller, users has to execute both __service__ and __action__ launch files as explained down below.

```
rosrun multirobot_state_publisher mobile_platform_controller.py

```

---

## Multirobot Services

ROS Service packages for multirobot project.

__ground_truth_listener_mobile_tool0_server__

It creates a server that listens to the ground truth of the mobile arm's "tool0" link, and returns a Response that includes the position and orientation information of the link when there is a request.

__ground_truth_listener_base_link_server.py__
It creates a server that listens to the ground truth of the mobile platform's "base_link", and returns a Response that includes the position and orientation information of the link when there is a request.

Launching the services:

```
roslaunch multirobot_services multirobot_service.launch

```

## Multirobot Actions

ROS Action packages for multirobot project.

__freeze_mobile_ee_server.py__

It locks the end-effector of the mobile platform to the current position and orientation. It handles the disturbances and movements that are coming from the platform. So to say, it fixes the end-effector to the 'world' rather than 'mobile_base_link'.

__send_ee_opposite_server.py__

It sends the fixed arm to the opposite side of the mobile arm, aligning the Z-axis.

__mobile_platform_rotation_controller_server.py__

PID rotation controller for mobile robot platform in order to be more accurate when rotating.

__move_both_server.py__

It includes the algorithms to move both fixed and mobie arm synchronously.

Launching the Actions:

```
roslaunch multirobot_actions multirobot_action.launch

```

After, launching both Services and Actions, users should execute the following lines in order:

First, set both arms to their "operation_ready" pose by usinng their MoveIt! packages.

To move both arms, first execute:

```
rosrun multirobot_actions send_ee_oppsite_client.py

```

Second, execute:

```
rosrun multirobot_actions move_both_client.py

```

With changing the action message parameters in 'move_both_client.py' script, users can alter the linear position and angle.

To freeze the mobile end-effector, first execute:

```
rosrun multirobot_actions freeze_mobile_ee_server.py

```

Second, execute:

```
rosrun multirobot_actions freeze_mobile_ee_client.py

```