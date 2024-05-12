#! /usr/bin/env python3

from __future__ import print_function
import sys
import rospy
import actionlib
import multirobot_actions.msg

def move_mobile_platform_localization_client():
    client = actionlib.SimpleActionClient('move_mobile_localization_action', multirobot_actions.msg.traj_planAction)

    # Wait until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = multirobot_actions.msg.traj_planGoal(start = True, linear = -0.2, tetha = 0)

    # Send the goal to the action server
    client.send_goal(goal)
    # Wait for the server to finish performing the action
    client.wait_for_result()

if __name__ == '__main__':
    try:
        # Initialize a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS
        rospy.init_node('move_mobile_platform_localization_client_py')
        result = move_mobile_platform_localization_client()
        

    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)