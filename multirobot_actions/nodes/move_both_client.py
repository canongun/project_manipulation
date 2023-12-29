#! /usr/bin/env python3

from __future__ import print_function
import sys
import rospy
import actionlib
import multirobot_actions.msg

def send_ee_opposite_client():
    
    client = actionlib.SimpleActionClient('move_both_action', multirobot_actions.msg.ee_planAction)

    # Wait until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = multirobot_actions.msg.ee_planGoal(start = True)

    # Send the goal to the action server
    client.send_goal(goal)
    # Wait for the server to finish performing the action
    client.wait_for_result()
    rospy.sleep(2)

if __name__ == '__main__':
    try:
        # Initialize a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS
        rospy.init_node('move_both_client_py')
        result = send_ee_opposite_client()
        

    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)