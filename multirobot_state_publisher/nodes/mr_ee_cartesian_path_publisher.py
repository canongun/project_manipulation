#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from math import pi, tau, dist, fabs, cos

class EECartesianPathPublisherNode:
    """
    The Multirobot End Effector Cartesian Path Publisher
    """
    def __init__(self, name):
    
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node(name, anonymous=False)

        # Instantiating a RobotCommander object. 
        # Provides information such as the robot’s kinematic model and the robot’s current joint states
        self.robot = moveit_commander.RobotCommander()

        # Instantiating a PlanningSceneInterface object. 
        # This provides a remote interface for getting, setting, 
        # and updating the robot’s internal understanding of the surrounding world
        self.scene = moveit_commander.PlanningSceneInterface()

        # Instantiating a MoveGroupCommander object. 
        # This object is an interface to a planning group (group of joints).
        group_name = "arm1"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz.
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

    def get_info(self):
        # Getting basic information

        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        # eef_link = move_group.get_end_effector_link()
        # print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

    def publish_position(self):
        # One can plan a cartesian path directly by specifying a list of waypoints for the end-effector to go through

        waypoints = []

        # Start with the current pose
        waypoints.append(self.move_group.get_current_pose().pose)

        # Orient the end-effector and move in the specified direction
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.x = waypoints[0].orientation.x
        wpose.orientation.y = waypoints[0].orientation.y
        wpose.orientation.z = waypoints[0].orientation.z
        wpose.orientation.w = waypoints[0].orientation.w
        wpose.position.x = waypoints[0].position.x
        wpose.position.y = waypoints[0].position.y
        wpose.position.z = waypoints[0].position.z
        waypoints.append(copy.deepcopy(wpose))
        
        # Enter the direction and the offset amount
        wpose.position.y = waypoints[0].position.y + 0.2
        waypoints.append(copy.deepcopy(wpose))

        # We want the cartesian path to be interpolated at a resolution of 1 cm which is why we will specify 0.01 as the eef_step in cartesian translation. 
        # We will specify the jump threshold as 0.0, effectively disabling it.

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                        waypoints,  # waypoints to follow
                                        0.01,       # eef_step
                                        0.0)        # jump_threshold
        
        # We are putting an ofsset in order to observe the trajectory in RViz first
        rospy.sleep(5)

        self.move_group.set_pose_target(wpose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    

def main(argv):
    
    node = EECartesianPathPublisherNode('ph_ee_coordinate_publisher_node')
    
    node.publish_position()

if __name__ == '__main__':
    main(sys.argv)