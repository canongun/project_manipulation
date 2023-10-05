#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from math import pi, tau, dist, fabs, cos

class EEPositionPublisherNode:
    """
    The Multirobot End Effector Position Publisher
    """
    def __init__(self, name, oritentation_x, oritentation_y, oritentation_z, oritentation_w, pos_x, pos_y, pos_z):
        self.orientation_x = oritentation_x
        self.orientation_y = oritentation_y
        self.orientation_z = oritentation_z
        self.orientation_w = oritentation_w
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_z = pos_z

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

        # Important: Setting up the reference frame of the kinematic chain
        self.move_group.set_pose_reference_frame('table_base_link')

        self.get_info()

        # We can plan a motion for this group to a desired pose for the end-effector
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = self.orientation_x
        pose_goal.orientation.y = self.orientation_y
        pose_goal.orientation.z = self.orientation_z
        pose_goal.orientation.w = self.orientation_w
        pose_goal.position.x = self.pos_x
        pose_goal.position.y = self.pos_y
        pose_goal.position.z = self.pos_z

        self.move_group.set_pose_target(pose_goal)

        # Now, we call the planner to compute the plan and execute it

        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

    

def main(argv):
    if len(argv) < 8:
        print("Usage: rosrun pinhandling_state_publisher ph_ee_coordinate_publisher.py <orientation_x> <orientation_y> <orientation_z> <orientation_w> <pos_x> <pos_y> <pos_z>")
        return
    
    orientation_x = float(argv[1])
    orientation_y = float(argv[2])
    orientation_z = float(argv[3])
    orientation_w = float(argv[4])
    pos_x = float(argv[5])
    pos_y = float(argv[6])
    pos_z = float(argv[7])

    node = EEPositionPublisherNode('ph_ee_coordinate_publisher_node',
                                   orientation_x,
                                   orientation_y,
                                   orientation_z,
                                   orientation_w,
                                   pos_x,
                                   pos_y,
                                   pos_z
                                   )
    
    node.publish_position()

if __name__ == '__main__':
    main(sys.argv)