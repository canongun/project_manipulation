#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from multirobot_actions.msg import ee_planAction, ee_planFeedback, ee_planResult

class SendEEOppositeServer():
    # Create messages that are used to publish feedback/result

    def __init__(self, name):
        
        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiating a RobotCommander object. 
        # Provides information such as the robot’s kinematic model and the robot’s current joint states
        self.robot = moveit_commander.RobotCommander(robot_description="/fixed/robot_description", ns="fixed")
        self.robot_2 = moveit_commander.RobotCommander(robot_description="/robot_description")

        # Instantiating a PlanningSceneInterface object. 
        # This provides a remote interface for getting, setting, 
        # and updating the robot’s internal understanding of the surrounding world
        self.scene = moveit_commander.PlanningSceneInterface(ns="fixed")
        self.scene_2 = moveit_commander.PlanningSceneInterface()

        # Instantiating a MoveGroupCommander object. 
        # This object is an interface to a planning group (group of joints).
        group_name_1 = "fixed_arm"
        group_name_2 = "mobile_arm"
        self.move_group_1 = moveit_commander.MoveGroupCommander(group_name_1, robot_description="/fixed/robot_description", ns="fixed")
        self.move_group_2 = moveit_commander.MoveGroupCommander(group_name_2)

        self.move_group_1.set_pose_reference_frame('fixed/base_link')
        self.move_group_2.set_pose_reference_frame('mobile_base_link')

        # Action Server Initialization part
        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name,      # Server name string
                                    ee_planAction,                # Action message type
                                    self.publish_position,              # Action Function
                                    auto_start = False 
                                    )
        
        self._as.start()

    def publish_position(self, goal):

        if goal:
            _feedback = ee_planFeedback()
            _result = ee_planResult()
            apose = self.move_group_2.get_current_pose().pose

            _feedback.achieved_orientation_x = apose.orientation.x
            _feedback.achieved_orientation_y = apose.orientation.y
            _feedback.achieved_orientation_z = apose.orientation.z
            _feedback.achieved_orientation_w = apose.orientation.w
            _feedback.achieved_position_x = apose.position.x
            _feedback.achieved_position_y = apose.position.y
            _feedback.achieved_position_z = apose.position.z
            
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.orientation.x = apose.orientation.x 
            pose_goal.orientation.y = -apose.orientation.y 
            pose_goal.orientation.z = apose.orientation.z 
            pose_goal.orientation.w = apose.orientation.w 
            pose_goal.position.x = apose.position.x - 0.443 # 0.450-0.007 because of the transformation from odom to fixed_base_link
            pose_goal.position.y = apose.position.y + 1.4
            pose_goal.position.z = apose.position.z + 0.350 # 0.4-0.05 because of the transformation from odom to fixed_base_link

            self.move_group_1.set_pose_target(pose_goal)

            # Call the planner to compute the plan and execute it
            # 'go()' returns a boolean indicating whether the plan and execute it
            success = self.move_group_1.go(wait=True)

            self._as.publish_feedback(_feedback)

            if success:
                _result.finish = True

                rospy.loginfo('%s: Operation Result: Succeeded' % self._action_name)
                self._as.set_succeeded(_result, "True")
                
                # Call 'stop()' to ensure that there is no residual movement
                self.move_group_1.stop()
                # Clear targets after planning poses
                self.move_group_1.clear_pose_targets()

            else:
                _result.finish = False
                rospy.loginfo('%s: Operation Result: Failed' % self._action_name)
                self._as.set_aborted(_result, "False")

                # Call 'stop()' to ensure that there is no residual movement
                self.move_group_1.stop()
                # Clear targets after planning poses
                self.move_group_1.clear_pose_targets()

if __name__ == '__main__':
    # rospy.set_param('/robot_description', rospy.get_param('/fixed/robot_description'))
    rospy.init_node('send_ee_opposite_action')
    server = SendEEOppositeServer(rospy.get_name())
    # rospy.spin()