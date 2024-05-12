#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import actionlib
import math

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from multirobot_actions.msg import traj_planAction, traj_planResult

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

        self.move_group_1.set_planner_id('PTP')

        # Action Server Initialization part
        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name,      # Server name string
                                    traj_planAction,                # Action message type
                                    self.publish_position,              # Action Function
                                    auto_start = False 
                                    )
        
        self._as.start()

    def publish_position(self, goal):

        if goal:
            _result = traj_planResult()
            apose = self.move_group_2.get_current_pose().pose
            #IMPORTANT: apose (mobile_arm)'s planning frame is base_link! So the 'tf' is from 'base_link' to 'mobile_tool0'
            print("MOBILE ARM POSE= {}".format(apose))

            # Get Quaternions of the mobile arm and convert into Euler
            orientation_q = apose.orientation
            orientation_q_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_q_list)
            
            # Calculate the roration for the fixed arm the send it to the opposite side
            self.yaw += math.pi + goal.tetha * math.pi/180
            (self.x_alt, self.y_alt, self.z_alt, self.w_alt) = quaternion_from_euler(self.roll, self.pitch, self.yaw)

            center_mobile_x = apose.position.x
            center_mobile_y = apose.position.y
        
            r = math.sqrt(center_mobile_x**2 + center_mobile_y**2) #  Hypotenuse from mobile_base_link to mobile_tool0
            β = math.atan(center_mobile_y / center_mobile_x) #  Angle between x and y axis

            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.orientation.x = self.x_alt
            pose_goal.orientation.y = self.y_alt 
            pose_goal.orientation.z = self.z_alt
            pose_goal.orientation.w = self.w_alt
            pose_goal.position.x = r * math.cos(β + goal.tetha * math.pi/180) + rospy.get_param("/mobile_x") - 0.1 * math.sin(-goal.tetha * math.pi/180)
            pose_goal.position.y = r * math.sin(β + goal.tetha * math.pi/180) + rospy.get_param("/mobile_y") - 0.1 * math.cos(goal.tetha * math.pi/180)
            pose_goal.position.z = apose.position.z + rospy.get_param("/mobile_z") - 0.05 # -0.05 because of the transformation from odom to fixed_base_link

            print('\nBETA= {}, r= {}, POSE= {}'.format(β, r, pose_goal))

            self.move_group_1.set_pose_target(pose_goal)

            # Call the planner to compute the plan and execute it
            # 'go()' returns a boolean indicating whether the plan and execute it
            success = self.move_group_1.go(wait=True)
            
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
    rospy.spin()