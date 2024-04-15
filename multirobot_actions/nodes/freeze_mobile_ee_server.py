#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import actionlib
import math

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from multirobot_actions.msg import ee_planAction, ee_planResult
from multirobot_services.srv import GroundTruthListener

class FreezeMobileEEServer():
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

        self.move_group_2.set_planner_id('RRTConnect')

        self.move_group_2.set_max_acceleration_scaling_factor(1)
        self.move_group_2.set_max_velocity_scaling_factor(1)

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
            _result = ee_planResult()

            # Starting the ground truth listener service
            rospy.loginfo("Requesting ground truth listener service")
            rospy.wait_for_service('/ground_truth_listener_base_link')

            ground_truth_listener_base_link = rospy.ServiceProxy('/ground_truth_listener_base_link', # service name
                                            GroundTruthListener   # service type
                                            )
            
            mobile_arm_pose = self.move_group_2.get_current_pose().pose

            while True:
                resp_base_link = ground_truth_listener_base_link(True)

                position_base_link = [resp_base_link.link_info[3], 
                                    resp_base_link.link_info[4], 
                                    resp_base_link.link_info[5], 
                                    resp_base_link.link_info[6]
                                    ]
            
                (roll_base_link, pitch_base_link, yaw_base_link) = euler_from_quaternion(position_base_link)

                roll_base_link += -math.pi / 2
                yaw_base_link += -math.pi

                position_mobile_tool0 = [mobile_arm_pose.orientation.x, 
                                        mobile_arm_pose.orientation.y, 
                                        mobile_arm_pose.orientation.z, 
                                        mobile_arm_pose.orientation.w
                                        ]
                
                (roll_mobile_tool0, pitch_mobile_tool0, yaw_mobile_tool0) = euler_from_quaternion(position_mobile_tool0)

                self.move_group_2.set_start_state_to_current_state()

                Δor_R = roll_base_link - roll_mobile_tool0 
                Δor_P = pitch_base_link - pitch_mobile_tool0
                Δor_Y = yaw_base_link - yaw_mobile_tool0

                rospy.loginfo("ΔR= {},  ΔP= {},  ΔY= {}".format(Δor_R, Δor_P, Δor_Y))

                (Δor_x, Δor_y, Δor_z, Δor_w) = quaternion_from_euler(Δor_R, Δor_P, Δor_Y)
                
                Δpos_x = rospy.get_param("/mobile_x") - resp_base_link.link_info[0] 
                Δpos_y = rospy.get_param("/mobile_y") - resp_base_link.link_info[1]
                Δpos_z = rospy.get_param("/mobile_z") - resp_base_link.link_info[2]

                pose_goal = geometry_msgs.msg.Pose()
                pose_goal.orientation.x = mobile_arm_pose.orientation.x #+ Δor_x
                pose_goal.orientation.y = mobile_arm_pose.orientation.y #+ Δor_y
                pose_goal.orientation.z = mobile_arm_pose.orientation.z #+ Δor_z
                pose_goal.orientation.w = mobile_arm_pose.orientation.w #+ Δor_w
                pose_goal.position.x = mobile_arm_pose.position.x + Δpos_x
                pose_goal.position.y = mobile_arm_pose.position.y + Δpos_y
                pose_goal.position.z = mobile_arm_pose.position.z + Δpos_z

                rospy.loginfo("{}".format(pose_goal))

                self.move_group_2.set_pose_target(pose_goal)

                success, self._trajectory, self.planning_time, error_code = self.move_group_2.plan()

                print("\nPlanning Time: {}".format(self.planning_time))

                self.move_group_2.execute(self._trajectory, True)

                rospy.sleep(1/120)

                # if success:
                #     _result.finish = True

                #     rospy.loginfo('%s: Operation Result: Succeeded' % self._action_name)
                #     self._as.set_succeeded(_result, "True")
                    
                #     # Call 'stop()' to ensure that there is no residual movement
                #     self.move_group_2.stop()
                #     # Clear targets after planning poses
                #     self.move_group_2.clear_pose_targets()

                # else:
                #     _result.finish = False
                #     rospy.loginfo('%s: Operation Result: Failed' % self._action_name)
                #     self._as.set_aborted(_result, "False")

                #     # Call 'stop()' to ensure that there is no residual movement
                #     self.move_group_2.stop()
                #     # Clear targets after planning poses
                #     self.move_group_2.clear_pose_targets()

if __name__ == '__main__':
    rospy.init_node('freeze_mobile_ee_action')
    server = FreezeMobileEEServer(rospy.get_name())
    rospy.spin()