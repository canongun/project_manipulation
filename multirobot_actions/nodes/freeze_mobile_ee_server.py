#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import actionlib
import math
# import time

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from multirobot_actions.msg import ee_planAction, ee_planResult
from multirobot_services.srv import GroundTruthListener

LINK_DIST = 0.081
PUB_HZ = 1/100

class FreezeMobileEEServer():
    # Create messages that are used to publish feedback/result

    def __init__(self, name):
        
        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiating a RobotCommander object. 
        # Provides information such as the robot’s kinematic model and the robot’s current joint states
        self.robot_2 = moveit_commander.RobotCommander(robot_description="/robot_description")

        # Instantiating a PlanningSceneInterface object. 
        # This provides a remote interface for getting, setting, 
        # and updating the robot’s internal understanding of the surrounding world
        self.scene_2 = moveit_commander.PlanningSceneInterface()

        # Instantiating a MoveGroupCommander object. 
        # This object is an interface to a planning group (group of joints).
        group_name_2 = "mobile_arm"
        self.move_group_2 = moveit_commander.MoveGroupCommander(group_name_2)

        self.move_group_2.set_planner_id('PTP')

        self.move_group_2.set_max_acceleration_scaling_factor(1)
        self.move_group_2.set_max_velocity_scaling_factor(1)

        # Transformation is from base_link -> mobile_tool0
        self.mobile_arm_pose = self.move_group_2.get_current_pose().pose

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
            rospy.wait_for_service('/ekf_odometry_listener')

            ekf_odom_listener = rospy.ServiceProxy('/ekf_odometry_listener', # service name
                                            GroundTruthListener   # service type
                                            )

            position_mobile_tool0 = [self.mobile_arm_pose.orientation.x, 
                                    self.mobile_arm_pose.orientation.y, 
                                    self.mobile_arm_pose.orientation.z, 
                                    self.mobile_arm_pose.orientation.w
                                    ]
               
            (roll_mobile_tool0, pitch_mobile_tool0, yaw_mobile_tool0) = euler_from_quaternion(position_mobile_tool0)

            resp_efk_odom = ekf_odom_listener(True)

            x0 = resp_efk_odom.link_info[0] + LINK_DIST * math.cos(0)
            y0 = resp_efk_odom.link_info[1] + LINK_DIST * math.sin(0)

            # counter = 0
            # start_time = time.time()
            
            while True:
                resp_efk_odom = ekf_odom_listener(True)

                position_base_link = [resp_efk_odom.link_info[3], 
                                    resp_efk_odom.link_info[4], 
                                    resp_efk_odom.link_info[5], 
                                    resp_efk_odom.link_info[6]
                                    ]
            
                (roll_base_link, pitch_base_link, yaw_base_link) = euler_from_quaternion(position_base_link)

                yaw_real = -yaw_base_link

                roll_base_link -= math.pi / 2
                yaw_base_link -= math.pi

                Δor_R = roll_mobile_tool0 - roll_base_link
                Δor_P = pitch_mobile_tool0 - pitch_base_link
                Δor_Y = yaw_mobile_tool0 - yaw_base_link

                al_R = roll_mobile_tool0 + Δor_R
                al_P = pitch_mobile_tool0 - Δor_P
                al_Y = yaw_mobile_tool0 + Δor_Y

                (arm_or_x, arm_or_y, arm_or_z, arm_or_w) = quaternion_from_euler(al_R, al_P, al_Y)
                
                Δpos_x = resp_efk_odom.link_info[0] + LINK_DIST * math.cos(yaw_real)
                Δpos_y = resp_efk_odom.link_info[1] - LINK_DIST * math.sin(yaw_real)
                # Δpos_z = resp_base_link.link_info[2] - rospy.get_param("/mobile_z")

                print("POS_X= {},   POS_Y= {},  YAW= {}".format(Δpos_x, Δpos_y, yaw_real))

                trans_x = self.mobile_arm_pose.position.x + x0 - Δpos_x
                trans_y = self.mobile_arm_pose.position.y + y0 - Δpos_y

                rot_x = trans_x * math.cos(yaw_real) - trans_y * math.sin(yaw_real) + LINK_DIST - LINK_DIST * math.cos(yaw_real)
                rot_y = trans_x * math.sin(yaw_real) + trans_y * math.cos(yaw_real) - LINK_DIST * math.sin(yaw_real)

                pose_goal = geometry_msgs.msg.Pose()
                pose_goal.orientation.x = arm_or_x
                pose_goal.orientation.y = arm_or_y
                pose_goal.orientation.z = arm_or_z
                pose_goal.orientation.w = arm_or_w
                pose_goal.position.x = rot_x
                pose_goal.position.y = rot_y
                pose_goal.position.z = self.mobile_arm_pose.position.z #+ Δpos_z - 0.05

                rospy.loginfo("{}".format(pose_goal))

                self.move_group_2.set_pose_target(pose_goal)

                success, self._trajectory, self.planning_time, error_code = self.move_group_2.plan()

                print("\nPlanning Time: {}".format(self.planning_time))

                self.move_group_2.execute(self._trajectory, True)

                # print("COUNTER:", counter)
                # counter += 1
                # # Check elapsed time every 1000 iterations
                # if counter % 100 == 0:
                #     elapsed_time = time.time() - start_time
                #     frequency = counter / elapsed_time
                #     print(f"Loop frequency: {frequency:.2f} iterations per second")

                # rospy.sleep(PUB_HZ)

if __name__ == '__main__':
    rospy.init_node('freeze_mobile_ee_action')
    server = FreezeMobileEEServer(rospy.get_name())
    rospy.spin()