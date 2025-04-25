#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import actionlib
import math

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from multirobot_actions.msg import traj_planAction, traj_planResult

from multirobot_services.srv import GroundTruthListener

class AlignEEOppositeServer():
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

        if goal.start:
            _result = traj_planResult()
            
            # Starting the ground truth listener service
            rospy.loginfo("Requesting ground truth listener service")
            rospy.wait_for_service('/ground_truth_listener_mobile_tool0')

            ground_truth_listener = rospy.ServiceProxy('/ground_truth_listener_mobile_tool0', # service name
                                            GroundTruthListener   # service type
                                            )
            resp1 = ground_truth_listener(True)

            orientation_q_list = [resp1.link_info[3], resp1.link_info[4], resp1.link_info[5], resp1.link_info[6]]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_q_list)

            yaw += math.pi

            (x_alt, y_alt, z_alt, w_alt) = quaternion_from_euler(roll, pitch, yaw)

            # Pose information for the final pose of the slave arm
            final_pose = geometry_msgs.msg.Pose()
            final_pose.position.x = resp1.link_info[0] - 0.1 * math.sin(-goal.tetha * math.pi/180)
            final_pose.position.y = resp1.link_info[1] - 0.1 * math.cos(goal.tetha * math.pi/180)
            final_pose.position.z = resp1.link_info[2]
            final_pose.orientation.x = x_alt
            final_pose.orientation.y = y_alt
            final_pose.orientation.z = z_alt
            final_pose.orientation.w = w_alt

            self.move_group_1.set_pose_target(final_pose)

            success, self._trajectory, self.planning_time, error_code = self.move_group_1.plan()

            self.move_group_1.execute(self._trajectory, True)
            
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
    rospy.init_node('align_ee_opposite_action')
    server = AlignEEOppositeServer(rospy.get_name())
    rospy.spin()