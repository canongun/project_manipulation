#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import actionlib
import geometry_msgs.msg
import math
from moveit_msgs.msg import RobotTrajectory

import multirobot_actions.msg
from multirobot_actions.msg import traj_planAction, traj_planFeedback, traj_planResult


class MoveBothServer():
    # Create messages that are used to publish feedback/result
    planning_time: float
    tooltip_frame: str
    _trajectory: RobotTrajectory
    _target_joint_values: float
    

    def __init__(self, name):
        
        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiating a RobotCommander object. 
        # Provides information such as the robot’s kinematic model and the robot’s current joint states
        self.robot = moveit_commander.RobotCommander(robot_description="/fixed/robot_description", ns="fixed")

        # Instantiating a PlanningSceneInterface object. 
        # This provides a remote interface for getting, setting, 
        # and updating the robot’s internal understanding of the surrounding world
        self.scene = moveit_commander.PlanningSceneInterface(ns="fixed")

        # Instantiating a MoveGroupCommander object. 
        # This object is an interface to a planning group (group of joints).
        group_name_1 = "fixed_arm"
    
        self.move_group_1 = moveit_commander.MoveGroupCommander(group_name_1, robot_description="/fixed/robot_description", ns="fixed")

        # Set robot planner, necessary for Pilz Motion Planner
        self.move_group_1.set_planner_id('LIN')
        # The LIN motion command:
        # This planner generates linear Cartesian trajectory between goal and start poses. 
        # The planner uses the Cartesian limits to generate a trapezoidal velocity profile in Cartesian space. 
        # The translational motion is a linear interpolation between start and goal position vector. 
        # The rotational motion is quaternion slerp between start and goal orientation. 
        # The translational and rotational motion is synchronized in time. 
        # This planner only accepts start state with zero velocity. 
        # Planning result is a joint trajectory. 
        # The user needs to adapt the Cartesian velocity/acceleration scaling factor if the motion plan fails due to violation of joint space limits.
        
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
            _feedback = traj_planFeedback()
            _result = traj_planResult()

            self.linear = goal.linear
            self.tetha = goal.tetha

            client = actionlib.SimpleActionClient('send_ee_opposite_action', multirobot_actions.msg.traj_planAction)
            # Wait until the action server has started up and started listening for goals.
            client.wait_for_server()

            # Set client for mobile platform rotation
            client2 = actionlib.SimpleActionClient('mobile_platform_rotation_controller_action', multirobot_actions.msg.traj_planAction)
            # Wait until the action server has started up and started listening for goals.
            client2.wait_for_server()

            # Send goal to send the slave arm in front of the master arm
            goal = multirobot_actions.msg.traj_planGoal(start = True, linear = self.linear, tetha = self.tetha)
            # Send the goal to the action server
            client.send_goal(goal)

            # Send goal to start rotation controller of the mobile platform
            goal2 = multirobot_actions.msg.traj_planGoal(start = True, linear = self.linear, tetha = self.tetha)
            # Send the goal to the action server
            client2.send_goal(goal2)

            # Wait for the server to finish performing the action
            client.wait_for_result()
            # Wait for the server to finish performing the action
            client2.wait_for_result()
            rospy.sleep(2)

            self.move_group_1.set_planner_id('LIN')
            
            wpose = self.move_group_1.get_current_pose().pose

            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.orientation.x = wpose.orientation.x 
            pose_goal.orientation.y = wpose.orientation.y 
            pose_goal.orientation.z = wpose.orientation.z 
            pose_goal.orientation.w = wpose.orientation.w 
            pose_goal.position.x = wpose.position.x + self.linear * math.cos(self.tetha * math.pi/180)
            pose_goal.position.y = wpose.position.y + self.linear * math.sin(self.tetha * math.pi/180)
            pose_goal.position.z = wpose.position.z

            self.move_group_1.set_pose_target(pose_goal)

            success, self._trajectory, self.planning_time, error_code = self.move_group_1.plan()

            secs = self._trajectory.joint_trajectory.points[-1].time_from_start.secs
            nsecs = self._trajectory.joint_trajectory.points[-1].time_from_start.nsecs

            print("secs:", secs)
            print("nsecs:", nsecs)

            if success:

                client = actionlib.SimpleActionClient('move_mobile_action', multirobot_actions.msg.traj_planAction)
                # Wait until the action server has started up and started listening for goals.
                client.wait_for_server()
                # Creates a goal to send to the action server.
                goal = multirobot_actions.msg.traj_planGoal(start = True, linear = self.linear, tetha = self.tetha)
                # Send the goal to the action server
                client.send_goal(goal)
                
                self.move_group_1.execute(self._trajectory, True)

                # Wait for the server to finish performing the action
                client.wait_for_result()


                client = actionlib.SimpleActionClient('align_ee_opposite_action', multirobot_actions.msg.traj_planAction)
                # Wait until the action server has started up and started listening for goals.
                client.wait_for_server()
                # Creates a goal to send to the action server.
                goal = multirobot_actions.msg.traj_planGoal(start = True, linear = self.linear, tetha = self.tetha)
                # Send the goal to the action server
                client.send_goal(goal)
                
                self.move_group_1.execute(self._trajectory, True)

                # Wait for the server to finish performing the action
                client.wait_for_result()


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
    rospy.init_node('move_both_action')
    server = MoveBothServer(rospy.get_name())
    rospy.spin()