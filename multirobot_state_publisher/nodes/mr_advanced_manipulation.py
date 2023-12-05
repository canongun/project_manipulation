#! /usr/bin/env python3

import sys
import os
import copy
import rospy
import PyKDL 
import time
import moveit_commander
from moveit_msgs.msg import *
from moveit_msgs.srv import *
import geometry_msgs.msg
from std_msgs.msg import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import math
from sensor_msgs.msg import *
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import xml.etree.ElementTree as ET


# ROS init
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('mr_moveit_extension', anonymous=True)

# Define the movegroup
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_left = moveit_commander.MoveGroupCommander("arm1")
arm_right = moveit_commander.MoveGroupCommander("arm2")
arms = moveit_commander.MoveGroupCommander("multirobot")

arm_left.clear_pose_targets()
arm_right.clear_pose_targets()
arms.clear_pose_targets()

class EEF(object):
    """This class defines an end effector"""
    def __init__(self, EE_end_frame=PyKDL.Frame(), x=0, y=0, z=0, ATC_frame=PyKDL.Frame(), name="", path=""):
        """
        - EE_end_frame: Transform from the EEF base frame (the ATC part of the tool) to its actuation frame [PyKDL.Frame]
        - x: Tool size in x dimension [float]
        - y: Tool size in y dimension [float]
        - z: Tool size in z dimension. This includes the ATC part of the tool [float]
        - ATC_frame: Frame of the EEF slot in the tool changer station [PyKDL.Frame]
        - name: Name of the tool. Name of the collision object created for the tool [string]
        - path: Path of the stl model of the tool [string]
        """
        self.EE_end_frame = EE_end_frame
        self.x = x
        self.y = y
        self.z = z
        self.ATC_frame = ATC_frame
        self.name = name
        self.path = path

class ATC(object):
    """
    This class manages everything related with the EEFs of the scene, including tool changing, goal poses corrections to the EEF action frames, etc.
    """
    global scene

    def __init__(self, frame_id = "table_base_link", left_tool = EEF(), eef_link_left = "", right_tool = EEF(), eef_link_right = "", ATC_tools = [], left_ATC_angle=0, right_ATC_angle=0, left_ATC_dist=0, right_ATC_dist=0):
        """
        The constructor function initializes the tools state, adds the tools to the scene, and updates the allowed collision matrix to consider EEF collisions.
        - frame_id: Name of the parent frame used to define the ATC_frames of the EEFs [String]
        - left_tool: EEF attached to the left arm of the robot [EEF]
        - eef_link_left: Name of the link attached to the EEF of the left arm [string]
        - right_tool: EEF attached to the right arm of the robot [EEF]
        - eef_link_right: Name of the link attached to the EEF of the right arm [string]
        - ATC_tools: List of EEFs located initially in the ATC station [list(EEF)]
        - left_ATC_angle: Z angle between the left arm wrist and the left arm tool changer/EEF
        - right_ATC_angle: Z angle between the left arm wrist and the right arm tool changer/EEF
        - left_ATC_dist: Length of the robot part of the left ATC
        - right_ATC_dist: Length of the robot part of the right ATC
        """
        self.left_ATC_angle = left_ATC_angle
        self.right_ATC_angle = right_ATC_angle
        self.left_ATC_dist = left_ATC_dist
        self.right_ATC_dist = right_ATC_dist
        self.eef_link_left = eef_link_left
        self.eef_link_right = eef_link_right

        #Remove other EEF collision objects from the scene
        attached_objects = scene.get_attached_objects()
        if len(attached_objects)>0:
            for object_name in attached_objects:
                if "EEF" in object_name:
                    try:
                        scene.remove_attached_object(eef_link_left, name=object_name)
                        self.wait_update_object(object_name, EE_is_attached=False, EE_is_known=True)
                        rospy.sleep(0.5)
                    except:
                        pass
                    try:
                        scene.remove_attached_object(eef_link_right, name=object_name)
                        self.wait_update_object(object_name, EE_is_attached=False, EE_is_known=True)
                        rospy.sleep(0.5)
                    except:
                        pass

                scene_objects = scene.get_known_object_names()
                for object_name in scene_objects:   
                    if "EEF" in object_name:     
                        scene.remove_world_object(object_name)
                        self.wait_update_object(object_name, EE_is_attached=False, EE_is_known=False)
                        rospy.sleep(0.5)

        #Add collision objects for the EEFs
        self.EEF_left = ""
        self.EEF_right = ""
        #Dictionary with all the EEF
        self.EEF_dict = {}

        #Add Left EEF
        if left_tool.name != "":
            self.EEF_dict[left_tool.name] = left_tool
            EE_pose = geometry_msgs.msg.PoseStamped()
            EE_pose.header.frame_id = frame_id
            left_wrist_pose = arm_left.get_current_pose().pose
            EE_pose.pose = self.correctPose(left_wrist_pose, "left", ATC_sign = 1)
            self.EEF_left = left_tool.name
            try:
                scene.add_mesh(self.EEF_left, EE_pose, left_tool.path)
                self.wait_update_object(self.EEF_left, EE_is_attached=False, EE_is_known=True)
                #Attach gripper
                rospy.sleep(0.5)
                scene.attach_mesh(eef_link_left, self.EEF_left, touch_links=[eef_link_left])
                self.wait_update_object(self.EEF_left, EE_is_attached=True, EE_is_known=False)
            except:
                print("Error adding left EEF to the scene")

        #Add right EEF
        if right_tool.name != "":
            self.EEF_dict[right_tool.name] = right_tool
            EE_pose = geometry_msgs.msg.PoseStamped()
            EE_pose.header.frame_id = frame_id
            right_wrist_pose = arm_right.get_current_pose().pose
            EE_pose.pose = self.correctPose(right_wrist_pose, "right", ATC_sign = 1)
            self.EEF_right = right_tool.name
            try:
                scene.add_mesh(self.EEF_right, EE_pose, right_tool.path)
                self.wait_update_object(self.EEF_right, EE_is_attached=False, EE_is_known=True)
                #Attach gripper
                rospy.sleep(0.5)
                scene.attach_mesh(eef_link_right, self.EEF_right, touch_links=[eef_link_right])
                self.wait_update_object(self.EEF_right, EE_is_attached=True, EE_is_known=False)
            except:
                print("Error adding right EEF to the scene")

        #Add the rest of tools in the ATC platform
        for tool in ATC_tools:
            self.EEF_dict[tool.name] = tool
            EE_pose = geometry_msgs.msg.PoseStamped()
            EE_pose.header.frame_id = frame_id
            EE_pose.pose = frame_to_pose(tool.ATC_frame)
            try:
                scene.add_mesh(tool.name, EE_pose, tool.path)
                self.wait_update_object(tool.name, EE_is_attached=False, EE_is_known=True)
            except:
                print("Error adding tool to the scene")

        #Add attached EEFs to the initial ACM
        #Get initial ACM
        rospy.wait_for_service('/get_planning_scene')
        my_service = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        sceneReq = PlanningSceneComponents(components=128)
        sceneResponse = my_service(sceneReq)
        acm = sceneResponse.scene.allowed_collision_matrix

        rospy.wait_for_service('/apply_planning_scene')
        my_service = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        sceneReq = PlanningScene()

        #Detect index of tool changer links (wrists)
        i=0
        left_attach_link_index = -1
        right_attach_link_index = -1
        for link_name in acm.entry_names:
                if link_name == eef_link_left:
                        left_attach_link_index = i
                if link_name == eef_link_right:
                        right_attach_link_index = i        
                i+=1
        if left_attach_link_index == -1 or right_attach_link_index == -1:
                print("Error")

        #Add two new columns to the existig rows
        acm.entry_names.append(self.EEF_left) #-2 index
        acm.entry_names.append(self.EEF_right) #-1 index
        i=0
        for val in range(len(acm.entry_values)):
                acm.entry_values[i].enabled.append(False)
                acm.entry_values[i].enabled.append(False)
                i+=1
        #Collisions are allowed just with the tool changers (adjacent to the tools)
        acm.entry_values[left_attach_link_index].enabled[-2] = True #Left tool changer - Left EEF
        acm.entry_values[right_attach_link_index].enabled[-1] = True #Right tool changer - Right EEF

        #Add the values of the two new rows
        entry_value_left = AllowedCollisionEntry()
        entry_value_left.enabled = []
        entry_value_right = AllowedCollisionEntry()
        entry_value_right.enabled = []
        for link_name in acm.entry_names:
                entry_value_left.enabled.append(False)
                entry_value_right.enabled.append(False)
        #Collisions are allowed just with the tool changers (adjacent to the tools)
        
        entry_value_left.enabled[left_attach_link_index] = True
        entry_value_right.enabled[right_attach_link_index] = True
        acm.entry_values.append(entry_value_left)
        acm.entry_values.append(entry_value_right)

        #Update ACM
        sceneReq.allowed_collision_matrix = acm
        sceneReq.is_diff = True
        sceneResponse = my_service(sceneReq)

        self.ACM = acm

    def correctPose(self, target_pose, arm_side, rotate = False, ATC_sign = -1):
        """
        Corrects a target pose. Moveit plans the movement to the last link of the move_group, that is in the wrist. This function corrects the target pose so
        The action frame of the EEF is the one that moves to the desired target pose.
        - target_pose: Target pose for the EEF action frame [Pose]
        - arm_side: Arm side in which to change the tool ["left" or "right"]
        - rotate: True to rotate the pose 180 degrees in the EEF X axis. Useful to correct the Z axis direction of the EEF [bool]
        - ATC_sign: Determines the direction of the EEF base frame angle and distance difference with the arm wrist frame [1 or -1]
        """
        target_frame = pose_to_frame(target_pose)

        #Transform from target_pose frame to EEF action frame
        if rotate:
                target_frame.M.DoRotX(3.14) #Z axis pointing inside the tool

        #Transform from EEF action frame to EEF base frame
        if arm_side == "right":
                ATC_dist = self.right_ATC_dist
                ATC_angle = self.right_ATC_angle
                if self.EEF_right != "" and self.EEF_right != "None":
                        frame_world_EEF_base = target_frame * get_inverse_frame(self.EEF_dict[self.EEF_right].EE_end_frame)
                else:
                        frame_world_EEF_base = copy.deepcopy(target_frame)
        else:
                ATC_dist = self.left_ATC_dist
                ATC_angle = self.left_ATC_angle
                if self.EEF_left != "" and self.EEF_left != "None":
                        frame_world_EEF_base = target_frame * get_inverse_frame(self.EEF_dict[self.EEF_left].EE_end_frame)
                else:
                        frame_world_EEF_base = copy.deepcopy(target_frame)

        #Transform from EEF base frame to arm wrist
        frame_base_wrist = PyKDL.Frame()
        frame_base_wrist.p = PyKDL.Vector(0, 0, ATC_sign*ATC_dist) #Adds the offset from the tool changer to the wrist
        frame_world_wrist = frame_world_EEF_base * frame_base_wrist
        if rotate:
                frame_world_wrist.M.DoRotZ(ATC_sign*ATC_angle) #Z axis difference between the tool changer and the arm wrist
        else:
                frame_world_wrist.M.DoRotZ(-ATC_sign*ATC_angle)

        pose_world_wrist = frame_to_pose(frame_world_wrist)

        return pose_world_wrist

def frame_to_pose(frame):
    """
    Convert PyKDL.Frame into a Pose
    """
    pose_result = Pose()
    pose_result.position.x = frame.p[0] 
    pose_result.position.y = frame.p[1] 
    pose_result.position.z = frame.p[2] 
    ang = frame.M.GetQuaternion() 
    pose_result.orientation.x = ang[0] 
    pose_result.orientation.y = ang[1] 
    pose_result.orientation.z = ang[2] 
    pose_result.orientation.w = ang[3]
    return pose_result

def pose_to_frame(pose):
    """
    Converts a Pose into a PyKDL.Frame
    """
    frame_result = PyKDL.Frame() 
    frame_result.p = PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z) 
    frame_result.M = PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    return frame_result

def get_transpose_rot(rot_original):
    """
    Retrieves the transpose of a rotation matrix [PyKDL.Rotation]
    """
    rot_trans = PyKDL.Rotation(rot_original[0,0], rot_original[1,0], rot_original[2,0], rot_original[0,1], rot_original[1,1], rot_original[2,1], rot_original[0,2], rot_original[1,2], rot_original[2,2])
    return rot_trans

def get_inverse_frame(frame_original):
    """
    Retrieves the inverse of a frame [PyKDL.Frame]
    """
    frame_inv = PyKDL.Frame() 
    x = -(frame_original.p[0]*frame_original.M[0,0] + frame_original.p[1]*frame_original.M[1,0] + frame_original.p[2]*frame_original.M[2,0])
    y = -(frame_original.p[0]*frame_original.M[0,1] + frame_original.p[1]*frame_original.M[1,1] + frame_original.p[2]*frame_original.M[2,1])
    z = -(frame_original.p[0]*frame_original.M[0,2] + frame_original.p[1]*frame_original.M[1,2] + frame_original.p[2]*frame_original.M[2,2])
    frame_inv.p = PyKDL.Vector(x,y,z) 
    frame_inv.M = get_transpose_rot(frame_original.M)
    return frame_inv

def compute_distance(pose1, pose2):
    """
    Retrieves the linear distance [mm] between two poses.
    """
    dist = math.sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2 + (pose1.position.z - pose2.position.z)**2)
    return dist

def compute_angle_distance(pose1, pose2):
    """
    Retrieves the angular [rad] distance between two poses.
    """
    frame1 = pose_to_frame(pose1)
    frame2 = pose_to_frame(pose2)
    frame12 = frame1.Inverse() * frame2
    rot = abs(frame12.M.GetRotAngle()[0])
    return rot

def compute_lin_or_ang_distance(pose1, pose2, linear = True):
    """
    Retrieves the linear (linear = True) distance [mm] or angular (linear = False) [rad] distance between two poses.
    """
    if linear:
        return compute_distance(pose1, pose2)
    else:
        return compute_angle_distance(pose1, pose2)

def adjust_plan_speed(traj_poses, EE_speed_aux, v_change, traj_mov, max_accel, all_plans, linear = True):
    """
    Recalculate the times of an EEF trajectory to follow some target speeds.
    This function is just called internally by another function.
    - traj_poses: List of lists of EEF poses. Each sublist corresponds to the poses of a different target speed plan
    - EE_speed_aux: List of target speeds (mm/s or deg/s)
    - v_change: Dictionary with the distance needed to accelerate between succesive speed sections (mm/s or deg/s)
    - traj_mov: List of total distance/angle travelled in each speed section (mm or deg)
    - max_accel: Maximum EEF acceleration (mm/s^2 or rad/s^2)
    - all_plans: All the plans of the different speed sections without EEF speed control
    - linear: True for linear motion, False for angular motion
    """

    #Variables initialization
    thres = 0.05
    if not linear:
        thres *= (0.7*(math.pi/180))
    corrected_traj = []
    zero_Jvel = []
    for i in range(len(all_plans[0].joint_trajectory.points[0].positions)):
        zero_Jvel.append(0.0)
    corrected_traj.append({'pose': traj_poses[0][0], 'state': all_plans[0].joint_trajectory.points[0].positions, 'EE_speed': 0, 'EE_accel': 0, 'time': 0, 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})

    success = True
    i=0
    s_i = 1
    v_chg_i = 0
    init_speed_change = False
    final_speed_change = False
    for plan_poses in traj_poses:
            if traj_mov[i] < thres: #No movement so the plan is skipped
                j=-1
                for pose in plan_poses:
                    j+=1
                    if pose == corrected_traj[-1]['pose']:
                            continue
                    corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': 0, 'EE_accel': 0, 'time': corrected_traj[-1]['time'], 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})
                continue

            init_speed_change = EE_speed_aux[s_i] > EE_speed_aux[s_i-1] #Transition at the beginning if the previous velocity was lower
            final_speed_change = EE_speed_aux[s_i] > EE_speed_aux[s_i+1] #Transition at the end if the next velocity is higher
            j=-1
            x_plan = 0
            init_transition_indexes = []
            first_final = True
            for pose in plan_poses:
                    j+=1
                    if pose == corrected_traj[-1]['pose']: #Repeated point, skipped
                            continue
                    
                    if compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'], linear) < thres and not init_speed_change:
                            corrected_traj.append({'pose': corrected_traj[-1]['pose'], 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': 0, 'EE_accel': 0, 'time': corrected_traj[-1]['time'], 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})
                            continue
                    
                    #Acceleration/decelarion at the beginning of the current speed section
                    if init_speed_change:
                            if final_speed_change:
                                if (v_change[v_chg_i]['x_min_req'] + v_change[v_chg_i+1]['x_min_req']) > traj_mov[i]: #It is not possible to reach the target speed and decelerate on time
                                    t2= (-2*EE_speed_aux[s_i+1] + math.sqrt(2*(EE_speed_aux[s_i+1]**2 + EE_speed_aux[s_i-1]**2 + 2*max_accel*traj_mov[i])))/(2*max_accel)
                                    v_change[v_chg_i+1]['x_min_req'] = (EE_speed_aux[s_i+1] + max_accel*t2)*t2 - (0.5 * max_accel * t2**2)
                                    v_change[v_chg_i]['x_min_req'] = traj_mov[i] - v_change[v_chg_i+1]['x_min_req']
                            x_plan += compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'], linear) #Distance travelled until the moment
                            #Adds the new configuration with empty speed, accel and time, which will be calculated later
                            corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': 0, 'EE_accel': 0, 'time': 0, 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})
                            init_transition_indexes.append(len(corrected_traj)-1)

                            #When the distance travelled until the moment is higher than the required distance for the initial acceleration, the speed, acceleration and time for all these points is calculated
                            if (x_plan + compute_lin_or_ang_distance(pose, plan_poses[min(j+1,len(plan_poses))], linear)) > v_change[v_chg_i]['x_min_req'] or (traj_mov[i] < v_change[v_chg_i]['x_min_req'] and j >= (len(plan_poses)-1)): #Determine the real discrete point where the speed change is completed
                                    speed_diff = EE_speed_aux[s_i] - EE_speed_aux[s_i-1] #Speed difference between consecutive target speed sections
                                    trans_accel = min((2*speed_diff*EE_speed_aux[s_i-1] + speed_diff**2)/(2*x_plan),max_accel) #Acceleration is constant as we are considering an u.a.r.m.
                                    for index in init_transition_indexes:
                                            #Apply the equations of a trapezoidal speed profile to calculate the times during the acceleration period
                                            corrected_traj[index]['EE_accel'] = trans_accel
                                            #Calculate the time difference between consecutive points
                                            t_A = trans_accel/2
                                            t_B = corrected_traj[index-1]['EE_speed']
                                            t_C = -compute_lin_or_ang_distance(corrected_traj[index]['pose'], corrected_traj[index-1]['pose'], linear)
                                            new_time_1 = (-t_B + math.sqrt(t_B**2 - 4*t_A*t_C))/(2*t_A)
                                            new_time_2 = (-t_B - math.sqrt(t_B**2 - 4*t_A*t_C))/(2*t_A)
                                            if new_time_1 < 0 and new_time_2 < 0:
                                                new_time = 0
                                                success = False
                                            elif new_time_1 < 0:
                                                new_time = new_time_2
                                            elif new_time_2 < 0:
                                                new_time = new_time_1
                                            elif new_time_1 < new_time_2:
                                                new_time = new_time_1
                                            else:
                                                new_time = new_time_2
                                            corrected_traj[index]['time'] = corrected_traj[index-1]['time'] + new_time
                                            corrected_traj[index]['EE_speed'] = corrected_traj[index-1]['EE_speed'] + trans_accel*new_time #Calculate the EEF speed reached in the next point
                                    corrected_traj[-1]['EE_accel'] = 0 #Finish the movement with accel 0, velocity will be constant after the transition
                                    corrected_traj[init_transition_indexes[0]-1]['EE_accel'] = trans_accel #Start the movement accelerating
                                    v_chg_i += 1
                                    init_speed_change = False

                    #Acceleration/decelarion at the end of the current speed section
                    elif final_speed_change:
                            x_plan += compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'], linear)
                            x_left = traj_mov[i] - x_plan #Remaining distance to travel before the end of the current speed section
                            #When the remaining distance is lower than the minimum distance required for the final acceleration/deceleration, it must start from the prev point
                            if x_left < v_change[v_chg_i]['x_min_req']: 
                                    if first_final:
                                            x_trans = x_left + compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'], linear) #The prev x
                                            speed_diff = EE_speed_aux[s_i+1] - corrected_traj[-1]['EE_speed']
                                            trans_accel_1 = (2*speed_diff*EE_speed_aux[s_i] + speed_diff**2)/(2*x_trans)
                                            trans_accel_2 = copy.deepcopy(-max_accel)
                                            if abs(trans_accel_1)>abs(trans_accel_2):
                                                trans_accel = trans_accel_2
                                            else:
                                                trans_accel = trans_accel_1
                                            first_final = False
                                            corrected_traj[-1]['EE_accel'] = trans_accel

                                    if speed_diff == 0:
                                        corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': 0, 'EE_accel': 0, 'time': corrected_traj[-1]['time'], 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})
                                        continue

                                    #Apply the equations of a trapezoidal speed profile to calculate the times during the acceleration/deceleration period
                                    t_A = trans_accel/2
                                    t_B = corrected_traj[-1]['EE_speed']
                                    t_C = -compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'], linear)
                                    t_shorter = False
                                    
                                    if (t_B**2 - 4*t_A*t_C) <= 0 or t_shorter:
                                        new_time = (-t_B)/(2*t_A)
                                    else:
                                        new_time_1 = (-t_B + math.sqrt(t_B**2 - 4*t_A*t_C))/(2*t_A)
                                        new_time_2 = (-t_B - math.sqrt(t_B**2 - 4*t_A*t_C))/(2*t_A)
                                        if new_time_1 < 0 and new_time_2 < 0:
                                            new_time = 0
                                            success = False
                                        elif new_time_1 < 0:
                                            new_time = new_time_2
                                        elif new_time_2 < 0:
                                            new_time = new_time_1
                                        elif new_time_1 < new_time_2:
                                            new_time = new_time_1
                                        else:
                                            new_time = new_time_2
                                        if new_time > corrected_traj[-1]['time'] - corrected_traj[-2]['time']:
                                            pass
                                        else:
                                            t_shorter = False

                                    new_total_time = corrected_traj[-1]['time'] + new_time
                                    new_speed = corrected_traj[-1]['EE_speed'] + trans_accel*new_time

                                    corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': new_speed, 'EE_accel': trans_accel, 'time': new_total_time, 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})
                                    if (linear and x_left < 0.1) or (not linear and x_left < 0.07*(math.pi/180)): #Last point
                                            corrected_traj[-1]['EE_accel'] = 0    
                                            v_chg_i += 1
                                            final_speed_change = False

                            #When there is still time before the final acceleration/deceleration the speed is kept constant at the target speed and the acceleration is 0                
                            else:
                                    corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': EE_speed_aux[s_i], 'EE_accel': 0, 'time': corrected_traj[-1]['time'] + compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'],linear)/EE_speed_aux[s_i], 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)}) 

                    #If there are no target speed changes between sections, or the next target speed is higher than the current, the speed is kept constant until the end of the plan
                    else:
                            corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': EE_speed_aux[s_i], 'EE_accel': 0, 'time': corrected_traj[-1]['time'] + compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'],linear)/EE_speed_aux[s_i], 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})   

            i+=1
            s_i += 1
    return corrected_traj, success

def master_slave_constant_distance(waypoints_gripper, ATC1, speed = 30, arm = "left"):
    """
    Funtion to generate dual-arm master-slave plans where the slave arm motion are calculated to keep the distance between the EEFs constant
    - waypoints_gripper: List of waypoints of the master arm [poses]
    - speed: EEF speed of the master arm [mm/s]
    - arm: Master arm side ["left" or "right"]
    """
    left_wrist_pose = arm_left.get_current_pose().pose
    right_wrist_pose = arm_right.get_current_pose().pose
    left_wrist_frame = pose_to_frame(left_wrist_pose)
    right_wrist_frame = pose_to_frame(right_wrist_pose)

    #Transform initial poses from the wrist to the EEF frame
    aux_left_wrist2_pose = ATC1.correctPose(left_wrist_pose, "left")
    aux_left_wrist2_frame = pose_to_frame(aux_left_wrist2_pose)
    transform_left_wrist_EEF = get_inverse_frame(aux_left_wrist2_frame) * left_wrist_frame
    left_EEF_frame = left_wrist_frame * transform_left_wrist_EEF
    
    aux_right_wrist2_pose = ATC1.correctPose(right_wrist_pose, "right")
    aux_right_wrist2_frame = pose_to_frame(aux_right_wrist2_pose)
    transform_right_wrist_EEF = get_inverse_frame(aux_right_wrist2_frame) * right_wrist_frame
    right_EEF_frame = right_wrist_frame * transform_right_wrist_EEF
    
    #Get the transformation matrix between EEFs, that will be kept constant
    arms_diff_frame = PyKDL.Frame()
    arms_diff_frame = get_inverse_frame(left_EEF_frame) * right_EEF_frame #Left to Right frame

    if arm == "right":
        arms_diff_frame = get_inverse_frame(arms_diff_frame) #Right to Left frame
        
    #Calculate the waypoints of the slave arm keeping the initial transformation matrix between the EEFs constant
    sec_arm_wp_gripper = []
    for wp_g in waypoints_gripper:
        if arm == "left":
            wp_frame = pose_to_frame(wp_g) * transform_left_wrist_EEF
            sec_arm = "right"
        elif arm == "right":
            wp_frame = pose_to_frame(wp_g) * transform_right_wrist_EEF
            sec_arm = "left"
        else:
            return [], False
        sec_wp = frame_to_pose(wp_frame * arms_diff_frame)
        sec_arm_wp_gripper.append(ATC1.correctPose(sec_wp, sec_arm))
        

    #Generate the dual-arm plan      
    if arm == "left":
        plan, success = dual_arm_cartesian_plan([waypoints_gripper], [speed], [sec_arm_wp_gripper], [speed], ATC1= ATC1, sync_policy=1)
    elif arm == "right":
        plan, success = dual_arm_cartesian_plan([sec_arm_wp_gripper], [speed], [waypoints_gripper], [speed], ATC1= ATC1, sync_policy=1)
    else:
        plan = []
        success = False


    return plan, success
    
def merge_plans(planL, planR):
    """
    Merge a left arm plan (planL) and a right arm plan (planR) into a single dual-arm plan, modifying their speed so 
    both arms finish their motion at the same time
    """
    
    #speed_limit = 1
    length_L = len(planL.joint_trajectory.points)
    duration_L = float(planL.joint_trajectory.points[-1].time_from_start.secs) + 0.000000001 * float(planL.joint_trajectory.points[-1].time_from_start.nsecs)
    length_R = len(planR.joint_trajectory.points)
    duration_R = float(planR.joint_trajectory.points[-1].time_from_start.secs) + 0.000000001 * float(planR.joint_trajectory.points[-1].time_from_start.nsecs)
    
    #Intialize variables
    joints_long = []
    vel_long = []        
    accel_long = []
    times_long = []
    joints_short = []
    vel_short = []        
    accel_short = []
    times_short = []
    joints_L = []
    vel_L = []        
    accel_L = []
    joints_R = []
    vel_R = []        
    accel_R = []
    times = []

    add_points = False
    if length_L > length_R:
        plan_long = planL
        plan_short = planR
        length_long = length_L
        length_short = length_R
        duration_long = duration_L
        duration_short = duration_R
        long_plan_name = "L"
        add_points = True
    elif length_L < length_R:
        plan_long = planR
        plan_short = planL
        length_long = length_R
        length_short = length_L
        duration_long = duration_R
        duration_short = duration_L
        long_plan_name = "R"
        add_points = True
    else: #Both plans have the same number of points, so it is no necessary to add new points (add_points = False)
        plan_long = planR
        plan_short = planL
        duration_long = duration_R
        duration_short = duration_L
        long_plan_name = "R"                

    #Extract the values of each plan
    for point in plan_long.joint_trajectory.points:
        joints_long.append(point.positions)
        vel_long.append(point.velocities)
        accel_long.append(point.accelerations)
        times_long.append(float(point.time_from_start.secs) + 0.000000001 * float(point.time_from_start.nsecs))

    for point in plan_short.joint_trajectory.points:
        joints_short.append(point.positions)
        vel_short.append(point.velocities)
        accel_short.append(point.accelerations)
        times_short.append(float(point.time_from_start.secs) + 0.000000001 * float(point.time_from_start.nsecs))        

    time_interpolation = False
    if duration_long >= duration_short:
        times = times_long[:]
    else:
        times = times_short[:]
        time_interpolation = True
    
    n_low_sep = 0
    n_high_sep = 0
    lower_sep = 0
    higher_sep = 0
    lower_amo = 0
    higher_amo = 0
    n_high_amo = 0
    n_low_amo = 0
    indexes = []

    #If one plan is larger than the other, add new points equally distributed to the shorter plan by linear interpolation.
    #The times used for merging the plans are the ones of the slowest plan.
    if add_points:

        if length_short >= ((length_long/2) - 1):
            separation = length_short/(length_long - length_short + 1)
            amount = 1
            lower_sep = int(separation)
            higher_sep = lower_sep + 1
            n_low_sep = int(((length_long - length_short + 1) * (separation - float(higher_sep)))/(float(lower_sep) - float(higher_sep))-1)
            n_high_sep = int(length_long - length_short - n_low_sep)

            indexes = []
            for i in range(n_high_sep):
                indexes.append(higher_sep + (i * (higher_sep + 1)))
            if len(indexes) != 0:
                start_index = indexes[-1]
            else:
                start_index = -1
            for i in range(n_low_sep):
                indexes.append(start_index + ((i + 1) * (lower_sep + 1)))                

            for index in indexes:
                new_pos_zip = zip(joints_short[index-1],joints_short[index])
                new_pos = [(x + y)/2 for (x, y) in new_pos_zip]
                joints_short.insert(index, new_pos)
                new_vel_zip = zip(vel_short[index-1],vel_short[index])
                new_vel = [(x + y)/2 for (x, y) in new_vel_zip]
                vel_short.insert(index, new_vel)
                new_accel_zip = zip(accel_short[index-1],accel_short[index])
                new_accel = [(x + y)/2 for (x, y) in new_accel_zip]
                accel_short.insert(index, new_accel)
                if time_interpolation:
                    new_time = (times[index-1] + times[index])/2
                    times.insert(index, new_time)
                            
        else:
            separation = 1
            amount = (length_long - length_short)/(length_short - 1)
            lower_amo = int(amount)
            higher_amo = lower_amo + 1
            n_high_amo = int((length_long + lower_amo - length_short * (1 + lower_amo))/(higher_amo - lower_amo))
            n_low_amo = (length_short - 1 - n_high_amo)

            indexes = []
            new_pos = []
            new_vel = []
            new_accel = []
            new_time = []
            for i in range(n_high_amo):
                new_pos_zip = zip(joints_short[i],joints_short[i+1])
                new_vel_zip = zip(vel_short[i],vel_short[i+1])
                new_accel_zip = zip(accel_short[i],accel_short[i+1])
                for j in range(higher_amo):
                    indexes.append(((higher_amo + 1) * i) + j + 1)
                    new_pos.append([((y - x)*(j+1)/(higher_amo+1))+x for (x, y) in new_pos_zip])
                    new_vel.append([((y - x)*(j+1)/(higher_amo+1))+x for (x, y) in new_vel_zip])
                    new_accel.append([((y - x)*(j+1)/(higher_amo+1))+x for (x, y) in new_accel_zip])
                    if time_interpolation:
                        new_time.append(((times_short[i+1] - times_short[i])*(j+1)/(higher_amo+1))+times_short[i])

            try:
                start_index = indexes[-1]
            except:
                start_index = -1
            for i in range(n_low_amo):
                new_pos_zip = zip(joints_short[i+n_high_amo],joints_short[i+1+n_high_amo])
                new_vel_zip = zip(vel_short[i+n_high_amo],vel_short[i+1+n_high_amo])
                new_accel_zip = zip(accel_short[i+n_high_amo],accel_short[i+1+n_high_amo])
                for j in range(lower_amo):
                    indexes.append((start_index + 1) + ((lower_amo + 1) * i) + j + 1)
                    new_pos.append([((y - x)*(j+1)/(lower_amo+1))+x for (x, y) in new_pos_zip])
                    new_vel.append([((y - x)*(j+1)/(lower_amo+1))+x for (x, y) in new_vel_zip])
                    new_accel.append([((y - x)*(j+1)/(lower_amo+1))+x for (x, y) in new_accel_zip])
                    if time_interpolation:
                        new_time.append(((times_short[i+1+n_high_amo] - times_short[i+n_high_amo])*(j+1)/(lower_amo+1))+times_short[i+n_high_amo])

            i = 0
            for index in indexes:
                joints_short.insert(index, new_pos[i])
                vel_short.insert(index, new_vel[i])
                accel_short.insert(index, new_accel[i])
                if time_interpolation:
                    times.insert(index, new_time[i])
                i += 1

    if long_plan_name == "L":
        joints_L = joints_long
        vel_L = vel_long        
        accel_L = accel_long
        joints_R = joints_short
        vel_R = vel_short        
        accel_R = accel_short
    else:
        joints_R = joints_long
        vel_R = vel_long        
        accel_R = accel_long
        joints_L = joints_short
        vel_L = vel_short        
        accel_L = accel_short

    #After adding all the points, both plans have the same length and they are ready to be merged in a single plan for the joints of both arms
    merged_plan = RobotTrajectory()
    merged_plan.joint_trajectory.header.frame_id = copy.copy(planL.joint_trajectory.header.frame_id)
    merged_plan.joint_trajectory.joint_names = copy.deepcopy(planL.joint_trajectory.joint_names) + copy.deepcopy(planR.joint_trajectory.joint_names)

    #times = [element/speed_limit for element in times] #*4

    for index in range(len(times)):
        point = JointTrajectoryPoint()
        point.positions = tuple(joints_L[index]) + tuple(joints_R[index])
        point.velocities = tuple(vel_L[index]) + tuple(vel_R[index])
        point.accelerations = tuple(accel_L[index]) + tuple(accel_R[index])
        point.effort = []
        point.time_from_start.secs = int(times[index])
        point.time_from_start.nsecs = int((times[index] - int(times[index]))*1000000000)
        merged_plan.joint_trajectory.points.append(point)

    return merged_plan

def fill_times_arm(values_1, times_1, times_2):
    """
    values_1 are the plan values of an arm at times_1. This function fills the plan with the values at times_2 too.
    This function is just called internally by the merge_plans_keep_speed() function
    """
    values_1_original = copy.deepcopy(values_1)
    for t in times_2:  
            if t not in times_1:
                    if t < max(times_1):
                            time_index = [n for n,i in enumerate(times_1) if i>t ][0]
                            prev_time = times_1[time_index-1]
                            next_time = times_1[time_index]
                            new_J = []
                            new_V = []
                            new_A = []
                            #Uses the trapezoidal profile equations to calculate the values at times_2, from the values in the previous and next times of times_1
                            for i in range(len(values_1_original[prev_time]['J'])):
                                    new_V_i = values_1_original[prev_time]['V'][i] + values_1_original[prev_time]['A'][i] * (t-prev_time)
                                    new_J_i = values_1_original[prev_time]['J'][i] + values_1_original[prev_time]['V'][i] * (t-prev_time) + 0.5*values_1_original[prev_time]['A'][i] * ((t-prev_time)**2)
                                    new_A_i = (2*(values_1_original[next_time]['J'][i] - new_J_i - new_V_i*(next_time-t)))/((next_time-t)**2)
                                    new_J.append(new_J_i)
                                    new_V.append(new_V_i)
                                    new_A.append(new_A_i)
                            values_1[t] = {'J': new_J, 'V': new_V, 'A': new_A}
                    else:
                            values_1[t] = copy.deepcopy(values_1_original[max(times_1)])
                            values_1[t]['V'] = [0]*7
                            values_1[t]['A'] = [0]*7
            sorted_keys = sorted(values_1.keys())
            values_1_sorted = {key:values_1[key] for key in sorted_keys}
    return values_1_sorted

def merge_plans_keep_speed(planL, planR):
    """
    Merge a left arm plan (planL) and a right arm plan (planR) into a single dual-arm plan keeping the speed of each individual arm
    """
    times_L = []
    times_R = []
    values_L = {}
    values_R = {}

    #Extract information from plans
    for point in planL.joint_trajectory.points:
            new_time_L = float(point.time_from_start.secs) + 0.000000001 * float(point.time_from_start.nsecs)
            times_L.append(new_time_L)
            values_L[new_time_L] = {'J': point.positions, 'V': point.velocities, 'A': point.accelerations}
    
    for point in planR.joint_trajectory.points:
            new_time_R = float(point.time_from_start.secs) + 0.000000001 * float(point.time_from_start.nsecs)
            times_R.append(new_time_R)
            values_R[new_time_R] = {'J': point.positions, 'V': point.velocities, 'A': point.accelerations}

    times_both = times_L + times_R
    times_both = list(dict.fromkeys(times_both)) #Remove repeated times
    times_both.sort()

    #For each arm plan, it creates new points for the times of the other arm plan, so it is possible to merge them later
    values_L = fill_times_arm(values_L, times_L, times_R)
    values_R = fill_times_arm(values_R, times_R, times_L)

    #After adding all the points, both plans have the same length and they are ready to be merged in a single plan for the joints of both arms
    merged_plan = RobotTrajectory()
    merged_plan.joint_trajectory.header.frame_id = copy.copy(planL.joint_trajectory.header.frame_id)
    merged_plan.joint_trajectory.joint_names = copy.deepcopy(planL.joint_trajectory.joint_names) + copy.deepcopy(planR.joint_trajectory.joint_names)
    
    for t in times_both:
            point = JointTrajectoryPoint()
            point.positions = tuple(values_L[t]['J']) + tuple(values_R[t]['J'])
            point.velocities = tuple(values_L[t]['V']) + tuple(values_R[t]['V'])
            point.accelerations = tuple(values_L[t]['A']) + tuple(values_R[t]['A'])
            point.effort = []
            point.time_from_start.secs = int(t)
            point.time_from_start.nsecs = int((t - int(t))*1000000000)
            merged_plan.joint_trajectory.points.append(point)

    return merged_plan
    
def compute_cartesian_path_velocity_control(waypoints_list, EE_speed, EE_ang_speed = [], arm_side = "left", max_linear_accel = 200.0, max_ang_accel = 140.0, extra_info = False, step = 0.002):
    """
    Function that generates a single-arm motion plan with control on the EEF speed, ready to be executed.
    - waypoints_list: List of lists of waypoints [Poses]. Every sublist has a different EEF target speed associated
    - EE_speed: The list of EEF target linear speeds for each speed section of the trajectory [mm/s]
    - EE_ang_speed: The List of EEF target angular speeds for each speed section of the trajectory [deg/s]
    - max_linear_accel: Maximum linear acceleration [mm/s^2]
    - max_angular_accel: Maximum linear acceleration [deg/s^2]
    - extra_info: Used for the sync policy 3 dual-arm function. Leave it as False otherwise
    - Step: Step used for the compute_cartesian_path function [mm]
    """
    
    success = True
    #Selects the arm to be planned
    if arm_side == "left":
            arm = arm_left
            fkln = ['arm1_tool0'] #Modify with the name of the final link of the left arm
    else:
            arm = arm_right
            fkln = ['arm2_tool0'] #Modify with the name of the final link of the right arm

    if len(EE_ang_speed)==0: #If the angular speed limit is not specified, it considers a limit of 1 mm/s --> 0.7 deg/s
            for s in EE_speed:
                    EE_ang_speed.append(s*0.7)

    #Convert to rads
    for i in range(len(EE_speed)):
        EE_ang_speed[i]*=(math.pi/180)

    max_ang_accel *= (math.pi/180)

    #Define the speed profile accelerations
    EE_speed_aux = copy.deepcopy(EE_speed)
    EE_speed_aux.insert(0,0)
    EE_speed_aux.append(0)
    EE_ang_speed_aux = copy.deepcopy(EE_ang_speed)
    EE_ang_speed_aux.insert(0,0)
    EE_ang_speed_aux.append(0)
    v_change = []
    v_change_ang = []

    #Calculates the minimum distance required to accelerate and/or decelerate the EEF between the different speed sections
    for i in range(len(EE_speed_aux)-1):
            max_linear_accel_sign = copy.deepcopy(max_linear_accel)
            if EE_speed_aux[i]>EE_speed_aux[i+1]:
                max_linear_accel_sign *= -1
            max_ang_accel_sign = copy.deepcopy(max_ang_accel)
            if EE_ang_speed_aux[i]>EE_ang_speed_aux[i+1]:
                max_ang_accel_sign *= -1
            
            t_req_lin = (EE_speed_aux[i+1]-EE_speed_aux[i])/max_linear_accel_sign
            t_req_ang = (EE_ang_speed_aux[i+1]-EE_ang_speed_aux[i])/max_ang_accel_sign
            change = {'t_req': t_req_lin, 'x_min_req': EE_speed_aux[i]*t_req_lin + (max_linear_accel_sign*(t_req_lin**2))/2}
            change_ang = {'t_req': t_req_ang, 'x_min_req': EE_ang_speed_aux[i]*t_req_ang + (max_ang_accel_sign*(t_req_ang**2))/2}

            v_change.append(change)
            v_change_ang.append(change_ang)

    #Plan the trajectory of every speed section using the compute_cartesian_path function or a custom IK trajectory solver.
    #This plan don't have any control over the speed of the end effector
    all_plans = []
    for traj in waypoints_list:
            (plan, fraction) = arm.compute_cartesian_path(traj, step, 0.0)
            all_plans.append(plan)
            rs = RobotState()
            for j_name in plan.joint_trajectory.joint_names:
                    rs.joint_state.name.append(j_name)
            for state in plan.joint_trajectory.points[-1].positions:
                    rs.joint_state.position.append(state)        
            arm.set_start_state(rs)
    arm.set_start_state_to_current_state()

    #Get all the EEF poses of the generated plan using a forward kinematics solver
    traj_poses = []
    traj_mov_position = []
    traj_mov_angle = []
    rospy.wait_for_service('compute_fk')
    fk_srv = rospy.ServiceProxy('compute_fk', GetPositionFK)
    rs = RobotState()
    for j_name in plan.joint_trajectory.joint_names:
            rs.joint_state.name.append(j_name)
    for plan in all_plans:
            plan_poses = []
            traj_mov_i_position = 0
            traj_mov_i_angle = 0
            for joint_state in plan.joint_trajectory.points:
                    rs.joint_state.position = []
                    for joint in joint_state.positions:
                            rs.joint_state.position.append(joint)
                    header = Header(0,rospy.Time.now(),"table_base_link") #Modify with the frame_id of the robot
                    header.frame_id = plan.joint_trajectory.header.frame_id
                    plan_pose_meters = fk_srv(header, fkln, rs).pose_stamped[0].pose
                    plan_pose_mm = copy.deepcopy(plan_pose_meters)
                    plan_pose_mm.position.x *= 1000
                    plan_pose_mm.position.y *= 1000
                    plan_pose_mm.position.z *= 1000
                    plan_poses.append(plan_pose_mm)
                    if len(plan_poses) > 1:
                            traj_mov_i_position += compute_distance(plan_poses[-2], plan_poses[-1])
                            traj_mov_i_angle += compute_angle_distance(plan_poses[-2], plan_poses[-1])
            traj_poses.append(plan_poses)
            traj_mov_position.append(traj_mov_i_position) #Total travelled distance in each speed section
            traj_mov_angle.append(traj_mov_i_angle) #Total travelled angle in each speed section

    #Get max joint velocities from the URDF robot description
    robot_desc = rospy.get_param('robot_description')
    root = ET.fromstring(robot_desc)
            
    vel_limit = {}
    for child in root: 
            if child.tag == "joint":
                    j_name = child.get("name")
                    if child.get("type") == "revolute":
                            for joint_attrib in child:
                                    if joint_attrib.tag == "limit":
                                            vel_limit[j_name] = float(joint_attrib.get("velocity"))*0.9 #The limit is set at a 90% of the URDF limit

    #Recalculate all the times of the trajectory according to the specified target speed profiles. 
    #The calculation is done both for the linear and the angular movements
    corrected_traj, success_lin = adjust_plan_speed(traj_poses, EE_speed_aux, v_change, traj_mov_position, max_linear_accel, all_plans, linear = True)
    corrected_traj_ang, success_ang = adjust_plan_speed(traj_poses, EE_ang_speed_aux, v_change_ang, traj_mov_angle, max_ang_accel, all_plans, linear = False)
    if not success_lin or not success_ang:
        success = False

    #Merges the linear and angular constraint plans by selecting the larger times, to not exceed any of the limits.
    first_accel = True
    full_corrected_traj = copy.deepcopy(corrected_traj)
    for i in range(len(corrected_traj)-1):
        if (corrected_traj_ang[i+1]['time'] - corrected_traj_ang[i]['time']) > (corrected_traj[i+1]['time'] - corrected_traj[i]['time']):
            full_corrected_traj[i+1]['time'] = full_corrected_traj[i]['time'] + (corrected_traj_ang[i+1]['time'] - corrected_traj_ang[i]['time'])
        else:
            full_corrected_traj[i+1]['time'] = full_corrected_traj[i]['time'] + (corrected_traj[i+1]['time'] - corrected_traj[i]['time'])
        if extra_info:
            if corrected_traj[i+1]['EE_accel'] == 0 and corrected_traj_ang[i+1]['EE_accel'] == 0 and first_accel:
                    first_accel = False
                    t_accel = full_corrected_traj[i+1]['time']
            if corrected_traj[i+1]['EE_accel'] == 0 and corrected_traj_ang[i+1]['EE_accel'] == 0 and ((i+1)<(len(corrected_traj)-1)):
                    t_dec = full_corrected_traj[i+1]['time']

    zero_Jvel = []
    for i in range(len(all_plans[0].joint_trajectory.points[0].positions)):
            zero_Jvel.append(0.0)

    #Recalculate the times of the trajectory when any of the joint velocity limits are exceeded (> 90% of limit)
    update_time = False
    full_corrected_traj_with_limits = copy.deepcopy(full_corrected_traj)
    for i in range(len(full_corrected_traj)-1):
            time_diff = full_corrected_traj[i+1]['time']-full_corrected_traj[i]['time']
            updated_new_times = []
            for j in range(len(full_corrected_traj[i]['state'])):
                    if time_diff != 0:
                        angle_diff = full_corrected_traj[i+1]['state'][j]-full_corrected_traj[i]['state'][j]
                        new_Jaccel = (angle_diff-(full_corrected_traj_with_limits[i]['Jspeed'][j]*time_diff))*(2/(time_diff**2))
                        new_Jspeed = full_corrected_traj_with_limits[i]['Jspeed'][j] + (new_Jaccel*time_diff)
                        new_time = full_corrected_traj_with_limits[i]['time'] + time_diff

                    joint_name = rs.joint_state.name[j]
                    if vel_limit[joint_name] < new_Jspeed or time_diff == 0:
                        new_Jspeed = vel_limit[joint_name]
                        new_Jaccel = ((2*full_corrected_traj_with_limits[i]['Jspeed'][j]*(new_Jspeed - full_corrected_traj_with_limits[i]['Jspeed'][j])) + (new_Jspeed - full_corrected_traj_with_limits[i]['Jspeed'][j])**2)/(2*angle_diff)
                        if abs(new_Jaccel) < 0.0001:
                            new_time_step = (angle_diff)/new_Jspeed
                        else:
                            new_time_step = (new_Jspeed - full_corrected_traj_with_limits[i]['Jspeed'][j])/new_Jaccel
                        updated_new_times.append(new_time_step)
                        update_time = True

                    full_corrected_traj_with_limits[i+1]['time'] = new_time
                    full_corrected_traj_with_limits[i+1]['Jaccel'][j] = new_Jaccel
                    full_corrected_traj_with_limits[i+1]['Jspeed'][j] = new_Jspeed

            if update_time: #If any of the limits is exceeded, recalculate times, joint velocities and accelerations
                print("Joint limits speed exceeded")
                update_time = False
                new_time_max = max(updated_new_times)
                time_diff = new_time_max
                full_corrected_traj_with_limits[i+1]['time'] = full_corrected_traj_with_limits[i]['time'] + time_diff
                for j in range(len(full_corrected_traj[i]['state'])):
                    angle_diff = full_corrected_traj[i+1]['state'][j]-full_corrected_traj[i]['state'][j]
                    new_Jaccel = (angle_diff-(full_corrected_traj_with_limits[i]['Jspeed'][j]*time_diff))*(2/(time_diff**2))
                    new_Jspeed = full_corrected_traj_with_limits[i]['Jspeed'][j] + (new_Jaccel*time_diff)
                    full_corrected_traj_with_limits[i+1]['Jaccel'][j] = new_Jaccel
                    full_corrected_traj_with_limits[i+1]['Jspeed'][j] = new_Jspeed

            if extra_info and t_accel == full_corrected_traj[i+1]['time']:
                t_accel = full_corrected_traj_with_limits[i+1]['time']
            if extra_info and t_dec == full_corrected_traj[i+1]['time']:
                t_dec = full_corrected_traj_with_limits[i+1]['time']

    full_corrected_traj_with_limits[-1]['Jspeed'] = copy.deepcopy(zero_Jvel)

    #Uncomment if we want to visualize all the information of the generated plan
    """
    i=0
    for traj_point in full_corrected_traj_with_limits:
            print(i)
            print("Position: " + str(traj_point['pose'].position.x) + ", " + str(traj_point['pose'].position.y) + ", " + str(traj_point['pose'].position.z))
            print("Velocity: " + str(traj_point['EE_speed']))
            print("Acceleration: " + str (traj_point['EE_accel']))
            print("Time: " + str (traj_point['time']))
            print("Joint angles: " + str (traj_point['state']))
            print("Joint velocities: " + str (traj_point['Jspeed']))
            print("Joint accelerations: " + str (traj_point['Jaccel']))
            print("----------------------")
            i+=1
    """

    #Adjust the generated plan to the RobotTrajectory() msg structure
    new_plan = RobotTrajectory()
    new_plan.joint_trajectory.header.frame_id = "base_link"
    new_plan.joint_trajectory.joint_names = copy.deepcopy(rs.joint_state.name)
    for state in full_corrected_traj_with_limits:
            point = JointTrajectoryPoint()
            point.positions = copy.deepcopy(state['state'])
            point.velocities = copy.deepcopy(state['Jspeed'])
            point.accelerations = copy.deepcopy(state['Jaccel'])
            point.effort = []
            point.time_from_start.secs = int(copy.deepcopy(state['time']))
            point.time_from_start.nsecs = int((copy.deepcopy(state['time']) - int(copy.deepcopy(state['time'])))*1000000000)
            new_plan.joint_trajectory.points.append(point)

    if extra_info:
            return new_plan, success, t_accel, t_dec
    else:
            return new_plan, success
    
def dual_arm_cartesian_plan(waypoints_left, speed_left, waypoints_right, speed_right, ATC1, sync_policy = 1, extra_info=False, rs=RobotState(), rs_l=RobotState(), rs_r=RobotState()):
    """
    This function generates a dual-arm motion plan ready to be executed. Three different synchronization policies can be used:
    1. Both arms finish their motion at the same time. The maximum EEF speed of each arm can be set.
    2. Each arm moves at its own speed.
    3. Both arms reach a sequence of intermediate dual-arm waypoints at the same time. A maximum EEF speed can be set.

    - waypoints_left: List of lists of waypoints [Poses] for the left EEF. Each sublist corresponds to a different target speed. In policy 3 there must be just one sublist.
    - speed_left: List of EEF speeds for each section of the left arm trajectory [mm/s]
    - waypoints_right: List of lists of waypoints [Poses] for the right EEF. Each sublist corresponds to a different target speed. In policy 3 there must be just one sublist.
    - speed_right: List of EEF speeds for each section of the right arm trajectory [mm/s]
    - sync_policy: Synchronization policy used [1, 2 or 3, explained above]
    - extra_info: Leave it as empty. Used for recursive calls of this function
    - rs: robot state. Leave it as empty. Used for recursive calls of this function
    - rs_l: left arm state. Leave it as empty. Used for recursive calls of this function
    - rs_r: right arm state. Leave it as empty. Used for recursive calls of this function
    - ATC1: ATC object of the simulation
    """
    
    #Clear previous targets
    arm_left.clear_pose_targets()
    arm_right.clear_pose_targets()
    arms.clear_pose_targets()

    #Managed by another function
    # if sync_policy == 3:
    #         if (len(waypoints_left[0]) != len(waypoints_right[0])) or (len(speed_left) != len(speed_right)):
    #                 plan = RobotTrajectory()
    #                 return plan, False
    #         else:
    #                 speed_3 = min([speed_left[0], speed_right[0]])
    #                 waypoints_3 = []
    #                 for i in range(len(waypoints_left[0])):
    #                         wp_i = [waypoints_left[0][i],waypoints_right[0][i]]
    #                         waypoints_3.append(wp_i)
    #                 plan, success = dual_arm_intermediate_waypoints_plan(waypoints_3, speed_3, ATC1=ATC1)
    #                 return plan, success

    #Get the allowed collision matrix (ACM)
    rospy.wait_for_service('/get_planning_scene')
    my_service = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
    sceneReq = PlanningSceneComponents(components=128)
    sceneResponse = my_service(sceneReq)
    acm = sceneResponse.scene.allowed_collision_matrix
    acm_correct = copy.deepcopy(acm)

    #Modify ACM to not consider collisions between arms
    rospy.wait_for_service('/apply_planning_scene')
    my_service = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
    sceneReq = PlanningScene()
    arms_joint_list = robot.get_joint_names("multirobot")
    for i in range(len(acm.entry_values)):
            for j in range(len(acm.entry_values)):
                    if i!=j and ((i not in arms_joint_list or "EEF" in i) and (j not in arms_joint_list or "EEF" in j)):
                            acm.entry_values[i].enabled[j] = True #No collisions
                    else:
                            acm.entry_values[i].enabled[j] = False
    sceneReq.allowed_collision_matrix = acm
    sceneReq.is_diff = True
    sceneResponse = my_service(sceneReq)

    if extra_info:
            planR, successR, t_acc_r, t_dec_r = compute_cartesian_path_velocity_control(waypoints_right, speed_right, EE_ang_speed = [], arm_side = "right", extra_info = extra_info)
            planL, successL, t_acc_l, t_dec_l = compute_cartesian_path_velocity_control(waypoints_left, speed_left, EE_ang_speed = [], extra_info = extra_info)
    else:
            planR, successR = compute_cartesian_path_velocity_control(waypoints_right, speed_right, EE_ang_speed = [], arm_side = "right")
            planL, successL = compute_cartesian_path_velocity_control(waypoints_left, speed_left, EE_ang_speed = [])
    if not successR or not successL:
            return [], False

    #Finish moving both arms at the same time
    if sync_policy == 1:
            if len(rs.joint_state.position) > 0: #In case this function is called by sync policy 3, it updates the start state of the arm
                    arm_left.set_start_state(rs_l) 
                    arm_right.set_start_state(rs_r)      
                    arms.set_start_state(rs)
            #Adjusts the step of the shorter plan and recalculates it, so both have approximately the same number of points
            if len(planR.joint_trajectory.points) > len(planL.joint_trajectory.points):
                    planL, success = compute_cartesian_path_velocity_control(waypoints_left, [speed_left[-1]], EE_ang_speed = [], arm_side = "left", step = 0.002*(float(len(planL.joint_trajectory.points))/float(len(planR.joint_trajectory.points))))
            else:
                    planR, success = compute_cartesian_path_velocity_control(waypoints_right, [speed_right[-1]], EE_ang_speed = [], arm_side = "right", step = 0.002*(float(len(planR.joint_trajectory.points))/float(len(planL.joint_trajectory.points))))
            plan_both = merge_plans(planL, planR)

    #Each arm moves at its own speed
    elif sync_policy == 2:
            plan_both = merge_plans_keep_speed(planL, planR)
    else:
            return [], False

    #In case both arms need to finish at the same time (sync policy 1), the real speed of one of the arms changes, therefore it is recalculated
    total_t_R = float(planR.joint_trajectory.points[-1].time_from_start.secs) + 0.000000001 * float(planR.joint_trajectory.points[-1].time_from_start.nsecs)
    total_t_L = float(planL.joint_trajectory.points[-1].time_from_start.secs) + 0.000000001 * float(planL.joint_trajectory.points[-1].time_from_start.nsecs)
    if extra_info:
            if total_t_R > total_t_L:
                    v_real = [speed_left[-1] * (total_t_L/total_t_R), speed_right[-1]]
                    t_acc = t_acc_r
                    t_dec = t_dec_r
            else:
                    v_real = [speed_left[-1], speed_right[-1] * (total_t_R/total_t_L)]
                    t_acc = t_acc_l
                    t_dec = t_dec_l

    #After calculating the plans, the correct ACM is restablished and then, the plan validity is checked for every configuration            
    sceneReq = PlanningScene()
    sceneReq.allowed_collision_matrix = acm_correct
    sceneReq.is_diff = True
    sceneResponse = my_service(sceneReq)

    plan_both_success = True
    rospy.wait_for_service('/check_state_validity')
    my_service = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
    check_stateReq = GetStateValidityRequest()
    check_robot_state = JointState()

    check_robot_state.name = copy.deepcopy(plan_both.joint_trajectory.joint_names)
    state_robot = robot.get_current_state()
    add_joints_index = []
    i = 0
    for joint_name in state_robot.joint_state.name:
            if joint_name not in check_robot_state.name:
                    add_joints_index.append(i)  
            i+=1 
    for i in add_joints_index: #Adds the joints that are not in the generated plan, for instance, the joints of the torso
            check_robot_state.name.append(state_robot.joint_state.name[i])

    #Add both end effectors to check the collisions
    # attached_gripper_left = scene.get_attached_objects([ATC1.EEF_left])[ATC1.EEF_left]
    # attached_gripper_right = scene.get_attached_objects([ATC1.EEF_right])[ATC1.EEF_right]
    # check_stateReq.robot_state.attached_collision_objects.append(attached_gripper_left)
    # check_stateReq.robot_state.attached_collision_objects.append(attached_gripper_right)

    #Check the validity of all the states of the trajectory    
    for joint_state in plan_both.joint_trajectory.points:
            check_robot_state.position = []
            for joint in joint_state.positions:
                    check_robot_state.position.append(joint)
            for i in add_joints_index:
                    check_robot_state.position.append(state_robot.joint_state.position[i])
            check_stateReq.robot_state.joint_state = check_robot_state
            stateValidityResp = my_service(check_stateReq)
            if not stateValidityResp.valid:
                    plan_both_success = False
                    break

    if plan_both_success:
            if extra_info: #Required when sync policy 3 is used
                    return plan_both, True, t_acc, t_dec, v_real
            else:
                    return plan_both, True
    else:
            print("Error generating dual-arm plan")
            plan = RobotTrajectory()
            if extra_info: #Required when sync policy 3 is used
                    return plan, False, 0, 0, 0
            else:
                    return plan, False


def master_slave_plan(waypoints, ATC1, speed=30, arm="left", type=1):
    """
    Funtion to generate dual-arm master-slave plans
        - waypoints_gripper: List of waypoints of the master arm [poses]
        - speed: EEF speed of the master arm [mm/s]
        - arm: Master arm side ["left" or "right"]
        - type: It can be 1 or 2.
                type 1: The distance between the EEFs is constant during the motion
                type 2: The motion of the slave arm is identical to the motion of the master arm
    """
    if type==1:
        plan, success = master_slave_constant_distance(waypoints, ATC1, speed, arm)
        return plan, success
    else:
        return [], False

######################################################################################################################################

if __name__ == '__main__':
    #Define parameters of the tools for the ATC objects
    eef_link_left = "left_tool_exchanger"
    touch_links_left = ["left_tool_exchanger"]
    eef_link_right = "right_tool_exchanger"
    touch_links_right = ["right_tool_exchanger"]

    gripper_left_ATC_frame = PyKDL.Frame()
    gripper_left_ATC_frame.p = PyKDL.Vector(0.85, 0.3, 1.4)
    gripper_left_ATC_frame.M.DoRotY(1.57)
    gripper_left_ATC_frame.M.DoRotZ(3.14)
    gripper_left_ATC_pose = frame_to_pose(gripper_left_ATC_frame)

    gun_ATC_frame = PyKDL.Frame()
    gun_ATC_frame.p = PyKDL.Vector(0.85, 0, 1.4)
    gun_ATC_frame.M.DoRotY(1.57)
    gun_ATC_frame.M.DoRotZ(3.14)
    gun_ATC_pose = frame_to_pose(gun_ATC_frame)

    gripper_right_ATC_frame = PyKDL.Frame()
    gripper_right_ATC_frame.p = PyKDL.Vector(0.85, -0.3, 1.4)
    gripper_right_ATC_frame.M.DoRotY(1.57)
    gripper_right_ATC_frame.M.DoRotZ(3.14)
    gripper_right_ATC_pose = frame_to_pose(gripper_right_ATC_frame)

    gripper_end_frame = PyKDL.Frame() 
    gripper_end_frame.p = PyKDL.Vector(0, 0, 0.275) 
    gun_end_frame = PyKDL.Frame()
    gun_end_frame.p = PyKDL.Vector(0.1, -0.055, 0.086) 
    gun_end_frame.M.DoRotY(1.57)

    EE_file_path_gripper = os.path.join(os.path.dirname(__file__), '/home/canongun/project_ws/src/EE/gripper_thin_simplified.stl')

    gripper_left = EEF(EE_end_frame = gripper_end_frame, x = 0.073, y = 0.025, z = 0.24, ATC_frame = gripper_left_ATC_frame, name = "EEF_gripper_left", path = EE_file_path_gripper)
    gripper_right = EEF(EE_end_frame = gripper_end_frame, x = 0.073, y = 0.025, z = 0.24, ATC_frame = gripper_right_ATC_frame, name = "EEF_gripper_right", path = EE_file_path_gripper)
        
    # print("Moving arms to initial position")
    # arms.set_named_target("multirobot_operation_ready")
    # arms.go(wait=True)
    # time.sleep(0.5)

    #Define the ATC instance
    ATC1 = ATC(left_tool=gripper_left, right_tool=gripper_right, eef_link_left=eef_link_left, eef_link_right=eef_link_right, ATC_tools=[], left_ATC_angle=0.7854, right_ATC_angle=0.7854, left_ATC_dist=0.09, right_ATC_dist=0.09)

    ### test: master-slave keep distance
    print("------------------")
    print("Test: master-slave keep distance")
    wrist_pose_left = arm_left.get_current_pose().pose
    wrist_pose_left_2 = copy.deepcopy(wrist_pose_left)
    # wrist_pose_left_2.position.x -= 0.15
    wrist_pose_left_2.position.y -= 0.1
    wrist_pose_left_2.position.z += 0.3

    #Linear movement
    plan, success = master_slave_plan([wrist_pose_left, wrist_pose_left_2], ATC1, 40.0, "left", type=1)
    if success:
            arms.execute(plan, wait=True)