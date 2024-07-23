#ifndef ARM_FREEZE_SERVER_H
#define ARM_FREEZE_SERVER_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <arm_freeze_cpp/ArmFreezeAction.h> 
#include <multirobot_services/GroundTruthListener.h>
#include <memory>

using namespace std;
using namespace moveit::planning_interface;

const double LINK_DIST = 0.081;
const double PUB_HZ = 10.0;

class ArmFreezeServer
{
    private:
        bool getCurrentPose(geometry_msgs::Pose& pose);
        void publishPosition(const arm_freeze_cpp::ArmFreezeGoalConstPtr& goal);

        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<arm_freeze_cpp::ArmFreezeAction> as_;
        std::string action_name_;

        // Move group interfaces for both arms
        moveit::planning_interface::MoveGroupInterface move_group_;
        // Planning scene interfaces
        moveit::planning_interface::PlanningSceneInterface scene_;
        
        arm_freeze_cpp::ArmFreezeFeedback feedback_;
        arm_freeze_cpp::ArmFreezeResult result_;

        geometry_msgs::Pose mobile_arm_pose_;
        geometry_msgs::Pose pose;
    
        ros::ServiceClient ekf_odom_listener_;

    public:
        ArmFreezeServer(const std::string& name);
};

#endif