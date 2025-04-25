#include <arm_freeze_cpp/arm_freeze_server.h>
#include <stdexcept>

// Constructor
ArmFreezeServer::ArmFreezeServer(const std::string& name) :
    // nh_(),
    as_(nh_, name, boost::bind(&ArmFreezeServer::publishPosition, this, _1), false),
    action_name_(name),
    move_group_("mobile_arm")
{

    if (!move_group_.startStateMonitor(10.0)) {
        ROS_ERROR("Failed to start state monitor");
        // Handle this error, perhaps by throwing an exception
    }

    // Initialize MoveIt
    move_group_.setPlannerId("PTP");
    move_group_.setMaxAccelerationScalingFactor(1.0);
    move_group_.setMaxVelocityScalingFactor(1.0);

    // Get current pose
    geometry_msgs::Pose mobile_arm_pose_ = move_group_.getCurrentPose().pose;
    
    ekf_odom_listener_ = nh_.serviceClient<multirobot_services::GroundTruthListener>("/ekf_odometry_listener");
    
    as_.start();
}

void ArmFreezeServer::publishPosition(const arm_freeze_cpp::ArmFreezeGoalConstPtr& goal){
    if (goal) {
        arm_freeze_cpp::ArmFreezeResult result;
        
        ROS_INFO("Requesting ground truth listener service");
        ros::service::waitForService("/ekf_odometry_listener");


        // Somehow getCurrentPose() function is not wroking properly, thus the quaternions are hard coded
        // tf2::Quaternion q_mobile_tool0(
        //     mobile_arm_pose_.orientation.x,
        //     mobile_arm_pose_.orientation.y,
        //     mobile_arm_pose_.orientation.z,
        //     mobile_arm_pose_.orientation.w
        // );

        tf2::Quaternion q_mobile_tool0(
                                        0,
                                        0,
                                        0,
                                        1
                                    );

        double roll_mobile_tool0, pitch_mobile_tool0, yaw_mobile_tool0;
        tf2::Matrix3x3(q_mobile_tool0).getRPY(roll_mobile_tool0, pitch_mobile_tool0, yaw_mobile_tool0);
        
        multirobot_services::GroundTruthListener srv;
        srv.request.start = true;
        ekf_odom_listener_.call(srv);

        double x0 = srv.response.link_info[0] + LINK_DIST * cos(0);
        double y0 = srv.response.link_info[1] + LINK_DIST * sin(0);

        while (ros::ok()){
            ekf_odom_listener_.call(srv);

            tf2::Quaternion q_base_link(
                srv.response.link_info[3],
                srv.response.link_info[4],
                srv.response.link_info[5],
                srv.response.link_info[6]
            );

            double roll_base_link, pitch_base_link, yaw_base_link;
            tf2::Matrix3x3(q_base_link).getRPY(roll_base_link, pitch_base_link, yaw_base_link);
            
            double yaw_real = -yaw_base_link;
            
            roll_base_link -= M_PI / 2;
            yaw_base_link -= M_PI;

            double delta_or_R = roll_mobile_tool0 - roll_base_link;
            double delta_or_P = pitch_mobile_tool0 - pitch_base_link;
            double delta_or_Y = yaw_mobile_tool0 - yaw_base_link;
            
            double al_R = roll_mobile_tool0 + delta_or_R;
            double al_P = pitch_mobile_tool0 - delta_or_P;
            double al_Y = yaw_mobile_tool0 + delta_or_Y;
            
            tf2::Quaternion q_arm;
            q_arm.setRPY(al_R, al_P, al_Y);

            double delta_pos_x = srv.response.link_info[0] + LINK_DIST * cos(yaw_real);
            double delta_pos_y = srv.response.link_info[1] - LINK_DIST * sin(yaw_real);
            
            ROS_INFO("POS_X= %f, POS_Y= %f, YAW= %f", delta_pos_x, delta_pos_y, yaw_real);
            
            double trans_x = mobile_arm_pose_.position.x + x0 - delta_pos_x;
            double trans_y = mobile_arm_pose_.position.y + y0 - delta_pos_y;
            
            double rot_x = trans_x * cos(yaw_real) - trans_y * sin(yaw_real) + LINK_DIST - LINK_DIST * cos(yaw_real);
            double rot_y = trans_x * sin(yaw_real) + trans_y * cos(yaw_real) - LINK_DIST * sin(yaw_real);
            
            geometry_msgs::Pose pose_goal;
            pose_goal.orientation.x = q_arm.x();
            pose_goal.orientation.y = q_arm.y();
            pose_goal.orientation.z = q_arm.z();
            pose_goal.orientation.w = q_arm.w();
            pose_goal.position.x = rot_x;
            pose_goal.position.y = rot_y;
            pose_goal.position.z = mobile_arm_pose_.position.z;

            ROS_INFO_STREAM(pose_goal);
                
            move_group_.setPoseTarget(pose_goal);

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            
            ROS_INFO("Planning time: %f", my_plan.planning_time_);
            
            if (success) {
                // Vitally important for real-time execution
                move_group_.asyncExecute(my_plan);
            }

            ros::Duration(1.0/PUB_HZ).sleep();
        }
    }
};