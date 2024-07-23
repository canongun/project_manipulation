#include <arm_freeze_cpp/arm_freeze_client.h>

ArmFreezeClient::ArmFreezeClient() 
    :client_("/arm_freeze", true)
{
    ROS_INFO("Waiting for action server to start.");
    ros::Rate rate(1); // 1 Hz
    int attempts = 0;
    while (!client_.waitForServer(ros::Duration(5.0)))
    {
        if (!ros::ok())
        {
            ROS_ERROR("Shutting down");
            return;
        }
        ROS_WARN_STREAM("Waiting for the action server. Attempt: " << ++attempts);
        rate.sleep();
    }
    ROS_INFO("Action server started.");
}

void ArmFreezeClient::sendGoal(bool start)
{
    ROS_INFO("Sending goal.");

    // Create a goal to send to the action server
    arm_freeze_cpp::ArmFreezeGoal goal;
    goal.start = true;

    // Send the goal to the action server
    client_.sendGoal(goal);

    // Wait for the server to finish performing the action
    client_.waitForResult();

    // Check the result
    if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Action completed successfully");
    else
        ROS_INFO("Action did not succeed");

    ros::Duration(2.0).sleep();
}