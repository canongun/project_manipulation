#include <arm_freeze_cpp/arm_freeze_client.h>

int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "arm_freeze_client_cpp");
        ArmFreezeClient client;
        ROS_INFO("SENDING GOAL");
        client.sendGoal(true);
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("Program interrupted before completion: %s", e.what());
        return 1;
    }
    return 0;
}