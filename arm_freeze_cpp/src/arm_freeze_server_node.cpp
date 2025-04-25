#include <arm_freeze_cpp/arm_freeze_server.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_freeze_server");
    ArmFreezeServer move_robot_server("arm_freeze");
    ros::spin();
    // ros::AsyncSpinner spinner(1);
    // spinner.start();
    // ros::waitForShutdown();
    return 0;
}