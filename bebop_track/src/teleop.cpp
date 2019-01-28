#include <bebop_track/bebop_teleop.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bebop2_control_node");
    ros::NodeHandle nodeHandle;

    BebopKeyBoardController controller(nodeHandle);

    controller.Control();

    return 0;
}