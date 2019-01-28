#include <bebop_track/bebop_track.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bebop_track_node");
    ros::NodeHandle nodeHandle;

    BebopTrack bebopTracker(nodeHandle);

    ros::spin();

    return 0;
}