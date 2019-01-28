#ifndef BEBOP_TRACK_BEBOP_TRACK_H
#define BEBOP_TRACK_BEBOP_TRACK_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

extern const std::string tracking;
extern const std::string darknet_Boxes_Topic;

class BebopTrack
{
private:
    ros::NodeHandle _nodeHandle;
    ros::Publisher _bebopControlMessagePublisher;
    ros::Subscriber _boxMessageSubscriber;

    geometry_msgs::Twist _bebopControlMessage;
    bool _isTracking;

    const int _frameMiddleX;
    const int _frameMiddleY;

    void _boxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxMessage);

public:
    explicit BebopTrack(const ros::NodeHandle& nodeHandle);
};

#endif //BEBOP_TRACK_BEBOP_TRACK_H
