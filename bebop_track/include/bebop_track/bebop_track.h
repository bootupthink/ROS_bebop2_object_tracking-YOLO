#ifndef BEBOP_TRACK_BEBOP_TRACK_H
#define BEBOP_TRACK_BEBOP_TRACK_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

class BebopTrack
{
private:
    ros::NodeHandle _nodeHandle;
    ros::Publisher _bebopControlMessagePublisher;
    ros::Subscriber _boxMessageSubscriber;

    geometry_msgs::Twist _bebopControlMessage;

    std::string _darknet_Boxes_TopicName;
    std::string _trackModeString;
    std::string _camModeString;

    bool _isTracking;
    bool _isCamMode;
    bool _Camera;


    long int _inCameraModeXmin;
    long int _inCameraModeXmax;
    long int _inCameraModeYmin;
    long int _inCameraModeYmax;

    const int _frameMiddleX;
    const int _frameMiddleY;

    void _boxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxMessage);
    void _runTracking(const darknet_ros_msgs::BoundingBox& boxMsg);
    void _selfcam(const darknet_ros_msgs::BoundingBox& boxMsg);

public:
    explicit BebopTrack(const ros::NodeHandle& nodeHandle);
};

#endif //BEBOP_TRACK_BEBOP_TRACK_H
