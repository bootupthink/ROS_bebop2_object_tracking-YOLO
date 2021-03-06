#ifndef BEBOP_TELEOP_BEBOP_TELEOP_H
#define BEBOP_TELEOP_BEBOP_TELEOP_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <termios.h>
#include <iostream>
#include <cstdlib>

#define UP 1
#define DOWN -1
#define FORWARD 1
#define BACKWARD -1
#define LEFT 1
#define RIGHT -1
#define CCW 1
#define CW -1

extern const bool trackOff;
extern const bool trackOn;
extern const std::string tracking;

class BebopKeyBoardController
{
private:
    ros::NodeHandle _nodeHandle;
    ros::Publisher _twistPublisher;
    ros::Publisher _takeOffPublisher;
    ros::Publisher _landPublisher;
    ros::Publisher _emergencyPublisher;

    geometry_msgs::Twist _controlValue;
    std_msgs::Empty _message;

    double _speedValue;
    double _speedIncreaseValue;
    bool _isTakeOff;
    bool _isTracking;
    static const char* Interface[];

    void _printInterface();
    void _move(double& value, int orientation);
    void _takeoff();
    void _land();
    void _emergency();
    void _speedUp();
    void _speedDown();
    char _getKey();

public:
    explicit BebopKeyBoardController(const ros::NodeHandle& nodeHandle);
    void Control();
};

#endif //BEBOP_TELEOP_BEBOP_TELEOP_H
