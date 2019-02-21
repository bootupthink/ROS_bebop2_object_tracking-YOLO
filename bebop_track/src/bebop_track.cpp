#include <bebop_track/bebop_track.h>

void BebopTrack::_boxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &boxMessage)
{
    _nodeHandle.getParam(_trackModeString, _isTracking);
    _nodeHandle.getParam(_camModeString, _isCamMode);
    darknet_ros_msgs::BoundingBox boxMsg = boxMessage->bounding_boxes[0];

    _bebopControlMessage.linear.x = 0;
    _bebopControlMessage.linear.y = 0;
    _bebopControlMessage.linear.z = 0;
    _bebopControlMessage.angular.x = 0;
    _bebopControlMessage.angular.y = 0;
    _bebopControlMessage.angular.z = 0;

    if(boxMsg.probability >= 0.65)
    {
        ROS_INFO("Class : %s", boxMsg.Class.c_str());
        ROS_INFO("probability : %f", boxMsg.probability);
        ROS_INFO("xmin : %ld", boxMsg.xmin);
        ROS_INFO("xmax : %ld", boxMsg.xmax);
        ROS_INFO("ymin : %ld", boxMsg.ymin);
        ROS_INFO("ymax : %ld", boxMsg.ymax);
        ROS_INFO("cameraWidth : %ld", boxMsg.cameraWidth);
        ROS_INFO("cameraHeight : %ld", boxMsg.cameraHeight);
    }

    if(_isTracking)
    {
        _runTracking(boxMsg);
        _Camera = false;
    }
    else if(_isCamMode)
        _selfcam(boxMsg);
    else
        _Camera = false;
}

BebopTrack::BebopTrack(const ros::NodeHandle& nodeHandle)
:_nodeHandle(nodeHandle),
 _darknet_Boxes_TopicName("darknet_ros/bounding_boxes"),
 _trackModeString("/bebop/tracking"),
 _camModeString("/bebop/selfcam"),
_isTracking(false),
_isCamMode(false),
_Camera(false),
_frameMiddleX(428),
_frameMiddleY(240)
{
    _bebopControlMessagePublisher = _nodeHandle.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 10);
    _boxMessageSubscriber = _nodeHandle.subscribe(_darknet_Boxes_TopicName, 1, &BebopTrack::_boxCallback, this);
}

void BebopTrack::_runTracking(const darknet_ros_msgs::BoundingBox& boxMsg)
{
    if(boxMsg.probability >= 0.65)
    {
        long int xMiddle = (boxMsg.xmin + boxMsg.xmax) / 2;
        long int yMiddle = (boxMsg.ymin + boxMsg.ymax) / 2;
        long int xDifference = xMiddle - _frameMiddleX;
        long int yDifference = yMiddle - _frameMiddleY;
        long int xDifferenceAbsolute = labs(xMiddle - _frameMiddleX);
        long int yDifferenceAbsolute = labs(yMiddle - _frameMiddleY);

        long int bboxHeight = boxMsg.ymax - boxMsg.ymin;

        if(xDifferenceAbsolute > 130)
        {
            if(xDifference > 30 ){
                _bebopControlMessage.angular.z = -0.35;
                ROS_INFO("CCW");
            }
            else if(xDifference < 30) {
                _bebopControlMessage.angular.z = 0.35;
                ROS_INFO("CW");
            }

        }

        else if(xDifferenceAbsolute >80)
        {
            if(xDifference > 20 ){
                _bebopControlMessage.angular.z = -0.2;
                ROS_INFO("CCW");
            }
            else if(xDifference < 20) {
                _bebopControlMessage.angular.z = 0.2;
                ROS_INFO("CW");
            }

        }

        else if(xDifferenceAbsolute >30)
        {
            if(xDifference > 0 ){
                _bebopControlMessage.angular.z = -0.1;
                ROS_INFO("CCW");
            }
            else if(xDifference < 0) {
                _bebopControlMessage.angular.z = 0.1;
                ROS_INFO("CW");
            }

        }

        else
        {
            _bebopControlMessage.angular.z = 0;
            ROS_INFO("WIDTH CLEAR");
        }
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////

        if(bboxHeight < 150)
        {
            _bebopControlMessage.linear.x = 0.23;
            if(xDifference > 60) {
                _bebopControlMessage.linear.y = -0.08;
                _bebopControlMessage.linear.x -= 0.18;
                _bebopControlMessage.angular.z += -0.13;
            }
            else if(xDifference < -60) {
                _bebopControlMessage.linear.y = 0.08;
                _bebopControlMessage.linear.x -= 0.18;
                _bebopControlMessage.angular.z += -0.13;
            }


            if(yMiddle >_frameMiddleY +15){
                _bebopControlMessage.linear.z = -0.28;
                ROS_INFO("DOWN");
            } else if(yMiddle < _frameMiddleY-15){
                _bebopControlMessage.linear.z = 0.28;
                ROS_INFO("UP");
            } else {
                _bebopControlMessage.linear.z = 0;
                ROS_INFO("HEIGHT CLEAR");
            }
        }

        else if(bboxHeight < 250 )
        {
            _bebopControlMessage.linear.x = 0.18;
            if(xDifference > 35) {
                _bebopControlMessage.linear.y = -0.03;
                _bebopControlMessage.linear.x -= 0.13;
                _bebopControlMessage.angular.z += -0.10;
            }
            else if(xDifference < -35) {
                _bebopControlMessage.linear.y = 0.03;
                _bebopControlMessage.linear.x -= 0.13;
                _bebopControlMessage.angular.z += -0.10;
            }


            if(yMiddle >_frameMiddleY +15){
                _bebopControlMessage.linear.z = -0.2;
                ROS_INFO("DOWN");
            } else if(yMiddle < _frameMiddleY-15){
                _bebopControlMessage.linear.z = 0.2;
                ROS_INFO("UP");
            } else {
                _bebopControlMessage.linear.z = 0;
                ROS_INFO("HEIGHT CLEAR");
            }
        }

        else if(bboxHeight < 290 )
        {
            _bebopControlMessage.linear.x = 0.1;
            if(xDifference > 10) {
                _bebopControlMessage.linear.y = -0.01;
                _bebopControlMessage.linear.x -= 0.03;
                _bebopControlMessage.angular.z += -0.07;
            }
            else if(xDifference < -10){
                _bebopControlMessage.linear.y = 0.01;
                _bebopControlMessage.linear.x -= 0.03;
                _bebopControlMessage.angular.z += -0.07;
            }


            if(yMiddle >_frameMiddleY +15){
                _bebopControlMessage.linear.z = -0.05;
                ROS_INFO("DOWN");
            } else if(yMiddle < _frameMiddleY-15){
                _bebopControlMessage.linear.z = 0.05;
                ROS_INFO("UP");
            } else {
                _bebopControlMessage.linear.z = 0;
                ROS_INFO("HEIGHT CLEAR");
            }
        }

        else if(bboxHeight <325)
            _bebopControlMessage.linear.x = 0;

        else
            _bebopControlMessage.linear.x = -0.1;

        _bebopControlMessagePublisher.publish(_bebopControlMessage);
    }
}

void BebopTrack::_selfcam(const darknet_ros_msgs::BoundingBox &boxMsg)
{
    if(!_Camera)
    {
        _Camera = true;
        _inCameraModeXmin = boxMsg.xmin;
        _inCameraModeXmax = boxMsg.xmax;
        _inCameraModeYmin = boxMsg.ymin;
        _inCameraModeYmax = boxMsg.ymax;
    }
    //xmin = 415
    //xmax = 465;
    //ymin = 210;
    //ymax = 370;

    if(boxMsg.probability >= 0.55) {
        long int originHeight = _inCameraModeYmax - _inCameraModeYmin;
        long int originWidth = _inCameraModeXmax - _inCameraModeXmin;
        long int boxHeight = boxMsg.ymax - boxMsg.ymin;
        long int boxWidth = boxMsg.xmax - boxMsg.xmin;
        long int boxMiddleX = (boxWidth / 2) + boxMsg.xmin;
        long int boxMiddleY = (boxHeight / 2) + boxMsg.ymin;

        long int heightGap = originHeight - boxHeight;
        long int widthGap = originWidth - boxWidth;
        long int middleXGap = _frameMiddleX - boxMiddleX;
        long int middleYGap = _frameMiddleY - boxMiddleY;

        double &Goback = _bebopControlMessage.linear.x;
        double &LeftRight = _bebopControlMessage.linear.y;
        double &Throttle = _bebopControlMessage.linear.z;
        double &Rotation = _bebopControlMessage.angular.z;

        if (labs(middleXGap) > 25)
            Rotation = copysign(labs(middleXGap) / 400.0, middleXGap);
        else
            Rotation = 0;

        if(labs(heightGap) > 30) {
            Goback = copysign(0.08, heightGap);
            LeftRight = 0.08;
        }
        else {
            Goback = 0.09;
            LeftRight = 0.11;
        }

        _bebopControlMessagePublisher.publish(_bebopControlMessage);
    }

}


