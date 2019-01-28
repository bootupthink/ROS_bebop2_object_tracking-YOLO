#include <bebop_track/bebop_track.h>

const std::string tracking = "/bebop/tracking";
const std::string darknet_Boxes_Topic = "darknet_ros/bounding_boxes";

void BebopTrack::_boxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &boxMessage)
{
    _nodeHandle.getParam(tracking, _isTracking);
    if(!_isTracking)
        return;
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

            if(yMiddle >_frameMiddleY +15){
                _bebopControlMessage.linear.z = -0.15;
                ROS_INFO("DOWN");
            } else if(yMiddle < _frameMiddleY-15){
                _bebopControlMessage.linear.z = 0.15;
                ROS_INFO("UP");
            } else {
                _bebopControlMessage.linear.z = 0;
                ROS_INFO("HEIGHT CLEAR");
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

            if(yMiddle >_frameMiddleY +15){
                _bebopControlMessage.linear.z = -0.15;
                ROS_INFO("DOWN");
            } else if(yMiddle < _frameMiddleY-15){
                _bebopControlMessage.linear.z = 0.15;
                ROS_INFO("UP");
            } else {
                _bebopControlMessage.linear.z = 0;
                ROS_INFO("HEIGHT CLEAR");
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


            if(yMiddle >_frameMiddleY +15){
                _bebopControlMessage.linear.z = -0.15;
                ROS_INFO("DOWN");
            } else if(yMiddle < _frameMiddleY-15){
                _bebopControlMessage.linear.z = 0.15;
                ROS_INFO("UP");
            } else {
                _bebopControlMessage.linear.z = 0;
                ROS_INFO("HEIGHT CLEAR");
            }
        }

        else
        {
            _bebopControlMessage.angular.z = 0;
            ROS_INFO("WIDTH CLEAR");

            if(yMiddle >_frameMiddleY +15){
                _bebopControlMessage.linear.z = -0.15;
                ROS_INFO("DOWN");
            } else if(yMiddle < _frameMiddleY-15){
                _bebopControlMessage.linear.z = 0.15;
                ROS_INFO("UP");
            } else {
                _bebopControlMessage.linear.z = 0;
                ROS_INFO("HEIGHT CLEAR");
            }
        }
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////

        if(bboxHeight < 150)
        {
            _bebopControlMessage.linear.x = 0.25;
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
        }

        else if(bboxHeight < 250 )
        {
            _bebopControlMessage.linear.x = 0.21;
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
        }

        else if(bboxHeight < 300 )
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
        }

        else
            _bebopControlMessage.linear.x = 0;

        _bebopControlMessagePublisher.publish(_bebopControlMessage);
    }
}

BebopTrack::BebopTrack(const ros::NodeHandle& nodeHandle)
:_nodeHandle(nodeHandle),
_bebopControlMessagePublisher(_nodeHandle.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 10)),
_boxMessageSubscriber(_nodeHandle.subscribe(darknet_Boxes_Topic, 1, &BebopTrack::_boxCallback, this)),
_isTracking(false),
_frameMiddleX(428),
_frameMiddleY(240)
{

}
