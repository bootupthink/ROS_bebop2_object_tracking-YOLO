#include <bebop_track/bebop_teleop.h>

const bool trackOff = false;
const bool trackOn = true;
const std::string tracking = "/bebop/tracking";
const char* BebopKeyBoardController::Interface[] = {
        "+-----------------------------------------------------+",
        "|                 Control your Bebop                  |",
        "|        W      R                            8        |",
        "|                                                     |",
        "|    A   S   D  F                        4   5   6    |",
        "|                                                     |",
        "| throttle UP / DOWN : W / S                          |",
        "| rotation CCW / CW  : A / D                          |",
        "| camera ?? / ??     : R / F                          |",
        "| FORWARD / BACKWARD : 8 / 5                          |",
        "| LEFT / RIGHT       : 4 / 6                          |",
        "| TAKE OFF / LAND    : Spacebar                       |",
        "| Tracking Mode      : T                              |",
        "| Emergency          : P                              |",
        "| Quit               : Q                              |",
        "+-----------------------------------------------------+"
};

void BebopKeyBoardController::_printInterface()
{
    _nodeHandle.getParam(tracking, _isTracking);
    int result = system("clear");
    for(int i = 0; i < 16; ++i)
        std::cout << Interface[i] << std::endl;
    std::cout << "[status] Takeoff : "<< ((_isTakeOff) ? "ON" : "OFF")
    << "\tTracking Mode : " << (_isTracking ? "ON" : "OFF") << std::endl;
    std::cout << "[Speed value] : " << _speedValue << std::endl;
    std::cout << "[Speed Increase Value] : " << _speedIncreaseValue << std::endl;
}

void BebopKeyBoardController::_move(double &value, int orientation)
{
    if(_isTakeOff)
    {
        _controlValue.linear.x = 0;
        _controlValue.linear.y = 0;
        _controlValue.linear.z = 0;
        _controlValue.angular.z = 0;

        value = _speedValue * orientation;
        _twistPublisher.publish(_controlValue);
    }
}

void BebopKeyBoardController::_takeoff()
{
    _isTakeOff = true;
    _takeOffPublisher.publish(_message);
    _printInterface();
}

void BebopKeyBoardController::_land()
{
    _isTakeOff = false;
    _landPublisher.publish(_message);
    _nodeHandle.setParam(tracking, trackOff);
    _printInterface();
}

void BebopKeyBoardController::_emergency()
{
    _isTakeOff = false;
    _emergencyPublisher.publish(_message);
    _nodeHandle.setParam(tracking, trackOff);
    _printInterface();
    ROS_INFO("\n\nDrone Emergency Land!");
}

void BebopKeyBoardController::_speedUp()
{
    _speedValue = _speedValue + _speedIncreaseValue;
}

void BebopKeyBoardController::_speedDown()
{
    _speedValue = _speedValue - _speedIncreaseValue;
    if(_speedValue < 0)
        _speedValue = 0;
}

char BebopKeyBoardController::_getKey()
{

    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return (char)ch;
}

BebopKeyBoardController::BebopKeyBoardController(const ros::NodeHandle& nodeHandle)
        :_nodeHandle(nodeHandle),
         _twistPublisher(_nodeHandle.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 10)),
         _takeOffPublisher(_nodeHandle.advertise<std_msgs::Empty>("bebop/takeoff", 1)),
         _landPublisher(_nodeHandle.advertise<std_msgs::Empty>("bebop/land", 1)),
         _emergencyPublisher(_nodeHandle.advertise<std_msgs::Empty>("bebop/reset", 1)),
         _isTakeOff(false),
         _isTracking(false),
         _speedIncreaseValue(0.05),
         _speedValue(0.5)
{
    _nodeHandle.setParam(tracking, trackOff);
    _printInterface();
}

void BebopKeyBoardController::Control()
{
    double& x = _controlValue.linear.x = 0;
    double& y = _controlValue.linear.y = 0;
    double& z = _controlValue.linear.z = 0;
    double& rotation = _controlValue.angular.z = 0;

    char key;

    while(ros::ok())
    {
        _printInterface();
        key = _getKey();
        if(!_isTakeOff && (key == 'q' || key == 'Q'))
            break;

        switch(key)
        {
            case ' ':
                _isTakeOff ? _land() : _takeoff();
                break;
            case 'P':
            case 'p':
                _emergency();
                break;
            case 'T':
            case 't':
                if(_isTakeOff)
                {
                    _nodeHandle.getParam(tracking, _isTracking);
                    if(!_isTracking)
                    {
                        _nodeHandle.setParam(tracking, trackOn);
                        _printInterface();
                    }
                    else
                    {
                        _nodeHandle.setParam(tracking, trackOff);
                        _printInterface();
                    }
                }
                break;
            case '+':
                _speedUp();
                _printInterface();
                break;
            case '-':
                _speedDown();
                _printInterface();
                break;
            case 'W':
            case 'w':
                _move(z, UP);
                break;
            case 'S':
            case 's':
                _move(z, DOWN);
                break;
            case 'A':
            case 'a':
                _move(rotation, CCW);
                break;
            case 'D':
            case 'd':
                _move(rotation, CW);
                break;
            case 'R':
            case 'r':
                break;
            case 'F':
            case 'f':
                break;
            case '8':
                _move(x, FORWARD);
                break;
            case '5':
                _move(x, BACKWARD);
                break;
            case '4':
                _move(y, LEFT);
                break;
            case '6':
                _move(y, RIGHT);
                break;
            default:
                break;
        }
    }
}