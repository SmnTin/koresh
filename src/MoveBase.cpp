#include "MoveBase.h"

bool MoveBase::init(ros::NodeHandle * _node, void (*_cb)(double linear, double angular, double orient)) {
    cb = _cb;
    node = _node;
    sub = node->subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &MoveBase::twistHandler, this);
    return true;
}

void MoveBase::twistHandler(const geometry_msgs::Twist::ConstPtr &msg) {
//    double _dirX = ((msg->angle_vel == 0) ? 0 : (msg->angle_vel > 0 ? 1 : -1));
//    double _dirY = 1;
//    double _speed = (int)(msg->linear_vel != 0);
//    ROS_INFO("JOYSTICK: %d %d %d", msg->linear_vel, msg->angle_vel, msg->orient);
//    ROS_INFO("CMD_VEL: linear:  orient: %d", atan2(msg->linear.y, msg->linear.x));
    cb(100*sqrt(msg->linear.x * msg->linear.x + msg->linear.y * msg->linear.y), msg->angular.z, atan2(msg->linear.y, msg->linear.x));
}