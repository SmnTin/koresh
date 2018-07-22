//
// Created by smntin on 08.07.18.
//

#include "Joy.h"

bool Joy::init(ros::NodeHandle * _node, void (*_cb)(double linear, double angular, double orient)) {
    cb = _cb;
    node = _node;
    sub = node->subscribe<koresh::NewTwist>("control/joystick", 1000, &Joy::twistHandler, this);
    return true;
}

void Joy::twistHandler(const koresh::NewTwist::ConstPtr &msg) {
//    double _dirX = ((msg->angle_vel == 0) ? 0 : (msg->angle_vel > 0 ? 1 : -1));
//    double _dirY = 1;
//    double _speed = (int)(msg->linear_vel != 0);
    ROS_INFO("JOYSTICK: %d %d %d", msg->linear_vel, msg->angle_vel, msg->orient);
    cb(msg->linear_vel, msg->angle_vel, msg->orient);
}