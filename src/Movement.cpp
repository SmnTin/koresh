//
// Created by smntin on 06.07.18.
//

#include "Movement.h"

Movement* Movement::s_pInstance = 0;

Movement* Movement::Instance() {
    if(s_pInstance == 0)
        s_pInstance = new Movement();
    return s_pInstance;
}

bool Movement::init(ros::NodeHandle * _node, string arduTwistTopic, string arduOdomTopic, double _angleVel) {
    node = _node;
    angleVel = _angleVel;

    sub = node->subscribe<koresh::ArduOdom>(arduOdomTopic, 1000, &Movement::odometryCallback, this );
    pub = node->advertise<koresh::NewTwist>(arduTwistTopic, 1000);
    return true;
}

//void Movement::move(double dirX, double dirY, double speed) {
void Movement::move(double linear, double angular, double orient) {
//    double vx = dirX * speed / sqrt(dirX * dirX + dirY * dirY);
//    double vy = dirY * speed / sqrt(dirX * dirX + dirY * dirY);
//
//    koresh::NewTwist msg;
//
//    msg.orient = atan2(vy,vx);
//    msg.angle_vel = (msg.orient < 3.1415926 ? angleVel : -angleVel);
//    msg.linear_vel = sqrt(vx * vx + vy * vy);

    koresh::NewTwist msg;
    msg.orient = orient;
    msg.angle_vel = angular;
    msg.linear_vel = linear;

    pub.publish(msg);
}

void Movement::move(koresh::NewTwist msg) {
    pub.publish(msg);
}

void Movement::odometryCallback(const koresh::ArduOdom::ConstPtr & odom) {
    stringstream ss;
    ss << odom->wfl << " " << odom->wfr << " " << odom->wrl << " " << odom->wrr;
    ROS_INFO(ss.str().c_str());
}

void Movement::registerMovementController(MovementController *controller) {
    if(controller->init(node, movementControllerCallback))
        movementControllers.push_back(controller);
}

void Movement::movementControllerCallback(double linear, double angular, double orient) {
    Movement::Instance()->move(linear, angular, orient);
}