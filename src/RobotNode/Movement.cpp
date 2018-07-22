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

bool Movement::init(ros::NodeHandle * _node, string arduTwistTopic, string arduOdomTopic, string odomTopic, double _angleVel) {
    node = _node;
    angleVel = _angleVel;

    previousOdomTime = ros::Time::now();

    odomSub = node->subscribe<koresh::ArduOdom>(arduOdomTopic, 1000, &Movement::odometryCallback, this );
    cmdPub = node->advertise<koresh::NewTwist>(arduTwistTopic, 1000);
    odomPub = node->advertise<nav_msgs::Odometry>(odomTopic, 1000);
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

    cmdPub.publish(msg);
}

void Movement::move(koresh::NewTwist msg) {
    cmdPub.publish(msg);
}

void Movement::odometryCallback(const koresh::ArduOdom::ConstPtr & wheel) {
//    ROS_INFO("RAW: %lf %lf %lf %lf", wheel->wfl, wheel->wfr, wheel->wrl, wheel->wrr);

    ros::Time currentOdomTime = ros::Time::now();

    vx = -0.25*rwheel*( -wheel->wfl + wheel->wfr - wheel->wrl + wheel->wrr );
    vy = -0.25*rwheel*( -wheel->wfl - wheel->wfr + wheel->wrl + wheel->wrr );
    vth = 0.25*rwheel/(lx+ly)*(-wheel->wfl - wheel->wfr - wheel->wrl - wheel->wrr );

    double dt = (currentOdomTime - previousOdomTime).toSec();

    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
//    ROS_INFO("AAA %lf", delta_y);
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = currentOdomTime;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odomBroadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = currentOdomTime;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
//    odom.pose.covariance = cov_matrix;
    odom.pose.covariance =  boost::assign::list_of (1e-3) (0) (0)  (0)  (0)  (0)
                                                   (0) (1e-3)  (0)  (0)  (0)  (0)
                                                   (0)   (0)  (1e6) (0)  (0)  (0)
                                                   (0)   (0)   (0) (1e6) (0)  (0)
                                                   (0)   (0)   (0)  (0) (1e6) (0)
                                                   (0)   (0)   (0)  (0)  (0)  (0.03) ; // 1e3

    odom.child_frame_id = "base_link";

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
//    odom.twist.covariance = cov_matrix;
    odom.twist.covariance =  boost::assign::list_of (1e-3) (0)   (0)  (0)  (0)  (0)
                                                  (0) (1e-3)  (0)  (0)  (0)  (0)
                                                  (0)   (0)  (1e6) (0)  (0)  (0)
                                                  (0)   (0)   (0) (1e6) (0)  (0)
                                                  (0)   (0)   (0)  (0) (1e6) (0)
                                                  (0)   (0)   (0)  (0)  (0)  (0.03) ;

    odomPub.publish(odom);

    previousOdomTime = currentOdomTime;
}

void Movement::registerMovementController(MovementController *controller) {
    if(controller->init(node, movementControllerCallback))
        movementControllers.push_back(controller);
}

void Movement::movementControllerCallback(double linear, double angular, double orient) {
    Movement::Instance()->move(linear, angular, orient);
}