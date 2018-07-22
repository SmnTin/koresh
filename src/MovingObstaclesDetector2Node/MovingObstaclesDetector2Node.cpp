#define _USE_MATH_DEFINES

#include "ros/ros.h"

#include <obstacle_detector/Obstacles.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>

#include <string>
#include <cmath>

#include "../Point.h"

using namespace std;

string odomTopic = (string)"/sensor/odom";
string obstaclesTopic = (string)"/tracked_obstacles";
string resTopic = (string)"/moving_obstacles";

ros::Publisher resPub;
int seq = 0;

double velocityThreshold = 0.2;

nav_msgs::Odometry::ConstPtr odom;

void odomCb(const nav_msgs::Odometry::ConstPtr & _odom) {
    odom = _odom;
}

void obstaclesCb(const obstacle_detector::Obstacles::ConstPtr & obstacles ) {
    if(odom == 0)
        return;

    sensor_msgs::PointCloud res;

    res.header.frame_id = "laser";
    res.header.stamp = odom->header.stamp;
    res.header.seq = seq++;

    res.channels.push_back(sensor_msgs::ChannelFloat32());
    res.channels[0].name = "moving";

    for(auto t = obstacles->circles.begin(); t != obstacles->circles.end(); t++) {
        double w = odom->twist.twist.angular.z;
        double ang = atan2(t->center.y, t->center.x);
        double r = sqrt(t->center.x * t->center.x + t->center.y * t->center.y);
        Point va (-odom->twist.twist.linear.x, -odom->twist.twist.linear.y);
        Point vb (-w * r * cos(ang + M_PI / 2), -w * r * sin(ang + M_PI / 2));
        Point staticVelocity = va + vb;
        Point bodyVelocity(t->velocity.x, t->velocity.y);

        ROS_INFO("va %lf %lf", va.x, va.y);
        ROS_INFO("vb %lf %lf", vb.x, vb.y);
        ROS_INFO("sv %lf %lf", staticVelocity.x, staticVelocity.y);
        ROS_INFO("bv %lf %lf", bodyVelocity.x, bodyVelocity.y);

        geometry_msgs::Point32 p;
        p.x = t->center.x;
        p.y = t->center.y;
        p.z = 0;
        res.points.push_back(p);

//        if((staticVelocity - bodyVelocity).vecLength() > velocityThreshold) {
        if((bodyVelocity).vecLength() > velocityThreshold) {
//            ROS_INFO("MOOOOVING SUKA");
            res.channels[0].values.push_back(100);
        }
        else
            res.channels[0].values.push_back(0);
    }

    resPub.publish(res);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "moving_obstacles_detector_2_node");
    ros::NodeHandle node;

    ros::Subscriber odomSub = node.subscribe<nav_msgs::Odometry>(odomTopic, 1, odomCb);
    ros::Subscriber obstaclesSub = node.subscribe<obstacle_detector::Obstacles>(obstaclesTopic, 1, obstaclesCb);
    resPub = node.advertise<sensor_msgs::PointCloud>(resTopic, 1);

    ros::spin();
    return 0;
}