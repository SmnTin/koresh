#ifndef PROJECT_LINEDETECTOR_H
#define PROJECT_LINEDETECTOR_H

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include "LineType.h"

using namespace std;

class LineDetector {
public:
    static LineDetector * Instance();

    bool init(ros::NodeHandle * _node);
    void detect();
    void registerLineType(LineType * lineType);
    void clean();

    // img = binary mat
    void convertToOccupancyGridAndPublish(const cv::Mat & img);

private:
    static LineDetector * s_pInstance;
    LineDetector() {};
    ~LineDetector() {};

    ros::NodeHandle * node;

    string imageTopic = (string)"a";
    string odometryTopic = (string)"b";
    string occupancyGridTopicToSubscribe = (string)"c";
    string occupancyGridTopicToPublish = (string)"d";

    int imageHeight = 720;
    int imageWidth = 1280;
    double realHeight = 2;
    double realWidth = 2;

    vector<LineType *> lineTypes;
    int erosion_size = 2;
    int bigRoiHeight = 480;
    int rowHeight = 50;

    struct sortByDistanceFromPoint {
        sortByDistanceFromPoint(cv::Point _p) : p(_p) {}
        cv::Point p;
        const bool operator() (const Line & a, const Line & b);
    };

    nav_msgs::OccupancyGrid::ConstPtr occupancyGrid;
    nav_msgs::Odometry::ConstPtr odometry;

    ros::Subscriber occupancyGridSubscriber;
    ros::Publisher occupancyGridPublisher;
    ros::Subscriber odometrySubscriber;

    void occupancyGridCb(const nav_msgs::OccupancyGrid::ConstPtr& grid);
    void odometryCb(const nav_msgs::Odometry::ConstPtr& odom);

    cv::Point2f imgToLocalSpace(const cv::Point2f& point);
    cv::Point2f rotateVector(const cv::Point2f& v, double r);

    cv::VideoCapture capture;
};

#endif //PROJECT_LINEDETECTOR_H
