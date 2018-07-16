#ifndef PROJECT_LINEDETECTOR_H
#define PROJECT_LINEDETECTOR_H

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdlib.h>

#include <std_msgs/String.h>

#include <ros/ros.h>
#include <ros/console.h>
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

    bool captureOn = false; //If OpenCV camera on

    void convertToOccupancyGridAndPublish(const cv::Mat & img);
    //ros::Publisher occupancyGridPublisher;

    cv::Mat openWarpPerspective(const cv::Mat& _image
            , const cv::Point2f& _lu
            , const cv::Point2f& _ru
            , const cv::Point2f& _rd
            , const cv::Point2f& _ld
            , const cv::Point2f& _lu_result
            , const cv::Point2f& _ru_result
            , const cv::Point2f& _rd_result
            , const cv::Point2f& _ld_result);

    vector<Line> transformToTheTopView(const vector<Line> & src);

private:

    static LineDetector * s_pInstance;
    LineDetector() {};
    ~LineDetector() {};

    ros::NodeHandle * node;

    string imageTopic = (string)"/zed/rgb/image_rect_color";
    string odometryTopic = (string)"/zed/odom";
    string occupancyGridTopicToSubscribe = (string)"/rtabmap/grid_map";
    string occupancyGridTopicToPublish = (string)"new_grid_map";

    double perspectiveTransformCoef = 0.325;
    int imageHeight = 376; //720
    int imageWidth = 672; //1280
    double realHeight = 2;
    double realWidth = 2;

    vector<LineType *> lineTypes;
    int erosion_size = 1;
    int bigRoiHeight = 120;
    int rowHeight = 20;

    struct sortByDistanceFromPoint {
        sortByDistanceFromPoint(cv::Point _p) : p(_p) {}
        cv::Point p;
        const bool operator() (const Line & a, const Line & b);
    };

    nav_msgs::OccupancyGrid::ConstPtr occupancyGrid;
    nav_msgs::Odometry::ConstPtr odometry;

    image_transport::ImageTransport * imgTransport;
    image_transport::Subscriber imgSubsriber;

    ros::Subscriber occupancyGridSubscriber;
    ros::Publisher occupancyGridPublisher;
    ros::Subscriber odometrySubscriber;

    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void occupancyGridCb(const nav_msgs::OccupancyGrid::ConstPtr& grid);
    void odometryCb(const nav_msgs::Odometry::ConstPtr& odom);

    cv::Point2f imgToLocalSpace(const cv::Point2f& point);
    cv::Point2f rotateVector(const cv::Point2f& v, double r);

    cv::VideoCapture capture;
    cv::Mat originalMat;
};

#endif //PROJECT_LINEDETECTOR_H
