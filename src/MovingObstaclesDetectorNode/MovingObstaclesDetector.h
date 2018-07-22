#ifndef PROJECT_MOVINGOBSTACLESDETECTOR_H
#define PROJECT_MOVINGOBSTACLESDETECTOR_H

#define _USE_MATH_DEFINES

#include "ros/ros.h"
//#include "tf/TransformListener.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/Odometry.h"
#include "koresh/Obstacles.h"

#include <vector>
#include <utility>
#include <math.h>

#include "../Point.h"
#include "Kalman.h"

using namespace std;

struct Body {
    vector<Point> points;
    Point mediumPoint;
    Point minPoint = Point(100000,100000);
    Point maxPoint = Point(-100000,-100000);
    double mediumAngle = 0;
    KalmanFilterSimple1D vxf, vyf;
    Point velocity;
    ros::Time stamp;
};

class MovingObstaclesDetector {
public:
    static MovingObstaclesDetector * Instance();
    bool init(ros::NodeHandle * _node);

private:
    MovingObstaclesDetector() {};
    ~MovingObstaclesDetector() {};
    static MovingObstaclesDetector * s_pInstance;

    ros::NodeHandle * node;

    string scanTopic = (string)"/scan";
    string odomTopic = (string)"/sensor/odom";
    string resTopic  = (string)"/moving_obstacles";

    string laserFrameId = (string)"laser";

    sensor_msgs::LaserScan::ConstPtr scan;
    nav_msgs::Odometry::ConstPtr odom;

    ros::Subscriber scanSubscriber;
    ros::Subscriber odomSubscriber;
    ros::Publisher  resPublisher;

//    tf::TransformListener transformListener;

    double oneBodyThreshold = 0.3;
    double anotherBodyThreshold = 0.3;
    double bodyLifetime = 0.5;
    double movingObstacleVelocityThreshold = 0.2;
    int    minPointsNumToBeBody = 10;
    int    maxPointsNumToBeBody = 20;

    double filterSize = 0.92;

    int seq;

    vector<Body> prevFrameBodies;

    void scanCb(const sensor_msgs::LaserScan::ConstPtr & _scan);
    void odomCb(const nav_msgs::Odometry::ConstPtr & _odom);
};

#endif
