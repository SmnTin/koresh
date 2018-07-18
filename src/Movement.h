#ifndef PROJECT_MOVEMENT_H
#define PROJECT_MOVEMENT_H

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

#include <koresh/NewTwist.h>
#include <koresh/ArduOdom.h>
#include <nav_msgs/Odometry.h>

#include "MovementController.h"

#include <string>
#include <cmath>
#include <functional>
#include <vector>
#include <sstream>
//#include <string.h>
//#include <boost/format.h>
#include <boost/assign.hpp>

using namespace std;

class Movement {
public:
    static Movement* Instance();
    bool init(ros::NodeHandle * _node, string arduTwistTopic = (string)"ardu_twist", string arduOdomTopic = (string)"ardu_odom", string odomTopic = (string) "sensor/odom", double _angleVel = 3.14/8);
    void move(double linear, double angular, double orient);
    void move(koresh::NewTwist msg);
    void registerMovementController(MovementController * controller);

private:
    Movement() {};
    ~Movement() {};
    static Movement * s_pInstance;

    ros::Publisher cmdPub;
    ros::Subscriber odomSub;
    ros::Publisher odomPub;

    ros::Time previousOdomTime;

    tf::TransformBroadcaster odomBroadcaster;

    double angleVel;

    double vx = 0.0, vy = 0.0, vth = 0.0;

    double x = 0.0, y = 0.0, th = 0.0;

    const double rwheel = 0.05;
    const double lx = 0.245;
    const double ly = 0.1925;
//    double speed;

//    void (Movement::* f)(koresh::ArduOdom::ConstPtr & odom);

    vector<MovementController *> movementControllers;

    static void movementControllerCallback(double linear, double angular, double orient);

    ros::NodeHandle * node;
    void odometryCallback(const koresh::ArduOdom::ConstPtr & odom);
};

#endif //PROJECT_MOVEMENT_H
