#ifndef PROJECT_MOVEMENT_H
#define PROJECT_MOVEMENT_H

#include "ros/ros.h"
#include <koresh/NewTwist.h>
#include <koresh/ArduOdom.h>

#include "MovementController.h"

#include <string>
#include <cmath>
#include <functional>
#include <vector>
#include <sstream>
//#include <string.h>
//#include <boost/format.h>

using namespace std;

class Movement {
public:
    static Movement* Instance();
    bool init(ros::NodeHandle * _node, string arduTwistTopic = (string)"ardu_twist", string arduOdomTopic = (string)"ardu_odom", double _angleVel = 3.14/8);
    void move(double linear, double angular, double orient);
    void move(koresh::NewTwist msg);
    void registerMovementController(MovementController * controller);

private:
    Movement() {};
    ~Movement() {};
    static Movement * s_pInstance;

    ros::Publisher pub;
    ros::Subscriber sub;

    double angleVel;
//    double speed;

//    void (Movement::* f)(koresh::ArduOdom::ConstPtr & odom);

    vector<MovementController *> movementControllers;

    static void movementControllerCallback(double linear, double angular, double orient);

    ros::NodeHandle * node;
    void odometryCallback(const koresh::ArduOdom::ConstPtr & odom);
};

#endif //PROJECT_MOVEMENT_H
