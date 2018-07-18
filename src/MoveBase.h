#ifndef PROJECT_MOVEBASE_H
#define PROJECT_MOVEBASE_H

#include "MovementController.h"

#include "geometry_msgs/Twist.h"

class MoveBase : public MovementController {
public:
    bool init(ros::NodeHandle * _node, void (*_cb)(double linear, double angular, double orient));

private:
    void (*cb)(double linear, double angular, double orient);

    ros::NodeHandle * node;

    ros::Subscriber sub;
    void twistHandler(const geometry_msgs::Twist::ConstPtr & msg);
};

#endif //PROJECT_MOVEBASE_H
