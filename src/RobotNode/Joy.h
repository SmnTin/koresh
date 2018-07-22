#ifndef PROJECT_JOY_H
#define PROJECT_JOY_H

#include "MovementController.h"

#include "koresh/NewTwist.h"

class Joy : public MovementController {
public:
    bool init(ros::NodeHandle * _node, void (*_cb)(double linear, double angular, double orient));

private:
    void (*cb)(double linear, double angular, double orient);

    ros::NodeHandle * node;

    ros::Subscriber sub;
    void twistHandler(const koresh::NewTwist::ConstPtr & msg);
};

#endif //PROJECT_JOY_H
