//
// Created by smntin on 08.07.18.
//

#ifndef PROJECT_MOVEMENTCONTROLLER_H
#define PROJECT_MOVEMENTCONTROLLER_H

#include "ros/ros.h"

class MovementController {
public:
    virtual bool init(ros::NodeHandle * node, void (*cb)(double linear, double angular, double orient)) {};
//    virtual void directMove(double dirX, double dirY, double speed) {};
};

#endif //PROJECT_MOVEMENTCONTROLLER_H
