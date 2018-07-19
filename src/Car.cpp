#include "Car.h"

#include "Movement.h"

#include "Joy.h"
#include "MoveBase.h"

Car* Car::s_pInstance = 0;

Car* Car::Instance() {
    if(s_pInstance == 0)
        s_pInstance = new Car();
    return s_pInstance;
}

bool Car::init(ros::NodeHandle * _node) {
    node = _node;
    running = true;
    if(!Movement::Instance()->init(node))
    {
        ROS_ERROR("Could not init Movement manager");
        return false;
    }
    Movement::Instance()->registerMovementController(new Joy());
    Movement::Instance()->registerMovementController(new MoveBase());

    return true;
}

void Car::update() {

}

void Car::applyChanges() {

}