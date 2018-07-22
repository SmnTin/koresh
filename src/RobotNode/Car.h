#ifndef PROJECT_CAR_H
#define PROJECT_CAR_H

#include "ros/ros.h"
#include <koresh/NewTwist.h>
#include <koresh/ArduOdom.h>

#include "../Callback.h"

#include <map>
#include <string>
#include <iostream>

using namespace std;

class Car
{
public:
    static Car * Instance();
    bool init(ros::NodeHandle * _node);
    void update();
    void applyChanges();

    bool isRunning() { return running; }
    bool quit() { running = false; }
//
//    map<string, ros::Subscriber> subscribers;
//    map<string, ros::Publisher> publishers;

//    ros::NodeHandle node;

private:
    static Car* s_pInstance;
    Car() {};
    ~Car() {};

    ros::NodeHandle * node;

    bool running = false;
};

#endif //PROJECT_CAR_H
