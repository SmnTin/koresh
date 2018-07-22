#include "MovingObstaclesDetector.h"

MovingObstaclesDetector * MovingObstaclesDetector::s_pInstance = 0;

MovingObstaclesDetector * MovingObstaclesDetector::Instance() {
    if(s_pInstance == 0)
        s_pInstance = new MovingObstaclesDetector();
    return s_pInstance;
}

bool MovingObstaclesDetector::init(ros::NodeHandle * _node) {
    node = _node;
    scanSubscriber = node->subscribe<sensor_msgs::LaserScan>(scanTopic, 1, &MovingObstaclesDetector::scanCb, this);
    odomSubscriber = node->subscribe<nav_msgs::Odometry>(odomTopic, 1, &MovingObstaclesDetector::odomCb, this);
    resPublisher = node->advertise<sensor_msgs::PointCloud>(resTopic,1);
//    resPublisher = node->advertise<koresh::Obstacles>(resTopic,1);
    return true;
}

void MovingObstaclesDetector::odomCb(const nav_msgs::Odometry::ConstPtr &_odom) {
    odom = _odom;
}

void MovingObstaclesDetector::scanCb(const sensor_msgs::LaserScan::ConstPtr &_scan) {
    scan = _scan;
    if(odom == 0)
        return;

//    tf::StampedTransform transform;
//    try{
//        listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
//    }
//    catch (tf::TransformException &ex) {
//        ROS_ERROR("%s",ex.what());
//        ros::Duration(1.0).sleep();
//        return;
//    }

    //removing old
    vector<vector<Body>::iterator> bodiesToBeRemoved;
    for(vector<Body>::iterator t = prevFrameBodies.begin(); t != prevFrameBodies.end(); t++)
        if((ros::Time::now() - t->stamp).toSec() > bodyLifetime )
            bodiesToBeRemoved.push_back(t);
    for(vector<vector<Body>::iterator>::iterator t = bodiesToBeRemoved.begin(); t != bodiesToBeRemoved.end(); t++)
        prevFrameBodies.erase(*t);


    //find object
    vector<Body> thisFrameBodies;

//    bool isBody = false;
//    Point prevP (cos(scan->angle_min) * scan->ranges[0], sin(scan->angle_min) * scan->ranges[0]);
//    Body newBody;

//    for(int i = 0; i < (int)scan->ranges.size(); i++) {
//        if(scan->intensities[i] > 0.0 && scan->ranges[i] > 0.0) {
//            double ang = scan->angle_min + scan->angle_increment * i;
//            Point p (cos(ang) * scan->ranges[i], sin(ang) * scan->ranges[i]);
//
//            if((p - prevP).vecLength() <= oneBodyThreshold) {
//                if(p.x < newBody.minPoint.x)
//                    newBody.minPoint.x = p.x;
//                if(p.y < newBody.minPoint.y)
//                    newBody.minPoint.y = p.y;
//                if(p.x > newBody.maxPoint.x)
//                    newBody.maxPoint.x = p.x;
//                if(p.y > newBody.maxPoint.y)
//                    newBody.maxPoint.y = p.y;
////                newBody.mediumPoint += p;
//                newBody.mediumAngle += ang;
//                newBody.points.push_back(p);
//            }
//            else {
//                newBody.mediumAngle /= (int)newBody.points.size();
//                newBody.mediumPoint = (newBody.minPoint + newBody.maxPoint) / 2;
//                if((int)newBody.points.size() >= minPointsNumToBeBody && (int)newBody.points.size() <= maxPointsNumToBeBody)
//                    thisFrameBodies.push_back(newBody);
//
//
//                newBody = Body();
//                newBody.minPoint = p;
//                newBody.maxPoint = p;
//                newBody.mediumPoint = p;
//                newBody.mediumAngle = ang;
//                newBody.points.push_back(p);
//                newBody.stamp = ros::Time::now();
//            }
//            prevP = p;
//        }
//    }
//
    vector<Point> points;
    for(int i = 0; i < (int)scan->ranges.size(); i++) {
        if (scan->intensities[i] > 0.0 && scan->ranges[i] > 0.0) {
            double ang = scan->angle_min + scan->angle_increment * i;
            Point p(cos(ang) * scan->ranges[i], sin(ang) * scan->ranges[i]);
            double minDist = 100000;
            double minDistJ = -1;
            for(int j = 0; j < (int)thisFrameBodies.size(); j++) {
                double dist = (p - thisFrameBodies[j].points.back()).vecLength();
                if(dist <= oneBodyThreshold && dist < minDist) {
                    minDistJ = j;
                }
            }

            if(minDistJ > -1) {
                thisFrameBodies[minDistJ].points.push_back(p);
                if (p.x < thisFrameBodies[minDistJ].minPoint.x)
                    thisFrameBodies[minDistJ].minPoint.x = p.x;
                if (p.y < thisFrameBodies[minDistJ].minPoint.y)
                    thisFrameBodies[minDistJ].minPoint.y = p.y;
                if (p.x > thisFrameBodies[minDistJ].maxPoint.x)
                    thisFrameBodies[minDistJ].maxPoint.x = p.x;
                if (p.y > thisFrameBodies[minDistJ].maxPoint.y)
                    thisFrameBodies[minDistJ].maxPoint.y = p.y;
                thisFrameBodies[minDistJ].mediumAngle += ang;
            }
            else {
                Body newBody;
                newBody.minPoint = p;
                newBody.maxPoint = p;
                newBody.mediumPoint = p;
                newBody.mediumAngle = ang;
                newBody.points.push_back(p);
                newBody.vxf.setState(0, 0.1);
                newBody.vyf.setState(0, 0.1);
                newBody.stamp = ros::Time::now();
                thisFrameBodies.push_back(newBody);
            }
        }
    }

    //forming result and brute-forcing the current and the previous frames to determine velocities
//    koresh::Obstacles res;
    sensor_msgs::PointCloud res;
    res.header.stamp = scan->header.stamp;
    res.header.seq = seq++;
    res.header.frame_id = laserFrameId;

    res.channels.push_back(sensor_msgs::ChannelFloat32());
    res.channels[0].name = "moving";
    res.channels.push_back(sensor_msgs::ChannelFloat32());
    res.channels[1].name = "velocity";

    vector<Body> filteredThisFrameBodies;
    for(int i = 0; i < (int)thisFrameBodies.size(); i++) {
        bool moving = false;

        if((int)thisFrameBodies[i].points.size() < minPointsNumToBeBody || (int)thisFrameBodies[i].points.size() > maxPointsNumToBeBody)
            continue;

        filteredThisFrameBodies.push_back(thisFrameBodies[i]);

        thisFrameBodies[i].mediumAngle /= (int)thisFrameBodies[i].points.size();
        thisFrameBodies[i].mediumPoint = (thisFrameBodies[i].minPoint + thisFrameBodies[i].maxPoint) / 2;

        double w = odom->twist.twist.angular.z;
        double r = thisFrameBodies[i].mediumPoint.vecLength();
        double ang = thisFrameBodies[i].mediumAngle;

        Point va (odom->twist.twist.linear.x, odom->twist.twist.linear.y);
        Point vb (-w * r * cos(ang + M_PI / 2), -w * r * sin(ang + M_PI / 2));
        Point staticVelocity = va + vb;
//        cout << staticVelocity.vecLength() << endl;

        double minDist = 100000;
        double minDistJ = -1;
        for(int j = 0; j < (int)prevFrameBodies.size(); j++) {
            double dist = (thisFrameBodies[i].mediumPoint - prevFrameBodies[j].mediumPoint).vecLength();
//            cout << "d " << dist << endl;
            if (dist <= anotherBodyThreshold && dist < minDist) {
                minDistJ = j;
                minDist = dist;
            }
        }
        if(minDistJ > -1) {
            thisFrameBodies[i].vxf = prevFrameBodies[minDistJ].vxf;
            thisFrameBodies[i].vyf = prevFrameBodies[minDistJ].vyf;
            Point bodyVelocity = (thisFrameBodies[i].mediumPoint - prevFrameBodies[minDistJ].mediumPoint) / (thisFrameBodies[i].stamp - prevFrameBodies[minDistJ].stamp).toSec();
//            Point bodyVelocity = thisFrameBodies[i].velocity = prevFrameBodies[minDistJ].velocity * filterSize + (thisFrameBodies[i].mediumPoint - prevFrameBodies[minDistJ].mediumPoint) * (1 - filterSize) / (thisFrameBodies[i].stamp - prevFrameBodies[minDistJ].stamp).toSec();
            thisFrameBodies[i].vxf.correct(bodyVelocity.x);
            thisFrameBodies[i].vyf.correct(bodyVelocity.y);

            thisFrameBodies[i].velocity.x = thisFrameBodies[i].vxf.State;
            thisFrameBodies[i].velocity.y = thisFrameBodies[i].vyf.State;
            cout << thisFrameBodies[i].velocity.vecLength() << endl;
            if ((thisFrameBodies[i].velocity - staticVelocity).vecLength()  > movingObstacleVelocityThreshold)
                moving = true;
        }

        if(moving)
            ROS_INFO("MOVING BITCH!!!");

//        res.moving.push_back(moving);
//        res.ranges.push_back(r);
//        res.angles.push_back(ang);
        geometry_msgs::Point32 curP;
        curP.x = thisFrameBodies[i].mediumPoint.x;
        curP.y = thisFrameBodies[i].mediumPoint.y;
        curP.z = 0;

        res.points.push_back(curP);
        res.channels[0].values.push_back((int)moving * 100);
        res.channels[1].values.push_back(thisFrameBodies[i].velocity.vecLength());
    }
    prevFrameBodies = filteredThisFrameBodies;

    resPublisher.publish(res);
}