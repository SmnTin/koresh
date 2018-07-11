#ifndef PROJECT_LINEDETECTOR_H
#define PROJECT_LINEDETECTOR_H

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

#include "LineType.h"

using namespace std;

class LineDetector {
public:
    static LineDetector * Instance();

    bool init(string imageTopic = (string)"");
    void detect();
    void registerLineType(LineType * lineType);
    void clean();

private:
    static LineDetector * s_pInstance;
    LineDetector() {};
    ~LineDetector() {};

    vector<LineType *> lineTypes;
    int erosion_size = 2;
    int bigRoiHeight = 480;
    int rowHeight = 50;

    struct sortByDistanceFromPoint {
        sortByDistanceFromPoint(cv::Point _p) : p(_p) {}
        cv::Point p;
        const bool operator() (const Line & a, const Line & b);
    };

    cv::VideoCapture capture;
};

#endif //PROJECT_LINEDETECTOR_H
