#ifndef PROJECT_LINE_H
#define PROJECT_LINE_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>

using namespace std;

struct Line {
    vector<cv::Point> path;
    bool discontinuous = false;
};

#endif //PROJECT_LINE_H
