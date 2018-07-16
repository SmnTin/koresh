#ifndef PROJECT_LINETYPE_H
#define PROJECT_LINETYPE_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/utils/trace.hpp"
#include "opencv2/core/core.hpp"

#include <vector>
#include <cmath>
#include <iostream>
//#include <stdlib>

#include "Line.h"

using namespace std;

class LineType {
public:
    cv::Scalar lb, ub;

    //-1 if no color conversion needed (leave BGR)
    int colorConversionFlag;
    virtual void processResults(const vector<Line> & lines, cv::Mat drawing) {}
};

#endif //PROJECT_LINETYPE_H
