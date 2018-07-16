#ifndef PROJECT_REDLINE_H
#define PROJECT_REDLINE_H

#include "LineType.h"

class BlueLine : public LineType {
public:
    BlueLine();
    void processResults(const vector<Line> & lines, cv::Mat drawing);
    ~BlueLine();

private:
    cv::VideoWriter videoWriter;
};

#endif //PROJECT_REDLINE_H
