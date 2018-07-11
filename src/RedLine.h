#ifndef PROJECT_REDLINE_H
#define PROJECT_REDLINE_H

#include "LineType.h"

class RedLine : public LineType {
public:
    RedLine();
    void processResults(const vector<Line> & lines, cv::Mat drawing);
};

#endif //PROJECT_REDLINE_H
