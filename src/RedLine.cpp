#include "RedLine.h"

RedLine::RedLine() {
    LineType::lb = cv::Scalar(0, 0, 200);
    LineType::ub = cv::Scalar(255, 30, 255);
//    cout << "CALLED" << endl;
}

void RedLine::processResults(const vector <Line> &lines, cv::Mat drawing) {
    for(int i = 0; i < (int)lines.size(); i++)
    {
        cv::Scalar color (min(i*50, 255), 255, 255);
//        cv::cvtColor(color, color, cv::COLOR_HSV2BGR);
        for(int j = 0; j < (int)lines[i].path.size(); j++)
        {
            cv::circle (drawing, lines[i].path[j], 5, color, 4, 8, 0);
            if(j != 0)
                cv::line(drawing,lines[i].path[j], lines[i].path[j-1],color,2);
        }
    }
    imshow("redLine", drawing);
}
