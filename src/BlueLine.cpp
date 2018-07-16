#include "BlueLine.h"

#include "LineDetector.h"

BlueLine::BlueLine() {
    LineType::lb = cv::Scalar(50,150,140);
    LineType::ub = cv::Scalar(255,255,200);
//    LineType::channel = 1;
    LineType::colorConversionFlag = cv::COLOR_BGR2Lab;
    videoWriter.open("/work/catkin_ws/src/koresh/assets/output1.mpeg",
                     CV_FOURCC('P','I','M','1'),
                     30, cv::Size(672,376));
    if(!videoWriter.isOpened())
        ROS_ERROR("Blue line: Could not open video writer.");
    //LineType::lb = cv::Scalar(0, 0, 200);
    //LineType::ub = cv::Scalar(255, 30, 255);
//    cout << "CALLED" << endl;
}

void BlueLine::processResults(const vector <Line> &lines, cv::Mat drawing) {
    cv::Mat bin = cv::Mat::zeros(drawing.size(), CV_8U);
    vector<Line> linesCorrected = LineDetector::Instance()->transformToTheTopView(lines);
    for(int i = 0; i < (int)lines.size(); i++)
    {
        cv::Scalar color (min(i*50, 255), 255, 255);
//        cv::cvtColor(color, color, cv::COLOR_HSV2BGR);
        for(int j = 0; j < (int)lines[i].path.size(); j++)
        {
            cv::circle (drawing, lines[i].path[j], 5, color, 4, 8, 0);
            cv::circle (bin, linesCorrected[i].path[j], 5, cv::Scalar(255), 4, 8, 0);
            if(j != 0) {
                cv::line(drawing, lines[i].path[j], lines[i].path[j - 1], color, 2);
                cv::line(bin, linesCorrected[i].path[j], linesCorrected[i].path[j - 1], cv::Scalar(255), 8);
            }
        }
    }

    LineDetector::Instance()->convertToOccupancyGridAndPublish(bin);

    if(videoWriter.isOpened())
        videoWriter.write(drawing);
    imshow("redLine", drawing);
    imshow("redLineBin", bin);
}

BlueLine::~BlueLine() {
    videoWriter.release();
}
