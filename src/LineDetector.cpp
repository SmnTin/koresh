#include "LineDetector.h"

LineDetector * LineDetector::s_pInstance = 0;

LineDetector * LineDetector::Instance() {
    if(s_pInstance == 0)
        s_pInstance = new LineDetector();
    return s_pInstance;
}

bool LineDetector::init(string imageTopic) {
    if(!capture.open("/work/catkin_ws/src/koresh/assets/Lol.mp4"))
        return false;
    capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    return true;
}

void LineDetector::registerLineType(LineType *lineType) {
    lineTypes.push_back(lineType);
}

const bool LineDetector::sortByDistanceFromPoint::operator()(const Line &a, const Line &b) {
    cv::Point pA = a.path.back();
    cv::Point pB = b.path.back();
    return sqrt((p.x - pA.x) * (p.x - pA.x) + (p.y - pA.y) * (p.y - pA.y)) < sqrt((p.x - pB.x) * (p.x - pB.x) + (p.y - pB.y) * (p.y - pB.y));
}

void LineDetector::detect() {
    cv::Mat src;
    capture >> src;
    cv::Rect bigRoiRect (0,src.rows - bigRoiHeight, src.cols, bigRoiHeight);
    cv::Mat bigRoi = src(bigRoiRect);
    cv::Mat hsv;
    cv::cvtColor(bigRoi, hsv, cv::COLOR_BGR2HSV);

    cv::Mat drawing = src.clone();

    vector<cv::Mat> hsvPlanes(3);
    cv::split(hsv, hsvPlanes);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(25);
    cv::Mat dst;
    clahe->apply(hsvPlanes[2], dst);
    imshow("clahe", hsvPlanes[2]);
    dst.copyTo(hsvPlanes[2]);
    cv::merge(hsvPlanes, hsv);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                            cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),cv::Point(erosion_size, erosion_size) );
    cv::GaussianBlur( hsv, hsv, cv::Size( 9, 9 ), 0, 0 );
    for(int t = 0; t < (int)lineTypes.size(); t++) {
        LineType * lineType = lineTypes[t];
        cv::Mat thr;
        cv::Scalar lb = lineType->lb, ub = lineType->ub;
//        cout << ub << endl;
        vector<Line> lines;

    	cv::inRange(hsv,lb,ub,thr);
        cv::erode(thr,thr,element);
        cv::dilate(thr,thr,element);
        cv::dilate(thr,thr,element);
        cv::erode(thr,thr,element);

    	cv::imshow("thr", thr);
        cv::Mat masked;
        hsv.copyTo(masked,thr);
        cv::Mat edges;
        cv::Canny(masked, edges, 0, 50, 3);
//        cv::imshow("canny", edges);
    	for(int k = 1; k <= bigRoiHeight / rowHeight; k++) {
            cv::Rect roiRect ( 0, bigRoiHeight - k*rowHeight, bigRoi.cols, rowHeight);
            cv::Mat roi = thr(roiRect);

            vector<vector<cv::Point> > contours;
            vector<cv::Vec4i> hierarchy;
            vector<cv::Rect> bounds;
            cv::findContours(roi, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
            for(int i = 0; i < (int)contours.size(); i++) {
                cv::Rect bound = cv::boundingRect(contours[i]);
                if(bound.width > bigRoi.cols / 15 && bound.width < bigRoi.cols / 10 && bound.height > rowHeight - 10) {}
                cv::Point center (bound.x + bound.width/2, bound.y + bound.height/2 + src.rows - k*rowHeight);
                sort(lines.begin(), lines.end(), sortByDistanceFromPoint(center));
                if(!lines.empty())
                {
                    Line & nearestLine = lines.front();
                    cv::Point & nearestPoint = nearestLine.path.back();
                    double dist = sqrt((center.x - nearestPoint.x) * (center.x - nearestPoint.x) + (center.y - nearestPoint.y) * (center.y - nearestPoint.y));
                    if(dist <= rowHeight)
                        nearestLine.path.push_back(center);
                    else if(dist <= 2*rowHeight) {
                        nearestLine.path.push_back(center);
                        nearestLine.discontinuous = true;
                    }
                    else
                    {
                        Line newLine;
                        newLine.path.push_back(center);
                        lines.push_back(newLine);
                    }
                }
                else
                {
                    Line newLine;
                    newLine.path.push_back(center);
                    lines.push_back(newLine);
                }
            }
        }

        lineType->processResults(lines, drawing);
    }
}

void LineDetector::clean() {
    capture.release();
}