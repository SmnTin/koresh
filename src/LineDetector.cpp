#include "LineDetector.h"

LineDetector * LineDetector::s_pInstance = 0;

LineDetector * LineDetector::Instance() {
    if(s_pInstance == 0)
        s_pInstance = new LineDetector();
    return s_pInstance;
}

bool LineDetector::init(ros::NodeHandle * _node) {

    node = _node;

    if(captureOn) {
        if(!capture.open("/work/catkin_ws/src/koresh/assets/Lol.mp4"))
            return false;
        capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    }
    else {
        imgTransport = new image_transport::ImageTransport(*node);
        imgSubsriber = imgTransport->subscribe(imageTopic, 1, &LineDetector::imageCb, this);
    }

    occupancyGridSubscriber = node->subscribe<nav_msgs::OccupancyGrid>(occupancyGridTopicToSubscribe, 1, &LineDetector::occupancyGridCb, this);
    odometrySubscriber = node->subscribe<nav_msgs::Odometry>(odometryTopic, 1, &LineDetector::odometryCb, this);
    occupancyGridPublisher = node->advertise<nav_msgs::OccupancyGrid>(occupancyGridTopicToPublish, 1); //node->advertise<nav_msgs::OccupancyGrid>(occupancyGridTopicToPublish, 1); node->advertise<std_msgs::String>(occupancyGridTopicToPublish, 1)

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
    if (captureOn) {
        capture >> originalMat;
    }
    cv::Mat src = originalMat.clone();
    cv::Mat drawing = src.clone();

    cv::imshow("original", originalMat);

    cv::Rect bigRoiRect (0,src.rows - bigRoiHeight, src.cols, bigRoiHeight);
    cv::Mat bigRoi = src(bigRoiRect).clone();
//    cv::Mat bigRoiUnwarped = src(bigRoiRect).clone();

//    cv::Mat bigRoi = openWarpPerspective(bigRoiUnwarped, cv::Point(bigRoiRect.width*(perspectiveTransformCoef),0), cv::Point(bigRoiRect.width*(1-perspectiveTransformCoef), 0), cv::Point(bigRoiRect.width, bigRoiRect.height), cv::Point(0, bigRoiRect.height), cv::Point(0,0), cv::Point(bigRoiRect.width, 0), cv::Point(bigRoiRect.width, bigRoiRect.height), cv::Point(0, bigRoiRect.height))(cv::Rect(0,0,bigRoiRect.width,bigRoiRect.height));

//    for(int i = 0; i < bigRoiHeight; i++)
//        bigRoi.row(i).copyTo(drawing.row(src.rows - bigRoiHeight + i));
    //cv::Mat colorSpace;
    //cv::cvtColor(bigRoi, colorSpace, CV_BGR2Lab);
    //std::vector<cv::Mat> channels;
    //cv::split(lab, channels);*/
    //cv::Mat hsv;
    //cv::cvtColor(bigRoi, hsv, cv::COLOR_BGR2HSV);



    /*vector<cv::Mat> hsvPlanes(3);
    cv::split(hsv, hsvPlanes);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(25);
    cv::Mat dst;
    clahe->apply(hsvPlanes[2], dst);
    //imshow("clahe", hsvPlanes[2]);
    dst.copyTo(hsvPlanes[2]);
    //cv::merge(hsvPlanes, hsv);*/

    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),cv::Point(erosion_size, erosion_size) );
    for(int t = 0; t < (int)lineTypes.size(); t++) {
        LineType * lineType = lineTypes[t];

        cv::Mat currentDrawing = drawing.clone();

        cv::Mat colorSpace;
        if(lineType->colorConversionFlag >= 0)
            cv::cvtColor(bigRoi, colorSpace, lineType->colorConversionFlag);

        cv::imshow("A", colorSpace);

        cv::Mat thr;
        cv::Scalar lb = lineType->lb, ub = lineType->ub;

        vector<Line> lines;

        cv::GaussianBlur(colorSpace, colorSpace, cv::Size( 9, 9 ), 0, 0 );
    	cv::inRange(colorSpace,lb,ub,thr);
        cv::erode(thr,thr,element);
        cv::dilate(thr,thr,element);
        cv::dilate(thr,thr,element);
        cv::erode(thr,thr,element);

    	cv::imshow("thr", thr);
        cv::Mat masked;
        colorSpace.copyTo(masked,thr);
        cv::Mat edges;
        cv::Canny(masked, edges, 0, 50, 3);
        //cv::imshow("canny", edges);
    	for(int k = 1; k <= bigRoiHeight / rowHeight; k++) {
            cv::Rect roiRect ( 0, bigRoiHeight - k*rowHeight, bigRoi.cols, rowHeight);
            cv::Mat roi = thr(roiRect);

            vector<vector<cv::Point> > contours;
            vector<cv::Vec4i> hierarchy;
            vector<cv::Rect> bounds;
            cv::findContours(roi, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
            for(int i = 0; i < (int)contours.size(); i++) {
                cv::Rect bound = cv::boundingRect(contours[i]);
                if(bound.width > bigRoi.cols / 15 && bound.width < bigRoi.cols / 10 && bound.height > rowHeight - 10) {} //PEREDELAT
                cv::Point center (bound.x + bound.width/2, bound.y + bound.height/2 + src.rows - k*rowHeight);
                sort(lines.begin(), lines.end(), sortByDistanceFromPoint(center));
                if(!lines.empty())
                {
                    Line & nearestLine = lines.front();
                    cv::Point & nearestPoint = nearestLine.path.back();
                    double dist = sqrt((center.x - nearestPoint.x) * (center.x - nearestPoint.x) + (center.y - nearestPoint.y) * (center.y - nearestPoint.y));
                    if(dist <= 2*rowHeight)
                        nearestLine.path.push_back(center);
                    else if(dist <= 4*rowHeight) {
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

        lineType->processResults(lines, currentDrawing);
    }
}

cv::Point2f LineDetector::imgToLocalSpace(const cv::Point2f& point) {
    float x = (imageHeight - point.y) / imageHeight * realHeight;
    float y = (point.x - (imageWidth / 2)) / imageWidth * realWidth;
    return cv::Point2f(x, y);
}

cv::Point2f LineDetector::rotateVector(const cv::Point2f& v, double r){
    double ca = cos(r);
    double sa = sin(r);
    return cv::Point2f(ca*v.x - sa*v.y, sa*v.x + ca*v.y);
}

void LineDetector::imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImageConstPtr cvImg;
    try
    {
        if (sensor_msgs::image_encodings::isColor(msg->encoding))
            cvImg = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        else
            cvImg = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cvImg->image.copyTo(originalMat);
    imageHeight = originalMat.rows;
    imageWidth = originalMat.cols;
    detect();
}

void LineDetector::odometryCb(const nav_msgs::Odometry::ConstPtr &odom) {
    odometry = odom;
    ROS_INFO("subOdom");
}

void LineDetector::occupancyGridCb(const nav_msgs::OccupancyGrid::ConstPtr &grid) {
    occupancyGrid = grid;

    /*std_msgs::String msg;
    std::stringstream ss;
    ss << kek
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());*/

    /*std_msgs::String kek;
    kek.data = occupancyGrid->header.frame_id;
    ROS_INFO("%s", kek.data.c_str());*/
    ROS_INFO("subGrid");
}

void LineDetector::convertToOccupancyGridAndPublish(const cv::Mat &img) {
    if(occupancyGrid == 0 || odometry == 0)
        return;
    nav_msgs::OccupancyGrid msg;

    /*std_msgs::String kek;
    kek.data = occupancyGrid->header.frame_id;
    ROS_INFO("%s", kek.data.c_str());*/

    const int gw = occupancyGrid->info.width;
    const int gh = occupancyGrid->info.height;
    const float resolution = occupancyGrid->info.resolution;

    msg.info.height = gh;
    msg.info.width = gw;
    msg.info.resolution = resolution;
    msg.info.origin = occupancyGrid->info.origin;
    msg.header = occupancyGrid->header;
    msg.data = occupancyGrid->data;
    for (int i = 0; i < gh * gw; ++i){
        msg.data[i] = -1;
    }

    for(int y = 0; y < img.rows; y++) {
        for(int x = 0; x < img.cols; x++) {
            if(img.at<unsigned char>(cv::Point(x,y)) > 0) {
//                cout << "HERE" << endl;
//                cout << ((int)img.at<unsigned char>(cv::Point(x, y))) << endl;
               // ROS_ERROR("cv_bridge exception: %s");
                cv::Point2f point(x,y);

                cv::Point2f position = imgToLocalSpace(point);

                cout << "px: " << position.x << ", py: " << position.y << endl;

                double yaw = tf::getYaw(odometry->pose.pose.orientation);

                position = rotateVector(position, yaw);

                cout << "px: " << position.x << ", py: " << position.y << endl;

                position += cv::Point2f(odometry->pose.pose.position.x, odometry->pose.pose.position.y);

                cout << "px: " << position.x << ", py: " << position.y << endl;
                cout << "ox: " << odometry->pose.pose.position.x << ", oy: " << odometry->pose.pose.position.y << endl;

                position += cv::Point2f(msg.info.origin.position.x, msg.info.origin.position.y);

                cout << "orx: " << msg.info.origin.position.x << ", ory: " << msg.info.origin.position.y << endl;

                cout << "px: " << position.x << ", py: " << position.y << endl;

                int py = (std::round(position.y / resolution));

                int px = (std::round(position.x / resolution));

                cout << "x: " << px << ", y: " << py << ", res: " << resolution << ", gw: " << gw << ", gh: " << gh << endl;

                //ROS_INFO("NOprepiatstvie");
                if (py < 0 || py >= gw || px < 0 || px >= gh)
                    continue;

                ROS_INFO("prepiatstvie");
//                msg.data[py + px * gw] = 100;
            }
        }
    }
    ROS_INFO("I send this ditch");
    occupancyGridPublisher.publish(msg);
}

cv::Mat LineDetector::openWarpPerspective(const cv::Mat& _image
        , const cv::Point2f& _lu
        , const cv::Point2f& _ru
        , const cv::Point2f& _rd
        , const cv::Point2f& _ld
        , const cv::Point2f& _lu_result
        , const cv::Point2f& _ru_result
        , const cv::Point2f& _rd_result
        , const cv::Point2f& _ld_result)
{

    cv::Point2f source_points[4];
    cv::Point2f dest_points[4];


    source_points[0] = _lu;
    source_points[1] = _ru;
    source_points[2] = _rd;
    source_points[3] = _ld;

    dest_points[0] = _lu_result;
    dest_points[1] = _ru_result;
    dest_points[2] = _rd_result;
    dest_points[3] = _ld_result;

    cv::Mat dst;
    cv::Mat _transform_matrix = cv::getPerspectiveTransform(source_points, dest_points);
    cv::warpPerspective(_image, dst, _transform_matrix, cv::Size(_image.cols, _image.cols));

    return dst;
}

vector<Line> LineDetector::transformToTheTopView(const vector <Line> &src) {
    vector<Line> res;
    for(int i = 0; i < (int)src.size(); i++){
        Line newLine;
        for(int j = 0; j < (int)src[i].path.size(); j++) {
            double Rx = src[i].path[j].x;
            double Ry = src[i].path[j].y - imageHeight + bigRoiHeight;
            double z = bigRoiHeight / (bigRoiHeight * perspectiveTransformCoef + Ry * (1 - perspectiveTransformCoef));
//            cout << z << endl;
            newLine.path.push_back(cv::Point(
                    (Rx - imageWidth / 2) * z + imageWidth / 2, Ry + imageHeight - bigRoiHeight));

        }
        res.push_back(newLine);
    }
    return res;
}

void LineDetector::clean() {
    capture.release();
}