#include "LineDetector.h"

LineDetector * LineDetector::s_pInstance = 0;

LineDetector * LineDetector::Instance() {
    if(s_pInstance == 0)
        s_pInstance = new LineDetector();
    return s_pInstance;
}

bool LineDetector::init(ros::NodeHandle * _node) {
    if(!capture.open("/work/catkin_ws/src/koresh/assets/Lol.mp4"))
        return false;
    capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);

    node = _node;

    occupancyGridSubscriber = node->subscribe<nav_msgs::OccupancyGrid>(occupancyGridTopicToSubscribe, 1, &LineDetector::occupancyGridCb, this);
    odometrySubscriber = node->subscribe<nav_msgs::Odometry>(odometryTopic, 1, &LineDetector::odometryCb, this);
    occupancyGridPublisher = node->advertise<nav_msgs::OccupancyGrid>(occupancyGridTopicToPublish, 1);

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
    //Берём кадр
    cv::Mat src;
    capture >> src;
    //Берём часть кадра, где есть разметка
    cv::Rect bigRoiRect (0,src.rows - bigRoiHeight, src.cols, bigRoiHeight);
    cv::Mat bigRoi = src(bigRoiRect);
    //Преобразуем в HSV
    cv::Mat hsv;
    cv::cvtColor(bigRoi, hsv, cv::COLOR_BGR2HSV);

//    vector<cv::Mat> hsvPlanes(3);
//    cv::split(hsv, hsvPlanes);
//    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
//    clahe->setClipLimit(50);
//    cv::Mat dst;
//    clahe->apply(hsvPlanes[2], dst);
//    dst.copyTo(hsvPlanes[2]);
//    cv::merge(hsvPlanes, hsv);

    //Создаём ядро морфологической обработки
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                            cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),cv::Point(erosion_size, erosion_size) );
    //Сглаживаем изображение размытием
    cv::GaussianBlur( hsv, hsv, cv::Size( 9, 9 ), 0, 0 );

    imshow("clahe", hsv);
    //Цикл по всем зарегистрированным типам линии
    for(int t = 0; t < (int)lineTypes.size(); t++) {
        LineType * lineType = lineTypes[t];
        cv::Mat thr;

        //Дополнительный Mat для отрисовки
        cv::Mat drawing = src.clone();

        //Вытаскиваем границы цветов в hsv
        cv::Scalar lb = lineType->lb, ub = lineType->ub;
//        cout << ub << endl;
        //Вектов линий
        vector<Line> lines;

        //Пороговое преобразование
    	cv::inRange(hsv,lb,ub,thr);

    	//Морфологическая обработка
        cv::erode(thr,thr,element);
        cv::dilate(thr,thr,element);
        cv::dilate(thr,thr,element);
        cv::erode(thr,thr,element);

    	cv::imshow("thr", thr);
//        cv::Mat masked;
//        hsv.copyTo(masked,thr);
//        cv::Mat edges;
//        cv::Canny(masked, edges, 0, 50, 3);
//        cv::imshow("canny", edges);

        //Цикл по каждой строке
    	for(int k = 1; k <= bigRoiHeight / rowHeight; k++) {

            //Берём строку
            cv::Rect roiRect ( 0, bigRoiHeight - k*rowHeight, bigRoi.cols, rowHeight);
            cv::Mat roi = thr(roiRect);

            //Находим контуры
            vector<vector<cv::Point> > contours;
            vector<cv::Vec4i> hierarchy;
            vector<cv::Rect> bounds;
            cv::findContours(roi, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

            //Цикл по каждому найденому контуру
            for(int i = 0; i < (int)contours.size(); i++) {
                //Описанный около контура прямоугольник
                cv::Rect bound = cv::boundingRect(contours[i]);

                //Фильтрация по размеру Я ЗАБЫЛ ЗАКЛЮЧИТЬ ВЕСЬ БЛОК В ФИГУРНЫЕ СКОБКИ ПОСлЕ ЭТОГО IF, НЕ ЗАБУДЬ ЭТО СДЕЛАТЬ
                if(bound.width > bigRoi.cols / 25 && bound.width < bigRoi.cols / 5 && bound.height > rowHeight - 10) {
                    //Берём центр в размерности src, а не roi
                    cv::Point center (bound.x + bound.width/2, bound.y + bound.height/2 + src.rows - k*rowHeight);

                    //Сортируем уже найденные линии по дальности от центра
                    sort(lines.begin(), lines.end(), sortByDistanceFromPoint(center));
                    if(!lines.empty())
                    {
                        //Берём ближайшую линию
                        Line & nearestLine = lines.front();
                        //Берём ближайшую точку ближайшей линии
                        cv::Point & nearestPoint = nearestLine.path.back();
                        //Сравниваем по расстоянию
                        double dist = sqrt((center.x - nearestPoint.x) * (center.x - nearestPoint.x) + (center.y - nearestPoint.y) * (center.y - nearestPoint.y));
                        if(dist <= 2*rowHeight)
                            nearestLine.path.push_back(center);
                        else if(dist <= 4*rowHeight) {
                            nearestLine.path.push_back(center);
                            //Discontinuous - прерывистая
                            nearestLine.discontinuous = true;
                        }
                        else
                        {
                            //Если достаточно далеко, значит добавляем новую линию
                            Line newLine;
                            newLine.path.push_back(center);
                            lines.push_back(newLine);
                        }
                    }
                    else
                    {
                        //Самая первая линия
                        Line newLine;
                        newLine.path.push_back(center);
                        lines.push_back(newLine);
                    }
                }
            }
        }

        //Передаём результат в метод-обработчик
        lineType->processResults(lines, drawing);
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

void LineDetector::odometryCb(const nav_msgs::Odometry::ConstPtr &odom) {
    odometry = odom;
}

void LineDetector::occupancyGridCb(const nav_msgs::OccupancyGrid::ConstPtr &grid) {
    occupancyGrid = grid;
}

void LineDetector::convertToOccupancyGridAndPublish(const cv::Mat &img) {
    if(occupancyGrid == 0 || odometry == 0)
        return;
    nav_msgs::OccupancyGrid msg;

    const int gw = occupancyGrid->info.width;
    const int gh = occupancyGrid->info.height;
    const float resolution = occupancyGrid->info.resolution;

    msg.info.height = gh;
    msg.info.width = gw;
    msg.info.resolution = resolution;
    msg.info.origin = occupancyGrid->info.origin;
    msg.header = occupancyGrid->header;
    msg.data = occupancyGrid->data;

    for(int y = 0; y < img.rows; y++) {
        for(int x = 0; x < img.cols; x++) {
            if(img.at<unsigned char>(cv::Point(x,y)) > 0) {
                cout << "HERE" << endl;
                cv::Point2f point(x,y);

                cv::Point2f position = imgToLocalSpace(point);

                double yaw = tf::getYaw(odometry->pose.pose.orientation);

                position = rotateVector(position, yaw);

                position += cv::Point2f(odometry->pose.pose.position.x, odometry->pose.pose.position.y);

                position += cv::Point2f(msg.info.origin.position.x, msg.info.origin.position.y);

                int py = static_cast<int>(std::round(position.y / resolution));

                int px = static_cast<int>(std::round(position.x / resolution));

                if (py < 0 || py >= gw || px < 0 || px >= gh)
                    continue;

                msg.data[py + px * gw] = 100;
            }
        }
    }

    occupancyGridPublisher.publish(msg);
}

void LineDetector::clean() {
    capture.release();
}