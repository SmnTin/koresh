#ifndef PROJECT_POINT_H
#define PROJECT_POINT_H

class Point {
public:
    double x = 0, y = 0;

    Point() {}
    Point(double _x, double _y) : x(_x), y(_y) {};

    Point operator+ (const Point & b) const{
        return Point(x + b.x, y + b.y);
    }
    Point operator+= (const Point & b){
        x+=b.x;
        y+=b.y;
        return Point(x,y);
    }
    Point operator- (const Point & b) const{
        return Point(x - b.x, y - b.y);
    }
    Point operator-= (const Point & b){
        x-=b.x;
        y-=b.y;
        return Point(x,y);
    }
    Point operator/ (const double & b) const{
        return Point(x / b, y / b);
    }
    Point operator/= (const double & b){
        x/=b;
        y/=b;
        return Point(x,y);
    }
    Point operator* (const double & b) const{
        return Point(x * b, y * b);
    }
    Point operator*= (const double & b){
        x*=b;
        y*=b;
        return Point(x,y);
    }
    double vecLength() const{
        return sqrt(x*x + y*y);
    }
};

#endif //PROJECT_POINT_H
