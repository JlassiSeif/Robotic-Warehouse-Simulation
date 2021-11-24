#ifndef POINT_H
#define POINT_H

class point{
    public :
        double x=0;
        double y=0;
        double z=0;
        double yaw=0;
        point();
        point(double,double,double,double);
        point(double,double,double);
};
point::point() = default;
point::point(double a,double b,double c,double d) : x{a}, y{b}, z{c}, yaw{d} {};
point::point(double a,double b,double c) : x{a}, y{b}, z{c} {};

#endif