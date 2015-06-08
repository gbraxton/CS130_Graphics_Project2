// CS130 Fall 2013: Computer Graphics
// point2d.h
//
// This file does not need to be modified
/////////////////////////////////////////
#ifndef __POINT2D_H__
#define __POINT2D_H__

struct Point2D
{
	double x;
	double y;

	Point2D() : x(0.0), y(0.0) {}
	Point2D(const double & nx, const double & ny) : x(nx), y(ny) {}
	
	Point2D operator+(const Point2D & rhs) const { return Point2D(x + rhs.x, y + rhs.y); }
	Point2D operator-(const Point2D & rhs) const { return Point2D(x - rhs.x, y - rhs.y); }
	Point2D operator*(double val) const { return Point2D(x * val, y * val); }
	Point2D operator/(double val) const { return Point2D(x / val, y / val); }
	Point2D operator+=(const Point2D & rhs) { x += rhs.x; y += rhs.y; return *this; }
	Point2D operator-=(const Point2D & rhs) { x -= rhs.x; y -= rhs.y; return *this; }
	Point2D operator*=(double val) { x *= val; y *= val; return *this; }
	Point2D operator/=(double val) { x /= val; y /= val; return *this; }
};

#endif
