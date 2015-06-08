// CS130 Fall 2013: Computer Graphics
// point3d.h
//
// This file does not need to be modified
/////////////////////////////////////////
#ifndef __POINT3D_H__
#define __POINT3D_H__

struct Point3D
{
	double x;
	double y;
	double z;

	Point3D() : x(0.0), y(0.0), z(0.0) {}
	Point3D(const double & nx, const double & ny) : x(nx), y(ny), z(0) {}
	Point3D(const double & nx, const double & ny, const double & nz) : x(nx), y(ny), z(nz) {}
	
	Point3D operator+(const Point3D & rhs) const { return Point3D(x + rhs.x, y + rhs.y, z + rhs.z); }
	Point3D operator-(const Point3D & rhs) const { return Point3D(x - rhs.x, y - rhs.y, z - rhs.z); }
	Point3D operator*(double val) const { return Point3D(x * val, y * val, z * val); }
	Point3D operator/(double val) const { return Point3D(x / val, y / val, z / val); }
	Point3D operator+=(const Point3D & rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
	Point3D operator-=(const Point3D & rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
	Point3D operator*=(double val) { x *= val; y *= val; z *= val; return *this; }
	Point3D operator/=(double val) { x /= val; y /= val; z /= val; return *this; }
};

#endif
