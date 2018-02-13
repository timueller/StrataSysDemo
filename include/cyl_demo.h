#pragma once
#include <exception>

const double APX_EQ_EPS = 1e-5;

///////////////////////////////////////////////////////////////////////////////
// Base geometry classes
class E3Coords
{
public:
	E3Coords(double x, double y, double z);
	double X() const { return p_coords[0]; };
	double Y() const { return p_coords[1]; };
	double Z() const { return p_coords[2]; };
	bool ApproxEq(const E3Coords& point, double eps = APX_EQ_EPS) const;
	static bool ApproxEq(double x, double y, double eps = APX_EQ_EPS);

protected:
	void Scale(double s);

private:
	double p_coords[3];
};

///////////////////////////////////////////////////////////////////////////////
// 3D Euclidean vector
class E3Point;
class E3Vector : public E3Coords
{
public:
	E3Vector(double x, double y, double z);
	E3Vector(const E3Point& head);

	E3Vector operator+(const E3Vector& vector) const;
	const E3Vector& operator*=(double scale);

	double Length() const;
	double DotProduct(const E3Vector& vector) const;
};

///////////////////////////////////////////////////////////////////////////////
// 3D Euclidean point
class E3Point : public E3Coords
{
public:
	E3Point(double x, double y, double z);

	E3Vector operator-(const E3Point& base_pt) const;
	E3Point operator+(const E3Vector& offset) const;

	bool ApproxEq(const E3Point& point, double eps = APX_EQ_EPS) const;
	double Distance(const E3Point& point) const;
};

///////////////////////////////////////////////////////////////////////////////
// Right circular cylinder
//
// Finite cylinder respresented by a base point, axis vector, height and radius.
// The axis is normalized giving a parametric 3D line equation for points on the axis of
//      P(t) = base + t * axis
// where t is in the range [0, h] for points inside the cylinder.
//
// The boundary representation of the cylinder consist of 3 surfaces, one for the
// cylinder body and two for the disks (or "caps") at each end. 
class RightCirCylinder
{
public:
	RightCirCylinder(const E3Point& base_pt, const E3Vector& axis, 
		double radius, double height);

	// Closest distance to cylinder surfaces
	double Distance(const E3Point& point) const;

private:
	E3Point p_base_pt;
	E3Vector p_axis;
	double p_radius;
	double p_height;
};

// Exception class for RightCirCylinder constructor
class RightCirCylinderInvalid : public std::exception
{
	virtual const char* what() const throw()
	{
		return "Invalid data in RightCirCylinder constructor";
	}
};
