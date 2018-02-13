#include "stdafx.h"
#include <cyl_demo.h>
#include <cmath>
#include <algorithm>

E3Coords::E3Coords(double x, double y, double z)
{
	p_coords[0] = x;
	p_coords[1] = y;
	p_coords[2] = z;
}

// Coordinate-wise approximately equal test. IE NOT a distance comparison.
bool E3Coords::ApproxEq(const E3Coords& point, double eps) const
{
	return ApproxEq(point.X(), X()) && ApproxEq(point.Y(), Y()) && 
		ApproxEq(point.Y(), Y());
}

bool E3Coords::ApproxEq(double x, double y, double eps)
{
	return fabs(x - y) < eps;
}

void E3Coords::Scale(double s)
{
	p_coords[0] *= s;
	p_coords[1] *= s;
	p_coords[2] *= s;
}

///////////////////////////////////////////////////////////////////////////////
// 3D Euclidean point
E3Point::E3Point(double x, double y, double z) : E3Coords(x, y, z)
{
}

bool E3Point::ApproxEq(const E3Point& point, double eps) const
{
	return E3Coords::ApproxEq(point, eps);
}

// Vector FROM base_pt TO this
E3Vector E3Point::operator-(const E3Point& base_pt) const
{
	return E3Vector(X() - base_pt.X(), Y() - base_pt.Y(), Z() - base_pt.Z());
}

// Point offset by a vector
E3Point E3Point::operator+(const E3Vector& offset) const
{
	return E3Point(X() + offset.X(), Y() + offset.Y(), Z() + offset.Z());
}

double E3Point::Distance(const E3Point& point) const
{
	E3Vector pt_to_pt = *this - point;
	return pt_to_pt.Length();
}

///////////////////////////////////////////////////////////////////////////////
// 3D Euclidean vector
E3Vector::E3Vector(double x, double y, double z) : E3Coords(x, y, z)
{
}

E3Vector::E3Vector(const E3Point& head)	: E3Coords(head.X(), head.Y(), head.Z())
{
}

E3Vector E3Vector::operator+(const E3Vector& vector) const
{
	return E3Vector(X() + vector.X(), Y() + vector.Y(), Z() + vector.Z());
}

const E3Vector& E3Vector::operator*=(double scale)
{
	Scale(scale);
	return *this;
}

double E3Vector::Length() const
{
	return sqrt(X()*X() + Y()*Y() + Z()*Z());
}

double E3Vector::DotProduct(const E3Vector& vector) const
{
	return X()*vector.X() + Y()*vector.Y() + Z()*vector.Z();
}

///////////////////////////////////////////////////////////////////////////////
// Right circular cylinder
//
// Solid model b-rep of a finite cylinder represented by 3 surfaces: the cylinder
// body plus one disk at each end.
//
// Exceptions: Dimensions must be positive, axis vector non-degenerate. 
//
RightCirCylinder::RightCirCylinder(const E3Point& base_pt, const E3Vector& axis, 
	double radius, double height) :
	p_base_pt(base_pt), p_axis(axis), p_radius(radius), p_height(height)
{
	// Radius and height must be positive
	if (p_radius < 0 || p_height < 0)
		throw RightCirCylinderInvalid();

	// Axis vector can't be degenerate.
	double vec_length = axis.Length();
	if (E3Coords::ApproxEq(vec_length, 0.0))
		throw RightCirCylinderInvalid();

	// Normalize axis. This gives a parametric 3D line equation for cylinder axis of 
	//     P(t) = base + t * axis 
	// where t is in the range [0, h] for points inside the finite cylinder.
	p_axis *= 1 / vec_length;
}

// Compute the closest distance from a point to the boundary of the cylinder. The 
// closest distance considers all 3 surfaces. The point may be inside, outside, or
// on one of the surfaces of the cylinder. Distance returned is >= 0.
//
double RightCirCylinder::Distance(const E3Point& point) const
{
	// Vector from base of cylinder to the given point.
	E3Vector to_point = point - p_base_pt;

	// The dot product of to_point and the axis vector is the parameter of the point
	// projected onto the axis.
	// NOTE: p_axis is a normalized vector. 
	double line_param = to_point.DotProduct(p_axis);

	// Compute the point projected onto the axis.
	E3Vector scaled_axis = p_axis;
	scaled_axis *= line_param;
	E3Point pt_on_axis = p_base_pt + scaled_axis;

	// Compute distances we need to use.
	// Distance from input point to the axis.
	double d_to_axis = pt_on_axis.Distance(point);
	
	// Distance from input point to the infinite cylinder.
	double d_to_cyl = fabs(d_to_axis - p_radius);

	// Distance to plane of the closest end cap of the cylinder.
	double d_to_cap = std::min(fabs(line_param), fabs(p_height - line_param));

	// Is pt_on_axis inside the finite cylinder? ie t is in [0,h]
	bool pt_on_axis_inside_cyl = line_param >= 0 && line_param <= p_height;
	
	// Closest distance depends of where the input point is relative to the cyl.
	if (pt_on_axis_inside_cyl)
	{
		// If the point is inside the cyl radius, it might be closer to the cap.
		return std::min(d_to_cyl, d_to_cap);
	}
	else if (d_to_axis < p_radius)
	{
		// It's outside the finite cyl, but within radius distance to the axis.
		// Closest point is just distance to the cap plane
		return d_to_cap;
	}
	else
	{
		// It's outside the infinite cyl and the pt_on_axis is outside the finite cyl. 
		// Compute the distance to the edge of the end cap. The input point and the axis
		// form a plane. In this plane, the cross-section of the cyl is a rectangle. 
		// We are computing the distance from the input point to a corner of this rect. 
		return sqrt(d_to_cap*d_to_cap + d_to_cyl*d_to_cyl);
	}
}
