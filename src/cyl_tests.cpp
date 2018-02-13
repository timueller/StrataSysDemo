#include "stdafx.h"
#include <cyl_demo.h>
#include <gtest/gtest.h>

const E3Point origin(0.0, 0.0, 0.0);
const E3Point sample_pt(1.0, 2.0, 3.0);
const E3Vector sample_vec(4.0, 5.0, 6.0);

TEST(basic_geom_access, pt_access)
{
	ASSERT_EQ(sample_pt.X(), 1.0);
	ASSERT_EQ(sample_pt.Y(), 2.0);
	ASSERT_EQ(sample_pt.Z(), 3.0);
}

TEST(basic_geom_access, vec_access)
{
	ASSERT_EQ(sample_vec.X(), 4.0);
	ASSERT_EQ(sample_vec.Y(), 5.0);
	ASSERT_EQ(sample_vec.Z(), 6.0);
}

TEST(basic_geom_access, vec_from_point)
{
	E3Vector vec(sample_pt);
	ASSERT_EQ(vec.X(), 1.0);
	ASSERT_EQ(vec.Y(), 2.0);
	ASSERT_EQ(vec.Z(), 3.0);
}

TEST(basic_geom_access, vec_from_2_points)
{
	E3Vector diff = origin - sample_pt;
	ASSERT_EQ(diff.X(), -1.0);
	ASSERT_EQ(diff.Y(), -2.0);
	ASSERT_EQ(diff.Z(), -3.0);
}

TEST(basic_geom_offset, pt_vec_arithmetic)
{
	E3Vector degenerate(origin);
	ASSERT_NEAR(degenerate.Length(), 0.0, APX_EQ_EPS);

	E3Vector offset = degenerate + sample_vec;
	offset *= 2.0;
	E3Point offset_point = origin + offset;

	E3Point double_sample(8.0, 10.0, 12.0);
	ASSERT_TRUE(offset_point.ApproxEq(double_sample));

	E3Vector double_eps(APX_EQ_EPS * 2, APX_EQ_EPS * 2, APX_EQ_EPS * 2);
	E3Point not_equal = offset_point + double_eps;
	ASSERT_FALSE(not_equal.ApproxEq(double_sample));
}

TEST(vector_calculation, dot_product)
{
	double length = sample_vec.Length();
	// Dot product should be length^2
	ASSERT_TRUE(E3Coords::ApproxEq(length*length, sample_vec.DotProduct(sample_vec)));
}

TEST(cylinder_exceptions, cylinder_validation)
{
	E3Point base = origin;
	E3Vector axis(1,0,0);

	ASSERT_ANY_THROW(RightCirCylinder(base, axis, -1, 1));
	ASSERT_ANY_THROW(RightCirCylinder(base, axis, 1, -1));

	E3Vector degenerate(origin);
	ASSERT_ANY_THROW(RightCirCylinder(base, degenerate, 1, 1));
}

TEST(cylinder_distance, cylinder_body)
{
	E3Point base = origin;
	E3Vector axis(2, 0, 0);
	// X axis cyl, radius 1/2. t in [0,2]
	RightCirCylinder cyl(base, axis, 0.5, 2.0);
	
	// Outside cyl
	E3Point p1(1, 1, 0);
	EXPECT_TRUE(E3Coords::ApproxEq(cyl.Distance(p1), 0.5));

	// Inside cyl
	E3Point p2(1, 0.25, 0);
	EXPECT_TRUE(E3Coords::ApproxEq(cyl.Distance(p2), 0.25));

	// Outside cyl
	E3Point p3(1, -1, 0);
	EXPECT_TRUE(E3Coords::ApproxEq(cyl.Distance(p3), 0.5));

	// On cyl and cap
	E3Point p4(2, .5, 0);
	EXPECT_TRUE(E3Coords::ApproxEq(cyl.Distance(p4), 0));

	// On cap and axis
	E3Point p5(2, 0, 0);
	EXPECT_TRUE(E3Coords::ApproxEq(cyl.Distance(p5), 0));

	// Inside, closer to cap
	E3Point p6(.25, 0, 0);
	EXPECT_TRUE(E3Coords::ApproxEq(cyl.Distance(p6), 0.25));
}

TEST(cylinder_distance, outside_cyl)
{
	E3Point base = origin;
	E3Vector axis(2, 0, 0);
	RightCirCylinder cyl(base, axis, 0.5, 2.0);

	// Outside cyl in t and and outside radius. Check 4 corner cases.
	double dist = sqrt(2);
	E3Point p1(-1, -1.5, 0);
	EXPECT_TRUE(E3Coords::ApproxEq(cyl.Distance(p1), dist));

	E3Point p2(-1, 1.5, 0);
	EXPECT_TRUE(E3Coords::ApproxEq(cyl.Distance(p2), dist));

	E3Point p3(3, 1.5, 0);
	EXPECT_TRUE(E3Coords::ApproxEq(cyl.Distance(p3), dist));

	E3Point p4(3, -1.5, 0);
	EXPECT_TRUE(E3Coords::ApproxEq(cyl.Distance(p4), dist));

	// Outside cyl in t, but inside radius. Closest to cap (plane).
	E3Point p5(-1, 0, 0);
	EXPECT_TRUE(E3Coords::ApproxEq(cyl.Distance(p5), 1));

	E3Point p6(3, .5, 0);
	EXPECT_TRUE(E3Coords::ApproxEq(cyl.Distance(p6), 1));

	E3Point p7(3, -0.1, 0);
	EXPECT_TRUE(E3Coords::ApproxEq(cyl.Distance(p7), 1));
}

