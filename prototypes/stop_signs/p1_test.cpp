#include "p1.cpp"
#include <gtest/gtest.h>
#include <vector>

/**
 * Test cases for p1.
 * +x is RIGHT and +y is UP.
 */

// Lane points DOWN to UP
TEST(StopSignP1Test, VerticalLaneTest) {
    const vector<Vec2f> left = {
        Vec2f(-1, -2),
        Vec2f(-1, -1),
        Vec2f(-1, 0),
        Vec2f(-1, 1),
        Vec2f(-1, 2)
    };
    const vector<Vec2f> right = {
        Vec2f(1, -2),
        Vec2f(1, -1),
        Vec2f(1, 0),
        Vec2f(1, 1),
        Vec2f(1, 2)
    };

    EXPECT_EQ(Side::Right, determine_point_orientation(Vec2f(2, 2), left, right));
    EXPECT_EQ(Side::Left, determine_point_orientation(Vec2f(-2, 2), left, right));
    EXPECT_EQ(Side::Inside, determine_point_orientation(Vec2f(0, 0), left, right));
}

// Lane points RIGHT TO LEFT.
TEST(StopSignP1Test, HorizontalLaneTest) {
    const vector<Vec2f> left = {
        Vec2f(2, -1),
        Vec2f(1, -1),
        Vec2f(0, -1),
        Vec2f(-1, -1),
        Vec2f(-2, -1)
    };
    const vector<Vec2f> right = {
        Vec2f(2, 1),
        Vec2f(1, 1),
        Vec2f(0, 1),
        Vec2f(-1, 1),
        Vec2f(-2, 1)
    };

    EXPECT_EQ(Side::Left, determine_point_orientation(Vec2f(0, -5), left, right));
    EXPECT_EQ(Side::Right, determine_point_orientation(Vec2f(5, 5), left, right));
    EXPECT_EQ(Side::Inside, determine_point_orientation(Vec2f(0, 0), left, right));
}

// TOP LEFT to BOTTOM RIGHT
TEST(StopSignP1Test, DiagonalLaneTest) {
    const vector<Vec2f> left = {
        Vec2f(-2, 2),
        Vec2f(-1, 1),
        Vec2f(0, 0),
        Vec2f(1, -1),
        Vec2f(2, -2)
    };
    const vector<Vec2f> right = {
        Vec2f(-2, 0),
        Vec2f(-1, -1),
        Vec2f(0, -2),
        Vec2f(1, -3),
        Vec2f(2, -4)
    };

    EXPECT_EQ(Side::Left, determine_point_orientation(Vec2f(2, 5), left, right));
    EXPECT_EQ(Side::Right, determine_point_orientation(Vec2f(-0.5, -3), left, right));
    EXPECT_EQ(Side::Inside, determine_point_orientation(Vec2f(1.5, -2.5), left, right));
}

// BOTTOM LEFT TO TOP RIGHT
TEST(StopSignP1Test, OppositeDiagonalLaneTest) {
    const vector<Vec2f> left = {
        Vec2f(-2, 0),
        Vec2f(-1, 1),
        Vec2f(0, 2),
        Vec2f(1, 3),
        Vec2f(2, 4)
    };
    const vector<Vec2f> right = {
        Vec2f(-2, -2),
        Vec2f(-1, -1),
        Vec2f(0, 0),
        Vec2f(1, 1),
        Vec2f(2, 2)
    };

    EXPECT_EQ(Side::Left, determine_point_orientation(Vec2f(-1, 5), left, right));
    EXPECT_EQ(Side::Right, determine_point_orientation(Vec2f(4, -5), left, right));
    EXPECT_EQ(Side::Inside, determine_point_orientation(Vec2f(-0.5, 0.5), left, right));
}

TEST(StopSignP1Test, ConcaveLaneTest) {
    // The lane curves inward and up, from Right-to-Left (x=5 to x=-5)
    const vector<Vec2f> left = {
        Vec2f(5.0f, 0.25f),    // y = 5^2/20 - 1 = 1.25 - 1 = 0.25
        Vec2f(2.0f, -0.8f),    // y = 2^2/20 - 1 = 0.2 - 1 = -0.8
        Vec2f(0.0f, -1.0f),    // y = 0 - 1 = -1
        Vec2f(-2.0f, -0.8f),
        Vec2f(-5.0f, 0.25f)
    };
    const vector<Vec2f> right = {
        Vec2f(5.0f, 2.25f),    // y = 1.25 + 1 = 2.25
        Vec2f(2.0f, 1.2f),     // y = 0.2 + 1 = 1.2
        Vec2f(0.0f, 1.0f),     // y = 0 + 1 = 1
        Vec2f(-2.0f, 1.2f),
        Vec2f(-5.0f, 2.25f)
    };
    
    // Test point 1: Inside the U-curve, well below the left boundary
    EXPECT_EQ(Side::Left, determine_point_orientation(Vec2f(0.0f, -5.0f), left, right)) << "Point is well below the concave lane.";

    // Test point 2: Outside the U-curve, well above the right boundary
    EXPECT_EQ(Side::Right, determine_point_orientation(Vec2f(0.0f, 5.0f), left, right)) << "Point is well above the concave lane.";
}

// Input Data Errors:
// - Lanes are the same
// - Lanes have different lengths
TEST(StopSignP1Test, LaneInErrors) {
    // Zero-width lane: left and right boundaries are identical
    const vector<Vec2f> boundary = {
        Vec2f(2, 0),
        Vec2f(1, 0),
        Vec2f(0, 0),
        Vec2f(-1, 0),
        Vec2f(-2, 0)
    };
    
    Side result = determine_point_orientation(Vec2f(0, 0), boundary, boundary);
    EXPECT_EQ(Side::OnBoundary, result) << "Degenerate zero-width lane should be marked as Onboundary.";
}

// Stop sign is directly on a lane
TEST(StopSignP1Test, IncidentStopSign) {
    const vector<Vec2f> left = {
        Vec2f(-1, -1),
        Vec2f(-1, 0),
        Vec2f(-1, 1)
    };
    const vector<Vec2f> right = {
        Vec2f(1, -1),
        Vec2f(1, 0),
        Vec2f(1, 1)
    };

    EXPECT_EQ(Side::Inside, determine_point_orientation(Vec2f(1, 0), left, right)) << "Targets on the lane boundary should be counted as inside.";
}

