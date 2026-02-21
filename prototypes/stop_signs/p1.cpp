#include <iostream>
#include <vector>
#include <stdexcept>
#include <cmath>

using std::invalid_argument;
using std::vector;

/// Helpers
struct Vec2f {
    float x, y;

    Vec2f(float x, float y): x(x), y(y) {}
    
    inline Vec2f operator+(const Vec2f& other) const {
        return Vec2f(x + other.x, y + other.y);
    }

    inline Vec2f operator-(const Vec2f& other) const {
        return Vec2f(x - other.x, y - other.y);
    }

    inline Vec2f operator*(const float f) const {
        return Vec2f(x * f, y * f);
    }

    inline Vec2f operator/(const float f) const {
        return Vec2f(x / f, y / f);
    }

    inline float cross(const Vec2f& other) const {
        return x * other.y - y * other.x;
    }

    inline float dot(const Vec2f& other) const {
        return x * other.x + y * other.y;
    }

    inline float mag() const {
        return std::sqrt(x * x + y * y);
    }

    inline Vec2f norm() const {
        float l = mag();
        return Vec2f(x / l, y / l);
    }

    inline float distance(const Vec2f& other) const {
        return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }

    void normalize() {
        float l = mag();
        x /= l;
        y /= l;
    }
};

enum class Side {
    Inside,
    Left,
    Right,
    OnBoundary,
    None
};

// Function Helpers

/// @brief Finds the closest point to target in vecs
/// Returns the first point if all points are the same distance.
/// Returns -1 if vecs is empty.
int find_closest_point(const Vec2f& target, const vector<Vec2f>& vecs) {
    if (vecs.empty()) return -1;

    // find 2 closest boundary points to target
    float min_distance = target.distance(vecs.at(0));
    int minimum_point_idx = 0;

    for (int i = 1; i < vecs.size(); ++i) {
        const Vec2f& v = vecs[i];
        float dist = target.distance(v);

        if (dist < min_distance) {
            minimum_point_idx = i;
            min_distance = dist;
        }
    }

    return minimum_point_idx;
}

/// @brief Determines the lateral orientation of a point relative to 2 other points that form a region between them.
/// @param p_left 
/// @param p_right 
/// @param target 
/// @return Side::Left if target is left of p_left, Side::Right if target is right of p_right, Side::Inside if target is between the two.
Side solve_3_point_orientation_lateral(const Vec2f& p_left, const Vec2f& p_right, const Vec2f& target) {
    // Build coordinate system around the "right" vector.
    const Vec2f center = (p_left + p_right) * 0.5f; // determine the center point
    const Vec2f right = (p_right - center).norm();

    // Transform target to new coord system
    const float target_lateral = (target - center).dot(right);

    // Compute boundaries
    const float left_lateral = (p_left - center).dot(right);
    const float right_lateral = (p_right - center).dot(right);

    // Classify
    if (target_lateral < left_lateral) return Side::Left;
    if (target_lateral > right_lateral) return Side::Right;
    return Side::Inside;
}

/// End Helpers

/// @brief Determine the points orientation
/// @param target 
/// @param left_waypoints 
/// @param right_waypoints
/// @invariant waypoints with the same id in left and right must be directly across from each other. 
/// @return Side::Left, Side::Right, or Side::Inside depending on Orientation. A failure is returned as Side::None.
Side determine_point_orientation(const Vec2f& target, const vector<Vec2f>& left_waypoints, const vector<Vec2f>& right_waypoints) {
    // Ensure invariants
    if (left_waypoints.size() != right_waypoints.size())
        throw invalid_argument("Left and right vectors must have the same size.");

    // Find 2 closest boundary points to target
    //      this is inefficienty as it calculates the distance to these points twice
    //      this is not that bad but could be optimized maybe using a cache build into
    //      a lane class or by passing the distance out of find_closest_point.
    const int closest_left_idx = find_closest_point(target, left_waypoints);
    const int closest_right_idx = find_closest_point(target, right_waypoints);

    // Find the single closest single point
    int closest_idx = -1;
    const float dist_left = left_waypoints[closest_left_idx].distance(target);
    const float dist_right = right_waypoints[closest_right_idx].distance(target);
    if(dist_left == 0 || dist_right == 0) return Side::OnBoundary;  // if the target is ON a boundary, bail now
    if (dist_left < dist_right) closest_idx = closest_left_idx;     // determine which is closest
    else closest_idx = closest_right_idx;

    // Solve the 3 point orientation
    return solve_3_point_orientation_lateral(left_waypoints.at(closest_idx), right_waypoints.at(closest_idx), target);
}