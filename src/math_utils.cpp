#include "ap1/planning/math_utils.hpp"
#include <cmath>

namespace ap1::planning {

float distance(const vec2f& next, const vec2f& target)
{
    // Formula: sqrt((b.x - a.x)^2 + (b.y - a.y)^2)
    float distance = std::sqrt(
                std::pow((target.x - next.x), 2) +
                std::pow((target.y - next.y), 2)
                );
    return distance;
}

} // namespace ap1::planning
