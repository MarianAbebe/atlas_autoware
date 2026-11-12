#ifndef ATLAS_NAV2_DEMO__ACKERMANN_CONTROLLER_MATH_HPP_
#define ATLAS_NAV2_DEMO__ACKERMANN_CONTROLLER_MATH_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace atlas_nav2_demo
{
namespace controller_math
{
inline double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double purePursuitCurvature(double local_x, double local_y)
{
  const double distance_squared = local_x * local_x + local_y * local_y;
  if (!std::isfinite(distance_squared) || distance_squared < 1.0e-8) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return 2.0 * local_y / distance_squared;
}

inline double steeringFromCurvature(double curvature, double wheelbase)
{
  return std::atan(wheelbase * curvature);
}

inline double curvatureFromTwist(double longitudinal, double angular)
{
  if (!std::isfinite(longitudinal) || !std::isfinite(angular) ||
    std::abs(longitudinal) < 1.0e-9)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return angular / longitudinal;
}

inline double boundedStep(double current, double target, double rise,
                          double fall, double dt)
{
  if (current * target < 0.0) {
    const double next = std::max(0.0, std::abs(current) - fall * dt);
    return next <= 1.0e-9 ? 0.0 : std::copysign(next, current);
  }
  const double rate = std::abs(target) > std::abs(current) ? rise : fall;
  return current + std::clamp(target - current, -rate * dt, rate * dt);
}

inline bool sourceStampFresh(int64_t source_ns, int64_t now_ns,
                             int64_t timeout_ns)
{
  return source_ns > 0 && now_ns >= source_ns && timeout_ns > 0 &&
         now_ns - source_ns <= timeout_ns;
}
}  // namespace controller_math
}  // namespace atlas_nav2_demo

#endif  // ATLAS_NAV2_DEMO__ACKERMANN_CONTROLLER_MATH_HPP_
