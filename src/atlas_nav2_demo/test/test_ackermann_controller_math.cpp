#include <cmath>

#include "gtest/gtest.h"
#include "atlas_nav2_demo/ackermann_controller_math.hpp"

TEST(AckermannControllerMath, PurePursuitGoldenVectors)
{
  EXPECT_NEAR(atlas_nav2_demo::controller_math::purePursuitCurvature(1.0, 0.25),
    0.47058823529411764, 1.0e-12);
  EXPECT_NEAR(atlas_nav2_demo::controller_math::purePursuitCurvature(-1.0, -0.25),
    -0.47058823529411764, 1.0e-12);
  EXPECT_NEAR(atlas_nav2_demo::controller_math::steeringFromCurvature(0.4, 1.2),
    std::atan(0.48), 1.0e-12);
}

TEST(AckermannControllerMath, TwistCurvatureContract)
{
  const double curvature =
    atlas_nav2_demo::controller_math::curvatureFromTwist(0.5, 0.2);
  EXPECT_NEAR(curvature, 0.4, 1.0e-12);
  EXPECT_TRUE(std::isnan(
      atlas_nav2_demo::controller_math::curvatureFromTwist(0.0, 0.2)));
}

TEST(AckermannControllerMath, BoundedAccelerationAndReverseStop)
{
  using atlas_nav2_demo::controller_math::boundedStep;
  EXPECT_NEAR(boundedStep(0.0, 0.2, 0.8, 1.2, 0.1), 0.08, 1.0e-12);
  EXPECT_NEAR(boundedStep(0.08, -0.08, 0.8, 1.2, 0.1), 0.0, 1.0e-12);
  EXPECT_NEAR(boundedStep(0.0, -0.08, 0.8, 1.2, 0.1), -0.08, 1.0e-12);
}

TEST(AckermannControllerMath, ObstacleSourceTimestampFreshness)
{
  using atlas_nav2_demo::controller_math::sourceStampFresh;
  constexpr int64_t now = 10'000'000'000LL;
  constexpr int64_t timeout = 300'000'000LL;
  EXPECT_TRUE(sourceStampFresh(now - timeout, now, timeout));
  EXPECT_FALSE(sourceStampFresh(now - timeout - 1, now, timeout));
  EXPECT_FALSE(sourceStampFresh(now + 1, now, timeout));
  EXPECT_FALSE(sourceStampFresh(0, now, timeout));
}
