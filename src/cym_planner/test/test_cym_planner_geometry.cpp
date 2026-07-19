#include "cym_planner.h"

#include <cmath>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace cym_planner
{

class CymPlannerTestPeer
{
public:
    static std::vector<double> resampleX(
        CymPlanner& planner,
        const std::vector<std::pair<double, double>>& positions,
        double resolution)
    {
        std::vector<CymPlanner::PathPoint> path;
        for (const auto& position : positions)
        {
            CymPlanner::PathPoint point;
            point.x = position.first;
            point.y = position.second;
            path.push_back(point);
        }
        planner.computePathGeometry(path);
        const std::vector<CymPlanner::PathPoint> sampled =
            planner.resamplePath(path, resolution);

        std::vector<double> x_values;
        for (const CymPlanner::PathPoint& point : sampled)
        {
            x_values.push_back(point.x);
        }
        return x_values;
    }

    static double clampVelocityDelta(
        const CymPlanner& planner,
        double target,
        double previous,
        double increase_limit,
        double decrease_limit,
        double dt)
    {
        return planner.clampVelocityDelta(
            target, previous, increase_limit, decrease_limit, dt);
    }
};

TEST(CymPlannerGeometryTest, ResamplesStraightPathAtFixedSpacingAndKeepsEndpoint)
{
    CymPlanner planner;
    const std::vector<double> x_values = CymPlannerTestPeer::resampleX(
        planner,
        {{0.0, 0.0}, {0.13, 0.0}, {0.31, 0.0}},
        0.04);

    ASSERT_GE(x_values.size(), 8U);
    EXPECT_NEAR(x_values.front(), 0.0, 1e-9);
    EXPECT_NEAR(x_values.back(), 0.31, 1e-9);
    for (std::size_t index = 1; index + 1 < x_values.size(); ++index)
    {
        EXPECT_NEAR(x_values[index] - x_values[index - 1], 0.04, 1e-9);
    }
}

TEST(CymPlannerGeometryTest, AppliesSeparateAccelerationAndDecelerationLimits)
{
    CymPlanner planner;
    EXPECT_NEAR(
        CymPlannerTestPeer::clampVelocityDelta(
            planner, 1.0, 0.0, 0.4, 0.8, 0.1),
        0.04,
        1e-9);
    EXPECT_NEAR(
        CymPlannerTestPeer::clampVelocityDelta(
            planner, 0.0, 0.5, 0.4, 0.8, 0.1),
        0.42,
        1e-9);
}

}  // namespace cym_planner

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
