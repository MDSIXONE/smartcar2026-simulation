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
    static std::vector<std::pair<double, double>> resampleXY(
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

        std::vector<std::pair<double, double>> result;
        for (const CymPlanner::PathPoint& point : sampled)
        {
            result.emplace_back(point.x, point.y);
        }
        return result;
    }

    static std::vector<double> resampleX(
        CymPlanner& planner,
        const std::vector<std::pair<double, double>>& positions,
        double resolution)
    {
        const std::vector<std::pair<double, double>> sampled =
            resampleXY(planner, positions, resolution);
        std::vector<double> x_values;
        for (const auto& point : sampled)
        {
            x_values.push_back(point.first);
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

    static void refineOffsets(
        CymPlanner& planner,
        const std::vector<std::pair<double, double>>& positions,
        std::vector<double>& offsets,
        int locked_side)
    {
        std::vector<CymPlanner::PathPoint> reference;
        for (const auto& position : positions)
        {
            CymPlanner::PathPoint point;
            point.x = position.first;
            point.y = position.second;
            reference.push_back(point);
        }
        planner.computePathGeometry(reference);
        planner.refineOffsets(reference, offsets, locked_side);
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

TEST(CymPlannerGeometryTest, RemovesMillimetreScaleLoopsFromResampledPath)
{
    CymPlanner planner;
    const double resolution = 0.04;
    const std::vector<std::pair<double, double>> sampled =
        CymPlannerTestPeer::resampleXY(
            planner,
            {
                {0.0, 0.0},
                {0.20, 0.0},
                {0.24, 0.0},
                {0.205, 0.002},
                {0.242, 0.004},
                {0.40, 0.01},
            },
            resolution);

    ASSERT_GE(sampled.size(), 3U);
    EXPECT_NEAR(sampled.front().first, 0.0, 1e-9);
    EXPECT_NEAR(sampled.back().first, 0.40, 1e-9);
    EXPECT_NEAR(sampled.back().second, 0.01, 1e-9);
    for (std::size_t index = 1; index + 1 < sampled.size(); ++index)
    {
        EXPECT_GE(
            std::hypot(
                sampled[index].first - sampled[index - 1].first,
                sampled[index].second - sampled[index - 1].second),
            0.5 * resolution - 1e-9);
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

TEST(CymPlannerGeometryTest, ElasticRefinementIsNotClampedToLockedSide)
{
    CymPlanner planner;
    std::vector<double> offsets(9, -0.10);
    CymPlannerTestPeer::refineOffsets(
        planner,
        {{0.00, 0.0}, {0.05, 0.0}, {0.10, 0.0}, {0.15, 0.0}, {0.20, 0.0},
         {0.25, 0.0}, {0.30, 0.0}, {0.35, 0.0}, {0.40, 0.0}},
        offsets,
        1);

    // A positive lock is now hysteresis metadata only; the elastic band can
    // retain a negative offset when the continuous force points right.
    EXPECT_LT(offsets[4], -1e-4);
}

}  // namespace cym_planner

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
