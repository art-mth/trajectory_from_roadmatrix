#include "trajectory_from_roadmatrix_impl.h"

#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <lms/math/polyline.h>
#include <street_environment/obstacle.h>
#include <street_environment/roadmatrix.h>
#include <street_environment/street_environment.h>

using lms::math::polyLine2f;
using lms::math::vertex2f;
using street_environment::RoadMatrix;
using street_environment::EnvironmentObjectPtr;

class TrajectoryFromRoadMatrixTest : public testing::Test {
   public:
    TrajectoryFromRoadMatrixTest()
        : lineLength(20),
          laneWidth(0.4),
          cellsPerLane(4),
          cellLength(0.1),
          carWidthMeter(0.2),
          obstacleClearanceMeter(0.5) {
        polyLine2f straightLine;
        const int lineLength = 20;
        for (int i = 0; i < lineLength; i++) {
            straightLine.points().push_back(vertex2f(i * 0.1, 0.2));
        }

        roadMatrix.aroundLine(straightLine, laneWidth, cellsPerLane,
                              cellLength);

        trajectory_creator.setCarWidthMeter(carWidthMeter);
        trajectory_creator.calculateCycleConstants(roadMatrix);
    }

    RoadMatrix roadMatrix;
    TrajectoryFromRoadmatrixImpl trajectory_creator;

    const int lineLength;
    const float laneWidth;
    const int cellsPerLane;
    const float cellLength;
    const float carWidthMeter;
    const float obstacleClearanceMeter;
};

TEST_F(TrajectoryFromRoadMatrixTest, valueFunction) {
    std::vector<EnvironmentObjectPtr> obstacles;

    street_environment::ObstaclePtr obst1(new street_environment::Obstacle());
    obst1->addPoint(lms::math::vertex2f(0.55, 0.05));
    obstacles.push_back(obst1);

    roadMatrix.markEnvironmentObjects(obstacles);

    EXPECT_EQ(
        trajectory_creator.valueFunction(roadMatrix.cell(0, 0), roadMatrix),
        12);
    EXPECT_EQ(
        trajectory_creator.valueFunction(roadMatrix.cell(0, 5), roadMatrix), 5);
}

TEST_F(TrajectoryFromRoadMatrixTest, getOptimalLanePieceTrajectoryBlockedRoad) {
    std::vector<EnvironmentObjectPtr> obstacles;

    street_environment::ObstaclePtr obst1(new street_environment::Obstacle());
    obst1->addPoint(lms::math::vertex2f(0.7, -0.15));
    obst1->addPoint(lms::math::vertex2f(0.7, -0.05));
    obst1->addPoint(lms::math::vertex2f(0.7, 0.05));
    obst1->addPoint(lms::math::vertex2f(0.7, 0.15));
    obst1->addPoint(lms::math::vertex2f(0.7, 0.25));
    obst1->addPoint(lms::math::vertex2f(0.7, 0.35));
    obst1->addPoint(lms::math::vertex2f(0.7, 0.45));
    obst1->addPoint(lms::math::vertex2f(0.7, 0.55));
    obstacles.push_back(obst1);

    roadMatrix.markEnvironmentObjects(obstacles);

    std::unique_ptr<LanePieceMatrix> lanePieceMatrix =
        trajectory_creator.createLanePieceMatrix(roadMatrix);

    std::unique_ptr<LanePieceTrajectory> lanePieceTrajectory =
        trajectory_creator.getOptimalLanePieceTrajectory(*lanePieceMatrix);

    EXPECT_EQ(lanePieceTrajectory->size(), 2);
    EXPECT_EQ(lanePieceTrajectory->back().stop, true);
}
