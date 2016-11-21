/**
 * Use this file for quick test prototyping as an alternative to logging. This
 * module is bound to change and it is hard to write ever lasting tests for it.
 * When there are tests that are not compiling anymore look at them and make
 * sure you know what the tests idea was. If you understand the tests purpose
 * and are sure your change did not break this constraint, but it is just a
 * value problem delete it or write a new test. Use this to make sure your code
 * is doing what you are expecting it to do.
 */

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
          obstacleClearanceLeftFrontMeter(1.0),
          obstacleClearanceRightFrontMeter(0.5),
          obstacleClearanceLeftBackMeter(0.5),
          obstacleClearanceRightBackMeter(0.5) {
        polyLine2f straightLine;
        for (int i = 0; i < lineLength; i++) {
            straightLine.points().push_back(vertex2f(i * 0.1, 0.2));
        }

        roadMatrix.aroundLine(straightLine, laneWidth, cellsPerLane,
                              cellLength);

        trajectory_creator.setCarWidthMeter(carWidthMeter);
        trajectory_creator.setObstacleClearanceLeftFrontMeter(obstacleClearanceLeftFrontMeter);
        trajectory_creator.setObstacleClearanceRightFrontMeter(obstacleClearanceRightFrontMeter);
        trajectory_creator.setObstacleClearanceLeftBackMeter(obstacleClearanceLeftBackMeter);
        trajectory_creator.setObstacleClearanceRightBackMeter(obstacleClearanceRightBackMeter);
        trajectory_creator.calculateCycleConstants(roadMatrix);
    }

    RoadMatrix roadMatrix;
    TrajectoryFromRoadmatrixImpl trajectory_creator;

    const int lineLength;
    const float laneWidth;
    const int cellsPerLane;
    const float cellLength;
    const float carWidthMeter;
    const float obstacleClearanceLeftFrontMeter;
    const float obstacleClearanceRightFrontMeter;
    const float obstacleClearanceLeftBackMeter;
    const float obstacleClearanceRightBackMeter;
};

TEST_F(TrajectoryFromRoadMatrixTest, createLanePieceMatrixValuesScaled) {
    std::vector<EnvironmentObjectPtr> obstacles;
    street_environment::ObstaclePtr obst1(new street_environment::Obstacle());
    obst1->addPoint(lms::math::vertex2f(0.55, 0.05));
    obst1->addPoint(lms::math::vertex2f(0.55, -0.05));
    obstacles.push_back(obst1);

    roadMatrix.markEnvironmentObjects(obstacles);

    std::unique_ptr<LanePieceMatrix> lanePieceMatrix =
        trajectory_creator.createLanePieceMatrix(roadMatrix);
    EXPECT_GT(lanePieceMatrix->at(4).at(5).value, lanePieceMatrix->at(4).at(4).value);
    EXPECT_GE(lanePieceMatrix->at(4).at(5).value, lanePieceMatrix->at(4).at(6).value);
    EXPECT_LT(lanePieceMatrix->at(4).at(5).value, lanePieceMatrix->at(4).at(3).value);
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
