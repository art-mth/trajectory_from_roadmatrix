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
#include <street_environment/roadmatrix.h>

using lms::math::polyLine2f;
using lms::math::vertex2f;
using street_environment::RoadMatrix;

class TrajectoryFromRoadMatrixTest : public testing::Test {
   public:
    TrajectoryFromRoadMatrixTest()
        : lineLength(20),
          laneWidth(0.4),
          cellsPerLane(4),
          cellLength(0.1),
          maxTranslation(5),
          carWidthMeter(0.2),
          obstacleClearanceLeftFrontMeter(1.0),
          obstacleClearanceRightFrontMeter(0.5),
          obstacleClearanceLeftBackMeter(0.5),
          obstacleClearanceRightBackMeter(0.5) {
        polyLine2f straightLine;
        for (int i = 0; i < lineLength; i++) {
            straightLine.points().push_back(vertex2f(i * 0.1, 0.2));
        }

        roadMatrix.initialize(laneWidth, cellsPerLane, cellLength, maxTranslation);
        roadMatrix.aroundLine(straightLine, vertex2f(0, 0), 0);

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
    const int maxTranslation;
    const float carWidthMeter;
    const float obstacleClearanceLeftFrontMeter;
    const float obstacleClearanceRightFrontMeter;
    const float obstacleClearanceLeftBackMeter;
    const float obstacleClearanceRightBackMeter;
};

TEST_F(TrajectoryFromRoadMatrixTest, createLanePieceMatrixValuesScaled) {
    {
        std::unique_ptr<LanePieceMatrix> lanePieceMatrix =
            trajectory_creator.createLanePieceMatrix(roadMatrix);
        EXPECT_GT(lanePieceMatrix->at(4).at(2).value, lanePieceMatrix->at(4).at(3).value);
        EXPECT_GE(lanePieceMatrix->at(4).at(1).value, lanePieceMatrix->at(4).at(2).value);
        EXPECT_LT(lanePieceMatrix->at(4).at(0).value, lanePieceMatrix->at(4).at(1).value);
    }
    {
        roadMatrix.cell(4, 2).hasObstacle = true;
        std::unique_ptr<LanePieceMatrix> lanePieceMatrix =
            trajectory_creator.createLanePieceMatrix(roadMatrix);

        EXPECT_LT(lanePieceMatrix->at(4).at(2).value, lanePieceMatrix->at(4).at(3).value);
        EXPECT_GE(lanePieceMatrix->at(4).at(1).value, lanePieceMatrix->at(4).at(2).value);
        EXPECT_GT(lanePieceMatrix->at(4).at(0).value, lanePieceMatrix->at(4).at(1).value);
    }


}

TEST_F(TrajectoryFromRoadMatrixTest, getOptimalLanePieceTrajectoryBlockedRoad) {
    roadMatrix.cell(7, 0).hasObstacle = true;
    roadMatrix.cell(7, 1).hasObstacle = true;
    roadMatrix.cell(7, 2).hasObstacle = true;
    roadMatrix.cell(7, 3).hasObstacle = true;
    roadMatrix.cell(7, 4).hasObstacle = true;
    roadMatrix.cell(7, 5).hasObstacle = true;
    roadMatrix.cell(7, 6).hasObstacle = true;
    roadMatrix.cell(7, 7).hasObstacle = true;

    std::unique_ptr<LanePieceMatrix> lanePieceMatrix =
        trajectory_creator.createLanePieceMatrix(roadMatrix);

    std::unique_ptr<LanePieceTrajectory> lanePieceTrajectory =
        trajectory_creator.getOptimalLanePieceTrajectory(*lanePieceMatrix);

    EXPECT_EQ(lanePieceTrajectory->size(), 3);
    EXPECT_EQ(lanePieceTrajectory->back().stop, true);
}
