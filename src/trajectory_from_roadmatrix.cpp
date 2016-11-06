#include "trajectory_from_roadmatrix.h"

#include <math.h>
#include <vector>
#include <memory>

namespace {
using road_matrix_to_trajectory::LanePiece;
using road_matrix_to_trajectory::LanePieceMatrix;
using road_matrix_to_trajectory::LanePieceTrajectory;
}

bool TrajectoryFromRoadmatrix::initialize() {
    roadMatrix = readChannel<street_environment::RoadMatrix>("ROADMATRIX");
    trajectory = writeChannel<street_environment::Trajectory>("TRAJECTORY");
    return true;
}

bool TrajectoryFromRoadmatrix::deinitialize() {
    return true;
}

bool TrajectoryFromRoadmatrix::fillTrajectory(
    const LanePieceTrajectory& lanePieceTrajectory) {
    street_environment::TrajectoryPoint tp;
    tp.velocity = 1;
    for (const auto& piece : lanePieceTrajectory) {
        float tp_x = (piece.cells.front().points[1].x +
                      piece.cells.back().points[2].x) / 2;
        float tp_y = (piece.cells.front().points[1].y +
                      piece.cells.back().points[2].y) / 2;
        tp.position = lms::math::vertex2f(tp_x, tp_y);
        trajectory->push_back(tp);
    }
    return true;
}

bool TrajectoryFromRoadmatrix::cycle() {
    trajectory->clear();

    float carWidthMeter = config().get<float>("carWidth", 0.2);
    int carWidthCells = ceil(carWidthMeter / roadMatrix->cellWidth());

    std::unique_ptr<LanePieceMatrix> lanePieceMatrix =
        road_matrix_to_trajectory::createLanePieceMatrix(
            carWidthCells, *roadMatrix);

    std::unique_ptr<LanePieceTrajectory> lanePieceTrajectory =
        road_matrix_to_trajectory::getOptimalLanePieceTrajectory(
            *lanePieceMatrix);

    fillTrajectory(*lanePieceTrajectory);

    return true;
}
