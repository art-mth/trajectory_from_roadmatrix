#include "trajectory_from_roadmatrix.h"

#include <math.h>
#include <vector>
#include <memory>

bool TrajectoryFromRoadmatrix::initialize() {
    roadMatrix = readChannel<street_environment::RoadMatrix>("ROADMATRIX");
    trajectory = writeChannel<street_environment::Trajectory>("TRAJECTORY");

    impl = std::unique_ptr<TrajectoryFromRoadmatrixImpl>(
               new TrajectoryFromRoadmatrixImpl);
    configureImpl();

    return true;
}

bool TrajectoryFromRoadmatrix::deinitialize() {
    return true;
}

void TrajectoryFromRoadmatrix::configsChanged() {
    configureImpl();
}

bool TrajectoryFromRoadmatrix::cycle() {
    trajectory->clear();

    float carWidthMeter = config().get<float>("carWidth", 0.2);
    int carWidthCells = ceil(carWidthMeter / roadMatrix->cellWidth());

    std::unique_ptr<LanePieceMatrix> lanePieceMatrix =
        impl->createLanePieceMatrix(carWidthCells, *roadMatrix);

    std::unique_ptr<LanePieceTrajectory> lanePieceTrajectory =
        impl->getOptimalLanePieceTrajectory(*lanePieceMatrix);

    fillTrajectory(*lanePieceTrajectory);

    return true;
}

void TrajectoryFromRoadmatrix::configureImpl() {}

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
