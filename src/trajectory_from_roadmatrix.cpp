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

    impl->fillTrajectory(*lanePieceTrajectory, *trajectory);

    return true;
}

void TrajectoryFromRoadmatrix::configureImpl() {}
