#include "trajectory_from_roadmatrix.h"

#include <memory>
#include <vector>

bool TrajectoryFromRoadmatrix::initialize() {
    roadMatrix = readChannel<street_environment::RoadMatrix>("ROADMATRIX");
    trajectory = writeChannel<street_environment::Trajectory>("TRAJECTORY");

    impl = std::unique_ptr<TrajectoryFromRoadmatrixImpl>(
        new TrajectoryFromRoadmatrixImpl);
    configureImpl();

    return true;
}

bool TrajectoryFromRoadmatrix::deinitialize() { return true; }

void TrajectoryFromRoadmatrix::configsChanged() { configureImpl(); }

bool TrajectoryFromRoadmatrix::cycle() {
    trajectory->clear();

    std::unique_ptr<LanePieceMatrix> lanePieceMatrix =
        impl->createLanePieceMatrix(*roadMatrix);

    std::unique_ptr<LanePieceTrajectory> lanePieceTrajectory =
        impl->getOptimalLanePieceTrajectory(*lanePieceMatrix);

    impl->fillTrajectory(*lanePieceTrajectory, *trajectory);

    return true;
}

void TrajectoryFromRoadmatrix::configureImpl() {
    impl->setCarWidthMeter(config().get<float>("carWidthMeter", 0.2));
    impl->setObstacleClearanceMeter(
        config().get<float>("setObstacleClearanceMeter", 0.5));
}
