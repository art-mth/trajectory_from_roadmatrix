#include "trajectory_from_roadmatrix.h"

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
    logger.error("1");
    trajectory->clear();
    logger.error("2");

    std::unique_ptr<LanePieceMatrix> lanePieceMatrix =
        impl->createLanePieceMatrix(*roadMatrix);
    logger.error("3");

    std::unique_ptr<LanePieceTrajectory> lanePieceTrajectory =
        impl->getOptimalLanePieceTrajectory(*lanePieceMatrix);
    logger.error("4");

    impl->fillTrajectory(*lanePieceTrajectory, *trajectory);
    logger.error("5");

    return true;
}

void TrajectoryFromRoadmatrix::configureImpl() {
    impl->setCarWidthMeter(config().get<float>("carWidthMeter", 0.2));
    impl->setObstacleClearanceMeter(
        config().get<float>("setObstacleClearanceMeter", 0.5));
}
