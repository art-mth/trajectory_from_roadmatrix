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
    impl->calculateCycleConstants(*roadMatrix);

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

    impl->setObstacleClearanceMeterFrontCurrentLane(
        config().get<float>("obstacleClearanceMeterFrontCurrentLane", 0.5));
    impl->setObstacleClearanceMeterFrontOtherLane(
        config().get<float>("obstacleClearanceMeterFrontOtherLane", 1.5));
    impl->setObstacleClearanceMeterBackCurrentLane(
        config().get<float>("obstacleClearanceMeterBackCurrentLane", 0.5));
    impl->setObstacleClearanceMeterBackOtherLane(
        config().get<float>("obstacleClearanceMeterBackOtherLane", 0.5));
}
