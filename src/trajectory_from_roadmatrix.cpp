#include "trajectory_from_roadmatrix.h"

#include <memory>
#include <vector>

bool TrajectoryFromRoadmatrix::initialize() {
    roadmatrix = readChannel<street_environment::RoadMatrix>("ROADMATRIX");
    trajectory = writeChannel<street_environment::Trajectory>("TRAJECTORY");

    impl = std::unique_ptr<TrajectoryFromRoadmatrixImpl>(
        new TrajectoryFromRoadmatrixImpl);
    configureImpl();

    return true;
}

bool TrajectoryFromRoadmatrix::deinitialize() { return true; }

void TrajectoryFromRoadmatrix::configsChanged() { configureImpl(); }

bool TrajectoryFromRoadmatrix::cycle() {
    impl->calculateCycleConstants(*roadmatrix);

    trajectory->clear();

    std::unique_ptr<LanePieceMatrix> lanePieceMatrix =
        impl->createLanePieceMatrix(*roadmatrix);

    std::unique_ptr<LanePieceTrajectory> lanePieceTrajectory =
        impl->getOptimalLanePieceTrajectory(*lanePieceMatrix);

    impl->fillTrajectory(*lanePieceTrajectory, *trajectory);

    return true;
}

void TrajectoryFromRoadmatrix::configureImpl() {
    impl->setCarWidthMeter(config().get<float>("carWidthMeter", 0.2));

    impl->setObstacleClearanceLeftFrontMeter(
        config().get<float>("obstacleClearanceLeftFrontMeter", 1.5));
    impl->setObstacleClearanceRightFrontMeter(
        config().get<float>("obstacleClearanceRightFrontMeter", 0.5));
    impl->setObstacleClearanceLeftBackMeter(
        config().get<float>("obstacleClearanceLeftBackMeter", 0.5));
    impl->setObstacleClearanceRightBackMeter(
        config().get<float>("obstacleClearanceRightBackMeter", 0.5));
}
