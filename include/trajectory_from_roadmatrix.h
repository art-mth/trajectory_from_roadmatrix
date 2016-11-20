#ifndef TRAJECTORY_FROM_ROADMATRIX_H
#define TRAJECTORY_FROM_ROADMATRIX_H

#include <lms/module.h>
#include <street_environment/roadmatrix.h>
#include <street_environment/trajectory.h>

#include "trajectory_from_roadmatrix_impl.h"

/**
 * @brief LMS module trajectory_from_roadmatrix
 * Given a matrix representation of the road creates a trajectory that tries to
 * optimize on certain criteria.
 *
 * Current criteria for trajectory choice include:
 * - Avoiding obstacles
 * - Staying close to the "perfect" trajectory (middle of right lane)
 *
 * This also means trying to wait close to the perfect trajectory in uncertain
 * situations (e.g. Even though the road is not blocked there is not enough
 * space to overtake).
 **/
class TrajectoryFromRoadmatrix : public lms::Module {
   public:
    bool initialize() override;
    bool deinitialize() override;
    void configsChanged() override;
    bool cycle() override;

   private:
    lms::ReadDataChannel<street_environment::RoadMatrix> roadmatrix;
    lms::WriteDataChannel<street_environment::Trajectory> trajectory;

    std::unique_ptr<TrajectoryFromRoadmatrixImpl> impl;

    void configureImpl();
};

#endif  // TRAJECTORY_FROM_ROADMATRIX_H
