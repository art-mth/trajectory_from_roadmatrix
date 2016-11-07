#ifndef TRAJECTORY_FROM_ROADMATRIX_H
#define TRAJECTORY_FROM_ROADMATRIX_H

#include <street_environment/road_matrix/trajectory_from_roadmatrix_impl.h>

#include <lms/module.h>
#include <street_environment/road_matrix/road_matrix.h>
#include <street_environment/trajectory.h>

/**
 * @brief LMS module trajectory_from_roadmatrix
 **/
class TrajectoryFromRoadmatrix : public lms::Module {
  public:
    bool initialize() override;
    bool deinitialize() override;
    void configsChanged() override;
    bool cycle() override;

  private:
    lms::ReadDataChannel<street_environment::RoadMatrix> roadMatrix;
    lms::WriteDataChannel<street_environment::Trajectory> trajectory;

    std::unique_ptr<TrajectoryFromRoadmatrixImpl> impl;

    void configureImpl();
};

#endif // TRAJECTORY_FROM_ROADMATRIX_H
