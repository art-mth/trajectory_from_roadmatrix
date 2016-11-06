#ifndef TRAJECTORY_FROM_ROADMATRIX_H
#define TRAJECTORY_FROM_ROADMATRIX_H

#include <lms/module.h>
#include <street_environment/road_matrix/road_matrix_to_trajectory.h>
#include <street_environment/road_matrix/road_matrix.h>
#include <street_environment/trajectory.h>

/**
 * @brief LMS module trajectory_from_roadmatrix
 **/
class TrajectoryFromRoadmatrix : public lms::Module {
  private:
    lms::ReadDataChannel<street_environment::RoadMatrix> roadMatrix;
    lms::WriteDataChannel<street_environment::Trajectory> trajectory;

    bool fillTrajectory(const road_matrix_to_trajectory::LanePieceTrajectory&
                        lanePieceTrajectory);
  public:
    bool initialize() override;
    bool deinitialize() override;
    bool cycle() override;
};

#endif // TRAJECTORY_FROM_ROADMATRIX_H
