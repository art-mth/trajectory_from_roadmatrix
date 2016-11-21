#ifndef TRAJECTORY_FROM_ROADMATRIX_IMPL_H_
#define TRAJECTORY_FROM_ROADMATRIX_IMPL_H_

#include <memory>
#include <vector>

#include <street_environment/roadmatrix.h>
#include <street_environment/trajectory.h>

struct LanePiece {
    std::vector<street_environment::RoadMatrixCell> cells;
    int value;
    bool stop;
};

using LanePieceMatrix = std::vector<std::vector<LanePiece>>;
using LanePieceTrajectory = std::vector<LanePiece>;

struct TrajectoryFromRoadmatrixImpl {
    void calculateCycleConstants(
        const street_environment::RoadMatrix& roadMatrix);

    std::unique_ptr<LanePieceMatrix> createLanePieceMatrix(
        const street_environment::RoadMatrix& roadMatrix) const;

    std::unique_ptr<LanePieceTrajectory> getOptimalLanePieceTrajectory(
        const LanePieceMatrix& lanePieceMatrix) const;

    bool fillTrajectory(const LanePieceTrajectory& lanePieceTrajectory,
                        street_environment::Trajectory& trajectory) const;

    int valueFunction(const street_environment::RoadMatrixCell& cell,
                      const street_environment::RoadMatrix& roadMatrix) const;

    void setCarWidthMeter(float carWidthMeter) {
        m_carWidthMeter = carWidthMeter;
    }
    void setObstacleClearanceLeftFrontMeter(float obstacleClearanceMeter) {
        m_obstacleClearanceLeftFrontMeter = obstacleClearanceMeter;
    }
    void setObstacleClearanceRightFrontMeter(float obstacleClearanceMeter) {
        m_obstacleClearanceRightFrontMeter = obstacleClearanceMeter;
    }
    void setObstacleClearanceLeftBackMeter(float obstacleClearanceMeter) {
        m_obstacleClearanceLeftBackMeter = obstacleClearanceMeter;
    }
    void setObstacleClearanceRightBackMeter(float obstacleClearanceMeter) {
        m_obstacleClearanceRightBackMeter = obstacleClearanceMeter;
    }

   private:
    ////////////////////////////// Config Values ///////////////////////////////
    float m_carWidthMeter;

    float m_obstacleClearanceLeftFrontMeter;
    float m_obstacleClearanceRightFrontMeter;
    float m_obstacleClearanceLeftBackMeter;
    float m_obstacleClearanceRightBackMeter;

    ///////////////////////////// Cycle Constants //////////////////////////////
    int m_cellsPerLane;
    int m_carWidthCells;
    int m_perfectTrajectory;
    int m_numLanes;
    // Defines the maximum value a cell can have without the obstacle bonus.
    int m_maxCellValue;
    // Defines the maximum value a LanePiece can have without the obstacle
    // bonus.
    int m_maxLanePieceValue;

    int m_obstacleClearanceLeftFrontCells;
    int m_obstacleClearanceRightFrontCells;
    int m_obstacleClearanceLeftBackCells;
    int m_obstacleClearanceRightBackCells;
};

#endif  // TRAJECTORY_FROM_ROADMATRIX_IMPL_H_
