#include "trajectory_from_roadmatrix_impl.h"

#include <math.h>
#include <stdlib.h>

#include <lms/math/vertex.h>

namespace {
const int kLaneValueStep = 1;
const float kPerfectTrajectoryFactor = 0.75;
const lms::math::vertex2f kCarPosition = lms::math::vertex2f(0, 0);
}

void TrajectoryFromRoadmatrixImpl::calculateCycleConstants(
    const street_environment::RoadMatrix& roadMatrix) {
    m_cellsPerLane = roadMatrix.width() / 2;
    m_carWidthCells = ceil(m_carWidthMeter / roadMatrix.cellWidth());
    m_perfectTrajectory = roadMatrix.width() * kPerfectTrajectoryFactor;
    m_numLanes = roadMatrix.width() - m_carWidthCells + 1;
    m_maxCellValue = m_perfectTrajectory * kLaneValueStep;
    m_maxLanePieceValue = m_maxCellValue * m_carWidthCells + 1;

    m_obstacleClearanceLeftFrontCells =
        ceil(m_obstacleClearanceLeftFrontMeter / roadMatrix.cellLength());
    m_obstacleClearanceRightFrontCells =
        ceil(m_obstacleClearanceRightFrontMeter / roadMatrix.cellLength());
    m_obstacleClearanceLeftBackCells =
        ceil(m_obstacleClearanceLeftBackMeter / roadMatrix.cellLength());
    m_obstacleClearanceRightBackCells =
        ceil(m_obstacleClearanceRightBackMeter / roadMatrix.cellLength());
}

std::unique_ptr<LanePieceMatrix>
TrajectoryFromRoadmatrixImpl::createLanePieceMatrix(
    const street_environment::RoadMatrix& roadMatrix) const {
    std::unique_ptr<LanePieceMatrix> lanePieceMatrix(new LanePieceMatrix(
        roadMatrix.length(), std::vector<LanePiece>(m_numLanes)));
    for (int x = 0; x < roadMatrix.length(); x++) {
        for (int y = 0; y < m_numLanes; y++) {
            int value = 0;
            LanePiece lanePiece;
            for (int i = 0; i < m_carWidthCells; i++) {
                const auto& cell = roadMatrix.cell(x, y + i);
                value += valueFunction(cell, roadMatrix);
                lanePiece.cells.push_back(cell);
            }

            // Scale the whole lane piece down so that lane pieces that are
            // partially blocked and fully blocked are comparable.
            if (value < m_carWidthCells * m_maxLanePieceValue) {
                int free_cells = value / m_maxLanePieceValue;
                value -= free_cells * m_maxLanePieceValue;
            }
            lanePiece.value = value;
            lanePieceMatrix->at(x).at(y) = lanePiece;
        }
    }
    return lanePieceMatrix;
}

std::unique_ptr<LanePieceTrajectory>
TrajectoryFromRoadmatrixImpl::getOptimalLanePieceTrajectory(
    const LanePieceMatrix& lanePieceMatrix) const {
    std::unique_ptr<LanePieceTrajectory> cellLane(new LanePieceTrajectory);
    for (const auto& pieces : lanePieceMatrix) {
        if (pieces.size() > 0) {
            const LanePiece* bestPiece = &pieces[0];
            for (const auto& piece : pieces) {
                if (piece.value > bestPiece->value) {
                    bestPiece = &piece;
                }
            }
            // The road is blocked. No need to calculate the trajectory any
            // further.
            if (bestPiece->value < m_maxLanePieceValue * m_carWidthCells) {
                cellLane->push_back(*bestPiece);
                cellLane->back().stop = true;
                return cellLane;
            }
            cellLane->push_back(*bestPiece);
        }
    }
    return cellLane;
}

bool TrajectoryFromRoadmatrixImpl::fillTrajectory(
    const LanePieceTrajectory& lanePieceTrajectory,
    street_environment::Trajectory& trajectory) const {
    street_environment::TrajectoryPoint prevTp;
    street_environment::TrajectoryPoint curTp;
    prevTp.position = kCarPosition;

    for (const LanePiece& piece : lanePieceTrajectory) {
        curTp.position =
            (piece.cells.front().points[1] + piece.cells.back().points[2]) / 2;
        if (piece.stop) {
            curTp.velocity = 0;
        } else {
            curTp.velocity = 1;
        }
        curTp.directory = (curTp.position - prevTp.position).normalize();
        trajectory.push_back(curTp);
        prevTp = curTp;
    }

    return true;
}

int TrajectoryFromRoadmatrixImpl::valueFunction(
    const street_environment::RoadMatrixCell& cell,
    const street_environment::RoadMatrix& roadMatrix) const {
    int value =
        m_maxCellValue - (abs(m_perfectTrajectory - cell.y) * kLaneValueStep);

    if (cell.hasObstacle) {
        return value - (m_maxLanePieceValue * m_carWidthCells);
    }

    if (obstacleInClearanceArea(cell, roadMatrix)) {
        return value;
    }

    value += m_maxLanePieceValue;
    return value;
}

bool TrajectoryFromRoadmatrixImpl::obstacleInClearanceArea(
    const street_environment::RoadMatrixCell& cell,
    const street_environment::RoadMatrix& roadMatrix) const {
    if (cell.y < m_perfectTrajectory - m_carWidthCells / 2) {
        for (int x = 1; x <= m_obstacleClearanceLeftFrontCells; x++) {
            if ((cell.x + x < roadMatrix.length()) &&
                (roadMatrix.cell(cell.x + x, cell.y).hasObstacle)) {
                return true;
            }
        }
        for (int x = 1; x <= m_obstacleClearanceLeftBackCells; x++) {
            if ((cell.x - x >= 0) &&
                (roadMatrix.cell(cell.x - x, cell.y).hasObstacle)) {
                return true;
            }
        }
    } else {
        for (int x = 1; x <= m_obstacleClearanceRightFrontCells; x++) {
            if ((cell.x + x < roadMatrix.length()) &&
                (roadMatrix.cell(cell.x + x, cell.y).hasObstacle)) {
                return true;
            }
        }
        for (int x = 1; x <= m_obstacleClearanceRightBackCells; x++) {
            if ((cell.x - x >= 0) &&
                (roadMatrix.cell(cell.x - x, cell.y).hasObstacle)) {
                return true;
            }
        }
    }

    return false;
}
