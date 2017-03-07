#pragma once

#include <3dplane/PlaneStateSpace3d.h>
#include <3dplane/ObstacleGrid3d.h>
#include <Eigen/Dense>


/**
 * @brief A 3d plane with continuous states and discretized obstacles.
 * @details The state space is broken up into a grid with the given discrete height and widths.
 */
class GridStateSpace3d : public PlaneStateSpace3d {
public:
    GridStateSpace3d(float width, float length, float height, int discretizedWidth, int discretizedLength, int discretizedHeight);

    /**
     * Returns a boolean indicating whether the given point is within bounds and obstacle-free.
     */
    bool stateValid(const Eigen::Vector3f &pt) const;
    bool transitionValid(const Eigen::Vector3f &from, const Eigen::Vector3f &to) const;

    Eigen::Vector3f intermediateState(const Eigen::Vector3f &source, const Eigen::Vector3f &target, float minStepSize, float maxStepSize) const;

    const ObstacleGrid3d &obstacleGrid() const;
    ObstacleGrid3d &obstacleGrid();

protected:
    bool transitionValid2d(const Eigen::Vector2f &from, const Eigen::Vector2f &to, int x, int y, float gridSqWidth, float gridSqHeight) const;

private:
    ObstacleGrid3d _obstacleGrid;
};

