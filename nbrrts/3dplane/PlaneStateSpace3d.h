#ifndef PLANESTATESPACE3D
#define PLANESTATESPACE3D

#pragma once

#include <statespace.h>
#include <Eigen/Dense>


/**
 * @brief A 3d plane with continuous states and no obstacles.
 */
class PlaneStateSpace3d : public StateSpace<Eigen::Vector3f> {
public:
    PlaneStateSpace3d(float width, float length, float height);

    Eigen::Vector3f randomState() const;

    Eigen::Vector3f intermediateState(const Eigen::Vector3f &source, const Eigen::Vector3f &target, float stepSize) const;

    double distance(const Eigen::Vector3f &from, const Eigen::Vector3f &to) const;

    /**
     * Returns a boolean indicating whether the given point is within bounds.
     */
    bool stateValid(const Eigen::Vector3f &pt) const;

    float width() const;
    float length() const;
    float height() const;

private:
    float _width, _length, _height;
};


#endif // PLANESTATESPACE


