#include <3dplane/PlaneStateSpace3d.h>

using namespace Eigen;


PlaneStateSpace3d::PlaneStateSpace3d(float width, float length, float height) {
    _width = width;
    _length=length;
    _height = height;
}

Vector3f PlaneStateSpace3d::randomState() const {
    return Vector3f(drand48() * width(),drand48() * length(), drand48() * height());
}

Vector3f PlaneStateSpace3d::intermediateState(const Vector3f &source, const Vector3f &target, float stepSize) const {
    Vector3f delta = target - source;
    delta = delta / delta.norm();   //  unit vector

    Vector3f val = source + delta * stepSize;
    return val;
}

double PlaneStateSpace3d::distance(const Eigen::Vector3f &from, const Eigen::Vector3f &to) const {
    Vector3f delta = from - to;
    return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2) + powf(delta.z(), 2));
}

bool PlaneStateSpace3d::stateValid(const Vector3f &pt) const {
        return pt.x() >= 0
            && pt.y() >= 0
            && pt.z() >= 0
            && pt.x() < width()
            && pt.y() < length()
            && pt.z() < height();
}

float PlaneStateSpace3d::width() const {
    return _width;
}

float PlaneStateSpace3d::height() const {
    return _height;
}

float PlaneStateSpace3d::length() const{
    return _length;
}
