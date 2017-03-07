#include <3dplane/ObstacleGrid3d.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

using namespace Eigen;
using namespace std;


ObstacleGrid3d::ObstacleGrid3d(float width, float length, float height, int discretizedWidth, int discretizedLength, int discretizedHeight) {
    _width = width;
    _length = length;
    _height = height;
    _discretizedWidth = discretizedWidth;
    _discretizedLength = discretizedLength;
    _discretizedHeight = discretizedHeight;

    _obstacles = (bool *)malloc(sizeof(bool) * discretizedWidth * discretizedLength * discretizedHeight);

    clear();
}

ObstacleGrid3d::~ObstacleGrid3d() {
    free(_obstacles);
}

Vector3i ObstacleGrid3d::gridSquareForLocation(const Vector3f &loc) const {
    return Vector3i(loc.x() / width() * discretizedWidth(),
                    loc.y() / length() * discretizedLength(),
                    loc.z() / height() * discretizedHeight());
}

float ObstacleGrid3d::nearestObstacleDist(const Vector3f &state, float maxDist) const {
    //x and y and z are the indices of the cell that state is located in
    float x = (state.x() / (_width / _discretizedWidth));
    float y = (state.y() / (_length / _discretizedLength));
    float z = (state.z() / (_height / _discretizedHeight));
    int xSearchRad = maxDist * _discretizedWidth / _width;
    int ySearchRad = maxDist * _discretizedLength / _length;
    int zSearchRad = maxDist * _discretizedHeight / _height;
    //here we loop through the cells around (x,y,z) to find the minimum distance of the point to the nearest obstacle
    for (int i = x - xSearchRad; i <= x + xSearchRad; i++) {
        for (int j = y - ySearchRad; j <= y + ySearchRad; j++) {
            for(int m = z - zSearchRad; m<= z + zSearchRad; m++)
            {
                bool obs = obstacleAt(i, j, m);
                if (obs) {
                    float xDist = (x-i)*_width / _discretizedWidth;
                    float yDist = (y-j)*_length / _discretizedLength;
                    float zDist = (z-m)*_height / _discretizedHeight;
                    float dist = sqrtf(powf(xDist, 2) + powf(yDist, 2) + powf(zDist, 2));
                    if (dist < maxDist) {
                        maxDist = dist;
                    }
                }
            }
        }
    }

    // the boundaries of the grid count as obstacles
    maxDist = std::min(maxDist, state.x());               // left boundary
    maxDist = std::min(maxDist, width() - state.x());     // right boundary
    maxDist = std::min(maxDist, state.y());               // front boundary
    maxDist = std::min(maxDist, length() - state.y());    // back boundary
    maxDist = std::min(maxDist, state.z());               // top boundary
    maxDist = std::min(maxDist, height() - state.z());    // bottom boundary

    return maxDist;
}

void ObstacleGrid3d::clear() {
    for (int x = 0; x < discretizedWidth(); x++) {
        for (int y = 0; y < discretizedLength(); y++) {
            for(int z = 0; z < discretizedHeight(); z++)
            {
                obstacleAt(x, y, z) = false;
            }
        }
    }
}

bool &ObstacleGrid3d::obstacleAt(double x, double y, double z) {
    int xint=floor(x),yint=floor(y),zint=floor(z);
    return _obstacles[xint + _discretizedWidth*yint + _discretizedLength*_discretizedWidth*zint];
}

bool ObstacleGrid3d::obstacleAt(double x, double y, double z) const {
    int xint=floor(x),yint=floor(y),zint=floor(z);
    return _obstacles[xint + _discretizedWidth*yint + _discretizedLength*_discretizedWidth*zint];
}

bool &ObstacleGrid3d::obstacleAt(const Vector3i &gridLoc) {
    return obstacleAt(gridLoc.x(), gridLoc.y(), gridLoc.z());
}

bool ObstacleGrid3d::obstacleAt(const Vector3i &gridLoc) const {
    return obstacleAt(gridLoc.x(), gridLoc.y(), gridLoc.z());
}

int ObstacleGrid3d::discretizedWidth() const {
    return _discretizedWidth;
}

int ObstacleGrid3d::discretizedLength() const {
    return _discretizedLength;
}

int ObstacleGrid3d::discretizedHeight() const {
    return _discretizedHeight;
}

float ObstacleGrid3d::width() const {
    return _width;
}

float ObstacleGrid3d::length() const{
    return _length;
}

float ObstacleGrid3d::height() const {
    return _height;
}

