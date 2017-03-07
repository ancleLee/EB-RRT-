#include <3dplane/GridStateSpace3d.h>
#include <util.h>
#include <stdexcept>
#include <math.h>
#include <QDebug>

using namespace Eigen;
using namespace std;

#include <iostream>


GridStateSpace3d::GridStateSpace3d(float width, float length, float height, int discretizedWidth, int discretizedLength, int discretizedHeight):
    PlaneStateSpace3d(width, length, height),
    _obstacleGrid(width, length, height, discretizedWidth, discretizedLength, discretizedHeight) {
}

bool GridStateSpace3d::stateValid(const Vector3f &pt) const {
    return PlaneStateSpace3d::stateValid(pt) && !_obstacleGrid.obstacleAt(_obstacleGrid.gridSquareForLocation(pt));
}

Vector3f GridStateSpace3d::intermediateState(const Vector3f &source, const Vector3f &target, float minStepSize, float maxStepSize) const {
    bool debug;

    Vector3f delta = target - source;
    delta = delta / delta.norm();   //  unit vector
    float dist = _obstacleGrid.nearestObstacleDist(source, maxStepSize * 2);


    float stepSize = (dist / maxStepSize) * minStepSize; // scale based on how far we are from obstacles
    if (stepSize > maxStepSize) stepSize = maxStepSize;
    if (stepSize < minStepSize) stepSize = minStepSize;
    if (debug) {
        cout << "ASC intermediateState" << endl;
        cout << "  stepsize: " << minStepSize << endl;
        cout << "  nearest obs dist: " << dist << endl;
        cout << "  maximum stepsize: " << maxStepSize << endl;
        cout << "  new step: " << stepSize << endl;
    }
    
    Vector3f val = source + delta * stepSize;
    return val;
}

bool GridStateSpace3d::transitionValid(const Vector3f &from, const Vector3f &to) const {
    //  make sure we're within bounds
    if (!stateValid(to)) return false;

    Vector3f delta = to - from;

    //  get the corners of this segment in integer coordinates.  This limits our intersection test to only the boxes in that square
    Vector3i discreteFrom = _obstacleGrid.gridSquareForLocation(from);
    Vector3i discreteTo = _obstacleGrid.gridSquareForLocation(to);
    int x1 = discreteFrom.x(), y1 = discreteFrom.y(), z1=discreteFrom.z();
    int x2 = discreteTo.x(), y2 = discreteTo.y(), z2=discreteTo.z();


    //  order ascending
    if (x1 > x2) swap<int>(x1, x2);
    if (y1 > y2) swap<int>(y1, y2);
    if (z1 > z2) swap<int>(z1, z2);

    float gridSqWidth = width() / _obstacleGrid.discretizedWidth();
    float gridSqLength = length() / _obstacleGrid.discretizedLength();
    float gridSqHeight = height() / _obstacleGrid.discretizedHeight();

    //  check all squares from (x1, y1) to (x2, y2)
    for (int x = x1; x <= x2; x++) {
        for (int y = y1; y <= y2; y++) {
            for (int z = z1; z <= z2; z++)
            {
                if(_obstacleGrid.obstacleAt(x,y,z))
                {
                    //plane xy
                    Vector2f from2dxy(from.x(),from.y());
                    Vector2f to2dxy(to.x(),to.y());
                    bool safexy=transitionValid2d(from2dxy,to2dxy,x,y,gridSqWidth,gridSqLength);
                    //plane yz
                    Vector2f from2dyz(from.y(),from.z());
                    Vector2f to2dyz(to.y(),to.z());
                    bool safeyz=transitionValid2d(from2dyz,to2dyz,y,z,gridSqLength,gridSqHeight);
                    //plane xz
                    Vector2f from2dxz(from.x(),from.z());
                    Vector2f to2dxz(to.x(),to.z());
                    bool safexz=transitionValid2d(from2dxz,to2dxz,x,z,gridSqWidth,gridSqHeight);
                    //_---------------------------------
                    //qDebug()<<"from:"<<discreteFrom.x()<<","<<discreteFrom.y()<<","<<discreteFrom.z();
                    //qDebug()<<"to:"<<discreteTo.x()<<","<<discreteTo.y()<<","<<discreteTo.z();
                    //qDebug()<<"(x,y,z)=:"<<x<<","<<y<<","<<z;
                    //qDebug()<<"xy?:"<<safexy<<" yz?:"<<safeyz<<" xz?:"<<safexz;
                    //----------------------------------

                    if(!(safexy||safeyz||safexz))
                    {
                        //qDebug()<<"find one path with ob!";
                        return false;
                    }
                }
            }


        }
    }

    return true;
}

bool GridStateSpace3d::transitionValid2d(const Vector2f &from, const Vector2f &to, int x, int y, float gridSqWidth, float gridSqHeight) const {

    Vector2f delta = to - from;

    //  there's an obstacle here, so check for intersection

    //  the corners of this obstacle square
    Vector2f ulCorner(x * gridSqWidth, y * gridSqHeight);
    Vector2f brCorner(ulCorner.x() + gridSqWidth, ulCorner.y() + gridSqHeight);

    if (delta.x() != 0)
    {
        /**
         * Find slope and y-intercept of the line passing through @from and @to.
         * y1 = m*x1+b
         * b = y1-m*x1
         */
        float slope = delta.y() / delta.x();
        float b = to.y() - to.x()*slope;

        /*
         * First check intersection with the vertical segments of the box.  Use y=mx+b for the from-to line and plug in the x value for each wall
         * If the corresponding y-value is within the y-bounds of the vertical segment, it's an intersection.
         */
        float yInt = slope*ulCorner.x() + b;
        if (inRange<float>(yInt, ulCorner.y(), brCorner.y())) return false;
        yInt = slope*brCorner.x() + b;
        if (inRange<float>(yInt, ulCorner.y(), brCorner.y())) return false;

        /*
         * Check intersection with horizontal sides of box
         * y = k;
         * y = mx+b;
         * mx+b = k;
         * mx = k - b;
         * (k - b) / m = x;  is x within the horizontal range of the box?
         */
        if (slope == 0) return false;
        float xInt = (ulCorner.y() - b) / slope;
        if (inRange<float>(xInt, ulCorner.x(), brCorner.x())) return false;
        xInt = (brCorner.y() - b) / slope;
        if (inRange<float>(xInt, ulCorner.x(), brCorner.x())) return false;
    }
    else
    {
        //  vertical line - slope undefined

        //  see if it's within the x-axis bounds of this obstacle box
        if (inRange<float>(from.x(), ulCorner.x(), brCorner.x()))
        {
            //  order by y-value
            //  note: @lower has a smaller value of y, but will appear higher visually on the screen due to qt's coordinate layout
            Vector2f lower(from);
            Vector2f higher(to);
            if (higher.y() < lower.y()) swap<Vector2f>(lower, higher);

            //  check for intersection based on y-values
            if (lower.y() < ulCorner.y() && higher.y() > ulCorner.y()) return false;
            if (lower.y() < brCorner.y() && higher.y() > brCorner.y()) return false;
        }
    }
    return true;
}

const ObstacleGrid3d &GridStateSpace3d::obstacleGrid() const {
    return _obstacleGrid;
}

ObstacleGrid3d &GridStateSpace3d::obstacleGrid() {
    return _obstacleGrid;
}

