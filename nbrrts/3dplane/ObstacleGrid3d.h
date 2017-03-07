#ifndef OBSTACLEGRID3D
#define OBSTACLEGRID3D

#include <Eigen/Dense>


/**
 * @brief Handles a grid of obstacles laid over a continuous 3d plane.
 * @details The state space is broken up into a grid with the given discrete height and widths.
 */
class ObstacleGrid3d {
public:
    ObstacleGrid3d(float width, float length, float height, int discretizedWidth, int discretizedLength, int discretizedHeight);
    ~ObstacleGrid3d();

    Eigen::Vector3i gridSquareForLocation(const Eigen::Vector3f &loc) const;

    /**
     * Finds the distance from state to its neareset obstacle. Only searches up to maxDist around
     * state so as to not waste time checking far away and irrelevant obstacles.
     *
     * @param state The location to search with respect to for the nearest obstacle dist
     * @param maxDist The maximum vertical and horizontal distance from state to search for obstacles
     */
    float nearestObstacleDist(const Eigen::Vector3f &state, float maxDist) const;
    void clear();
    bool &obstacleAt(double x, double y, double z);
    bool obstacleAt(double x, double y, double z) const;
    bool &obstacleAt(const Eigen::Vector3i &gridLoc);
    bool obstacleAt(const Eigen::Vector3i &gridLoc) const;

    int discretizedWidth() const;
    int discretizedLength() const;
    int discretizedHeight() const;

    float width() const;
    float length() const;
    float height() const;


private:
    int _discretizedWidth, _discretizedLength, _discretizedHeight;
    float _width, _length, _height;

    /// 3d array of obstacles
    bool *_obstacles;
};

#endif // OBSTACLEGRID


