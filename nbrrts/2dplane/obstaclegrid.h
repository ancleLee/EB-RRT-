#ifndef OBSTACLEGRID
#define OBSTACLEGRID

#include <Eigen/Dense>
#include <vector>

/**
 * @brief Handles a grid of obstacles laid over a continuous 2d plane.
 * @details The state space is broken up into a grid with the given discrete height and widths.
 */
class ObstacleGrid {
public:
    ObstacleGrid(float width, float height, int discretizedWidth, int discretizedHeight);
    ~ObstacleGrid();

    Eigen::Vector2i gridSquareForLocation(const Eigen::Vector2f &loc) const;

    /**
     * Finds the distance from state to its neareset obstacle. Only searches up to maxDist around
     * state so as to not waste time checking far away and irrelevant obstacles.
     *
     * @param state The location to search with respect to for the nearest obstacle dist
     * @param maxDist The maximum vertical and horizontal distance from state to search for obstacles
     */
    float nearestObstacleDist(const Eigen::Vector2f &state, float maxDist) const;
    //std::vector<Eigen::Vector2f> nearObstacle(Eigen::Vector2f state, float maxDist);
    int *nearObstacle(const Eigen::Vector2f &state, float maxDist);
    std::vector<Eigen::Vector2f> intermediateStates(const Eigen::Vector2f state, const int *inform);
    int getRelativePosition(const Eigen::Vector2f &state, const Eigen::Vector2f &to);
    int getLocationDest(const Eigen::Vector2f &state, int initW, int initH);
    
    void clear();
    bool &obstacleAt(int x, int y);
    bool obstacleAt(int x, int y) const;
    bool &obstacleAt(const Eigen::Vector2i &gridLoc);
    bool obstacleAt(const Eigen::Vector2i &gridLoc) const;

    int discretizedWidth() const;
    int discretizedHeight() const;
    float width() const;
    float height() const;


private:
    int _discretizedWidth, _discretizedHeight;
    float _width, _height;

    /// 2d array of obstacles
    bool *_obstacles;
};

#endif // OBSTACLEGRID

