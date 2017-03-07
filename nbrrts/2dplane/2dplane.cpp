#include "2dplane.h"
#include <functional>
#include <math.h>

using namespace Eigen;
using namespace std;


RRTStar::rrtStarTree<Vector2f> *rrtStarTreeFor2dPlane(shared_ptr<StateSpace<Eigen::Vector2f>> stateSpace,
    Vector2f goal,
    float step) {
    RRTStar::rrtStarTree<Vector2f> *rrt = new RRTStar::rrtStarTree<Vector2f>(stateSpace);

    rrt->setStepSize(step);

    rrt->setGoalState(goal);

    return rrt;
}

RRT::Tree<Vector2f> *rrtTreeFor2dPlane(shared_ptr<StateSpace<Eigen::Vector2f>> stateSpace,
    Vector2f goal,
    float step) {
    RRT::Tree<Vector2f> *rrt = new RRT::Tree<Vector2f>(stateSpace);

    rrt->setStepSize(step);

    rrt->setGoalState(goal);

    return rrt;
}

