#include <3dplane/3dplane.h>
#include <functional>
#include <math.h>

using namespace Eigen;
using namespace std;


RRTStar::rrtStarTree<Vector3f> *rrtStarTreeFor3dPlane(shared_ptr<StateSpace<Eigen::Vector3f>> stateSpace,
    Vector3f goal,
    float step) {
    RRTStar::rrtStarTree<Vector3f> *rrt = new RRTStar::rrtStarTree<Vector3f>(stateSpace);

    rrt->setStepSize(step);

    rrt->setGoalState(goal);

    return rrt;
}

RRT::Tree<Vector3f> *rrtTreeFor3dPlane(shared_ptr<StateSpace<Eigen::Vector3f>> stateSpace,
    Vector3f goal,
    float step) {
    RRT::Tree<Vector3f> *rrt = new RRT::Tree<Vector3f>(stateSpace);

    rrt->setStepSize(step);

    rrt->setGoalState(goal);

    return rrt;
}


