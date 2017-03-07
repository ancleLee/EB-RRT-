#ifndef TREE
#define TREE

#include <statespace.h>
#include <list>
#include <vector>
#include <memory>
#include <functional>
#include <stdexcept>
#include <stdlib.h>
#include <iostream>
#include <node.h>
//#include <QDebug>
namespace RRT
{

    /**
     * An RRT tree searches a state space by randomly filling it in and
     * connecting points to form a branching tree.  Once a branch of the tree
     * reaches the goal and satisifes all of the constraints, a solution is
     * found and returned.
     *
     * This provides a base class for RRT trees.  Because many parts of an RRT are
     * implementation-/domain-specific, several key functionalities are
     * placed in callbacks (C++ lambdas), which must be supplied by the
     * user of this class.
     *
     * If adaptive stepsize control (ASC) is enabled, then the stepsize for extending new nodes from
     * the tree will be dynamically updated depending on how close the nearest obstacle is. If there are
     * no nearby obstacles, then the stepsize will be extended in order to safely cover more ground. If
     * there are nearby obstacles, then the stepsize will shrink so that the RRT can take more precise steps.
     *
     * USAGE:
     * 1) Create a new Tree with the appropriate StateSpace
     *    RRT::Tree<My2dPoint> tree(stateSpace);
     *
     * 2) Set the start and goal states
     *    tree->setStartState(s);
     *    tree->setGoalState(g);
     *
     * 3) (Optional) If adaptive stepsize control is enabled:
     *    _stateSpace->setMaxStepSize sets the maximum stepsize the tree can take for any step.
     *
     * 4) Run the RRT algorithm!  This can be done in one of two ways:
     *    Option 1) Call the run() method - it will grow the tree
     *              until it finds a solution or runs out of iterations.
     *
     *    Option 2) Call grow() repeatedly
     *
     *    Either way works fine, just choose whatever works best for your
     *    application.
     *
     * 5) Use getPath() to get the series of states that make up the solution
     *
     * @param T The type that represents a state within the state-space that
     * the tree is searching.  This could be a 2D Point or something else,
     * but will generally be some sort of vector.
     */
    template<typename T>
    class Tree {
    public:
        Tree(std::shared_ptr<StateSpace<T>> stateSpace) {
            _stateSpace = stateSpace;

            //  default values
            setStepSize(0.1);
            setMaxStepSize(5);
            setMaxIterations(1000);
            setASCEnabled(false);
            setGoalBias(0);
            setWaypointBias(0);
            setGoalMaxDist(0.1);
        }

        virtual ~Tree() {
            reset();
        }


        StateSpace<T> &stateSpace() {
            return *_stateSpace;
        }
        const StateSpace<T> &stateSpace() const {
            return *_stateSpace;
        }


        /**
         * The maximum number of random states in the state-space that we will
         * try before giving up on finding a path to the goal.
         */
        int maxIterations() const {
            return _maxIterations;
        }
        void setMaxIterations(int itr) {
            _maxIterations = itr;
        }


        /**
         * Whether or not the tree is to run with adaptive stepsize control.
         */
        bool isASCEnabled() const {
            return _isASCEnabled;
        }
        void setASCEnabled(bool checked) {
            _isASCEnabled = checked;
        }

        /**
         * @brief The chance we extend towards the goal rather than a random point.
         * @details At each iteration of the RRT algorithm, we extend() towards a particular state.  The goalBias
         * is a number in the range [0, 1] that determines what proportion of the time we extend() towards the goal.
         * The rest of the time, we extend() towards a random state.
         */
        float goalBias() const {
            return _goalBias;
        }
        void setGoalBias(float goalBias) {
            if (goalBias < 0 || goalBias > 1) {
                throw std::invalid_argument("The goal bias must be a number between 0.0 and 1.0");
            }
            _goalBias = goalBias;
        }


        /**
         * @brief The chance that we extend towards a randomly-chosen point from the @waypoints vector
         */
        float waypointBias() const {
            return _waypointBias;
        }
        void setWaypointBias(float waypointBias) {
            if (waypointBias < 0 || waypointBias > 1) {
                throw std::invalid_argument("The waypoint bias must be a number between 0.0 and 1.0");
            }
            _waypointBias = waypointBias;
        }


        /**
         * The waypoints vector holds a series of states that were a part of a previously-generated successful path.
         * Setting these here and setting @waypointBias > 0 will bias tree growth towards these
         */
        const std::vector<T> &waypoints() const {
            return _waypoints;
        }
        void setWaypoints(const std::vector<T> &waypoints) {
            _waypoints = waypoints;
        }
        void clearWaypoints() {
            _waypoints.clear();
        }


        float stepSize() const {
            return _stepSize;
        }
        void setStepSize(float stepSize) {
            _stepSize = stepSize;
        }

        /// Max step size used in ASC
        float maxStepSize() const {
            return _maxStepSize;
        }
        void setMaxStepSize(float maxStep) {
            _maxStepSize = maxStep;
        }


        /**
         * @brief How close we have to get to the goal in order to consider it reached.
         * @details The RRT will continue to run unti we're within @goalMaxDist of the goal state.
         */
        float goalMaxDist() const {
            return _goalMaxDist;
        }
        void setGoalMaxDist(float maxDist) {
            _goalMaxDist = maxDist;
        }


        /**
         * Executes the RRT algorithm with the given start state.
         *
         * @return a bool indicating whether or not it found a path to the goal
         */
        bool run() {
            //  grow the tree until we find the goal or run out of iterations
            for (int i = 0; i < _maxIterations; i++) {
                Node<T> *newNode = grow();

                if (newNode && _stateSpace->distance(newNode->state(), _goalState) < _goalMaxDist) return true;
            }

            //  we hit our iteration limit and didn't reach the goal :(
            return false;
        }

        /**
         * Removes all Nodes from the tree so it can be run() again.
         */
        void reset(bool eraseRoot = false) {
            if (!_nodes.empty()) {
                Node<T> *root = _nodes.front();
                _nodes.erase(_nodes.begin());

                for (Node<T> *n : _nodes) delete n;
                _nodes.clear();

                if (eraseRoot) {
                    delete root;
                } else {
                    _nodes.push_back(root);
                }
            }
        }

        /**
         * Picks a random state and attempts to extend the tree towards it.
         * This is called at each iteration of the run() method.
         */
        Node<T> *grow() {
            //  extend towards goal, waypoint, or random state depending on the biases and a random number
            float r = rand() / (float)RAND_MAX; //  r is between 0 and one since we normalize it
            if (r < goalBias()) {
                return extend(goalState());
            } else if (r < goalBias() + waypointBias() && _waypoints.size() > 0) {
                const T &waypoint = _waypoints[rand() % _waypoints.size()];
                return extend(waypoint);
            } else {
                return extend(_stateSpace->randomState());
            }
        }

        /**
         * Find the node int the tree closest to @state.  Pass in a float pointer
         * as the second argument to get the distance that the node is away from
         * @state.
         */
        Node<T> *nearest(const T &state, float *distanceOut = nullptr) {
            float bestDistance = -1;
            Node<T> *best = nullptr;

            for (Node<T> *other : _nodes) {
                float dist = _stateSpace->distance(other->state(), state);
                if (bestDistance < 0 || dist < bestDistance) {
                    bestDistance = dist;
                    best = other;
                }
            }

            if (distanceOut) *distanceOut = bestDistance;

            return best;
        }



        /**
         * Grow the tree in the direction of @state
         *
         * @return the new tree Node (may be nullptr if we hit Obstacles)
         * @param target The point to extend the tree to
         * @param source The Node to connect from.  If source == nullptr, then
         *             the closest tree point is used
         */
        virtual Node<T> *extend(const T &target, Node<T> *source = nullptr) {
            //  if we weren't given a source point, try to find a close node
            if (!source) {
                source = nearest(target, nullptr);
                if (!source) {
                    return nullptr;
                }
            }

            //  Get a state that's in the direction of @target from @source.
            //  This should take a step in that direction, but not go all the
            //  way unless the they're really close together.
            T intermediateState;
            if (_isASCEnabled) {
                intermediateState = _stateSpace->intermediateState(
                    source->state(),
                    target,
                    stepSize(),
                    maxStepSize()
                );
            } else {
                intermediateState = _stateSpace->intermediateState(source->state(), target, stepSize());
            }

            //  Make sure there's actually a direct path from @source to
            //  @intermediateState.  If not, abort
            if (!_stateSpace->transitionValid(source->state(), intermediateState)) {
                return nullptr;
            }

            // Add a node to the tree for this state
            Node<T> *n = new Node<T>(intermediateState, source);
            n->setDistance(_stateSpace->distance(source->state(),intermediateState)+source->getDistance());
//            qDebug()<<n->getDistance();
            _nodes.push_back(n);
            return n;
        }

        /**
         * Get the path from the receiver's root point to the dest point
         *
         * @param callback The lambda to call for each state in the path
         * @param reverse if true, the states will be sent from @dest to the
         *                tree's root
         */
        void getPath(std::function<void(const T &stateI)> callback, Node<T> *dest, const bool reverse = false) {
            const Node<T> *node = dest;
            if (reverse) {
                while (node) {
                    callback(node->state());
                    node = node->parent();
                }
            } else {
                //  order them correctly in a list
                std::list<const Node<T> *> nodes;
                while (node) {
                    nodes.push_front(node);
                    node = node->parent();
                }

                //  then pass them one-by-one to the callback
                for (const Node<T> *n : nodes) callback(n->state());
            }
        }

        /**
         * Get the path from the receiver's root point to the dest point.
         *
         * @param vectorOut The vector to append the states along the path
         * @param reverse if true, the states will be sent from @dest to the
         *                tree's root
         */
        void getPath(std::vector<T> &vectorOut, Node<T> *dest, const bool reverse = false) {
            getPath([&](const T &stateI) {
                vectorOut.push_back(stateI);
            },
            dest,
            reverse);
        }

        /**
         * @return The root node or nullptr if none exists
         */
        Node<T> *rootNode() const {
            if (_nodes.empty()) return nullptr;

            return _nodes.front();
        }

        /**
         * @return The most recent Node added to the tree
         */
        Node<T> *lastNode() const {
            if (_nodes.empty()) return nullptr;

            return _nodes.back();
        }

        /**
         * All the nodes
         */
        const std::vector<Node<T> *> allNodes() const {
            return _nodes;
        }


        /**
         * @brief The start state for this tree
         */
        const T &startState() const {
            if (_nodes.empty()) throw std::logic_error("No start state specified for RRT::Tree");
            else return rootNode()->state();
        }
        void setStartState(const T &startState) {
            reset(true);

            //  create root node from provided start state
            Node<T> *root = new Node<T>(startState, nullptr);
            _nodes.push_back(root);
        }


        /**
         * @brief The goal this tree is trying to reach.
         */
        const T &goalState() const {
            return _goalState;
        }
        void setGoalState(const T &goalState) {
            _goalState = goalState;
        }


    protected:
        /**
         * A list of all Node objects in the tree.
         */
        std::vector<Node<T> *> _nodes;

        T _goalState;

        int _maxIterations;

        bool _isASCEnabled;

        float _goalBias;

        /// used for Extended RRTs where growth is biased towards waypoints from previously grown tree
        float _waypointBias;
        std::vector<T> _waypoints;

        float _goalMaxDist;

        float _stepSize;
        float _maxStepSize;

        std::shared_ptr<StateSpace<T>> _stateSpace;
    };
}

#endif // TREE

