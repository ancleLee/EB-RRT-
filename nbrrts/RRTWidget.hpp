#pragma once

#include <QtWidgets>
#include <birrtstar.h>
#include <2dplane/GridStateSpace.hpp>
#include <Eigen/Dense>
#include <birrt.h>

/**
 * This widget creates an RRT tree for searching a 2d space and draws it.
 * It has methods (slots) for stepping and resetting tree growth.
 * You can also draw and erase obstacles for by clicking and dragging.
 */
class RRTWidget : public QWidget {
    Q_OBJECT

public:
    RRTWidget();


private slots:
    void slot_run();
    void run_step();
    void slot_stop();
    void slot_reset();
    void slot_clearObstacles();
    void slot_step();
    void slot_stepBig();
    void slot_setGoalBias(int bias);        //  bias is from 0 to 100
    void slot_setWaypointBias(int bias);    //  bias is from 0 to 100
    void slot_setASC(int checked);
    void slot_setStepSize(double step);
    void slot_openmap();
    void slot_savemap();

signals:
    void signal_stepped(int iterationCount);
    void signal_solution(double solutionlength);

protected:
    void paintEvent(QPaintEvent *p);
    void drawTree(QPainter &painter,
        const RRTStar::rrtStarTree<Eigen::Vector2f> &rrt,
        const Node<Eigen::Vector2f> *solutionNode = NULL,
        QColor treeColor = Qt::blue,
        QColor solutionColor = Qt::red);
    void drawTerminalState(QPainter &painter, const Eigen::Vector2f &pos, const Eigen::Vector2f &vel, const QColor &color);

    QPointF pointFromNode(const Node<Eigen::Vector2f> *n);

    void step(int numTimes);
    
    void calBeziercurve();

    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);

    static bool mouseInGrabbingRange(QMouseEvent *event, const Eigen::Vector2f &pt);


private:
    std::shared_ptr<GridStateSpace> _stateSpace;
    RRTStar::BiRRTStar<Eigen::Vector2f> *_biRRTStar;

    Eigen::Vector2f _startVel, _goalVel;

    //  if you click down on an obstacle, you enter erase mode
    //  if you click down where there's no obstacle, you enter draw mode
    bool _editingObstacles, _erasingObstacles, treeflags;

    enum {
        DraggingNone = 0,
        DraggingStart,
        DraggingGoal,
        DraggingStartVel,
        DraggingGoalVel
    } _draggingItem;

    int _waypointCacheMaxSize;

    std::vector<Eigen::Vector2f> _previousSolution;
    
    std::vector<Eigen::Vector2f> _beziercurve;


    QTimer *_runTimer;
};
