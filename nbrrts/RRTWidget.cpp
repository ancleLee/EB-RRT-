#include "RRTWidget.hpp"
#include <planning/Path.hpp>
#include <2dplane/2dplane.h>
#include <QDebug>
#include <QFile>
#include <QTextCodec>
#include <bezier2d.h>

using namespace RRTStar;
using namespace Eigen;
using namespace std;

/// multiply velocity by this to get the length of the vector to draw
const float VelocityDrawingMultiplier = 12;


RRTWidget::RRTWidget() {
    _stateSpace = make_shared<GridStateSpace>(800,
                                        600,
                                        40,
                                        30);
    _biRRTStar = new BiRRTStar<Vector2f>(_stateSpace);


    //set the size of the widget
    setFixedSize(800, 600);

    _waypointCacheMaxSize = 15;

    //  setup birrtStar
    _biRRTStar->initVector(48, 100, 100);
    _biRRTStar->setStartState(Vector2f(400, 300));
    _biRRTStar->setGoalState(Vector2f(700, 300));
    _biRRTStar->setStepSize(10);
    _biRRTStar->setMaxStepSize(20);
    _biRRTStar->setGoalMaxDist(12);
    


    _startVel = Vector2f(1, 0);
    _goalVel = Vector2f(0, 1);

    //  register for mouse events
    setMouseTracking(true);

    _draggingItem = DraggingNone;

    _runTimer = nullptr;
}

void RRTWidget::slot_reset() {
    //  store waypoint cache
    vector<Vector2f> waypoints;
    if (_biRRTStar->startSolutionNode() && _biRRTStar->goalSolutionNode()) {
        waypoints = _previousSolution;
        if (waypoints.size() > 0) {
            //  don't keep the start or end states
            waypoints.erase(waypoints.begin());
            waypoints.erase(waypoints.end()-1);

            //  down-sample
            Planning::DownSampleVector<Vector2f>(waypoints, _waypointCacheMaxSize);
        }
    } else {
        _previousSolution.clear();
        _beziercurve.clear();
    }

    _biRRTStar->reset();

    _biRRTStar->setWaypoints(waypoints);

    emit signal_stepped(0);
    emit signal_solution(-1);

    repaint();
}

void RRTWidget::slot_clearObstacles() {
    _stateSpace->obstacleGrid().clear();

    repaint();
}

void RRTWidget::slot_setGoalBias(int bias) {
    _biRRTStar->setGoalBias((float)bias / 100.0f);
}

void RRTWidget::slot_setWaypointBias(int bias) {
    _biRRTStar->setWaypointBias((float)bias / 100.0f);
}

void RRTWidget::slot_setASC(int checked) {
    _biRRTStar->setASCEnabled(checked != 0);
}

void RRTWidget::slot_step() {
    step(1);
}

void RRTWidget::slot_stepBig() {
    step(100);
}

void RRTWidget::slot_setStepSize(double step) {
    _biRRTStar->setStepSize(step);
    _biRRTStar->setGoalMaxDist(step*1.2);
}

void RRTWidget::slot_run() {
    if (!_runTimer) {
        _runTimer = new QTimer(this);
        connect(_runTimer, SIGNAL(timeout()), this, SLOT(run_step()));
        _runTimer->start(0);
    }
}

void RRTWidget::slot_stop() {
    if (_runTimer) {
        delete _runTimer;
        _runTimer = nullptr;
    }
}

void RRTWidget::run_step() {
    if (_biRRTStar->startSolutionNode() == nullptr) {
        step(1);
    } else {
        delete _runTimer;
        _runTimer = nullptr;
    }
}

void RRTWidget::step(int numTimes) {
    for (int i = 0; i < numTimes; i++) {
        _biRRTStar->grow();
    }

    //  store solution
    _previousSolution.clear();
    _beziercurve.clear();
    if (_biRRTStar->startSolutionNode() != nullptr || _biRRTStar->goalSolutionNode() != nullptr) {
        _biRRTStar->getPath(_previousSolution);
        Planning::SmoothPath<Vector2f>(_previousSolution, *_stateSpace);
        calBeziercurve();
        emit signal_solution(_biRRTStar->getsolutionLength());
        _biRRTStar->print_num_r();
    }

    emit signal_stepped(_biRRTStar->iterationCount());

    repaint();
}

/*
 * @brief open the map file and read the obstacle from the map file
 */
void RRTWidget::slot_openmap()
{
    const QString directoryName = QFileDialog::getOpenFileName(this, tr("Open Text"), "/home/liyuwei/MBRRTs/build-MBRRTs-Desktop-Debug", tr("Text Files(*.txt)"));

    QFile file(directoryName);
    if(!file.open(QIODevice::ReadWrite | QIODevice::Text))
    {
            QMessageBox::information(this, tr("Failed to open this file"), tr("Failed to open this file"));
            return;
    }

    _stateSpace->obstacleGrid().clear();

    QTextStream stream(&file);
    while(!stream.atEnd())
    {
        int x=-1,y=-1;
        char m;
        stream>>x>>m>>y;
        if(stream.atEnd())
            break;
        Vector2f pos = Vector2f(x,y);
        Vector2i gridLoc = _stateSpace->obstacleGrid().gridSquareForLocation(pos);
        //  toggle the obstacle state of clicked square
        _stateSpace->obstacleGrid().obstacleAt(gridLoc) = true;

    }

    repaint();
}

/*
 * @brief save the current obstacle to the .txt file
 */
void RRTWidget::slot_savemap()
{
    QString filename = QFileDialog::getSaveFileName(this, tr("Save Text"),
                "/home/liyuwei/MBRRTs/build-MBRRTs-Desktop-Debug",
                tr("*.txt"));
    qDebug()<<filename;
    qDebug()<<QFile::exists(filename);
    if(!QFile::exists(filename))
    {
        QFile file(filename);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            QMessageBox::information(this,
                tr("Failed to save the file"),
                tr("Failed to save the file!"));
            return;
        }
        QTextStream out(&file);
        for(int x=0; x<40; x++)
            for(int y=0; y<30; y++)
            {
                Vector2i pos(x,y);
                if(_stateSpace->obstacleGrid().obstacleAt(pos))
                    out<<x*20<<","<<y*20<<"\n";

            }
        return;
    }
}

QPointF RRTWidget::pointFromNode(const Node<Vector2f> *n) {
    return QPointF(n->state().x(), n->state().y());
}

QPointF vecToPoint(const Vector2f &vec) {
    return QPointF(vec.x(), vec.y());
}

void RRTWidget::paintEvent(QPaintEvent *p) {
    QPainter painter(this);

    //  Draw black border around widget
    painter.setPen(QPen (Qt::black, 3));
    painter.drawRect(rect());

    //  Draw obstacles
    int rectW = rect().width() / _stateSpace->obstacleGrid().discretizedWidth(), rectH = rect().height() / _stateSpace->obstacleGrid().discretizedHeight();
    painter.setPen(QPen(Qt::black, 1));//2
    for (int x = 0; x < _stateSpace->obstacleGrid().discretizedWidth(); x++) {
        for (int y = 0; y < _stateSpace->obstacleGrid().discretizedHeight(); y++) {
            if (_stateSpace->obstacleGrid().obstacleAt(x, y)) {
                painter.fillRect(x * rectW, y * rectH, rectW, rectH, Qt::SolidPattern);
            }
        }
    }


    //  Draw previous solution
    if (_previousSolution.size() > 0) {
        painter.setPen(QPen(Qt::gray, 3));//5
        Vector2f prev;
        bool first = true;
        for (const Vector2f &curr : _previousSolution) {
            if (first) {
                first = false;
            } else {
                painter.drawLine(QPointF(prev.x(), prev.y()), QPointF(curr.x(), curr.y()));
            }
            prev = curr;
        }



        //  Draw cubic bezier interpolation of waypoints
        painter.setPen(QPen(Qt::darkRed, 3));//5

        //Draw the bezier curve
        if (_beziercurve.size() > 0) {
            Vector2f prev;
            bool first = true;
            for (const Vector2f &curr : _beziercurve) {
                if (first) {
                    first = false;
                } else {
                    painter.drawLine(QPointF(prev.x(), prev.y()), QPointF(curr.x(), curr.y()));
                }
                prev = curr;
            }
        }

  //      painter.drawPath(path);
    }


    //  draw waypoint cache
    if (_biRRTStar->waypoints().size() > 0) {
        float r = 2;    //  radius to draw waypoint dots

        painter.setPen(QPen(Qt::lightGray, 1));//3
        for (const Vector2f &waypoint : _biRRTStar->waypoints()) {
            painter.drawEllipse(QPointF(waypoint.x(), waypoint.y()), r, r);
        }
    }

    //  draw trees
    drawTree(painter, _biRRTStar->startTree(), _biRRTStar->startSolutionNode(), Qt::blue);
    drawTree(painter, _biRRTStar->goalTree(), _biRRTStar->goalSolutionNode(), Qt::darkGreen);

    //  draw start and goal points
    drawTerminalState(painter, _biRRTStar->startState(), _startVel, Qt::darkBlue);
    drawTerminalState(painter, _biRRTStar->goalState(), _goalVel, Qt::darkGreen);    
}


void RRTWidget::calBeziercurve() {
        
        /*_beziercurve.push_back(_previousSolution[0]);

        Vector2f prevControlDiff = -_startVel*VelocityDrawingMultiplier;
        for (int i = 1; i < _previousSolution.size(); i++) {
            //Calculate the two control points
            Vector2f waypoint = _previousSolution[i];
            Vector2f prevWaypoint = _previousSolution[i-1];

            Vector2f controlDir;
            float controlLength;
            if (i == _previousSolution.size() - 1) {
                controlLength = _goalVel.norm() * VelocityDrawingMultiplier;
                controlDir = -_goalVel.normalized();
            } else {

                Vector2f nextWaypoint = _previousSolution[i+1];
                controlLength = 0.5*min( (waypoint - prevWaypoint).norm(), (nextWaypoint - waypoint).norm() );
                controlDir = ((prevWaypoint - waypoint).normalized() - (nextWaypoint - waypoint).normalized()).normalized();
            }


            Vector2f controlDiff = controlDir * controlLength;

            vector<Vector2f> tempforbezier;

            //To add the start point and two control point and the aim point
            tempforbezier.push_back(_beziercurve.back ());
            tempforbezier.push_back(prevWaypoint - prevControlDiff);
            tempforbezier.push_back(waypoint + controlDiff);
            tempforbezier.push_back(waypoint);
            ComputeBezier3(tempforbezier, 10, _beziercurve);

            

            prevControlDiff = controlDiff;
        }        
        */
        
        _beziercurve.push_back(_previousSolution[0]);
        
        float scale = 0.5;
        float every_increase = _biRRTStar->stepSize()/2;
        
        for (int i = 1; i < _previousSolution.size()-1; i++) {
            Vector2f waypoint = _previousSolution[i];
            Vector2f prevWaypoint = _previousSolution[i-1];
            Vector2f nextWaypoint = _previousSolution[i+1];
            
            Vector2f controlDir1 = (prevWaypoint - waypoint).normalized();
            Vector2f controlDir2 = (nextWaypoint - waypoint).normalized();
            float mindis = min((prevWaypoint - waypoint).norm(), (nextWaypoint - waypoint).norm());
            float controlLength = 0.0;
            
            Vector2f p1 = waypoint+controlLength*controlDir1;
            Vector2f p4 = waypoint+controlLength*controlDir2;
            
            for(;_stateSpace->transitionValid(p1, p4) && controlLength < 0.5*mindis;controlLength += every_increase) {
                p1 = waypoint+controlLength*controlDir1;
                p4 = waypoint+controlLength*controlDir2;
            }
            
            if(controlLength > 0.5*mindis) {
                controlLength = 0.5*mindis;
            }
            else {
                controlLength -= every_increase;
            }
            
            if(controlLength ==  0.0) {
                _beziercurve.push_back(_previousSolution[i]);
                continue;
            }
            
            p1 = waypoint+controlLength*controlDir1;
            p4 = waypoint+controlLength*controlDir2;
            
            Vector2f p2 = waypoint+scale*controlLength*controlDir1;
            Vector2f p3 = waypoint+scale*controlLength*controlDir2;
            
            vector<Vector2f> tempforbezier;
            tempforbezier.push_back(p1);
            tempforbezier.push_back(p2);
            tempforbezier.push_back(p3);
            tempforbezier.push_back(p4);
            ComputeBezier3(tempforbezier, controlLength, _beziercurve);
        }
        _beziercurve.push_back(_previousSolution[_previousSolution.size()-1]);/**/
        
        
}





/*
 * Draw the start and goal point miss the arrow shaft
 */
void RRTWidget::drawTerminalState(QPainter &painter, const Vector2f &pos, const Vector2f &vel, const QColor &color) {
    //  draw point
    painter.setPen(QPen(color, 6));
    QPointF rootLoc(pos.x(), pos.y());
    painter.drawEllipse(rootLoc, 2, 2);


    Vector2f tipOffset = vel * VelocityDrawingMultiplier;
    Vector2f tipLocVec = pos + tipOffset;
    QPointF tipLoc(tipLocVec.x(), tipLocVec.y());

    //  draw arrow shaft
    painter.setPen(QPen(color, 3));
//    painter.drawLine(rootLoc, tipLoc);

    //  draw arrow head
    Vector2f headBase = tipLocVec - tipOffset.normalized()*4;
    Vector2f perp = Vector2f(-tipOffset.y(), tipOffset.x()).normalized();
    Vector2f tipLeftVec = headBase + perp*4;
    Vector2f tipRightVec = headBase - perp*4;
    QPointF trianglePts[] = {
        tipLoc,
        QPointF(tipLeftVec.x(), tipLeftVec.y()),
        QPointF(tipRightVec.x(), tipRightVec.y())
    };
//    painter.drawPolygon(trianglePts, 3);
}

void RRTWidget::drawTree(QPainter &painter,
    const rrtStarTree<Vector2f> &rrt,
    const Node<Vector2f> *solutionNode,
    QColor treeColor,
    QColor solutionColor)
{
    //  node drawing radius
    const float r = 1;

    //  draw all the nodes and connections
    for (const Node<Vector2f> *node : rrt.allNodes()) {
        painter.setPen(QPen (treeColor, 1));
        QPointF loc = pointFromNode(node);
        painter.drawEllipse(loc, r, r);

        if (node->parent()) {
            //  draw edge
            painter.setPen(QPen(treeColor, 1));
            QPointF parentLoc = pointFromNode(node->parent());
            painter.drawLine(loc, parentLoc);
        }
    }

    //  draw solution
    if (solutionNode) {
        painter.setPen(QPen(solutionColor, 2));//4

        const Node<Vector2f> *node = solutionNode, *parent = solutionNode->parent();
        while (parent) {
            //  draw the edge
            QPointF from = pointFromNode(node);
            QPointF to = pointFromNode(parent);
            painter.drawLine(from, to);

            node = parent;
            parent = parent->parent();
        }
    }
}


bool RRTWidget::mouseInGrabbingRange(QMouseEvent *event, const Vector2f &pt) {
    float dx = event->pos().x() - pt.x();
    float dy = event->pos().y() - pt.y();
    return sqrtf( dx*dx + dy*dy ) < 15;
}

//React to the mouse press event ,1.drag the start and goal 2.add and remove the obstacle
void RRTWidget::mousePressEvent(QMouseEvent *event) {
    if (mouseInGrabbingRange(event, _biRRTStar->startState())) {
        _draggingItem = DraggingStart;
    } else if (mouseInGrabbingRange(event, _biRRTStar->goalState())) {
        _draggingItem = DraggingGoal;
    } else if (mouseInGrabbingRange(event, _biRRTStar->startState() + _startVel*VelocityDrawingMultiplier)) {
        _draggingItem = DraggingStartVel;
    } else if (mouseInGrabbingRange(event, _biRRTStar->goalState() + _goalVel*VelocityDrawingMultiplier)) {
        _draggingItem = DraggingGoalVel;
    } else {
        _editingObstacles = true;
        Vector2f pos = Vector2f(event->pos().x(), event->pos().y());
        Vector2i gridLoc = _stateSpace->obstacleGrid().gridSquareForLocation(pos);
        _erasingObstacles = _stateSpace->obstacleGrid().obstacleAt(gridLoc);

        //  toggle the obstacle state of clicked square
        _stateSpace->obstacleGrid().obstacleAt(gridLoc) = !_erasingObstacles;
        repaint();
    }
}

void RRTWidget::mouseMoveEvent(QMouseEvent *event) {
    Vector2f point(event->pos().x(), event->pos().y());

    if (_draggingItem == DraggingStart) {
        //  reset the tree with the new start pos
        _biRRTStar->setStartState(point);
    } else if (_draggingItem == DraggingGoal) {
        //  set the new goal point
        _biRRTStar->setGoalState(point);
    } else if (_draggingItem == DraggingStartVel) {
        _startVel = (point - _biRRTStar->startState()) / VelocityDrawingMultiplier;
    } else if (_draggingItem == DraggingGoalVel) {
        _goalVel = (point - _biRRTStar->goalState()) / VelocityDrawingMultiplier;
    } else if (_editingObstacles) {
        Vector2i gridLoc = _stateSpace->obstacleGrid().gridSquareForLocation(point);
        if (gridLoc[1] >= 0 && gridLoc[1] < _stateSpace->obstacleGrid().discretizedHeight()
                && gridLoc[0] >= 0 && gridLoc[0] < _stateSpace->obstacleGrid().discretizedWidth()) {
            _stateSpace->obstacleGrid().obstacleAt(gridLoc) = !_erasingObstacles;
        }
    }

    if (_draggingItem != DraggingNone || _editingObstacles) repaint();
}

void RRTWidget::mouseReleaseEvent(QMouseEvent *event) {
    _draggingItem = DraggingNone;
    _editingObstacles = false;
}
