#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets>
#include "RRTWidget.hpp"

/**
 * This window subclass displays the Interactive RRT 'applet'.
 * It has an RRTWidget that takes up most of the space and a
 * buttons for interacting with the RRT.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow();

protected slots:
    void slot_updateGoalBiasLabel(int value);
    void slot_updateWaypointBiasLabel(int value);
    void slot_updateIterationCount(int iterationCount);
    void slot_updateSolutionLength(double solutionLength);

private:
    RRTWidget *_rrtWidget;
    QLabel *_goalBiasLabel;
    QLabel *_waypointBiasLabel;
    QLabel *_iterationCountLabel;
    QLabel *_SolutionLengthLabel;
};
#endif // MAINWINDOW_H
