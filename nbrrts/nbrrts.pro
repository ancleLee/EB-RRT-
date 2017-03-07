#-------------------------------------------------
#
# Project created by QtCreator 2016-12-05T09:28:02
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = nbrrts
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    2dplane/2dplane.cpp \
    2dplane/GridStateSpace.cpp \
    2dplane/obstaclegrid.cpp \
    2dplane/PlaneStateSpace.cpp \
    3dplane/3dplane.cpp \
    3dplane/GridStateSpace3d.cpp \
    3dplane/ObstacleGrid3d.cpp \
    3dplane/PlaneStateSpace3d.cpp \
    RRTWidget.cpp

HEADERS  += mainwindow.h \
    2dplane/2dplane.h \
    2dplane/GridStateSpace.hpp \
    2dplane/obstaclegrid.h \
    2dplane/planestatespace.h \
    3dplane/3dplane.h \
    3dplane/GridStateSpace3d.h \
    3dplane/ObstacleGrid3d.h \
    3dplane/PlaneStateSpace3d.h \
    planning/Path.hpp \
    birrt.h \
    birrtstar.h \
    node.h \
    NomalrrtStar.h \
    rrtstartree.h \
    RRTWidget.hpp \
    statespace.h \
    tree.h

FORMS    += mainwindow.ui

INCLUDEPATH += /usr/include/eigen3

QMAKE_CXXFLAGS += -std=c++0x
