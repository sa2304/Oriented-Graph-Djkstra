#-------------------------------------------------
#
# Project created by QtCreator 2015-01-30T23:03:53
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = kr2_OrientedGraph
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    mainmenu.cpp \
    testorientedgraph.cpp \
    IMenu.cpp \
    IMenuAction.cpp \
    imenuhotkeyhandler.cpp

HEADERS += \
    IMenu.h \
    IMenuAction.h \
    imenuhotkeyhandler.h \
    mainmenu.h \
    orientedgraph.h \
    testorientedgraph.h
