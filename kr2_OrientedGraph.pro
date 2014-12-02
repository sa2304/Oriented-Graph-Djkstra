TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    testorientedgraph.cpp

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    orientedgraph.h \
    testorientedgraph.h

