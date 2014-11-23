TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    orientedgraph.cpp

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    orientedgraph.h

