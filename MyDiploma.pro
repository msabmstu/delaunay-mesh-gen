TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        delaunayadf3d.cpp \
        exceptions.cpp \
        griddatatypes.cpp \
        main.cpp \
        mathutilities.cpp \
        meshloader3d.cpp \
        meshunloader3d.cpp \
        unstructuredgrid.cpp

HEADERS += \
    abstractgenerator.h \
    abstractloader.h \
    abstractunloader.h \
    delaunayadf3d.h \
    exceptions.h \
    griddatatypes.h \
    mathutilities.h \
    meshloader3d.h \
    meshunloader3d.h \
    unstructuredgrid.h
