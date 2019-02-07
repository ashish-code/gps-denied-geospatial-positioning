TARGET = osm_convert

QT += core
QT += gui
QT += opengl
QT += svg

CONFIG += console
CONFIG -= app_bundle
#CONFIG += debug

TEMPLATE = app

INCLUDEPATH += ../shared/cpp

LIBS += -lboost_serialization -lboost_program_options

SOURCES += \
    osm_convert.cpp \
    osm.cpp \
    mapviewer.cpp \
    mapwidget.cpp \
    ../shared/cpp/utils.cpp \
    ../shared/cpp/objects.cpp \
    ../shared/cpp/coordinates.cpp

HEADERS += \
    osm.h \
    mapviewer.h \
    mapwidget.h \
    ../shared/cpp/objects.h \
    ../shared/cpp/utils.h \
    ../shared/cpp/coordinates.h

FORMS += \
    mapviewer.ui

DESTDIR = .
OBJECTS_DIR = build
MOC_DIR = build
RCC_DIR = build
UI_DIR = build

