QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RobotExperimentViewer
TEMPLATE = app


SOURCES += main.cpp\
        rev.cpp \
    revinputviewer.cpp

HEADERS  += rev.h \
    revinputviewer.h
