QT       += core gui widgets

TARGET = REVLibrary
TEMPLATE = lib
CONFIG += staticlib

SOURCES += rev.cpp \
    revinit.cpp \
    revinputviewer.cpp \
    neuralnetworkviewer.cpp

HEADERS += rev.h \
    revinit.h \
    revinputviewer.h \
    neuralnetworkviewer.h
unix {
    target.path = /usr/lib
    INSTALLS += target
}
