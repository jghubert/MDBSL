#-------------------------------------------------
#
# Project created by QtCreator 2015-02-24T13:38:26
#
#-------------------------------------------------

QT       += core gui widgets

TARGET = REVLibrary
TEMPLATE = lib
CONFIG += staticlib

SOURCES += rev.cpp

HEADERS += rev.h
unix {
    target.path = /usr/lib
    INSTALLS += target
}
