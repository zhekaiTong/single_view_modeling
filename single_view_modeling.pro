#-------------------------------------------------
#
# Project created by QtCreator 2018-04-04T21:48:46
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = single_view_modeling
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        single_view_modeling.cpp

HEADERS += \
        single_view_modeling.h

FORMS += \
        single_view_modeling.ui

INCLUDEPATH += /usr/include/opencv

INCLUDEPATH += /usr/include/opencv2

INCLUDEPATH += /usr/include/eigen3/Eigen

LIBS +=  -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs
