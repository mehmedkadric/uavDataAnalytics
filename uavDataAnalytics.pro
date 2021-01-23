QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    helpdialog.cpp \
    hsvseparator.cpp \
    imageprocessing2d.cpp \
    main.cpp \
    pointcloudanalyzer.cpp \
    uav.cpp

HEADERS += \
    helpdialog.h \
    hsvseparator.h \
    imageprocessing2d.h \
    pointcloudanalyzer.h \
    uav.h

FORMS += \
    helpdialog.ui \
    uav.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


INCLUDEPATH += /usr/local/include/opencv4
LIBS += -lopencv_core
LIBS += -lopencv_imgproc
LIBS += -lopencv_highgui
LIBS += -lopencv_ml
LIBS += -lopencv_video
LIBS += -lopencv_features2d
LIBS += -lopencv_calib3d
LIBS += -lopencv_objdetect
LIBS += -lopencv_flann
LIBS += -lopencv_stitching
LIBS += -lopencv_xfeatures2d
LIBS += -lopencv_imgcodecs
LIBS += -L/usr/local/lib/
LIBS += -L/usr/local/Cellar/pcl/1.11.1_1/lib

INCLUDEPATH += /usr/local/Cellar/pcl/1.11.1_1/include/pcl-1.11
