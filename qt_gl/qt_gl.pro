# #####################################################################
# Automatically generated by qmake (2.01a) Sun May 30 02:48:28 2010
# #####################################################################
TEMPLATE = app
TARGET = 
DEPENDPATH += .
INCLUDEPATH += . \
    /usr/local/include/opencv
CONFIG += qt \
    debug

# Input
SOURCES += qt_gl.cpp \
    Face.cpp \
    Vector3.cpp \
    my_main_window.cpp \
    face_widget.cpp \
    svd.cpp \
    Tensor.cpp \
    FaceModel.cpp \
    Matrix.cpp \
    transferwidget.cpp \
    clickableqlabel.cpp \
    opticalflowengine.cpp \
    opticalflowfarneback.cpp \
    vectorfieldqlabel.cpp \
    featurepointqlabel.cpp \
    mysimplex.cpp \
    rosenerror.cpp \
    modelimageerror.cpp \
    facetabcontroller.cpp \
    optimizer.cpp \
    neldermeadoptimizer.cpp \
    closedformoptimizer.cpp
QT += opengl
FORMS += applform.ui \
    cameraDialog.ui
HEADERS += my_main_window.h \
    Face.h \
    Vector3.h \
    face_widget.h \
    svd.h \
    Tensor.h \
    FaceModel.h \
    Matrix.h \
    transferwidget.h \
    clickableqlabel.h \
    opticalflowengine.h \
    opticalflowfarneback.h \
    vectorfieldqlabel.h \
    featurepointqlabel.h \
    mysimplex.h \
    errorfunction.h \
    rosenerror.h \
    modelimageerror.h \
    facetabcontroller.h \
    optimizer.h \
    neldermeadoptimizer.h \
    closedformoptimizer.h
LIBS += -L/usr/local/lib \
    -lml \
    -lcvaux \
    -lhighgui \
    -lcv \
    -lcxcore
