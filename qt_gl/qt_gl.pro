# #####################################################################
# Automatically generated by qmake (2.01a) Sun May 30 02:48:28 2010
# #####################################################################
TEMPLATE = app
TARGET = 
DEPENDPATH += .
CONFIG += qt \
    debug

# Input
SOURCES += qt_gl.cpp \
    model/rosenerror.cpp \
    model/Tensor.cpp \
    model/Face.cpp \
    model/FaceModel.cpp \
    model/Matrix.cpp \
    model/Vector3.cpp \
    view/clickableqlabel.cpp \
    view/featurepointqlabel.cpp \
    view/vectorfieldqlabel.cpp \
    model/closedformoptimizer.cpp \
    model/neldermeadoptimizer.cpp \
    model/nnlsoptimizer.cpp \
    model/opticalflowengine.cpp \
    model/opticalflowfarneback.cpp \
    model/videoprocessor.cpp \
    model/modelimageerror.cpp \
    controller/facetabcontroller.cpp \
    controller/transfertabcontroller.cpp \
    controller/utility.cpp \
    controller/videotabcontroller.cpp \
    view/facewidget.cpp \
    view/exptranwindow.cpp \
    view/customizablefacewidget.cpp \
    model/poseestimator.cpp \
    model/optimizer.cpp \
    model/modelidentityerror.cpp
QT += opengl
FORMS += view/applform.ui \
    view/cameraDialog.ui \
    view/settingsDialog.ui
HEADERS += model/rosenerror.h \
    model/Tensor.h \
    model/Face.h \
    model/FaceModel.h \
    model/Matrix.h \
    model/Vector3.h \
    view/clickableqlabel.h \
    view/featurepointqlabel.h \
    view/vectorfieldqlabel.h \
    model/closedformoptimizer.h \
    model/neldermeadoptimizer.h \
    model/nnlsoptimizer.h \
    model/opticalflowengine.h \
    model/opticalflowfarneback.h \
    model/optimizer.h \
    model/videoprocessor.h \
    model/errorfunction.h \
    model/modelimageerror.h \
    controller/facetabcontroller.h \
    controller/transfertabcontroller.h \
    controller/utility.h \
    controller/videotabcontroller.h \
    view/facewidget.h \
    view/exptranwindow.h \
    view/customizablefacewidget.h \
    model/poseestimator.h \
    view/exptranapplication.h \
    model/exptranexception.h \
    view/exptranabstractview.h \
    model/modelidentityerror.h

OTHER_FILES += model.properties


INCLUDEPATH += . /usr/include/opencv
LIBS += -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_flann
LIBS += -lGL -lGLU
