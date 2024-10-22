#ifndef VIDEOTABCONTROLLER_H
#define VIDEOTABCONTROLLER_H

#include <opencv/highgui.h>
#include <QWidget>
#include <QString>
#include <QTimer>
#include <QLabel>
#include "view/clickableqlabel.h"
#include "view/vectorfieldqlabel.h"
#include "model/opticalflowengine.h"

#include "model/Face.h"
#include "view/facewidget.h"
#include "model/poseestimator.h"
#include "model/videoprocessor.h"

#include "view/exptranabstractview.h"
#include "ui_cameraDialog.h"

#include <opencv/cv.hpp>
#include <vector>
#include <string>


//really should be called transfer controller
class VideoTabController : public QWidget
{
    Q_OBJECT
public:
    VideoTabController(QString fileName, ClickableQLabel *picLabel, VectorFieldQLabel *flowLabel,
                       ExpTranAbstractView *view, FaceWidget *face_widget);
    ~VideoTabController();

    void selectGoodFeaturePoints(const cv::Mat& m);

    void setInteractiveLabel(QLabel *);   

public slots:   
    void playTransfer();
    void pauseTransfer();
    void restartCapturing();
    void computeFlow();
    void captureFrame();
    void toggleDrawable(bool);
    void video_file_changed(const QString str);
    void findGoodFeaturePoints();
    void extractPose();
    void dropFrame();
    void playBack();

    void replayFrame();
    void calibrate();
    void setCameraParameters();
    void processingFinished();
    void detect();

    //settings
    void setOptNelder(bool);
    void setOptReg(double regParam);
    void setOptType(int);
    void setFrameNum(int);
    void setIterNum(int);
    void setGenPoints(int);
    void setConstID(bool);
    void setPointGen2D(bool);
    void setPointGen3D(bool);
    void setHybrid(bool);
    void setNone(bool);
    void setProjModel(bool);
    void setWithFirstFrame(bool);
private:
    void calcIntrinsicParams();

    VideoProcessor::OptType opttype;
    VideoProcessor::IdConstraintType idconstype;
    VideoProcessor::PointGenerationType projtype;
    double regParam;
    int frame_num;
    int iter_num;
    int gen_point_num;
    bool projModel;
    bool withFirstFrame;

    //video fileName
    QString fileName;
    bool autoSelectFeaturePoints;

    QTimer *timer;

    cv::VideoCapture *capture;
    double frameCount;

    ClickableQLabel *picLabel;
    VectorFieldQLabel *flowLabel;
    ExpTranAbstractView *view;

    //camera parameters
    Mat_<double> cameraMatrix;
    Mat_<double> lensDist;

    //frameHistory .. this includes dropped frames and first frame
    std::vector<cv::Mat> frames;

    //object responsible for computing
    //optical flow
    OpticalFlowEngine *flowEngine;
    //3D pose parameters optimization
    PoseEstimator *poseEstimator;
    //model fitting engine
    VideoProcessor *videoProcessor;

    Face *face_ptr;
    FaceWidget *face_widget;

    //for now so that we can use them in a timer
    vector<cv::Mat> frameData;
    //points we use for optical flow .. more points will be used for model estimation
    vector<vector<cv::Point2f> > featurePoints;

    QTimer *timerReplay;

    //camera dialog ui
    Ui::cameraDialog cameraUi;
    QDialog *cameraDialog;
};

#endif // VIDEOTABCONTROLLER_H
