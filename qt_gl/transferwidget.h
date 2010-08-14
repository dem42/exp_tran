#ifndef TRANSFERWIDGET_H
#define TRANSFERWIDGET_H

#include <highgui.h>
#include <QWidget>
#include <QString>
#include <QTimer>
#include <QLabel>
#include "clickableqlabel.h"
#include "vectorfieldqlabel.h"
#include "opticalflowengine.h"

#include "Face.h"
#include "face_widget.h"
#include "optimizer.h"

#include "ui_cameraDialog.h"

#include <cv.h>
#include <vector>
#include <string>

//really should be called transfer controller
class TransferWidget : public QWidget
{
    Q_OBJECT
public:
    TransferWidget(QString fileName, FaceWidget *face_widget);
    ~TransferWidget();    
    ClickableQLabel* getPicLabel() const;
    ClickableQLabel* getFlowLabel() const;
    void grabThumbnailForVideo(std::string videoName,cv::Mat& thumbnail);
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
    void startFaceTransfer();
    void dropFrame();
    void playBack();

    void replayFrame();
    void calibrate();
private:
    void calcIntrinsicParams();
    void processVideo();


    //video fileName
    QString fileName;
    bool autoSelectFeaturePoints;

    QTimer *timer;

    cv::VideoCapture *capture;
    double frameCount;

    ClickableQLabel *picLabel;
    VectorFieldQLabel *flowLabel;

    //camera parameters
    Mat_<double> cameraMatrix;
    Mat_<double> lensDist;

    //frameHistory .. this includes dropped frames and first frame
    std::vector<cv::Mat> frames;

    //object responsible for computing
    //optical flow
    OpticalFlowEngine *flowEngine;
    //model parameters optimization
    Optimizer *paramOptimizer;
    
    Face *face_ptr;
    FaceWidget *face_widget;
    //static here could be read from an xml config
    static const int fPoints[20];
    static const int fPoints_size;

    //for now so that we can use them in a timer
    vector<cv::Mat> frameData;
    //points we use for optical flow .. more points will be used for model estimation
    vector<vector<cv::Point2f> > featurePoints;
    //pose data for every frame
    vector<cv::Mat> frameTranslation;
    vector<cv::Mat> frameRotation;
    //points used to calculate model parameters
    vector<vector<cv::Point2f> > generatedPoints;
    //recalculated weights
    vector<vector<double> >vector_weights_exp;
    vector<vector<double> >vector_weights_id;
    QTimer *timerReplay;

    //camera dialog ui
    Ui::cameraDialog cameraUi;
    QDialog *cameraDialog;
};

#endif // TRANSFERWIDGET_H
