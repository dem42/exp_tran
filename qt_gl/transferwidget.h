#ifndef TRANSFERWIDGET_H
#define TRANSFERWIDGET_H

#include <phonon>
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

#include <cv.h>
#include <vector>
#include <string>

class FaceWidget;

class TransferWidget : public QWidget
{
    Q_OBJECT
public:
    TransferWidget(QString fileName, FaceWidget *face_widget);
    ~TransferWidget();
    Phonon::VideoWidget* getVideoWidget() const;
    ClickableQLabel* getPicLabel() const;
    ClickableQLabel* getFlowLabel() const;
    void grabThumbnailForVideo(std::string videoName,cv::Mat& thumbnail);
    void selectGoodFeaturePoints(const cv::Mat& m);

    void setInteractiveLabel(QLabel *);   

public slots:
    void playSource();
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

private:
    void calcIntrinsicParams();
    void processVideo();
    void calculateTransformation(vector<cv::Point2f> imagePoints, Face* face_ptr,
                                 cv::Mat &rvec,cv::Mat &tvec,bool useExt=true);

    //video fileName
    QString fileName;
    bool autoSelectFeaturePoints;

    QTimer *timer;

    Phonon::MediaObject *media;
    Phonon::VideoWidget *vwidget;
    Phonon::AudioOutput *audioOutput;

    cv::VideoCapture *capture;
    double frameCount;

    ClickableQLabel *picLabel;
    VectorFieldQLabel *flowLabel;

    std::vector<cv::Mat> frames;

    //object responsible for computing
    //optical flow
    OpticalFlowEngine *flowEngine;
    
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
    QTimer *timerReplay;
};

#endif // TRANSFERWIDGET_H
