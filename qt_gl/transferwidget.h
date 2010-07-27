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

#include <cv.h>
#include <vector>
#include <string>

class TransferWidget : public QWidget
{
    Q_OBJECT
public:
    TransferWidget(QString fileName);
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
    void restartCapturing();
    void computeFlow();
    void captureFrame();
    void toggleDrawable(bool);
    void video_file_changed(const QString str);
    void findGoodFeaturePoints();
    void startFaceTransfer();
    void dropFrame();

private:
    void calcIntrinsicParams();

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

    //static here could be read from an xml config
    static const int fPoints[13];
    static const int fPoints_size;
};

#endif // TRANSFERWIDGET_H
