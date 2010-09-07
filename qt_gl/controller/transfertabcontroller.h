#ifndef TRANSFERTABCONTROLLER_H
#define TRANSFERTABCONTROLLER_H

#include <cv.h>
#include "view/clickableqlabel.h"
#include "model/videoprocessor.h"
#include <QString>
#include <QTimer>
#include <QMutex>
#include <QLineEdit>
#include <QFileDialog>

#include "model/Face.h"
#include "view/customizablefacewidget.h"

class TransferTabController : public QObject
{
    Q_OBJECT
public:
    TransferTabController(ClickableQLabel *sourceLabel,ClickableQLabel *targetLabel,
                          QLineEdit *srcText, QLineEdit *targetText, CustomizableFaceWidget *face_widget);

public slots:
    void targetFileSelected(const QString str);
    void srcFileSelected(const QString str);
    void srcBrowse();
    void targetBrowse();
    void dropSrc();
    void dropTarget();
    void beginTransfer();

    void replayFrame();
    void processingFinished();

private:
    void convertFrameIntoTexture(Mat &frame);    
    void initSrcSide();
    void initTargetSide();

    void getClonedMouth(const Mat& img, unsigned int frame_index, Mat &target);
    Mat getMaskForLeftEyebrow();
    Mat getMaskForRightEyebrow();

    ClickableQLabel *sourceLabel;
    ClickableQLabel *targetLabel;
    VideoProcessor *src_videoProcessor;
    VideoProcessor *target_videoProcessor;

    bool textureInterpolate;

    VideoCapture *capSrc;
    VideoCapture *capTarget;

    QString srcFile;
    QString targetFile;
    QLineEdit *srcText;
    QLineEdit *targetText;

    QFileDialog *srcDia;
    QFileDialog *targetDia;

    Mat_<double> cameraSrc;
    Mat_<double> cameraTarget;
    Mat_<double> lensDist;

    vector<Mat> targetFrames;
    vector<Mat> srcFrames;

    CustomizableFaceWidget *face_widget;
    Face *src_face_ptr;
    Face *target_face_ptr;

    //TEMPORARY to test texture
    //for now so that we can use them in a timer
    vector<cv::Mat> s_frameData, t_frameData;
    //points we use for optical flow .. more points will be used for model estimation
    vector<vector<cv::Point2f> > featurePoints;

    bool srcFinished;
    bool targetFinished;
    QMutex mutex;

    QTimer *timerReplay;
};

#endif // TRANSFERTABCONTROLLER_H
