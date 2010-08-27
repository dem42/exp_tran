#ifndef TRANSFERTABCONTROLLER_H
#define TRANSFERTABCONTROLLER_H

#include <cv.h>
#include "clickableqlabel.h"
#include "videoprocessor.h"
#include <QString>
#include <QTimer>
#include <QLineEdit>
#include <QFileDialog>

#include "Face.h"
#include "face_widget.h"

class TransferTabController : public QObject
{
    Q_OBJECT
public:
    TransferTabController(ClickableQLabel *sourceLabel,ClickableQLabel *targetLabel,
                          QLineEdit *srcText, QLineEdit *targetText, FaceWidget *face_widget);

public slots:
    void targetFileSelected(const QString str);
    void srcFileSelected(const QString str);
    void srcBrowse();
    void targetBrowse();
    void dropSrc();
    void dropTarget();
    void beginTransfer();

    void replayFrame();
private:
    void convertFrameIntoTexture(Mat &frame);

    ClickableQLabel *sourceLabel;
    ClickableQLabel *targetLabel;
    VideoProcessor *videoProcessor;

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

    FaceWidget *face_widget;
    Face *face_ptr;


    //TEMPORARY to test texture
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
};

#endif // TRANSFERTABCONTROLLER_H
