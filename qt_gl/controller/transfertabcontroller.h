#ifndef TRANSFERTABCONTROLLER_H
#define TRANSFERTABCONTROLLER_H

#include <cv.h>
#include "view/clickableqlabel.h"
#include "model/videoprocessor.h"
#include <QString>
#include <QTimer>
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

    CustomizableFaceWidget *face_widget;
    Face *face_ptr;


    //TEMPORARY to test texture
    //for now so that we can use them in a timer
    vector<cv::Mat> s_frameData, t_frameData;
    //points we use for optical flow .. more points will be used for model estimation
    vector<vector<cv::Point2f> > featurePoints;
    //pose data for every frame
    vector<cv::Mat> s_frameTranslation, t_frameTranslation;
    vector<cv::Mat> s_frameRotation, t_frameRotation;
    //points used to calculate model parameters
    vector<vector<cv::Point2f> > s_generatedPoints, t_generatedPoints;
    //recalculated weights
    vector<vector<double> >s_vector_weights_exp, t_vector_weights_exp;
    vector<vector<double> >s_vector_weights_id, t_vector_weights_id;
    QTimer *timerReplay;
};

#endif // TRANSFERTABCONTROLLER_H
