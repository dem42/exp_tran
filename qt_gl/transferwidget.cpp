#include "transferwidget.h"
#include "opticalflowfarneback.h"
#include <phonon>
#include <iostream>
#include <QImage>
#include <QTimer>
#include <cv.h>
#include <highgui.h>
#include <cstdlib>

using namespace cv;
using namespace std;

QImage mat2QImage(const Mat mat_rgb)
{
    QImage qframe;
    //converting to rgb not necessary it seems
    //cvtColor(mat_bgr, mat_rgb, CV_BGR2RGB);

    qframe = QImage((const unsigned char*)(mat_rgb.data), mat_rgb.cols,
    mat_rgb.rows, QImage::Format_RGB888);
    //rgb QImage::Format_RGB888
    return qframe;
}

TransferWidget::TransferWidget(QString fileName,
                               bool autoSelectFeaturePoints) : fileName(fileName),
                                                               autoSelectFeaturePoints(autoSelectFeaturePoints)
{
    picLabel = new ClickableQLabel();
    flowLabel = new VectorFieldQLabel();
    //video
    media = new Phonon::MediaObject(this);

    media->setCurrentSource(fileName);

    vwidget = new Phonon::VideoWidget(this);
    Phonon::createPath(media, vwidget);

    //no audio for now
    //audioOutput = new Phonon::AudioOutput(Phonon::VideoCategory, this);
    //Phonon::createPath(media, audioOutput);

    //initialize the optical flow engine
    flowEngine = new OpticalFlowEngine();

    //setup the timer
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(captureFrame()));

    media->play();

    //images

    Mat m;
    //m = imread("/home/martin/iowndis.png");
    //img = cvLoadImage("/home/martin/iowndis.png");
    //std::cout << img->nChannels <<" " << img->depth << " " <<   IPL_DEPTH_8U << std::endl;

    Mat col,row,reshaped;

    grabThumbnailForVideo(fileName.toStdString(),m);

    if(autoSelectFeaturePoints == true)
    {
        reshaped = m.reshape(1);
//        col = reshaped.colRange(117,240);
//        row = col.rowRange(170,330);
        col = reshaped.colRange(0,300);
        row = col.rowRange(0,300);


        vector<Point2f> marked;
        goodFeaturesToTrack(row,marked,40,0.01,0.01);
        cout << marked[1].x << " " << marked[1].y << endl;
        for(int k=0;k<40;k++)
        {
            //marked[k].x+=117;
           // marked[k].y+=170;
        }
        picLabel->setMarked(marked);
    }    

    QImage img_q = mat2QImage(m);

    QPixmap p_map;
    p_map = QPixmap::fromImage(img_q);

    picLabel->setPixmap(p_map);
    flowLabel->setPixmap(p_map);
    flowLabel->show();
    picLabel->show();
    //ui.picLabel->showFullScreen();

    //initialize the capture so that we can capture
    //successive frames
     capture = new VideoCapture(fileName.toStdString());
     frameCount = capture->get(CV_CAP_PROP_FRAME_COUNT); //SEEMS TO BE UNSUPPORTED AND RETURNS 0
     cout << "frame count is : " << frameCount << endl;

     connect(media,SIGNAL(aboutToFinish()),this,SLOT(playSource()));
}

Phonon::VideoWidget* TransferWidget::getVideoWidget() const
{
    return vwidget;
}

ClickableQLabel* TransferWidget::getPicLabel() const
{
    return picLabel;
}

ClickableQLabel* TransferWidget::getFlowLabel() const
{
    return flowLabel;
}

void TransferWidget::captureFrame()
{
    Mat frame, rgb_frame;

    if( capture->grab() == true)
        capture->retrieve(frame);
    else
    {
        timer->stop();
        return;
    }
    cvtColor(frame, rgb_frame, CV_BGR2RGB);

    frames.push_back(rgb_frame);

    QImage img_q = mat2QImage(rgb_frame);
    QPixmap p_map;
    p_map = QPixmap::fromImage(img_q);
    picLabel->setPixmap(p_map);

    computeFlow();
}

void TransferWidget::grabThumbnailForVideo(string videoName, Mat& thumbnail)
{
    Mat frame;

    VideoCapture cap(videoName);
    cap >> frame;
    cvtColor(frame, thumbnail, CV_BGR2RGB);

    cap.release();
}

//restart caputring by clearing marked points
//refreshing the picture to the thumbnail
//and stopping the timer and clearing old frames
void TransferWidget::restartCapturing()
{
    timer->stop();

    capture->release();
    delete capture;
    frames.clear();
    capture = new VideoCapture(fileName.toStdString());

    picLabel->clearMarked();
    flowLabel->clearMarked();

    cv::Mat m;

    grabThumbnailForVideo(fileName.toStdString(),m);
    QImage img_q = mat2QImage(m);

    QPixmap p_map;
    p_map = QPixmap::fromImage(img_q);

    picLabel->setPixmap(p_map);
    flowLabel->setPixmap(p_map);
}

void TransferWidget::playTransfer()
{
    captureFrame();
    timer->start(200);
}

void TransferWidget::playSource()
{
    QString fileName("/home/martin/project/TrackedSmiles/S003-024.avi");
    const Phonon::MediaSource m("/home/martin/project/TrackedSmiles/test.ogv");
    std::cout << "playing" << std::endl;
    media->enqueue(m);
}

void TransferWidget::computeFlow()
{
    if(frames.size() < 2)
        return;

    Mat f1;
    Mat f2;
    vector<Point2f> curPoints = picLabel->getMarked();
    vector<Point2f> nextPoints;
    vector<Vec2f> vectors;

    vector<Mat>::iterator it = frames.end();
    f2 = *(--it);
    f1 = *(--it);

    flowEngine->computeFlow(f1,f2,curPoints,nextPoints);
    picLabel->setMarked(nextPoints);
    flowLabel->setMarked(curPoints);

    for(int j=0;j<nextPoints.size();j++)
    {
        vectors.push_back(Vec2f(nextPoints[j].x - curPoints[j].x, nextPoints[j].y - curPoints[j].y));
    }
    flowLabel->setVectorField(vectors);

    QImage img_q = mat2QImage( *(frames.end()-1) );
    QPixmap p_map;
    p_map = QPixmap::fromImage(img_q);
    flowLabel->setPixmap(p_map);
    //flowLabel->setGeometry(800,0,400,400);
    flowLabel->show();
}

void TransferWidget::video_file_changed(const QString str)
{
    cout << str.toStdString() << endl;
    fileName = str;
    restartCapturing();
}

void TransferWidget::toggleDrawable(bool drawable)
{
    picLabel->clearMarked();
    flowLabel->clearMarked();
    picLabel->setDrawable(drawable);
    flowLabel->setDrawable(drawable);
}

TransferWidget::~TransferWidget()
{
    frames.clear();
    delete capture;
    delete media;
    delete vwidget;
    delete audioOutput;

    delete flowEngine;

    delete picLabel;
    delete flowLabel;
}


