#include "transferwidget.h"

#include <phonon>
#include <iostream>
#include <qimage.h>
#include <cv.h>
#include <highgui.h>

using namespace cv;


QImage mat2QImage(const Mat mat_rgb)
{
    QImage qframe;
    //converting to rgb not necessary it seems
    //cvtColor(mat_bgr, mat_rgb, CV_BGR2RGB);

    qframe = QImage((const unsigned char*)(mat_rgb.data), mat_rgb.cols,
    mat_rgb.rows, QImage::Format_RGB888);
    return qframe;
}

TransferWidget::TransferWidget(QLabel* pic)
        //: picLabel(pic)
{
    picLabel = new ClickableQLabel();
    //video
    media = new Phonon::MediaObject(this);

    QString fileName("/home/martin/project/TrackedSmiles/S003-024.avi");
    media->setCurrentSource(fileName);

    vwidget = new Phonon::VideoWidget(this);
    Phonon::createPath(media, vwidget);

    audioOutput = new Phonon::AudioOutput(Phonon::VideoCategory, this);
    Phonon::createPath(media, audioOutput);



    media->play();

    //images
    IplImage *img = 0;
    cv::Mat m;
    m = imread("/home/martin/iowndis.png");
    img = cvLoadImage("/home/martin/iowndis.png");
    std::cout << img->nChannels <<" " << img->depth << " " <<   IPL_DEPTH_8U << std::endl;


    QImage img_q = mat2QImage(m);

    QImage img_org("/home/martin/Pictures/spectre.jpg");
    QPixmap p_map;
    p_map = QPixmap::fromImage(img_q);
    picLabel->setPixmap(p_map);
    picLabel->show();
    //ui.picLabel->showFullScreen();


    //initialize the capture so that we can capture
    //successive frames
     capture = new VideoCapture("/home/martin/project/TrackedSmiles/S003-024.avi");
    //capture = cvCaptureFromAVI("/home/martin/project/TrackedSmiles/S003-024.avi");

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

void TransferWidget::refreshCapturing()
{
    Mat frame, rgb_frame;
    (*capture) >> frame;
    cvtColor(frame, rgb_frame, CV_BGR2RGB);

    QImage img_q = mat2QImage(rgb_frame);
    QPixmap p_map;
    p_map = QPixmap::fromImage(img_q);
    picLabel->setPixmap(p_map);

//    IplImage* img = 0;
//    // capture a frame
//    if(!cvGrabFrame(capture)){
//      printf("Could not grab a frame\n\7");
//      return;
//    }
//    // retrieve the captured frame
//    img=cvRetrieveFrame(capture);
//
//    QImage img_q = IplImage2QImage(img);
//    QPixmap p_map;
//    p_map = QPixmap::fromImage(img_q);
//    ui.picLabel->setPixmap(p_map);
}

void TransferWidget::playSource()
{
    QString fileName("/home/martin/project/TrackedSmiles/S003-024.avi");
    const Phonon::MediaSource m("/home/martin/project/TrackedSmiles/test.ogv");
    std::cout << "playing" << std::endl;
    media->enqueue(m);
}

TransferWidget::~TransferWidget()
{
    delete capture;
    delete media;
    delete vwidget;
    delete audioOutput;
}


