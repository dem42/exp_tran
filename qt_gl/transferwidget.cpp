#include "transferwidget.h"
#include "featurepointqlabel.h"
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

const int TransferWidget::fPoints[13] = {9521,7240,1183,8934,8945,6284,8197,2080,3923,6058,8825,1680,3907};
const int TransferWidget::fPoints_size = 13;


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

TransferWidget::TransferWidget(QString fileName) : fileName(fileName)
{
    picLabel = new FeaturePointQLabel();
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


    //m = imread("/home/martin/iowndis.png");
    //img = cvLoadImage("/home/martin/iowndis.png");
    //std::cout << img->nChannels <<" " << img->depth << " " <<   IPL_DEPTH_8U << std::endl;


    Mat m;
    grabThumbnailForVideo(fileName.toStdString(),m);

    selectGoodFeaturePoints(m);

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

void TransferWidget::calcIntrinsicParams()
{    
    Mat img = imread("../../chessboard.jpg");
    vector<Point2f> corners;
    //the size here is very important .. it cannot be a subset
    //of the inner corners in the image but the max number in there
    Size s(3,4);

    bool out = findChessboardCorners(img,s,corners);
    cout << out << endl;
    drawChessboardCorners(img,s,Mat(corners),out);

    QImage img_q = mat2QImage(img);
    QPixmap p_map;
    p_map = QPixmap::fromImage(img_q);
    picLabel->setPixmap(p_map);

    //order of corners is from top to bottom, left to right
    vector<Point3f> worldCoord;
    //top row
    worldCoord.push_back(Point3f(-3.0,3.0,-0.0));
    worldCoord.push_back(Point3f(0.0,3.0,-0.0));
    worldCoord.push_back(Point3f(3.0,3.0,-0.0));
    //middle row
    worldCoord.push_back(Point3f(-3.0,0.0,-0.0));
    worldCoord.push_back(Point3f(0.0,0.0,-0.0));
    worldCoord.push_back(Point3f(3.0,0.0,-0.0));
    //middle bottom
    worldCoord.push_back(Point3f(-3.0,-3.0,-0.0));
    worldCoord.push_back(Point3f(0.0,-3.0,-0.0));
    worldCoord.push_back(Point3f(3.0,-3.0,-0.0));
    //bottom row
    worldCoord.push_back(Point3f(-3.0,-6.0,-0.0));
    worldCoord.push_back(Point3f(0.0,-6.0,-0.0));
    worldCoord.push_back(Point3f(3.0,-6.0,-0.0));

    vector<vector<Point3f> > objectPoints;
    objectPoints.push_back(worldCoord);
    vector<vector<Point2f> > imagePoints;
    imagePoints.push_back(corners);

    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    calibrateCamera(objectPoints,imagePoints,img.size(),cameraMatrix,distCoeffs,rvecs,tvecs);

    cout << "CALIBRATED : " << endl;
    MatConstIterator_<double> it = cameraMatrix.begin<double>(), it_end = cameraMatrix.end<double>();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;
    cout << "distortions: " << endl;
    it = distCoeffs.begin<double>();
    it_end = distCoeffs.end<double>();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;
}

/**********************************************/
/* PUBLIC SLOTS */
/**********************************************/
//later do a pyramid version
void TransferWidget::startFaceTransfer()
{
    //first align our face guess over the marked feature points
    vector<Point2f> marked = picLabel->getMarked();
    //marked points should get recentered but we are gonna get rid of shifts anyway
    //marked points are ordered (should be ordered if the user clicked correctly)
    //the order is the same as the index order in fPoints

    calcIntrinsicParams();


    //face guess
    long double *w_id = new long double[56];
    long double *w_exp = new long double[7];
    Face *face_ptr = new Face();

    for(int i=0;i<56;i++)
    {
        if(i==33)w_id[i] = 0.2;
        else if(i==17)w_id[i] = 0.5;
        else if(i==50)w_id[i] = 0.3;
        else w_id[i] = 0;
    }
    w_exp[0] = 0.0;
    w_exp[1] = 0.0;
    w_exp[2] = 0.0;
    w_exp[3] = 0.2;
    w_exp[4] = 0.0;
    w_exp[5] = 0.8;
    w_exp[6] = 0.0;

    face_ptr->interpolate(w_id,w_exp);
    //use feature points to position the geometry

    //pick points from the face vertex data ... private atm .. then locate these points in
    //the image based on distance
//    for(int i=0; i < MAX_SAMPLES;i++)
//    {
//        vertex = vertexes[random()];
//        val_of_vertex = picLabel->getPointAt(vertex);
//    }
    //instead of using both point types overload operator Point3f in face.h including cv.h
    vector<Point3f> objectPoints;
    vector<Point2f> imagePoints;
    Point3 p3;
    for(int i=0;i<this->fPoints_size;i++)
    {
        p3 = face_ptr->getPointFromPolygon(fPoints[i]);        
        objectPoints.push_back(Point3f(p3.x,p3.y,p3.z));
    }
    imagePoints = picLabel->getMarked();

    //183.536 0 337.789 0 186.4 299.076 0 0 1
    //initialize intrinsic parameters of the camera
    double m[3][3] = {{183.536, 0, 337.789},{0, 186.4, 299.076},{0, 0, 1}};
    Mat cameraMatrix(3,3,CV_64F,m);

    //-0.0865513 0.197476 0.00430749 0.0072667 -0.114125
    double l[1][5] = {{-0.0865513, 0.197476, 0.00430749, 0.0072667, -0.114125}};
    Mat lensDist(1,5,CV_64F,l);

    //setup the result vectors for the extrinsic parameters
    Mat rvec, tvec;

    cout << "before solve pnp" << endl;
    //transform vectors into Mats with 3 (2) channels
    cv::solvePnP(Mat(objectPoints),Mat(imagePoints),cameraMatrix,lensDist,rvec,tvec);

    cout << "rvec" << endl;
    double sum=0;
    MatConstIterator_<double> it = rvec.begin<double>(), it_end = rvec.end<double>();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;
    cout << "tvec" << endl;
    it = tvec.begin<double>();
    it_end = tvec.end<double>();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;

    delete face_ptr;
    delete[] w_id;
    delete[] w_exp;
}

void TransferWidget::dropFrame()
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
    QImage img_q = mat2QImage(rgb_frame);
    QPixmap p_map;
    p_map = QPixmap::fromImage(img_q);
    picLabel->setPixmap(p_map);
    flowLabel->setPixmap(p_map);
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

void TransferWidget::findGoodFeaturePoints()
{
    Mat curFrame, curFrame2;

    if(frames.size() > 0)
        curFrame = *(frames.end()-1);
    else
        grabThumbnailForVideo(this->fileName.toStdString(),curFrame);

    cvtColor(curFrame,curFrame2,CV_RGB2GRAY);

    selectGoodFeaturePoints(curFrame);
}

void TransferWidget::selectGoodFeaturePoints(const Mat& m)
{
//    Mat col,row,reshaped;
//    reshaped = m.reshape(1);
//    //        col = reshaped.colRange(117,240);
//    //        row = col.rowRange(170,330);
//    col = reshaped.colRange(0,300);
//    row = col.rowRange(0,300);


    vector<Point2f> marked;
//    goodFeaturesToTrack(row,marked,40,0.01,0.01);
//    cout << marked[1].x << " " << marked[1].y << endl;
//    for(int k=0;k<40;k++)
//    {
//        //marked[k].x+=117;
//        // marked[k].y+=170;
//    }

    int number_of_features = 60;

    IplImage img, *frame, *eigen_image, *temp_img;

    CvSize size = m.size();
    img = m;
    frame = cvCreateImage(size,IPL_DEPTH_8U, 1);
    cvConvertImage(&img,frame,0);

    eigen_image = cvCreateImage(size,IPL_DEPTH_32F, 1);
    temp_img= cvCreateImage(size,IPL_DEPTH_32F, 1);

    CvPoint2D32f features[number_of_features];

    cvGoodFeaturesToTrack(frame,eigen_image,temp_img,features,&number_of_features,0.01,0.01,NULL,5,0,0.04);
    int x_shift = picLabel->getXShift();
    int y_shift = picLabel->getYShift();
    for(int i=0;i<number_of_features;i++)
    {
        //subtract the shift of the piclabel to account for shifted images
        marked.push_back(Point2f(features[i].x - x_shift,features[i].y - y_shift));
        cout << features[i].x << " " << features[i].y << endl;
    }
    picLabel->setMarked(marked);
}


//alot of shift nonesense .. plz just display the entire image
void TransferWidget::computeFlow()
{
    if(frames.size() < 2)
        return;

    Mat f1;
    Mat f2;
    vector<Point2f> curPoints = picLabel->getMarked();
    vector<Point2f> curPoints_centered;
    vector<Point2f> nextPoints;
    vector<Point2f> nextPoints_centered;
    vector<Vec2f> vectors;

    vector<Mat>::iterator it = frames.end();
    f2 = *(--it);
    f1 = *(--it);

    int x_shift = picLabel->getXShift();
    int y_shift = picLabel->getYShift();
    //readjust the shift in curPoints
    for(unsigned int j=0;j<curPoints.size();j++)
    {
        curPoints_centered.push_back(Point2f(curPoints[j].x + x_shift,curPoints[j].y + y_shift));
    }
    flowEngine->computeFlow(f1,f2,curPoints_centered,nextPoints);
    for(unsigned int j=0;j<nextPoints.size();j++)
    {
        nextPoints_centered.push_back(Point2f(nextPoints[j].x - x_shift,nextPoints[j].y - y_shift));
    }
    picLabel->setMarked(nextPoints_centered);
    flowLabel->setMarked(curPoints);

    for(unsigned int j=0;j<nextPoints.size();j++)
    {
        vectors.push_back(Vec2f(nextPoints[j].x - curPoints_centered[j].x, nextPoints[j].y - curPoints_centered[j].y));
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


