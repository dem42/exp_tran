#include "videotabcontroller.h"
#include "model/opticalflowfarneback.h"
#include "utility.h"

#include <iostream>
#include <fstream>
#include <QImage>
#include <QTimer>
#include <cv.h>
#include <highgui.h>
#include <cstdlib>
#include <cmath>
#include <ctime>

#include "model/errorfunction.h"
#include "model/modelimageerror.h"

#include "model/poseestimator.h"
#include "model/exptranexception.h"


using namespace cv;
using namespace std;



VideoTabController::VideoTabController(QString fileName, ClickableQLabel *picLabel,
                                       VectorFieldQLabel *flowLabel, ExpTranAbstractView *view,
                                       FaceWidget *face_widget) : fileName(fileName)
{
    this->picLabel = picLabel;
    this->flowLabel = flowLabel;
    this->view = view;
    this->face_widget = face_widget;
    this->face_widget->setCameraParameters(-200,-1,-200);
    face_ptr  = new Face();
    face_widget->setFace(face_ptr);

    cameraDialog = new QDialog();
    cameraUi.setupUi(cameraDialog);
    connect(cameraUi.buttonBox,SIGNAL(accepted()),this,SLOT(setCameraParameters()));

    //initialize the optical flow engine
    flowEngine = new OpticalFlowEngine();
    //init the optimizer
    poseEstimator = new PoseEstimator();

    opttype = VideoProcessor::OptType_INTERPOLATE;
    idconstype = VideoProcessor::IdConstraintType_CONST;
    projtype = VideoProcessor::PointGenerationType_2D;
    regParam = 2000.0;
    frame_num = 10;
    iter_num = 30;
    projModel = false;
    withFirstFrame = true;

    //setup the timer
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(captureFrame()));

    //initialize the capture so that we can capture
    //successive frames
    capture = new VideoCapture(fileName.toStdString());
    frameCount = capture->get(CV_CAP_PROP_FRAME_COUNT); //SEEMS TO BE UNSUPPORTED AND RETURNS 0

    Mat m;

    Utility::grabThumbnailForVideo(fileName.toStdString(),m);

    double c_x = m.size().width / 2.;
    double c_y = m.size().height / 2.;

    frames.push_back(m);

    QImage img_q = Utility::mat2QImage(m);

    QPixmap p_map;
    p_map = QPixmap::fromImage(img_q);

    picLabel->setPixmap(p_map);
    flowLabel->setPixmap(p_map);
    flowLabel->show();
    picLabel->show();
    //ui.picLabel->showFullScreen();


    //initalize the camera based on precalculated intrinsic parameters
    //283.536, 0, 337.789},{0, 286.4, 299.076},{0, 0, 1}
    //389.515 0 303.703 0 394.657 159.174 0 0 1
    //initialize intrinsic parameters of the camera
    //double mat[3][3] = {{389.515, 0, 303.703},{0, 394.657, 159.174},{0, 0, 1}};
    //     double mat[3][3] = {{283.536, 0, 337.789},{0, 286.4, 299.076},{0, 0, 1}};
    //    cameraMatrix = Mat(3,3,CV_64F,mat);
    cameraMatrix = Mat_<double>(3,3);
    cameraMatrix(0,0) = 900;//589.515;
    cameraMatrix(0,1) = 0;
    cameraMatrix(0,2) = c_x;//337.789;//303.703;
    cameraMatrix(1,0) = 0;
    cameraMatrix(1,1) = 900;//594.657;
    cameraMatrix(1,2) = c_y;//299.076;//159.174;
    cameraMatrix(2,0) = 0;
    cameraMatrix(2,1) = 0;
    cameraMatrix(2,2) = 1;


    //double l[1][5] = {{-0.0865513, 0.197476, 0.00430749, 0.0072667, -0.114125}};
    //-0.20357 0.423957 -0.00415811 -0.00633922 -0.295448
    //double l[1][5] = {{-0.20357, 0.423957, -0.00415811, -0.00633922, -0.295448}};
    lensDist = Mat_<double>(1,5);
    //    lensDist(0,0) = -0.20357;
    //    lensDist(0,1) = 0.423957;
    //    lensDist(0,2) = -0.00415811;
    //    lensDist(0,3) = -0.00633922;
    //    lensDist(0,4) = -0.295448;
    lensDist(0,0) = 0;
    lensDist(0,1) = 0;
    lensDist(0,2) = 0;
    lensDist(0,3) = 0;
    lensDist(0,4) = 0;
}

void VideoTabController::calcIntrinsicParams()
{    
    vector<Mat> imgs;

    imgs.push_back(imread("../../camera2.jpg"));
    imgs.push_back(imread("../../camera1.jpg"));


    //the size here is very important .. it cannot be a subset
    //of the inner corners in the image but the max number in there
    Size s(7,7);

    //order of corners is from top to bottom, left to right
    vector<Point3f> worldCoord;
    Point3f p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    p.y = (s.height) * -3;
    for(int i=0;i<s.height;i++)
    {
        p.y += 3;
        p.x = (s.width) * -3;
                for(int j=0;j<s.width;j++)
        {
            p.x += 3;            
            worldCoord.push_back(p);
        }
    }


    vector<vector<Point3f> >objectPoints;
    vector<vector<Point2f> >imagePoints;

    Mat img;
    vector<Mat>::iterator iter = imgs.begin(), iter_end = imgs.end();
    for(; iter != iter_end; ++iter)
    {
        img = *iter;
        vector<Point2f> corners;
        // use calibration matrix value to find opengl parameters calibrationMatrixValues();

        bool out = findChessboardCorners(img,s,corners);
        cout << out << endl;

        drawChessboardCorners(img,s,Mat(corners),out);

        QImage img_q = Utility::mat2QImage(img);
        QPixmap p_map;
        p_map = QPixmap::fromImage(img_q);
        picLabel->setPixmap(p_map);



        objectPoints.push_back(worldCoord);
        imagePoints.push_back(corners);

    }

    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    calibrateCamera(objectPoints,imagePoints,imgs[0].size(),cameraMatrix,distCoeffs,rvecs,tvecs);

    cout << "CALIBRATED : " << endl;
    MatConstIterator_<double> it = cameraMatrix.begin<double>(), it_end = cameraMatrix.end<double>();
    for(; it != it_end; ++it)
    {
        cout << *it << " ";
    }
    cout << endl;
    cout << "distortions: " << endl;
    it = distCoeffs.begin<double>();
    it_end = distCoeffs.end<double>();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;
    cout << "rvecs: " << endl;
    it = rvecs[0].begin<double>();
    it_end = rvecs[0].end<double>();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;
    cout << "tvecs: " << endl;
    it = tvecs[0].begin<double>();
    it_end = tvecs[0].end<double>();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;
}


/**********************************************/
/* PUBLIC SLOTS */
/**********************************************/
void VideoTabController::replayFrame()
{
    static unsigned int i;
    double euler_x, euler_y, euler_z;
    Mat_<double> rmatrix;
    Mat_<double> rot;
    Mat_<double> tran;
    vector<Point2f> points;

    videoProcessor->getFaceAndPoseForFrame(i,face_ptr,rot,tran);

    double tx = tran(0,0);
    double ty = tran(0,1);
    double tz = tran(0,2);
    //compute and set the pose parameters
    Rodrigues(rot,rmatrix);
    Utility::computeEulerAnglesFromRmatrix(rmatrix,euler_x,euler_y,euler_z);
    //only y needs to be negative so that it agrees with the transposes

    //set the parameters in face widget
    face_widget->setTransParams(euler_x,-euler_y,euler_z,tx,ty,tz);
    face_widget->setFace(face_ptr);

    if(projModel)
    {
        videoProcessor->getGeneratedPointsForFrame(i,points);
        picLabel->setMarked(points);
    }
    else
        picLabel->clearMarked();

    picLabel->setPixmap(Utility::mat2QPixmap(frameData[i]));


    cout << "FRAAME " << i << endl;
    i++;

    view->incrementVideoProgress();
    if(i == videoProcessor->getFrameNum())
    {
        timerReplay->stop();
        i = 0;
        frameData.clear();
        featurePoints.clear();

        view->setAllVideoTabButtonsDisabled(false);
        delete videoProcessor;
    }    
}

void VideoTabController::processingFinished()
{
    cout << "PROCESSING OVER" << endl;        
    if(!videoProcessor->getCrashed())
    {
        //after processing is complete start the timer
        timerReplay = new QTimer(this);
        connect(timerReplay,SIGNAL(timeout()),this,SLOT(replayFrame()));
        timerReplay->start(500);
    }
    else
    {
        ExpTranException e("execution of video processor failed .. see log for reason");
        view->displayException(e);
        view->setAllVideoTabButtonsDisabled(false);
        delete videoProcessor;
        restartCapturing();
    }
}

void VideoTabController::playBack()
{
    Mat frame, rgb_frame, copyFrame;
    int counter = 0;

    flowLabel->setVisible(false);
    face_widget->setVisible(true);
    view->setAllVideoTabButtonsDisabled(true);

    /**********************/
    /*first collect frames*/
    /**********************/
    //use the VideoCapture attribute capture of to collect frame data
    //note that the first frame is not captured (the one wherein we mark the
    //feature points so we add it first
    frameData.push_back(frames[frames.size()-1]);
    while( capture->grab() == true)
    {
        capture->retrieve(frame);
        cvtColor(frame, rgb_frame, CV_BGR2RGB);
        copyFrame = rgb_frame.clone();
        frameData.push_back(copyFrame);
        counter++;
    }
    if(counter < frame_num) frame_num = counter;

    videoProcessor = new VideoProcessor(picLabel->getMarked(),frameData,cameraMatrix,lensDist,opttype,
                                        regParam,idconstype,projtype,frame_num,iter_num,withFirstFrame);

    connect(videoProcessor,SIGNAL(finished()),this,SLOT(processingFinished()));

    videoProcessor->start();
}

void VideoTabController::calibrate()
{
    cameraDialog->show();
}


void VideoTabController::extractPose()
{
    vector<int> correspondences;
    vector<int>::iterator cit, cit_end;

    double euler_x, euler_y, euler_z;
    double tx, ty, tz;

    Mat_<double> rvec, tvec;
    Mat_<double> rmatrix;
    vector<Point2f> sampledPoints;

    Face *face_guess = new Face();
    double *w_id = new double[face_guess->getIdNum()];
    double *w_exp = new double[face_guess->getExpNum()];
    vector<Point2f> marked = picLabel->getMarked();

    vector<int> indices(Face::fPoints,Face::fPoints+Face::fPoints_size);

    flowLabel->setVisible(false);
    face_widget->setVisible(true);

    //marked points should get recentered but we are gonna get rid of shifts anyway
    //marked points are ordered (should be ordered if the user clicked correctly)
    //the order is the same as the index order in fPoints

    int ID = face_guess->getIdNum();
    int EXP = face_guess->getExpNum();
    for(int i=0;i<ID;i++)
    {
        w_id[i] = 1./(double)(ID);
    }
    for(int i=0;i<EXP;i++)
    {
        w_exp[i] = 1./(double)(EXP);
    }

    face_guess->setNewIdentityAndExpression(w_id,w_exp);

    poseEstimator->calculateTransformation(marked,face_guess,cameraMatrix,lensDist,indices,rvec,tvec,false);

    //convert the rotation vector to a rotation matrix
    cv::Rodrigues(rvec,rmatrix);
    Utility::computeEulerAnglesFromRmatrix(rmatrix,euler_x,euler_y,euler_z);

    tx = tvec.at<double>(0,0);
    ty = tvec.at<double>(0,1);
    tz = tvec.at<double>(0,2);

    Utility::sampleGoodPoints(marked,sampledPoints);
    //picLabel->setMarked(sampledPoints);


    vector<Point3f> obj;
    vector<Point2f> imagePoints;

    for(int i=0;i<Face::mouth_size;i++)
        obj.push_back(face_guess->getPointFromPolygon(Face::mouth[i]));


//    poseEstimator->reprojectInto3DUsingWeak(imagePoints,rvec,tvec,cameraMatrix,lensDist,face_guess,correspondences);
//    cit = correspondences.begin();
//    cit_end = correspondences.end();
//    cout << "correspondences are : " << endl;
//    for(;cit!=cit_end;cit++)
//        cout << *cit << " ";
//    cout << endl;
//
//    obj.clear();
//    for(int i=0;i<correspondences.size();i++)
//        obj.push_back(face_guess->getPoint(correspondences[i]));

    imagePoints.clear();
    projectPoints(Mat(obj),rvec,tvec,cameraMatrix,lensDist,imagePoints);
    picLabel->setMarked(imagePoints);
    //the sign here doesnt seem to make a difference as far as orientation of
    //the object is concerned .. however it does flip the extreme head
    //z is the one we dont need to inverse since opengl displays with upvector as 0,0,-1
    //x also changes if we the up vector is upside down
    //only y needs to be negative so that it agrees with the transposes
    //that the pose estimation returns .. (solvePnP return R = rzTryTrxT but a different upvector
    face_widget->setTransParams(euler_x,-euler_y,euler_z,tx,ty,tz);
    face_widget->setFace(face_guess);
    face_widget->refreshGL();

    delete[] w_id;
    delete[] w_exp;
}

void VideoTabController::dropFrame()
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

    QPixmap p_map = Utility::mat2QPixmap(rgb_frame);
    picLabel->setPixmap(p_map);
    flowLabel->setPixmap(p_map);    
}

void VideoTabController::captureFrame()
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

    picLabel->setPixmap(Utility::mat2QPixmap(rgb_frame));

    computeFlow();
}



//restart caputring by clearing marked points
//refreshing the picture to the thumbnail
//and stopping the timer and clearing old frames
void VideoTabController::restartCapturing()
{
    timer->stop();

    flowLabel->setVisible(true);
    face_widget->setVisible(false);

    capture->release();
    delete capture;
    frames.clear();

    featurePoints.clear();
    frameData.clear();

    //get rid of the old face
    delete face_ptr;
    face_ptr = new Face();

    capture = new VideoCapture(fileName.toStdString());

    picLabel->clearMarked();
    flowLabel->clearMarked();

    Mat m,um;

    Utility::grabThumbnailForVideo(fileName.toStdString(),m);
    double c_x = m.size().width / 2.;
    double c_y = m.size().height / 2.;
    //set this when you load a video
    cameraMatrix.at<double>(0,2) = c_x;
    cameraMatrix.at<double>(1,2) = c_y;
    //undistort(m,um,cameraMatrix,lensDist);
    frames.push_back(m);

    QImage img_q = Utility::mat2QImage(m);

    QPixmap p_map;
    p_map = QPixmap::fromImage(img_q);

    picLabel->setPixmap(p_map);
    flowLabel->setPixmap(p_map);
}

void VideoTabController::playTransfer()
{
    flowLabel->setVisible(true);
    face_widget->setVisible(false);

    captureFrame();
    timer->start(200);
}

void VideoTabController::pauseTransfer()
{
    timer->stop();
}


void VideoTabController::findGoodFeaturePoints()
{
    Utility::selectGoodFeaturePoints(picLabel);
}

void VideoTabController::computeFlow()
{
    if(frames.size() < 2)
        return;

    Mat f1;
    Mat f2;
    vector<Point2f> curPoints = picLabel->getMarked();
    vector<Point2f> nextPoints;
    vector<Vec2f> vectors;
    vector<int> a;
    vector<int> b;

    vector<Mat>::iterator it = frames.end();
    f2 = *(--it);
    f1 = *(--it);

    flowEngine->computeFlow(f1,f2,curPoints,nextPoints);

    picLabel->setMarked(nextPoints);
    flowLabel->setMarked(curPoints);

    for(unsigned int j=0;j<nextPoints.size();j++)
    {
        vectors.push_back(Vec2f(nextPoints[j].x - curPoints[j].x, nextPoints[j].y - curPoints[j].y));
    }
    flowLabel->setVectorField(vectors);

    QImage img_q = Utility::mat2QImage( *(frames.end()-1) );
    QPixmap p_map;
    p_map = QPixmap::fromImage(img_q);
    flowLabel->setPixmap(p_map);    
    flowLabel->show();
}

void VideoTabController::video_file_changed(const QString str)
{
    cout << str.toStdString() << endl;
    fileName = str;

    restartCapturing();
}

void VideoTabController::setCameraParameters()
{
    if(cameraUi.paramBox->isChecked())
    {
        cameraMatrix.at<double>(0,0) = cameraUi.fx->value();

        cameraMatrix.at<double>(1,1) = cameraUi.fy->value();

        cameraMatrix.at<double>(0,2) = cameraUi.cx->value();

        cameraMatrix.at<double>(1,2) = cameraUi.cy->value();
    }
    else if(cameraUi.cameraBox->isChecked())
        calcIntrinsicParams();
}

void VideoTabController::toggleDrawable(bool drawable)
{
    picLabel->clearMarked();
    flowLabel->clearMarked();
    picLabel->setDrawable(drawable);
    flowLabel->setDrawable(drawable);
}

//settings
void VideoTabController::setOptNelder(bool toggled)
{
    if(toggled == true)
        opttype = VideoProcessor::OptType_NELDER_INT;
}
void VideoTabController::setOptReg(double regParam)
{
    opttype = VideoProcessor::OptType_LIN_COMB;
    this->regParam = regParam;
}
void VideoTabController::setOptType(int t)
{    
    if(t == 0)
        opttype = VideoProcessor::OptType_INTERPOLATE;
    else
        opttype = VideoProcessor::OptType_LIN_COMB;
}
void VideoTabController::setFrameNum(int n)
{
    frame_num = n;
}
void VideoTabController::setIterNum(int n)
{
    iter_num = n;
}
void VideoTabController::setConstID(bool b)
{
    if(b == true)
        this->idconstype = VideoProcessor::IdConstraintType_CONST;
    else
        this->idconstype = VideoProcessor::IdConstraintType_NONE;
}
void VideoTabController::setPointGen2D(bool b)
{
    if(b == true)
        this->projtype = VideoProcessor::PointGenerationType_2D;
}
void VideoTabController::setPointGen3D(bool b)
{
    if(b == true)
        this->projtype = VideoProcessor::PointGenerationType_3D;
}
void VideoTabController::setHybrid(bool b)
{
    if(b == true)
        this->projtype = VideoProcessor::PointGenerationType_HYBRID;
}
void VideoTabController::setNone(bool b)
{
    if(b == true)
        this->projtype = VideoProcessor::PointGenerationType_NONE;
}
void VideoTabController::setProjModel(bool b)
{
    this->projModel = b;
}
void VideoTabController::setWithFirstFrame(bool b)
{
    this->withFirstFrame = b;
}

VideoTabController::~VideoTabController()
{
    frames.clear();
    delete capture;

    delete flowEngine;
    delete poseEstimator;
    delete picLabel;
    delete flowLabel;

    delete face_ptr;
}


