#include "videotabcontroller.h"
#include "model/opticalflowfarneback.h"
#include "utility.h"

#include <iostream>
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
    cout << "frame count is : " << frameCount << " " << c_x << " " << c_y << endl;

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
            cout << p.x << " " << p.y << endl;
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
        cout << *it << " ";
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
    cout << "setting trans param " << euler_x << " " << euler_y << " "
            << euler_z  << " " << tx << " " << ty << " " << tz  << endl;

    //set the parameters in face widget
    face_widget->setTransParams(euler_x,-euler_y,euler_z,tx,ty,tz);
    face_widget->setFace(face_ptr);

    videoProcessor->getGeneratedPointsForFrame(i,points);
    picLabel->setPixmap(Utility::mat2QPixmap(frameData[i]));
    picLabel->setMarked(points);

    cout << "FRAAME " << i << endl;
    i++;

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

    //after processing is complete start the timer
    timerReplay = new QTimer(this);
    connect(timerReplay,SIGNAL(timeout()),this,SLOT(replayFrame()));
    timerReplay->start(500);
}

void VideoTabController::playBack()
{
    Mat frame, rgb_frame, copyFrame;

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
    }

    videoProcessor = new VideoProcessor(picLabel->getMarked(),frameData,cameraMatrix,lensDist,
                                        VideoProcessor::OptType_LIN_COMB,5000.,20,3);

    connect(videoProcessor,SIGNAL(finished()),this,SLOT(processingFinished()));

    videoProcessor->start();    

//    videoProcessor->processVideo(picLabel->getMarked(),frameData, cameraMatrix, lensDist,
//                                 frameTranslation,frameRotation, generatedPoints,vector_weights_exp,vector_weights_id);

}

void VideoTabController::calibrate()
{
    cameraDialog->show();
}



//later do a pyramid version
void VideoTabController::startFaceTransfer()
{
    flowLabel->setVisible(false);
    face_widget->setVisible(true);

    //first align our face guess over the marked feature points
    vector<Point2f> marked = picLabel->getMarked();

    //marked points should get recentered but we are gonna get rid of shifts anyway
    //marked points are ordered (should be ordered if the user clicked correctly)
    //the order is the same as the index order in fPoints

    //calcIntrinsicParams();


    MatConstIterator_<double> it = cameraMatrix.begin(), it_end = cameraMatrix.end();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;
    lensDist(0,0) = 0;
    lensDist(0,1) = 0;
    lensDist(0,2) = 0;
    lensDist(0,3) = 0;
    lensDist(0,4) = 0;

    //setup the result vectors for the extrinsic parameters
    Mat_<double> rvec, tvec;

    Face *face_guess = new Face();
    double *w_id = new double[56];
    double *w_exp = new double[7];

    for(int i=0;i<56;i++)
    {
        if(i==33)w_id[i] = 0.2;
        else if(i==10)w_id[i] = 0.2;
        else if(i==20)w_id[i] = 0.2;
        else if(i==1)w_id[i] = 0.2;
        else if(i==50)w_id[i] = 0.2;
        else w_id[i] = 0;
    }
    w_exp[0] = 0.0;
    w_exp[1] = 0.0;
    w_exp[2] = 0.0;
    w_exp[3] = 0.0;
    w_exp[4] = 1.0;
    w_exp[5] = 0.0;
    w_exp[6] = 0.0;
    face_guess->interpolate(w_id,w_exp);
    vector<int> indices(Face::fPoints,Face::fPoints+Face::fPoints_size);
    poseEstimator->calculateTransformation(marked,face_guess,cameraMatrix,lensDist,indices,rvec,tvec,false);

    Mat_<double> rmatrix;
    //convert the rotation vector to a rotation matrix
    cv::Rodrigues(rvec,rmatrix);

    const double PI = 3.141593;


    double rot_y, rot_x, rot_z,cy, cx,sx,cz,sz;
    rot_y = asin( rmatrix.at<double>(2,0));        /* Calculate Y-axis angle */
    cy           =  cos( rot_y );

    if ( fabs( cy ) > 0.005 )             /* Gimball lock? */
      {
      cx      =  rmatrix.at<double>(2,2) / cy;           /* No, so get X-axis angle */
      sx      = rmatrix.at<double>(2,1)  / cy;

      rot_x  = atan2( sx, cx );

      cz      =  rmatrix.at<double>(0,0) / cy;            /* Get Z-axis angle */
      sz      = rmatrix.at<double>(1,0) / cy;

      rot_z  = atan2( sz, cz );
  }


    double rx[3][3] = {{1,0,0},{0,::cos(rot_x),-::sin(rot_x)},{0,::sin(rot_x),::cos(rot_x)}};
    double ry[3][3] = {{::cos(rot_y),0,-::sin(rot_y)},{0,1,0},{::sin(rot_y),0,::cos(rot_y)}};
    double rz[3][3] = {{::cos(rot_z),-::sin(rot_z),0},{::sin(rot_z),::cos(rot_z),0},{0,0,1}};

    Mat rX(3,3,CV_64F,rx), rY(3,3,CV_64F,ry), rZ(3,3,CV_64F,rz);
    Mat result = rZ*rY*rX;

    cout << "its so dumb i want to scream rZT*rYT*rXT" << Matrix(result);
    cout << "so soooo dumb rmatrix: " << Matrix(rmatrix);

    double r11 = rmatrix.at<double>(0,0), r12 = rmatrix.at<double>(0,1),r13 = rmatrix.at<double>(0,2);
    double r21 = rmatrix.at<double>(1,0), r22 = rmatrix.at<double>(1,1),r23 = rmatrix.at<double>(1,2);
    double r31 = rmatrix.at<double>(2,0), r32 = rmatrix.at<double>(2,1),r33 = rmatrix.at<double>(2,2);
    double tx = tvec.at<double>(0,0);
    double ty = tvec.at<double>(0,1);
    double tz = tvec.at<double>(0,2);    

    double r[3][4] = {{r11, r12, r13, tx},{r21,r22,r23,ty},{r31,r32,r33,tz}};
    Mat R(3,4,CV_64F,r);
    Mat res = cameraMatrix * R;


    //decompose
    Vec3d euler;
    Mat cam,rot,trans,rotX,rotY,rotZ;
    decomposeProjectionMatrix(res,cam,rot,trans,rotX,rotY,rotZ,euler);

    cout << "euler" << endl;    
    for(int i=0; i<3; ++i)
        cout << euler[i] << " ";
    cout << endl;

    Point3f p;
    Mat_<double> point3dMat(3,1);
    Mat_<double> point2dMat(3,1);
    vector<Point2f> points;

    Mat_<double> translation = tvec;

    cout << "TRANS : " << tvec.at<double>(0,0) << " " << tvec.at<double>(1,0)  << " " << tvec.at<double>(2,0) << endl;

    Point2d pp;

    double scale = 1;

    for(int i=0; i<Face::fPoints_size; i++)
    {
        p = face_guess->getPoint(Face::fPoints[i]);
        point3dMat(0,0) = p.x;
        point3dMat(1,0) = p.y;
        point3dMat(2,0) = p.z;

        point2dMat = cameraMatrix * ((rmatrix * point3dMat));
        point2dMat = scale*point2dMat;
        point2dMat = point2dMat + cameraMatrix*translation;

        //translation = cameraMatrix*tvec;
//        //homogenous coord
//        translation = (1/translation.at<double>(2,0)) * translation;

        //homogenous coord
        point2dMat(0,0) /= point2dMat(2,0);
        point2dMat(1,0) /= point2dMat(2,0);

        //point2dMat = point2dMat + translation;

        cout << "2d: " << point2dMat(0,0) << " " << point2dMat(1,0) << endl;

        //objectPoints.push_back(Point3f(p.x,p.y,p.z));

        points.push_back(Point2f(point2dMat(0,0) , point2dMat(1,0)));
    }

    //picLabel->setMarked(points);

    vector<Point2f> sampledPoints;
    Utility::sampleGoodPoints(points,sampledPoints);
    picLabel->setMarked(sampledPoints);


    vector<int> correspondences;
    vector<int>::iterator cit, cit_end;
    poseEstimator->reprojectInto3DUsingWeak(points,rvec,tvec,cameraMatrix,lensDist,face_guess,correspondences);
    cit = correspondences.begin();
    cit_end = correspondences.end();
    cout << "correspondences are : " << endl;
    for(;cit!=cit_end;cit++)
        cout << *cit << " ";
    cout << endl;
    double euler_x = rot_x * 180./PI;
    double euler_y = rot_y * 180./PI;
    double euler_z = rot_z * 180./PI;

    cout << "my euler : " << euler_x << " " << euler_y << " " << euler_z << endl;
    cout << "their euler : " << euler[0]<< " " << euler[1]<< " " << euler[2] << endl;

    //the sign here doesnt seem to make a difference as far as orientation of
    //the object is concerned .. however it does flip the extreme head
    //z is the one we dont need to inverse since opengl displays with upvector as 0,0,-1
    //x also changes if we the up vector is upside down
    //only y needs to be negative so that it agrees with the transposes
    //that the pose estimation returns .. (solvePnP return R = rzTryTrxT but a different upvector
    face_widget->setTransParams(euler[0],-euler[1],euler[2],tx,ty,tz);
    //face_widget->setTransParams(euler[0],euler[1],euler[2],tx,ty,tz);
    face_widget->setFace(face_guess);
    face_widget->refreshGL();


    Mat gradImg(frames[frames.size()-1].size().height,frames[frames.size()-1].size().width, CV_8UC1);
    Utility::filterForGradient(frames[frames.size()-1],gradImg);

    picLabel->setPixmap(Utility::mat2QPixmap(gradImg));

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
    Mat curFrame, curFrame2;

    if(frames.size() > 0)
        curFrame = *(frames.end()-1);
    else
        Utility::grabThumbnailForVideo(this->fileName.toStdString(),curFrame);

    cvtColor(curFrame,curFrame2,CV_RGB2GRAY);

    selectGoodFeaturePoints(curFrame);
}

void VideoTabController::selectGoodFeaturePoints(const Mat& m)
{
    vector<Point2f> marked;

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

    for(int i=0;i<number_of_features;i++)
    {    
        marked.push_back(Point2f(features[i].x,features[i].y));
    }
    picLabel->setMarked(marked);
}


//alot of shift nonesense .. plz just display the entire image
void VideoTabController::computeFlow()
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

    QImage img_q = Utility::mat2QImage( *(frames.end()-1) );
    QPixmap p_map;
    p_map = QPixmap::fromImage(img_q);
    flowLabel->setPixmap(p_map);
    //flowLabel->setGeometry(800,0,400,400);
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
    QString fxs = cameraUi.fx->selectedText();
    QString fys = cameraUi.fy->selectedText();
    QString cxs = cameraUi.cx->selectedText();
    QString cys = cameraUi.cy->selectedText();
    bool ok = true;

    double fxd = fxs.toDouble(&ok);
    if(ok == false)
    {
        cerr << "not a double" << endl;
        cameraMatrix.at<double>(0,0) = fxd;
    }
    double fyd = fys.toDouble(&ok);
    if(ok == false)
    {
        cerr << "not a double" << endl;
        cameraMatrix.at<double>(1,1) = fyd;
    }
    double cxd = cxs.toDouble(&ok);
    if(ok == false)
    {
        cerr << "not a double" << endl;
        cameraMatrix.at<double>(0,2) = cxd;
    }
    double cyd = cys.toDouble(&ok);
    if(ok == false)
    {
        cerr << "not a double" << endl;
        cameraMatrix.at<double>(1,2) = cyd;
    }
    calcIntrinsicParams();
}

void VideoTabController::toggleDrawable(bool drawable)
{
    picLabel->clearMarked();
    flowLabel->clearMarked();
    picLabel->setDrawable(drawable);
    flowLabel->setDrawable(drawable);
}

VideoTabController::~VideoTabController()
{
    frames.clear();
    delete capture;

    delete flowEngine;
    delete poseEstimator;
    delete videoProcessor;

    delete picLabel;
    delete flowLabel;

    delete face_ptr;
}


