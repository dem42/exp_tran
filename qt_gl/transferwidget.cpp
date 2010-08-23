#include "transferwidget.h"
#include "featurepointqlabel.h"
#include "opticalflowfarneback.h"

#include <iostream>
#include <QImage>
#include <QTimer>
#include <cv.h>
#include <highgui.h>
#include <cstdlib>
#include <cmath>
#include <ctime>

#include "errorfunction.h"
#include "modelimageerror.h"
#include "rosenerror.h"
#include "mysimplex.h"

#include "neldermeadoptimizer.h"
#include "closedformoptimizer.h"
#include "nnlsoptimizer.h"

using namespace cv;
using namespace std;


//const int TransferWidget::fPoints[5] = {9521,646,8144,4979,4525};
//const int TransferWidget::fPoints_size = 5;
const int TransferWidget::fPoints[20] = {9521,8899,310,7240,1183,8934,8945,6284,7140,8197,
                                         2080,2851,3580,6058,8825,1680,3907,8144,6540,2519};
const int TransferWidget::fPoints_size = 20;


QImage TransferWidget::mat2QImage(const Mat mat_rgb)
{
    QImage qframe;
    //converting to rgb not necessary it seems
    //cvtColor(mat_bgr, mat_rgb, CV_BGR2RGB);

    qframe = QImage((const unsigned char*)(mat_rgb.data), mat_rgb.cols,
    mat_rgb.rows, QImage::Format_RGB888);
    //rgb QImage::Format_RGB888
    return qframe;
}

TransferWidget::TransferWidget(QString fileName, FaceWidget *face_widget) : fileName(fileName), FRAME_MAX(2)
{
    picLabel = new FeaturePointQLabel();
    flowLabel = new VectorFieldQLabel();
    this->face_widget = face_widget;
    this->face_widget->setCameraParameters(-200,-1,-200);
    face_ptr  = new Face();
    face_widget->setFace(face_ptr);

    cameraDialog = new QDialog();
    cameraUi.setupUi(cameraDialog);

    //initialize the optical flow engine
    flowEngine = new OpticalFlowEngine();
    //init the optimizer
    paramOptimizer = new NelderMeadOptimizer();

    //setup the timer
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(captureFrame()));


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
    cameraMatrix(0,2) = 337.789;//303.703;
    cameraMatrix(1,0) = 0;
    cameraMatrix(1,1) = 900;//594.657;
    cameraMatrix(1,2) = 299.076;//159.174;
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


    //setup video
    Mat m;
    grabThumbnailForVideo(fileName.toStdString(),m);

    frames.push_back(m);

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


    NNLSOptimizer *nnls = new NNLSOptimizer();
    nnls->test();

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
    vector<Mat> imgs;

    imgs.push_back(imread("../../camera1.jpg"));
    imgs.push_back(imread("../../camera2.jpg"));

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

        QImage img_q = mat2QImage(img);
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

void TransferWidget::processVideo()
{
    Mat frame, rgb_frame, copyFrame;
    //point indices
    vector<vector<int> >point_indices;
    vector<int> indices;
    vector<int> nextIndices;
    bool useExt = false;
    Face *face_ptr = new Face();
    double *w_id = new double[56];
    double *w_exp = new double[7];

    Mat_<double> rvec, tvec;
    vector<Point2f> imagePoints;
    vector<Point2f> newPoints;

    vector<double> weights_id;
    vector<double> weights_exp;

    //make a face guess
    for(int i=0;i<56;i++)
    {
        if(i==33)w_id[i] = 0.1;
        else if(i==7)w_id[i] = 0.8;
        else if(i==20)w_id[i] = 0.1;
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

    /**********************/
    /*first collect frames*/
    /**********************/
    //use the VideoCapture attribute capture of this transferWidget instance
    //to collect frame data

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

    /**********************/
    /*calculate points in all
    frames using optical flow*/
    /**********************/
    vector<Point2f> currentPoints = picLabel->getMarked();
    vector<Point2f> nextPoints;

    featurePoints.push_back(currentPoints);

    cout << "feature points bfore" << endl;
    for(int i=0;i<featurePoints[0].size();i++)
        cout << featurePoints[0][i].x << " " << featurePoints[0][i].y << endl;

    vector<Mat>::iterator it = frameData.begin(),
    it_last = frameData.begin(),
    it_end = frameData.end();

    //estimate the pose in the first frame
    paramOptimizer = new NNLSOptimizer();    

    //place feature point indices in indices
    indices.insert(indices.begin(),Face::fPoints,Face::fPoints+Face::fPoints_size);
    point_indices.push_back(indices);

    cout << "now first frame pose " << endl;
    //calculate the rot,trans of the intial frame
    paramOptimizer->calculateTransformation(featurePoints[0],face_ptr,cameraMatrix,lensDist,indices,rvec,tvec,useExt);
    frameRotation.push_back(rvec.clone());
    frameTranslation.push_back(tvec.clone());
    useExt |= true;
    
    newPoints.clear();
    cout << "now new points " << endl;
    paramOptimizer->generatePoints(frameRotation[0],frameTranslation[0],cameraMatrix,lensDist,20,face_ptr,newPoints,indices);
    //add new points so that we start tracking them
    //we arent actually altering the first featurePoints[0] just the rest throught compute flow
    cout << "cur points b4 " << currentPoints.size() << endl;
    currentPoints.insert(currentPoints.end(),newPoints.begin(),newPoints.end());
    cout << " after " << currentPoints.size() << endl;

    for(;it != it_end; ++it)
    {
        flowEngine->computeFlow(*it_last,*it,currentPoints,nextPoints,indices,nextIndices);
        it_last = it;

        featurePoints.push_back(nextPoints);
        point_indices.push_back(nextIndices);
        currentPoints.clear();
        indices.clear();
        //assigns a copy of next points as the new content for currentPoints
        currentPoints = nextPoints;
        indices = nextIndices;
        nextPoints.clear();
        nextIndices.clear();
    }

    FRAME_MAX = 3;
    
    for(unsigned int i=1;i<FRAME_MAX;++i)
    {
        /**********************/
        /*estimate pose*/
        /**********************/

        //estimate the pose parameters and place estimations into vectors rotations and translations
        //the rotations vector holds the rodrigues rotation vectors which can be converted to a rotation matrix
        paramOptimizer->calculateTransformation(featurePoints[i],face_ptr,cameraMatrix,lensDist,point_indices[i],rvec,tvec,useExt);
        frameRotation.push_back(rvec.clone());
        frameTranslation.push_back(tvec.clone());
        useExt |= true;
    }

    cout << "feature points after" << endl;
    for(int i=0;i<featurePoints[0].size();i++)
        cout << featurePoints[0][i].x << " " << featurePoints[0][i].y << endl;

    for(unsigned int i=0;i<FRAME_MAX;++i)
    {
        /**********************/
        /*estimate model parameters + pose*/
        /**********************/
        weights_exp.clear();
        weights_id.clear();
        paramOptimizer->estimateModelParameters(frameData[i],featurePoints[i],cameraMatrix,lensDist,face_ptr,
                                                point_indices[i], frameRotation[i],frameTranslation[i],
                                                weights_id,weights_exp);
        vector_weights_exp.push_back(weights_exp);
        vector_weights_id.push_back(weights_id);

        //could overload generatePoints in closedform optim to do weak perspective
        cout << "frame : " << i << endl;
        newPoints.clear();
        paramOptimizer->weakPerspectiveProjectPoints(frameRotation[i],frameTranslation[i],cameraMatrix,lensDist,point_indices[i],face_ptr,newPoints);
        generatedPoints.push_back(newPoints);
    }
    timerReplay = new QTimer(this);
    connect(timerReplay,SIGNAL(timeout()),this,SLOT(replayFrame()));
    timerReplay->start(3000);

    //face_widget->setFace(face_ptr);

    delete[] w_id;
    delete[] w_exp;
}

/**********************************************/
/* PUBLIC SLOTS */
/**********************************************/
void TransferWidget::replayFrame()
{
    static unsigned int i;
    double euler_x, euler_y, euler_z;
    Mat_<double> rmatrix;

    double tx = frameTranslation[i].at<double>(0,0);
    double ty = frameTranslation[i].at<double>(0,1);
    double tz = frameTranslation[i].at<double>(0,2);

    double *w_id = new double[56];
    double *w_exp = new double[7];


    QImage img_q = mat2QImage(frameData[i]);
    QPixmap p_map;
    p_map = QPixmap::fromImage(img_q);
    picLabel->setPixmap(p_map);

    picLabel->setMarked(generatedPoints[i]);

    //compute and set the pose parameters
    Rodrigues(frameRotation[i],rmatrix);
    computeEulerAnglesFromRmatrix(rmatrix,euler_x,euler_y,euler_z);
    //only y needs to be negative so that it agrees with the transposes
    cout << "setting trans param " << euler_x << " " << euler_y << " "
         << euler_z  << " " << tx << " " << ty << " " << tz  << endl;

    face_widget->setTransParams(euler_x,-euler_y,euler_z,tx,ty,tz);

    //generate the corresponding face    
    cout << "begin w_id and w_exp in frame : " << i << endl;
    for(int j=0;j<56;j++)
    {
        w_id[j] = vector_weights_id[i][j];
        cout << w_id[j] << " ";
    }
    cout << endl;
    for(int j=0;j<7;j++)
    {
        w_exp[j] = vector_weights_exp[i][j];
        cout << w_exp[j] << " ";
    }
    cout << endl;

    face_ptr->interpolate(w_id,w_exp);
    face_widget->setFace(face_ptr);


    i++;
    if(i == FRAME_MAX)
    {
        timerReplay->stop();
        i = 0;
        frameData.clear();
        featurePoints.clear();
    }
    delete[] w_id;
    delete[] w_exp;
}

void TransferWidget::playBack()
{
    flowLabel->setVisible(false);
    face_widget->setVisible(true);

    processVideo();
}

void TransferWidget::calibrate()
{
    cameraDialog->show();
}

void TransferWidget::computeEulerAnglesFromRmatrix(const Mat &rmatrix,double &euler_x, double &euler_y, double &euler_z)
{
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

    euler_x = rot_x * 180./PI;
    euler_y = rot_y * 180./PI;
    euler_z = rot_z * 180./PI;
}

//later do a pyramid version
void TransferWidget::startFaceTransfer()
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


    vector<int> indices(Face::fPoints,Face::fPoints+Face::fPoints_size);
    paramOptimizer->calculateTransformation(marked,face_ptr,cameraMatrix,lensDist,indices,rvec,tvec,false);

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

    double fovy, fovx, fl, aratio;
    Point2d pp;
    cv::calibrationMatrixValues(cameraMatrix,frames[frames.size()-1].size(),200,200,fovx,fovy,fl,pp,aratio);

    cout << "fovx " << fovx << " fovy " << fovy << " focal length " << fl
         << " pp.x " << pp.x << " pp.y " << pp.y << " aratio " << aratio << endl;


    double scale = 1;

    for(int i=0; i<Face::fPoints_size; i++)
    {
        p = face_ptr->getPoint(Face::fPoints[i]);
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

    picLabel->setMarked(points);

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
    face_widget->setFace(face_ptr);
    face_widget->refreshGL();
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

    frames.push_back(rgb_frame);

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


    double c_x = thumbnail.size().width / 2.;
    double c_y = thumbnail.size().height / 2.;
    //set this when you load a video
    cameraMatrix.at<double>(0,2) = c_x;
    cameraMatrix.at<double>(1,2) = c_y;

    cap.release();
}

//restart caputring by clearing marked points
//refreshing the picture to the thumbnail
//and stopping the timer and clearing old frames
void TransferWidget::restartCapturing()
{
    timer->stop();

    flowLabel->setVisible(true);
    face_widget->setVisible(false);

    capture->release();
    delete capture;
    frames.clear();

    generatedPoints.clear();\
    featurePoints.clear();
    frameData.clear();
    frameRotation.clear();
    frameTranslation.clear();

    //get rid of the old face
    delete face_ptr;
    face_ptr = new Face();

    capture = new VideoCapture(fileName.toStdString());

    picLabel->clearMarked();
    flowLabel->clearMarked();

    Mat m,um;

    grabThumbnailForVideo(fileName.toStdString(),m);

    //undistort(m,um,cameraMatrix,lensDist);
    frames.push_back(m);

    QImage img_q = mat2QImage(m);

    QPixmap p_map;
    p_map = QPixmap::fromImage(img_q);

    picLabel->setPixmap(p_map);
    flowLabel->setPixmap(p_map);
}

void TransferWidget::playTransfer()
{
    flowLabel->setVisible(true);
    face_widget->setVisible(false);

    captureFrame();
    timer->start(200);
}

void TransferWidget::pauseTransfer()
{
    timer->stop();
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

    delete flowEngine;

    delete picLabel;
    delete flowLabel;

    delete face_ptr;
}


