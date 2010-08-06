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
#include <cmath>
#include <ctime>

#include "errorfunction.h"
#include "modelimageerror.h"
#include "rosenerror.h"
#include "mysimplex.h"

using namespace cv;
using namespace std;


//const int TransferWidget::fPoints[5] = {9521,646,8144,4979,4525};
//const int TransferWidget::fPoints_size = 5;
const int TransferWidget::fPoints[20] = {9521,8899,310,7240,1183,8934,8945,6284,7140,8197,
                                         2080,2851,3580,6058,8825,1680,3907,8144,6540,2519};
const int TransferWidget::fPoints_size = 20;


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

TransferWidget::TransferWidget(QString fileName, FaceWidget *face_widget) : fileName(fileName)
{
    picLabel = new FeaturePointQLabel();
    flowLabel = new VectorFieldQLabel();
    this->face_widget = face_widget;

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

    cout << " CALIBRATE SIZE" << endl;
    cout << img.size().height << " " << img.size().width << endl;


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

    vector<Mat>::iterator it = frameData.begin(),
                          it_last = frameData.begin(),
                          it_end = frameData.end();

    for(;it != it_end; ++it)
    {
        flowEngine->computeFlow(*it_last,*it,currentPoints,nextPoints);
        it_last = it;

        featurePoints.push_back(nextPoints);
        currentPoints.clear();
        //assigns a copy of next points as the new content for currentPoints
        currentPoints = nextPoints;
        nextPoints.clear();
    }

    /**********************/
    /*estimate model parameters + pose*/
    /**********************/    
    //make a face guess
    double *w_id = new double[56];
    double *w_exp = new double[7];
    Face *face_ptr = new Face();

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

    Mat rvec, tvec, rvec_clone, tvec_clone;

    vector<vector<Point2f> >::iterator it2 = featurePoints.begin(),
                                       it2_end = featurePoints.end();

    //do not use the guess in the first frame but do for every following frame
    bool useExt = false;
    for(; it2 != it2_end; ++it2)
    {
        //determine pose based on the preselected feature points (fPoints)
        //the function also uses the current values in rvec and tvec as a guess
        //for the extrinsic parameters (if it isnt the first frame
        calculateTransformation( *it2, face_ptr, rvec, tvec, useExt);
        rvec_clone = rvec.clone();
        tvec_clone = tvec.clone();
        frameRotation.push_back(rvec_clone);
        frameTranslation.push_back(tvec_clone);

        //useExt or true = useExt
        useExt |= true;
    }

    //seed our pseudorandom number generator to give different numbers each time
    srand( (unsigned int)time(0) );
    int random;
    double low = 0;
    double high = face_ptr->getPolyNum();
    vector<Point2f> imagePoints;

    vector<int> indices;
    vector<vector<int> >point_indices_for_frame;
    vector<int> point_indices;
    Point3f p;    
    Mat_<double> point3dMat(3,1);
    Mat_<double> point2dMat(3,1);


    double l[1][5] = {{-0.0865513, 0.197476, 0.00430749, 0.0072667, -0.114125}};
    Mat lens(1,5,CV_64F,l);

    Mat_<double> rmatrix;
    Mat_<double> transpose;

    vector<Point3f> objectPoints;

    double m[2][3] = {{283.536, 0, 337.789},{0, 286.4, 299.076}};
    Mat camera(2,3,CV_64F,m);

//
//    double m[3][3] = {{1, 0, 337.789},{0, 1, 299.076}};
//    Mat_<double> camera = Mat(2,3,CV_64F,m);


    //using the transformations generate a 1000 new points on the 2d image
    for(unsigned int j=0; j<frameTranslation.size(); j++)
    {
        cout << "frame : " << j << endl;
        imagePoints.clear();
        objectPoints.clear();
        point_indices.clear();


        for(int i=0;i<fPoints_size;i++)
        {
            //now generate random numbers from range based on unifrom sampling dist formula
            //we look at it as a parametrization of the interval a,b
            //with a param based on rand() from interval 0 to 1
            //with the +1 coz it will never go to 1 .. coz it will never be rand_max
            //U = a + (b-a+1)*rand()/(1+RAND_MAX);
            random = low + (high - low + 1.0)*rand()/(1.0 + RAND_MAX);
            cout << "is random: " << random << endl;
            indices.push_back(random);
            p = face_ptr->getPointFromPolygon(fPoints[i]);
            point_indices.push_back(face_ptr->getPointIndexFromPolygon(fPoints[i]));

            point3dMat(0,0) = p.x;
            point3dMat(1,0) = p.y;
            point3dMat(2,0) = p.z+1500.0;            

            Rodrigues(frameRotation[j],rmatrix);
            transpose = frameTranslation[j];

            cout << rmatrix.size().height << " " << rmatrix.size().width << endl;
            cout << transpose.size().height << " " << transpose.size().width << endl;


            point2dMat = camera*((rmatrix * point3dMat) + transpose);

////            //homogenous coord
//            point2dMat(0,0) /= point2dMat(2,0);
//            point2dMat(1,0) /= point2dMat(2,0);

            //objectPoints.push_back(Point3f(p.x,p.y,p.z+1500.0));

            imagePoints.push_back(Point2f(point2dMat(0,0) , point2dMat(1,0)));
        }
        //projectPoints(Mat(objectPoints),frameRotation[j],frameTranslation[j],camera,lens,imagePoints);

        point_indices_for_frame.push_back(point_indices);
        generatedPoints.push_back(imagePoints);
    }

    /***********************/
    /* now use the newly */
    /* projected points to */
    /* calculate how to change shape */
    /***********************/

    cout << "TESTING MYSIMPLEX (nelder mead impl) with ROSEN function" << endl;
    RosenError rosen;

    double start_array[] = {-1.2,1.0}; //,2.3,-1.3};
    vector<double> start;
    double min;
    int i;
    int size_t = sizeof(start_array) / sizeof(double);

    start.assign(start_array, start_array + size_t);
    min = rosen(start);
    cout << "min before simplex : " << min << endl;

    min=mysimplex(rosen,start,start.size(),1);
    //min=mysimplex(rosen,start,start.size(),0.55);
    cout <<  "min after simplex : " << min << endl;


    for (i=0;i<2;i++) {
        printf("%f\n",start[i]);
    }
    min = rosen(start);
    cout << min << endl;

    ModelImageError error;
    vector<double> weights_id;
    vector<double> weights_ex;
    weights_id.assign(w_id,w_id+56);
    weights_ex.assign(w_exp,w_exp+7);
//sizeof(w_exp)/sizeof(double)

    //TODO smaller coz its too slow
//    for(unsigned int i=0; i<1; i++)
//    {
//        Rodrigues(frameRotation[i],rmatrix);
//        error = ModelImageError(camera,rmatrix,frameTranslation[i]);
//        error.setWeights(weights_id);
//        error.setPoints(featurePoints[i]);
//        error.setPointIndices(point_indices_for_frame[i]);
//
//        min=mysimplex(error,weights_ex,weights_ex.size(),1);
//
//    }

    timerReplay = new QTimer(this);
    connect(timerReplay,SIGNAL(timeout()),this,SLOT(replayFrame()));
    timerReplay->start(400);

    delete face_ptr;
    delete[] w_id;
    delete[] w_exp;
}

void TransferWidget::calculateTransformation(vector<Point2f> imagePoints, Face *face_ptr, Mat &rvec, Mat &tvec, bool useExt)
{
    vector<Point3f> objectPoints;

    Point3f p3;
    for(int i=0;i<this->fPoints_size;i++)
    {
        p3 = face_ptr->getPointFromPolygon(fPoints[i]);
        p3.z += 1500.0;
        objectPoints.push_back(p3);
    }    

    //183.536 0 337.789 0 186.4 299.076 0 0 1
    //initialize intrinsic parameters of the camera
    double m[3][3] = {{283.536, 0, 337.789},{0, 286.4, 299.076},{0, 0, 1}};
    Mat cameraMatrix(3,3,CV_64F,m);

    //-0.0865513 0.197476 0.00430749 0.0072667 -0.114125
    double l[1][5] = {{-0.0865513, 0.197476, 0.00430749, 0.0072667, -0.114125}};
    Mat lensDist(1,5,CV_64F,l);

    //setup the result vectors for the extrinsic parameters


    cout << "before solve pnp" << endl;
    //transform vectors into Mats with 3 (2) channels
    cv::solvePnP(Mat(objectPoints),Mat(imagePoints),cameraMatrix,lensDist,rvec,tvec,useExt);

   // Mat_<double> rmatrix;
    //convert the rotation vector to a rotation matrix
   // cv::Rodrigues(rvec,rmatrix);

//    double r11 = rmatrix.at<double>(0,0), r12 = rmatrix.at<double>(0,1),r13 = rmatrix.at<double>(0,2);
//    double r21 = rmatrix.at<double>(1,0), r22 = rmatrix.at<double>(1,1),r23 = rmatrix.at<double>(1,2);
//    double r31 = rmatrix.at<double>(2,0), r32 = rmatrix.at<double>(2,1),r33 = rmatrix.at<double>(2,2);
//    double tx = tvec.at<double>(0,0);
//    double ty = tvec.at<double>(0,1);
//    double tz = tvec.at<double>(0,2);
//
//    double r[3][4] = {{r11, r12, r13, tx},{r21,r22,r23,ty},{r31,r32,r33,tz}};
//    Mat R(3,4,CV_64F,r);
//    Mat result = cameraMatrix * R;
//
//    MatConstIterator_<double> it,it_end;
//    cout << "camera matrix" << endl;
//    it = cameraMatrix.begin<double>();
//    it_end = cameraMatrix.end<double>();
//    for(; it != it_end; ++it)
//        cout << *it << " ";
//    cout << endl;
//    cout << "rot matrix" << endl;
//    it = rvec.begin();
//    it_end = rvec.end();
//    for(; it != it_end; ++it)
//        cout << *it << " ";
//    cout << endl;
//    cout << "trans mult" << endl;
//    it = tvec.begin();
//    it_end = tvec.end();
//    for(; it != it_end; ++it)
//        cout << *it << " ";
//    cout << endl;
//
//    return result;

}
/**********************************************/
/* PUBLIC SLOTS */
/**********************************************/
void TransferWidget::replayFrame()
{
    static unsigned int i;
    QImage img_q = mat2QImage(frameData[i]);
    QPixmap p_map;
    p_map = QPixmap::fromImage(img_q);
    picLabel->setPixmap(p_map);

    picLabel->setMarked(generatedPoints[i]);

    i++;
    if(i == frameData.size())
    {
        timerReplay->stop();
        i = 0;
        frameData.clear();
        featurePoints.clear();
    }
}

void TransferWidget::playBack()
{
    processVideo();
}

//later do a pyramid version
void TransferWidget::startFaceTransfer()
{
    //first align our face guess over the marked feature points
    vector<Point2f> marked = picLabel->getMarked();
    //marked points should get recentered but we are gonna get rid of shifts anyway
    //marked points are ordered (should be ordered if the user clicked correctly)
    //the order is the same as the index order in fPoints

    //calcIntrinsicParams();


    //face guess
    double *w_id = new double[56];
    double *w_exp = new double[7];
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
        cout << p3.x << " " << p3.y << " " << p3.z << endl;
        objectPoints.push_back(Point3f(p3.x,p3.y,p3.z+1500.0));
    }
    imagePoints = picLabel->getMarked();

    //183.536 0 337.789 0 186.4 299.076 0 0 1
    //initialize intrinsic parameters of the camera
    double m[3][3] = {{283.536, 0, 337.789},{0, 286.4, 299.076},{0, 0, 1}};
    Mat cameraMatrix(3,3,CV_64F,m);

    //-0.0865513 0.197476 0.00430749 0.0072667 -0.114125
    double l[1][5] = {{-0.0865513, 0.197476, 0.00430749, 0.0072667, -0.114125}};
    Mat lensDist(1,5,CV_64F,l);

    //setup the result vectors for the extrinsic parameters
    Mat_<double> rvec, tvec;

    cout << "before solve pnp" << endl;
    //transform vectors into Mats with 3 (2) channels
    cv::solvePnP(Mat(objectPoints),Mat(imagePoints),cameraMatrix,lensDist,rvec,tvec);

    Mat_<double> rmatrix;
    //convert the rotation vector to a rotation matrix
    cv::Rodrigues(rvec,rmatrix);
    //after that compute the euler angles from the rotation matrix (from wiki)
    //we extract based on the analytical formula of the
//    double rot_y = ::asin(-rmatrix.at<double>(2,0));
//    double cos_y = ::cos(rot_y);
//    double rot_x = ::acos(rmatrix.at<double>(2,2) / cos_y);
//    double rot_z = ::acos(rmatrix.at<double>(0,0) / cos_y);

    double rot_y, rot_x, rot_z, C, trx, tr_y;
    rot_y = asin( -rmatrix.at<double>(2,0));        /* Calculate Y-axis angle */
    C           =  cos( rot_y );

    if ( fabs( C ) > 0.005 )             /* Gimball lock? */
      {
      trx      =  rmatrix.at<double>(2,2) / C;           /* No, so get X-axis angle */
      tr_y      = rmatrix.at<double>(2,1)  / C;

      rot_x  = atan2( tr_y, trx );

      trx      =  rmatrix.at<double>(0,0) / C;            /* Get Z-axis angle */
      tr_y      = rmatrix.at<double>(1,0) / C;

      rot_z  = atan2( tr_y, trx );
  }

    const double PI = 3.141593;


    double rx[3][3] = {{1,0,0},{0,::cos(rot_x),-::sin(rot_x)},{0,::sin(rot_x),::cos(rot_x)}};
    double ry[3][3] = {{::cos(rot_y),0,-::sin(rot_y)},{0,1,0},{::sin(rot_y),0,::cos(rot_y)}};
    double rz[3][3] = {{::cos(rot_z),-::sin(rot_z),0},{::sin(rot_z),::cos(rot_z),0},{0,0,1}};

    Mat rX(3,3,CV_64F,rx), rY(3,3,CV_64F,ry), rZ(3,3,CV_64F,rz);
    Mat result = rZ*rX*rY;

    cout << "rvec" << endl;
    cout << rot_x*(180.0/PI) << " " << rot_y*(180.0/PI) << " " << rot_z*(180.0/PI) << " " << endl;
    cout << rmatrix.at<double>(2,0) << endl;

    //since we parametrised Mat_<double> we dont need to call it agaon for begin
    MatConstIterator_<double> it = rmatrix.begin(), it_end = rmatrix.end();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;
    it = result.begin<double>();
    it_end = result.end<double>();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;

    double n = 0;
    it = rvec.begin();
    it_end = rvec.end();
    for(; it != it_end; ++it)
    {
        cout << *it << " ";
        n += (*it)*(*it);
    }
    cout << endl;
    cout << "rvec norm" << endl;
    cout << ::sqrt(n) << endl;

    cout << "tvec" << endl;
    it = tvec.begin();
    it_end = tvec.end();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;

    vector<Point2f> output;
    projectPoints(Mat(objectPoints),rvec,tvec,cameraMatrix,lensDist,output);

    Mat frame = frames[frames.size()-1];

    cout << "FOCAL LENGTH : " <<  frame.size().height << " "
         <<  frame.size().width << endl;

//
//    cout << "FOCAL LENGTH : " <<  cameraMatrix.at<double>(0,0)*frame.size().height << " "
//         <<  cameraMatrix.at<double>(1,1)*frame.size().width << endl;

//    cout << "FOCAL LENGTH : " <<  cameraMatrix.at<double>(0,0) << " "
//         <<  cameraMatrix.at<double>(1,1) << endl;

    cout << "reprojection" << endl;
    vector<Point2f>::iterator iter = output.begin(), iter_end = output.end();
    for(;iter!=iter_end;++iter)
        cout << iter->x << " " << iter->y << endl;
    cout << "original" << endl;
    iter = imagePoints.begin();
    iter_end = imagePoints.end();
    for(;iter!=iter_end;++iter)
        cout << iter->x << " " << iter->y << endl;
    //picLabel->setMarked(output);

    double r11 = rmatrix.at<double>(0,0), r12 = rmatrix.at<double>(0,1),r13 = rmatrix.at<double>(0,2);
    double r21 = rmatrix.at<double>(1,0), r22 = rmatrix.at<double>(1,1),r23 = rmatrix.at<double>(1,2);
    double r31 = rmatrix.at<double>(2,0), r32 = rmatrix.at<double>(2,1),r33 = rmatrix.at<double>(2,2);
    double tx = tvec.at<double>(0,0);
    double ty = tvec.at<double>(0,1);
    double tz = tvec.at<double>(0,2);    

    double r[3][4] = {{r11, r12, r13, tx},{r21,r22,r23,ty},{r31,r32,r33,tz}};
    Mat R(3,4,CV_64F,r);
    Mat res = cameraMatrix * R;
    cout << "matrix mult" << endl;
    it = res.begin<double>();
    it_end = res.end<double>();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;

    //decompose
    Vec3d euler;
    Mat cam,rot,trans,rotX,rotY,rotZ;
    decomposeProjectionMatrix(res,cam,rot,trans,rotX,rotY,rotZ,euler);
    cout << "camera matrix" << endl;
    it = cam.begin<double>();
    it_end = cam.end<double>();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;
    cout << "rot matrix" << endl;
    it = rot.begin<double>();
    it_end = rot.end<double>();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;
    cout << "trans mult" << endl;
    it = trans.begin<double>();
    it_end = trans.end<double>();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;
    cout << "euler" << endl;
    Vec3d v;

    for(int i=0; i<3; ++i)
        cout << euler[i] << " ";
    cout << endl;

    Point3f p;
    Mat_<double> point3dMat(3,1);
    Mat_<double> point2dMat(3,1);
    vector<Point2f> points;

    double mat[3][3] = {{283.536, 0, 337.789},{0, 286.4, 299.076},{0,0,1}};
    cameraMatrix = Mat(3,3,CV_64F,mat);
    Mat_<double> translation = tvec;

    cout << "TRANS : " << tvec.at<double>(0,1) << " " << tvec.at<double>(1,0) << endl;

    double scale = 2;

    for(int i=0; i<fPoints_size; i++)
    {
        p = face_ptr->getPointFromPolygon(fPoints[i]);
        point3dMat(0,0) = p.x;
        point3dMat(1,0) = p.y;
        point3dMat(2,0) = p.z+1500.0;

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

    face_widget->setTransParams(euler[0],euler[1],euler[2],tx,ty,tz);

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

    generatedPoints.clear();\
    featurePoints.clear();
    frameData.clear();
    frameRotation.clear();
    frameTranslation.clear();

    capture = new VideoCapture(fileName.toStdString());

    picLabel->clearMarked();
    flowLabel->clearMarked();

    Mat m;

    grabThumbnailForVideo(fileName.toStdString(),m);
    frames.push_back(m);

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

void TransferWidget::pauseTransfer()
{
    timer->stop();
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


