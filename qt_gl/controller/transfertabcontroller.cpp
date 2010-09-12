#include "transfertabcontroller.h"
#include "utility.h"
#include <QBitmap>

TransferTabController::TransferTabController(ClickableQLabel *sourceLabel, ClickableQLabel *targetLabel, ExpTranAbstractView *view,
                                             QLineEdit *srcText, QLineEdit *targetText,CustomizableFaceWidget *face_widget)
{
    this->view = view;
    this->targetLabel = targetLabel;
    this->sourceLabel = sourceLabel;
    this->srcText = srcText;
    this->targetText = targetText;
    this->face_widget = face_widget;
    this->face_widget->setCameraParameters(-200,-1,-200);
    src_face_ptr = new Face();
    target_face_ptr = new Face();
    face_widget->setFace(src_face_ptr);

    opttype = VideoProcessor::OptType_INTERPOLATE;
    regParam = 2000.0;
    frame_num = 10;
    iter_num = 3;

    srcFileSelected("/home/martin/project/TrackedSmiles/S003-024.avi");
    targetFileSelected("/home/martin/project/TrackedSmiles/S008-005.avi");

    srcDia = new QFileDialog();
    targetDia = new QFileDialog();

    connect(srcDia,SIGNAL(fileSelected(QString)),this,SLOT(srcFileSelected(QString)));
    connect(targetDia,SIGNAL(fileSelected(QString)),this,SLOT(targetFileSelected(QString)));

    initSrcSide();
    initTargetSide();

    lensDist = Mat_<double>(1,5);
    lensDist(0,0) = 0;
    lensDist(0,1) = 0;
    lensDist(0,2) = 0;
    lensDist(0,3) = 0;
    lensDist(0,4) = 0;

    //set the texture functions
    textureInterpolate = false;
}

void TransferTabController::initSrcSide()
{
    Mat m;
    double c_x, c_y;

    Utility::grabThumbnailForVideo(srcFile.toStdString(),m);
    srcFrames.push_back(m.clone());
    sourceLabel->setPixmap(Utility::mat2QPixmap(m));
    c_x = m.size().width / 2.;
    c_y = m.size().height / 2.;

    sourceLabel->show();
    capSrc = new VideoCapture(srcFile.toStdString());

    cameraSrc = Mat_<double>(3,3);
    cameraSrc(0,0) = 900;
    cameraSrc(0,1) = 0;
    cameraSrc(0,2) = c_x;
    cameraSrc(1,0) = 0;
    cameraSrc(1,1) = 900;
    cameraSrc(1,2) = c_y;
    cameraSrc(2,0) = 0;
    cameraSrc(2,1) = 0;
    cameraSrc(2,2) = 1;

    srcFinished = false;
}

void TransferTabController::initTargetSide()
{
    Mat m;
    double c_x, c_y;
    Utility::grabThumbnailForVideo(targetFile.toStdString(),m);
    targetFrames.push_back(m.clone());
    targetLabel->setPixmap(Utility::mat2QPixmap(m));
    c_x = m.size().width / 2.;
    c_y = m.size().height / 2.;

    targetLabel->show();

    capTarget = new VideoCapture(targetFile.toStdString());

    cameraTarget = Mat_<double>(3,3);
    cameraTarget(0,0) = 900;
    cameraTarget(0,1) = 0;
    cameraTarget(0,2) = c_x;
    cameraTarget(1,0) = 0;
    cameraTarget(1,1) = 900;
    cameraTarget(1,2) = c_y;
    cameraTarget(2,0) = 0;
    cameraTarget(2,1) = 0;
    cameraTarget(2,2) = 1;

    targetFinished = false;
}

/*************************************************/
/* PUBLIC SLOTS */
/*************************************************/
void TransferTabController::dropSrc()
{
//    Mat img_src = *(srcFrames.end()-1);
//    Mat img_dest = *(targetFrames.end()-1);
//
//    Mat_<double> mask = Mat_<double>::zeros(img_src.rows,img_src.cols);
//    for(int i=385;i<420;i++)
//        for(int j=157;j<185;j++)
//            mask(i,j) = 1.0;
//
//    Point2f p1 = sourceLabel->getMarked()[0];
//    Point2f p2 = targetLabel->getMarked()[0];
//
//    cout << "hey " << p1.x << " " << p1.y << " " << p2.x << " " << p2.y << endl;
//    int o_x = p1.x - p2.x;
//    int o_y = p1.y - p2.y;
//
//    Utility::poissonClone(img_src,mask,img_dest,o_x,o_y);
//
//    targetLabel->setPixmap(Utility::mat2QPixmap(img_dest));

    Mat frame, rgb_frame;
    if( capSrc->grab() == true)
        capSrc->retrieve(frame);
    else
        return;

    cvtColor(frame, rgb_frame, CV_BGR2RGB);

    srcFrames.push_back(rgb_frame);
    sourceLabel->setPixmap(Utility::mat2QPixmap(rgb_frame));
}
void TransferTabController::dropTarget()
{
    Mat frame, rgb_frame;
    if( capTarget->grab() == true)
        capTarget->retrieve(frame);
    else
        return;

    cvtColor(frame, rgb_frame, CV_BGR2RGB);

    targetFrames.push_back(rgb_frame);
    targetLabel->setPixmap(Utility::mat2QPixmap(rgb_frame));
}
void TransferTabController::beginTransfer()
{
    Mat frame, rgb_frame, copyFrame;

//    targetLabel->setVisible(false);
//    face_widget->setVisible(true);

    /**********************/
    /*first collect frames*/
    /**********************/    
    s_frameData.push_back(srcFrames[srcFrames.size()-1]);
    sourceLabel->setPixmap(Utility::mat2QPixmap(s_frameData[0]));
    while( capSrc->grab() == true)
    {
        capSrc->retrieve(frame);
        cvtColor(frame, rgb_frame, CV_BGR2RGB);
        copyFrame = rgb_frame.clone();
        s_frameData.push_back(copyFrame);
    }

    src_videoProcessor = new VideoProcessor(sourceLabel->getMarked(),s_frameData, cameraSrc, lensDist,opttype,regParam,frame_num,iter_num);
    connect(src_videoProcessor,SIGNAL(finished()),this,SLOT(processingFinished()));
    src_videoProcessor->start();


    /**** now repeat the process for the other face ***/    
    t_frameData.push_back(targetFrames[targetFrames.size()-1]);
    targetLabel->setPixmap(Utility::mat2QPixmap(t_frameData[0]));
    while( capTarget->grab() == true)
    {
        capTarget->retrieve(frame);
        cvtColor(frame, rgb_frame, CV_BGR2RGB);
        copyFrame = rgb_frame.clone();
        t_frameData.push_back(copyFrame);
    }

    target_videoProcessor = new VideoProcessor(targetLabel->getMarked(),t_frameData, cameraSrc, lensDist,opttype,regParam,frame_num,iter_num);
    connect(target_videoProcessor,SIGNAL(finished()),this,SLOT(processingFinished()));
    target_videoProcessor->start();

}


void TransferTabController::getClonedMouth(const Mat& img, unsigned int frame_index, Mat &target)
{
   vector<Point2f> genPointsSrc;
   vector<Point2f> genPointsTarget;
   src_videoProcessor->getGeneratedPointsForFrame(frame_index,genPointsSrc);
   target_videoProcessor->getGeneratedPointsForFrame(frame_index,genPointsTarget);

   Point2f ta = genPointsSrc[3];

   Point2f pa = genPointsTarget[3];
   Point2f pb = genPointsTarget[5];
   Point2f pc = genPointsTarget[4];
   Point2f pd = genPointsTarget[6];
   cout << "points are " << pa.x << " " << pa.y << " " << pb.x << " " << pb.y << endl;

   int hx = (pc.x >= pa.x)?pc.x:pa.x;
   int lx = (pc.x >= pa.x)?pa.x:pc.x;
   int hy = (pd.y >= pb.y)?pd.y:pb.y;
   int ly = (pd.y >= pb.y)?pb.y:pd.y;
   int nx = hx - lx;
   int ny = hy - ly;
   cout << "nums are " << hx << " " << ny << endl;
   Mat_<double> mask = Mat_<double>::zeros(img.size());

   for(int i=ly;i<=hy;i++)
       for(int j=lx;j<=hx;j++)
           mask(i,j) = 1.0;

   cout << " the shift is " << ta.x - pa.x << ta.y - pa.y << endl;
   Utility::poissonClone(img,mask,target,ta.x - pa.x,ta.y - pa.y);
}

Mat TransferTabController::getMaskForLeftEyebrow()
{
   Mat result;

}


Mat TransferTabController::getMaskForRightEyebrow()
{
   Mat result;

}

void TransferTabController::processingFinished()
{
    //we dont actually distinguish which one is finished atm
    //we could use 2 functions to do that

    mutex.lock();
    if(!srcFinished && !targetFinished)
    {
        srcFinished = true;
        mutex.unlock();
        return;
    }
    else if(!srcFinished)
    {
        srcFinished = true;
    }
    else if(!targetFinished)
    {
        targetFinished = true;
    }
    mutex.unlock();
    timerReplay = new QTimer(this);
    connect(timerReplay,SIGNAL(timeout()),this,SLOT(replayFrame()));
    timerReplay->start(1000);
}


void TransferTabController::replayFrame()
{
    static unsigned int i;
    double euler_x, euler_y, euler_z;
    Mat_<double> rmatrix;
    Mat_<double> rot;
    Mat_<double> tran;
    vector<Point2f> points;

    Point2 *textureData;
    vector<Point2f> imagePoints;
    vector<Point3f> objectPoints;
    int img_width;
    int img_height;

    src_videoProcessor->getFaceAndPoseForFrame(i,src_face_ptr,rot,tran);
    target_videoProcessor->getFaceForFrame(i,target_face_ptr);

    double tx = tran(0,0);
    double ty = tran(0,1);
    double tz = tran(0,2);
    //compute and set the pose parameters
    Rodrigues(rot,rmatrix);
    Utility::computeEulerAnglesFromRmatrix(rmatrix,euler_x,euler_y,euler_z);
    //only y needs to be negative so that it agrees with the transposes   

    //set the parameters in face widget
    face_widget->setTransParams(euler_x,-euler_y,euler_z,tx,ty,tz);


    //transfer texture
    for(int j=0;j<target_face_ptr->getPointNum();j++)
        objectPoints.push_back(src_face_ptr->vertexes[j]);

    cv::projectPoints(Mat(objectPoints),rot,tran,cameraSrc,lensDist,imagePoints);
    textureData = new Point2[imagePoints.size()];
    //    img_width = srcFrames[i].size().width;
    //    img_height = srcFrames[i].size().height;
    img_width = Utility::closestLargetPowerOf2(s_frameData[i].size().width);
    img_height = Utility::closestLargetPowerOf2(s_frameData[i].size().height);

    for(unsigned int j=0;j<imagePoints.size();j++)
    {
        textureData[j].x = imagePoints[j].x / img_width;
        //our texture coordinate sysem is 0,0 top left corner ..
        //bingTexture doesnt do this
        textureData[j].y = imagePoints[j].y / img_height;
    }

    //create the 4x4 proj matrix for the face widget
    Mat_<double> projM = Mat_<double>::zeros(Size(4,4));
    for(int j=0;j<3;j++)
        for(int k=0;k<3;k++)
            projM(j,k) = cameraSrc.at<double>(j,k);


    face_widget->setProjectionMatrix(projM);

    //creat the 4x4 (homogen) trans matrix for face widget
    Mat_<double> tranM = Mat_<double>::zeros(Size(4,4));
    for(int j=0;j<3;j++)
        for(int k=0;k<3;k++)
            tranM(j,k) = rmatrix(j,k);
    tranM(0,3) = tx;
    tranM(1,3) = ty;
    tranM(2,3) = tz;
    tranM(3,3) = 1.0;

    //face_widget->setTransformationMatrix(tranM);


    //face_widget->bindTexture(Utility::mat2QImage(frameData[i]));
    srcFrames.push_back(s_frameData[i]);
    convertFrameIntoTexture(s_frameData[i]);

    src_face_ptr->transferExpressionFromFace(target_face_ptr);
    face_widget->setFace(src_face_ptr,textureData);
    delete[] textureData;


    if(textureInterpolate)
    {
        QImage qimg = face_widget->grabFrameBuffer(false);
        QPixmap pimg = QPixmap::fromImage(qimg);
        pimg.setMask(pimg.createMaskFromColor(Qt::white,Qt::MaskInColor));
        QPixmap result = Utility::composePixmaps(Utility::mat2QPixmap(s_frameData[i]),pimg);
        sourceLabel->setPixmap(result);
    }

    sourceLabel->clearMarked();
    targetLabel->clearMarked();

    getClonedMouth(s_frameData[i],i,t_frameData[i]);

    sourceLabel->setPixmap(Utility::mat2QPixmap(s_frameData[i]));
    targetLabel->setPixmap(Utility::mat2QPixmap(t_frameData[i]));

    i++;
    if(i == src_videoProcessor->getFrameNum())
    {
        timerReplay->stop();
        i = 0;
        s_frameData.clear();
        featurePoints.clear();
        cout << "DONE!!!!!!!!!" << endl;
    }

}
void TransferTabController::convertFrameIntoTexture(Mat &imgO)
{
    int img_width = Utility::closestLargetPowerOf2(imgO.size().width);
    int img_height = Utility::closestLargetPowerOf2(imgO.size().height);    
    Mat img = Mat::ones(img_height,img_width,CV_8UC3);

    int height,width,step,channels;
    uchar * data, *img_data;

    int count = 0;


    for(int i=0;i<imgO.rows;i++)
    {
        for(int j=0;j<imgO.cols;j++)
        {
            for(int k=0;k<imgO.channels();k++)
            {
                img.data[i*img.step+j*imgO.channels()+k] = imgO.data[i*imgO.step+j*imgO.channels()+k];
            }
        }
    }

    // get the image data
    height = img.rows;
    width = img.cols;
    step = img.step;
    channels = img.channels();
    data = img.data;    
    img_data = new uchar[width*height*3];

    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            for(int k=0;k<channels;k++)
            {
                img_data[count] = data[i*step+j*channels+k];
                count++;
            }
        }
    }
    face_widget->setTexture(img_data,img_height,img_width);
}


void TransferTabController::targetFileSelected(const QString str)
{
    targetFile = str;
    targetText->setText(str);
    initTargetSide();
}

void TransferTabController::srcFileSelected(const QString str)
{
    srcFile = str;
    srcText->setText(str);
    initSrcSide();
}

void TransferTabController::targetBrowse()
{
    targetDia->show();
}

void TransferTabController::srcBrowse()
{
    srcDia->show();
}

void TransferTabController::restart()
{
    srcFrames.clear();
    targetFrames.clear();
    s_frameData.clear();
    t_frameData.clear();
    featurePoints.clear();
    srcFileSelected(srcFile);
    targetFileSelected(targetFile);
}

void TransferTabController::setOptNelder(bool toggled)
{
    if(toggled == true)
        opttype = VideoProcessor::OptType_NELDER_INT;
}
void TransferTabController::setOptReg(double regParam)
{    
    this->regParam = regParam;
}
void TransferTabController::setOptType(int t)
{    
    if(t == 0)
        opttype = VideoProcessor::OptType_LIN_COMB;
    else
        opttype = VideoProcessor::OptType_INTERPOLATE;
}
void TransferTabController::setFrameNum(int n)
{
    frame_num = n;
}
void TransferTabController::setIterNum(int n)
{
    iter_num = n;
}
