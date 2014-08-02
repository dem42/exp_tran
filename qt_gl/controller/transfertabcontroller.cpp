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
    this->face_widget->setLabeled(false);

    this->face_widget->setCameraParameters(-200,-1,-200);
    src_face_ptr = new Face();
    target_face_ptr = new Face();
    face_widget->setFace(src_face_ptr);

    opttype = VideoProcessor::OptType_INTERPOLATE;
    idconstype = VideoProcessor::IdConstraintType_CONST;
    projtype = VideoProcessor::PointGenerationType_2D;
    regParam = 2000.0;
    frame_num = 50;
    iter_num = 30;
    gen_point_num = 400;

    srcFileSelected("obama.avi");
    targetFileSelected("clinton.avi");

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
    show3D = false;
    textured3D = false;
    projModel = false;
    withFirstFrame = true;
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
    int counter = 0;
    if(show3D == true)
    {
        targetLabel->setVisible(false);
        face_widget->setVisible(true);
    }

    view->setAllTransferTabButtonsDisabled(true);

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
        counter++;
    }
    if(counter < frame_num) frame_num = counter;
    src_videoProcessor = new VideoProcessor(sourceLabel->getMarked(),s_frameData, cameraSrc, lensDist,opttype,
                                            regParam,idconstype,projtype,frame_num,iter_num,withFirstFrame,
                                            gen_point_num);
    connect(src_videoProcessor,SIGNAL(finished()),this,SLOT(processingFinished()));
    src_videoProcessor->start();


    /**** now repeat the process for the other face ***/    
    counter = 0;
    t_frameData.push_back(targetFrames[targetFrames.size()-1]);
    targetLabel->setPixmap(Utility::mat2QPixmap(t_frameData[0]));
    while( capTarget->grab() == true)
    {
        capTarget->retrieve(frame);
        cvtColor(frame, rgb_frame, CV_BGR2RGB);
        copyFrame = rgb_frame.clone();
        t_frameData.push_back(copyFrame);
        counter++;
    }
    if(counter < frame_num) frame_num = counter;
    target_videoProcessor = new VideoProcessor(targetLabel->getMarked(),t_frameData, cameraSrc, lensDist,opttype,
                                               regParam,idconstype,projtype,frame_num,iter_num,withFirstFrame,
                                               gen_point_num);
    connect(target_videoProcessor,SIGNAL(finished()),this,SLOT(processingFinished()));
    target_videoProcessor->start();

}


void TransferTabController::getClonedMouth(const Mat& img, unsigned int frame_index, Mat &target)
{
   vector<Point2f> genPointsSrc;
   vector<Point2f> genPointsTarget;
   src_videoProcessor->getGeneratedPointsForFrame(frame_index,genPointsSrc);
   target_videoProcessor->getGeneratedPointsForFrame(frame_index,genPointsTarget);

   int lmc = Face::leftMouthCornerIndex;
   int rmc = Face::rightMouthCornerIndex;
   int tl = Face::topLipIndex;
   int bl = Face::bottomLipIndex;

   Point2f ta = genPointsSrc[lmc];
   Point2f tb = genPointsSrc[tl];
   Point2f tc = genPointsSrc[rmc];
   Point2f td = genPointsSrc[bl];

   Point2f pa = genPointsTarget[lmc];
   Point2f pb = genPointsTarget[tl];
   Point2f pc = genPointsTarget[rmc];
   Point2f pd = genPointsTarget[bl];

   int hx = (pc.x >= pa.x)?pc.x:pa.x;
   int lx = (pc.x >= pa.x)?pa.x:pc.x;
   int hy = (pd.y >= pb.y)?pd.y:pb.y;
   int ly = (pd.y >= pb.y)?pb.y:pd.y;
   int nx = (hx + lx)/2.;
   int ny = (hy + ly)/2.;


   int hx2 = (tc.x >= ta.x)?tc.x:ta.x;
   int lx2 = (tc.x >= ta.x)?ta.x:tc.x;
   int hy2 = (td.y >= tb.y)?td.y:tb.y;
   int ly2 = (td.y >= tb.y)?tb.y:td.y;
   int nx2 = (hx2 - lx2)/2.;
   int ny2 = (hy2 - ly2)/2.;

   Mat_<double> mask_mouth = Mat_<double>::zeros(img.size());

   for(int i=ny-ny2-8;i<=(ny+ny2+8);i++)
       for(int j=nx-nx2-10;j<=(nx+nx2+10);j++)
           mask_mouth(i,j) = 1.0;


   int shift_x = ta.x - (nx-nx2);

   Utility::poissonClone(img,mask_mouth,target,shift_x,ta.y - pa.y);

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
    if(!src_videoProcessor->getCrashed() && !target_videoProcessor->getCrashed())
    {        
        timerReplay = new QTimer(this);
        connect(timerReplay,SIGNAL(timeout()),this,SLOT(replayFrame()));
        timerReplay->start(200);
    }
    else
    {
        ExpTranException e("execution of video processor failed .. see log for reason");
        view->displayException(e);
        view->setAllTransferTabButtonsDisabled(false);
        delete src_videoProcessor;
        delete target_videoProcessor;
    }
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

    src_videoProcessor->getFaceForFrame(i,src_face_ptr);
    target_videoProcessor->getFaceAndPoseForFrame(i,target_face_ptr,rot,tran);

    double tx = tran(0,0);
    double ty = tran(0,1);
    double tz = tran(0,2);
    //compute and set the pose parameters
    Rodrigues(rot,rmatrix);
    Utility::computeEulerAnglesFromRmatrix(rmatrix,euler_x,euler_y,euler_z);
    //only y needs to be negative so that it agrees with the transposes   

    if(projModel)
    {
        vector<Point2f> points;
        src_videoProcessor->getGeneratedPointsForFrame(i,points);
        sourceLabel->setMarked(points);
    }

    if(textured3D && i == 10)
    {
        //transfer texture
        for(int j=0;j<target_face_ptr->getPointNum();j++)
            objectPoints.push_back(target_face_ptr->vertexes[j]);

        cv::projectPoints(Mat(objectPoints),rot,tran,cameraSrc,lensDist,imagePoints);
        textureData = new Point2[imagePoints.size()];
        //    img_width = srcFrames[i].size().width;
        //    img_height = srcFrames[i].size().height;
        img_width = Utility::closestLargetPowerOf2(t_frameData[i].size().width);
        img_height = Utility::closestLargetPowerOf2(t_frameData[i].size().height);

        for(unsigned int j=0;j<imagePoints.size();j++)
        {
            textureData[j].x = imagePoints[j].x / img_width;
            //our texture coordinate sysem is 0,0 top left corner ..
            //bingTexture doesnt do this
            textureData[j].y = imagePoints[j].y / img_height;
        }


        cout << "in text " << i << " " << textured3D << endl;
        convertFrameIntoTexture(t_frameData[i]);
        face_widget->setFace(target_face_ptr,textureData);

        delete[] textureData;
    }
    else if(!textured3D)
    {
        face_widget->disableTexture();
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
//
//    //face_widget->setTransformationMatrix(tranM);   

    //set the parameters in face widget
    face_widget->setTransParams(euler_x,-euler_y,euler_z,tx,ty,tz);
    target_face_ptr->transferExpressionFromFace(src_face_ptr);
    //face_widget->bindTexture(Utility::mat2QImage(frameData[i]));
    srcFrames.push_back(t_frameData[i]);

    sourceLabel->clearMarked();
    targetLabel->clearMarked();

    if(!show3D)
    {
         face_widget->setFace(target_face_ptr);
    }    

    if(textureInterpolate && !show3D)
    {
        QImage qimg = face_widget->grabFrameBuffer(false);
        QPixmap pimg = QPixmap::fromImage(qimg);
        pimg.setMask(pimg.createMaskFromColor(Qt::white,Qt::MaskInColor));
        QPixmap result = Utility::composePixmaps(Utility::mat2QPixmap(t_frameData[i]),pimg);
        targetLabel->setPixmap(result);
    }
    else if(!textureInterpolate && !show3D)
    {
        getClonedMouth(s_frameData[i],i,t_frameData[i]);        
        targetLabel->setPixmap(Utility::mat2QPixmap(t_frameData[i]));
    }

    sourceLabel->setPixmap(Utility::mat2QPixmap(s_frameData[i]));
    i++;
    view->incrementTransferProgress();

    if(i == src_videoProcessor->getFrameNum())
    {
        timerReplay->stop();
        i = 0;
        s_frameData.clear();
        t_frameData.clear();
        featurePoints.clear();

        view->setAllTransferTabButtonsDisabled(false);

        delete src_videoProcessor;
        delete target_videoProcessor;
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
    sourceLabel->clearMarked();
    targetLabel->clearMarked();

    capSrc->release();
    capTarget->release();

    srcFrames.clear();
    targetFrames.clear();
    s_frameData.clear();
    t_frameData.clear();
    featurePoints.clear();
    srcFileSelected(srcFile);
    targetFileSelected(targetFile);

    delete target_face_ptr;
    target_face_ptr = new Face();

    delete src_face_ptr;
    src_face_ptr = new Face();
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
        opttype = VideoProcessor::OptType_INTERPOLATE;
    else
        opttype = VideoProcessor::OptType_LIN_COMB;
}
void TransferTabController::setFrameNum(int n)
{
    frame_num = n;
}
void TransferTabController::setIterNum(int n)
{
    iter_num = n;
}
void TransferTabController::setGenPoints(int n)
{
    gen_point_num = n;
}
void TransferTabController::setTexture(bool tex)
{
    textureInterpolate = true;
}
void TransferTabController::setPoisson(bool tex)
{    
    textureInterpolate = false;
}
void TransferTabController::setUsingBackground(bool background)
{    
    this->show3D = !background;    
}
void TransferTabController::setTextured3D(bool b)
{
    cout << "tex 3d " << textured3D << endl;
    this->textured3D = b;
}
void TransferTabController::setConstID(bool b)
{
    if(b == true)
        this->idconstype = VideoProcessor::IdConstraintType_CONST;
    else
        this->idconstype = VideoProcessor::IdConstraintType_NONE;
}
void TransferTabController::setPointGen2D(bool b)
{
    if(b == true)
        this->projtype = VideoProcessor::PointGenerationType_2D;
}
void TransferTabController::setPointGen3D(bool b)
{
    if(b == true)
        this->projtype = VideoProcessor::PointGenerationType_3D;
}
void TransferTabController::setHybrid(bool b)
{
    if(b == true)
        this->projtype = VideoProcessor::PointGenerationType_HYBRID;
}
void TransferTabController::setNone(bool b)
{
    if(b == true)
        this->projtype = VideoProcessor::PointGenerationType_NONE;
}
void TransferTabController::setProjModel(bool b)
{
    this->projModel = b;
}
void TransferTabController::setWithFirstFrame(bool b)
{
    this->withFirstFrame = b;
}

void TransferTabController::selectGoodFeatureSrcPoints()
{
    Utility::selectGoodFeaturePoints(sourceLabel);
}
void TransferTabController::selectGoodFeatureTargetPoints()
{
    Utility::selectGoodFeaturePoints(targetLabel);
}
