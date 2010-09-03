#include "transfertabcontroller.h"
#include "utility.h"
#include <QBitmap>

TransferTabController::TransferTabController(ClickableQLabel *sourceLabel, ClickableQLabel *targetLabel,
                                             QLineEdit *srcText, QLineEdit *targetText,CustomizableFaceWidget *face_widget)
{
    this->targetLabel = targetLabel;
    this->sourceLabel = sourceLabel;
    this->srcText = srcText;
    this->targetText = targetText;
    this->face_widget = face_widget;
    this->face_widget->setCameraParameters(-200,-1,-200);
    face_ptr = new Face();
    face_widget->setFace(face_ptr);

    this->videoProcessor = new VideoProcessor();

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
}

/*************************************************/
/* PUBLIC SLOTS */
/*************************************************/
void TransferTabController::dropSrc()
{
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

    targetLabel->setVisible(false);
    face_widget->setVisible(true);

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

    videoProcessor->processVideo(sourceLabel->getMarked(),s_frameData, cameraSrc, lensDist,
                                 s_frameTranslation,s_frameRotation, s_generatedPoints,s_vector_weights_exp,s_vector_weights_id);


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

    videoProcessor->processVideo(targetLabel->getMarked(),t_frameData, cameraTarget, lensDist,
                                 t_frameTranslation,t_frameRotation, t_generatedPoints,t_vector_weights_exp,t_vector_weights_id);


    timerReplay = new QTimer(this);
    connect(timerReplay,SIGNAL(timeout()),this,SLOT(replayFrame()));
    timerReplay->start(1000);
}

void TransferTabController::replayFrame()
{
    static unsigned int i;
    double euler_x, euler_y, euler_z;
    Mat_<double> rmatrix;
    vector<Point2f> imagePoints;
    vector<Point3f> objectPoints;
    Point2 *textureData;
    double tx = s_frameTranslation[i].at<double>(0,0);
    double ty = s_frameTranslation[i].at<double>(0,1);
    double tz = s_frameTranslation[i].at<double>(0,2);

    double *w_id = new double[56];
    double *w_exp = new double[7];

    int img_width, img_height;

    sourceLabel->setPixmap(Utility::mat2QPixmap(s_frameData[i]));

    sourceLabel->setMarked(s_generatedPoints[i]);

    //compute and set the pose parameters
    Rodrigues(s_frameRotation[i],rmatrix);
    Utility::computeEulerAnglesFromRmatrix(rmatrix,euler_x,euler_y,euler_z);
    //only y needs to be negative so that it agrees with the transposes
    cout << "setting trans param " << euler_x << " " << euler_y << " "
         << euler_z  << " " << tx << " " << ty << " " << tz  << endl;

    face_widget->setTransParams(euler_x,-euler_y,euler_z,tx,ty,tz);

    //generate the corresponding face
    cout << "begin w_id and w_exp in frame : " << i << endl;
    for(int j=0;j<56;j++)
    {
        w_id[j] = s_vector_weights_id[i][j];
        cout << w_id[j] << " ";
    }
    cout << endl;
    for(int j=0;j<7;j++)
    {
        w_exp[j] = s_vector_weights_exp[i][j];
        cout << w_exp[j] << " ";
    }
    cout << endl;

    //converts and loads the texture into opengl
    //face_widget->bindTexture(Utility::mat2QImage(frameData[i]));
    convertFrameIntoTexture(s_frameData[i]);
    double zavg = face_ptr->getAverageDepth();
    face_ptr->interpolate(w_id,w_exp);
    face_ptr->setAverageDepth(zavg);

    for(int j=0;j<face_ptr->getPointNum();j++)
        objectPoints.push_back(face_ptr->vertexes[j]);

    cv::projectPoints(Mat(objectPoints),s_frameRotation[i],s_frameTranslation[i],cameraSrc,lensDist,imagePoints);
    textureData = new Point2[imagePoints.size()];

    sourceLabel->clearMarked();
    //sourceLabel->setMarked(imagePoints);
//
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
        if(j%1000==0)
            cout << "texture for j " << j << " is " << textureData[j].x << " " << textureData[j].y
                 <<  "from points " << imagePoints[j].x << " " << imagePoints[j].y << endl;
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
    cout << "tranM " << Matrix(tranM);
    //face_widget->setTransformationMatrix(tranM);

    Mat_<double> comp = projM*tranM;
    Mat_<double> p3a(4,1);
    Mat_<double> p3b(4,1);
    Mat_<double> p3c(3,1);
    Mat_<double> p3d(3,1);
    Mat_<double> tM(3,1);
    tM(0,0) = tx;
    tM(1,0) = ty;
    tM(2,0) = tz;

    Point3f p3f = face_ptr->getPointFromPolygon(10);
    p3a(0,0) = p3f.x;
    p3a(1,0) = p3f.y;
    p3a(2,0) = p3f.z;
    p3a(3,0) = 1.0;

    p3c(0,0) = p3f.x;
    p3c(1,0) = p3f.y;
    p3c(2,0) = p3f.z;

    p3b = comp*p3a;
    cout << "p3a " << Matrix(p3a);
    cout << "p3b " << Matrix(p3b);

    p3d = cameraSrc*(rmatrix*p3c + tM);
    p3d = (1./p3d(2,0)) * p3d;
    cout << "p3c " << Matrix(p3c);
    cout << "p3d " << Matrix(p3d);

    face_widget->setFace(face_ptr,textureData);
    delete[] textureData;

    QImage qimg = face_widget->grabFrameBuffer(false);
    QPixmap pimg = QPixmap::fromImage(qimg);
    pimg.setMask(pimg.createMaskFromColor(Qt::white,Qt::MaskInColor));



    QPixmap result = Utility::composePixmaps(Utility::mat2QPixmap(s_frameData[i]),pimg);
    sourceLabel->setPixmap(result);

    i++;
    if(i == s_generatedPoints.size())
    {
        timerReplay->stop();
        i = 0;
        s_frameData.clear();
        featurePoints.clear();
    }
    delete[] w_id;
    delete[] w_exp;
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
    cout << "in convert frame into texture " << endl;
    cout << height << " " << width << " " << channels << " " << step << endl;;
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
