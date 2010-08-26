#include "transfertabcontroller.h"
#include "utility.h"

TransferTabController::TransferTabController(ClickableQLabel *sourceLabel, ClickableQLabel *targetLabel,
                                             QLineEdit *srcText, QLineEdit *targetText,FaceWidget *face_widget)
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

    Mat m;
    double c_x, c_y;

    Utility::grabThumbnailForVideo(srcFile.toStdString(),m);
    srcFrames.push_back(m.clone());    
    sourceLabel->setPixmap(Utility::mat2QPixmap(m));
    c_x = m.size().width / 2.;
    c_y = m.size().height / 2.;

    Utility::grabThumbnailForVideo(targetFile.toStdString(),m);
    targetFrames.push_back(m.clone());
    targetLabel->setPixmap(Utility::mat2QPixmap(m));
    c_x = m.size().width / 2.;
    c_y = m.size().height / 2.;

    sourceLabel->show();
    targetLabel->show();

    capSrc = new VideoCapture(srcFile.toStdString());
    capTarget = new VideoCapture(targetFile.toStdString());


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
    lensDist = Mat_<double>(1,5);
    lensDist(0,0) = 0;
    lensDist(0,1) = 0;
    lensDist(0,2) = 0;
    lensDist(0,3) = 0;
    lensDist(0,4) = 0;
}

/*************************************************/
/* PUBLIC SLOTS */
/*************************************************/
void TransferTabController::dropSrc()
{

}
void TransferTabController::dropTarget()
{

}
void TransferTabController::beginTransfer()
{
    Mat frame, rgb_frame, copyFrame;

    targetLabel->setVisible(false);
    face_widget->setVisible(true);

    /**********************/
    /*first collect frames*/
    /**********************/    
    frameData.push_back(srcFrames[srcFrames.size()-1]);
    sourceLabel->setPixmap(Utility::mat2QPixmap(srcFrames[0]));
    while( capSrc->grab() == true)
    {
        capSrc->retrieve(frame);
        cvtColor(frame, rgb_frame, CV_BGR2RGB);
        copyFrame = rgb_frame.clone();
        frameData.push_back(copyFrame);
    }

    videoProcessor->processVideo(sourceLabel->getMarked(),frameData, cameraSrc, lensDist,
                                 frameTranslation,frameRotation, generatedPoints,vector_weights_exp,vector_weights_id);

    timerReplay = new QTimer(this);
    connect(timerReplay,SIGNAL(timeout()),this,SLOT(replayFrame()));
    timerReplay->start(3000);
}

void TransferTabController::replayFrame()
{
    static unsigned int i;
    double euler_x, euler_y, euler_z;
    Mat_<double> rmatrix;

    double tx = frameTranslation[i].at<double>(0,0);
    double ty = frameTranslation[i].at<double>(0,1);
    double tz = frameTranslation[i].at<double>(0,2);

    double *w_id = new double[56];
    double *w_exp = new double[7];

    sourceLabel->setPixmap(Utility::mat2QPixmap(frameData[i]));

    sourceLabel->setMarked(generatedPoints[i]);

    //compute and set the pose parameters
    Rodrigues(frameRotation[i],rmatrix);
    Utility::computeEulerAnglesFromRmatrix(rmatrix,euler_x,euler_y,euler_z);
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
    if(i == generatedPoints.size())
    {
        timerReplay->stop();
        i = 0;
        frameData.clear();
        featurePoints.clear();
    }
    delete[] w_id;
    delete[] w_exp;
}


void TransferTabController::targetFileSelected(const QString str)
{
    targetFile = str;
    targetText->setText(str);
}

void TransferTabController::srcFileSelected(const QString str)
{
    srcFile = str;
    srcText->setText(str);
}

void TransferTabController::targetBrowse()
{
    targetDia->show();
}

void TransferTabController::srcBrowse()
{
    srcDia->show();
}
