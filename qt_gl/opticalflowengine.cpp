#include "opticalflowengine.h"

using namespace std;
using namespace cv;

OpticalFlowEngine::OpticalFlowEngine()
{
}

void OpticalFlowEngine::computeFlow(const Mat& prevImg, const Mat& nextImg,
                                    const vector<Point2f>& prevPoints, vector<Point2f>& nextPoints)
{
    vector<uchar> status;
    vector<float> err;

//    cout << prevImg.size().height << " " << prevImg.size().width << " "
//         << nextImg.size().height << " " << nextImg.size().width << endl;

    Mat pImgG, nImgG;
    //convert to grayscale
//    cvtColor(prevImg, pImgG, CV_RGB2GRAY);
//    cvtColor(nextImg, nImgG, CV_RGB2GRAY);
//
//    calcOpticalFlowPyrLK(pImgG,nImgG,prevPoints,nextPoints,status,err,Size(10,10),
//                         3,TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03));


    IplImage pim, nim, *frame1, *frame2, *pyramid1, *pyramid2;

    CvSize size = prevImg.size();
    pim = prevImg;
    frame1 = cvCreateImage(size,IPL_DEPTH_8U, 1);
    cvConvertImage(&pim,frame1,0);

    size = nextImg.size();
    nim = nextImg;
    frame2 = cvCreateImage(size,IPL_DEPTH_8U, 1);
    cvConvertImage(&nim,frame2,0);

    CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
    pyramid1 = cvCreateImage(size,IPL_DEPTH_8U, 1);
    pyramid2 = cvCreateImage(size,IPL_DEPTH_8U, 1);

    int number_of_features = prevPoints.size();

    CvPoint2D32f features[number_of_features];
    CvPoint2D32f guess[number_of_features];

    for(int i=0;i<number_of_features;i++)
    {
        features[i].x = prevPoints[i].x;
        features[i].y = prevPoints[i].y;
    }

    CvSize window = cvSize(5,5);
    char flow_status[number_of_features];
    float flow_error[number_of_features];

    cvCalcOpticalFlowPyrLK(frame1,frame2,pyramid1,pyramid2,features,guess,number_of_features,window,5,flow_status,flow_error,criteria,0);

    for(int i=0;i<number_of_features;i++)
    {
        nextPoints.push_back(Point2f(guess[i].x,guess[i].y));
    }

//    for(int i=0;i<err.size();i++)
//        cout << err[i] << " and " << (float)status[i] << " ";
//    cout << endl;

}
