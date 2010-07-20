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
    calcOpticalFlowPyrLK(prevImg,nextImg,prevPoints,nextPoints,status,err,Size(3,3),
                         3,TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03));


//    for(int i=0;i<err.size();i++)
//        cout << err[i] << " and " << (float)status[i] << " ";
//    cout << endl;

}
