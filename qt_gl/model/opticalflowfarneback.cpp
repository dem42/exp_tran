#include "opticalflowfarneback.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;

OpticalFlowFarneback::OpticalFlowFarneback() : OpticalFlowEngine()
{
    pyrScale = 0.5;
    levels = 3;
    winsize = 3;
    iter = 5;
    polyN = 5;
    polySigma = 1.1;
    flags = 0;
}

void OpticalFlowFarneback::computeFlow(const Mat& prevImg, const Mat& nextImg,
                                    const vector<Point2f>& prevPoints, vector<Point2f>& nextPoints,
                                    const std::vector<int> &curIndices, std::vector<int> &nextIndices,bool indicesOn)
{
    Mat flowOutput;
    int sizeX,sizeY;

    int x,y;
    float dx,dy;


    calcOpticalFlowFarneback(prevImg.reshape(1),nextImg.reshape(1),flowOutput,pyrScale,levels,winsize,iter,polyN,polySigma,flags);

    Mat flowOutput_reshaped = flowOutput.reshape(3);
    sizeX = nextImg.size().width;
    sizeY = nextImg.size().height;

    //prevPoints holds the x,y coordiantes gained from the mouse event
    //these can be safely cast to int without loss of precision
    //cout << flowOutput2.at<float>(2,1) << endl;

    for(int i=0;i<prevPoints.size();i++)
    {
        x = prevPoints[i].x;
        y = prevPoints[i].y;        
        dx = flowOutput_reshaped.at<float>(y,2*x);
        dy = flowOutput_reshaped.at<float>(y,2*x+1);
        nextPoints.push_back(Point2f(x+dx,y+dy));
    }
}
