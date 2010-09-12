#ifndef OPTICALFLOWFARNEBACK_H
#define OPTICALFLOWFARNEBACK_H

#include "opticalflowengine.h"
#include <cv.h>
#include <highgui.h>
#include <vector>
#include <iostream>

class OpticalFlowFarneback : public OpticalFlowEngine
{
public:
    OpticalFlowFarneback();

protected:
    void computeFlow(const cv::Mat& prevImg, const cv::Mat& nextImg,
                     const std::vector<cv::Point2f>& prevPoints, std::vector<cv::Point2f>& nextPoints,
                     const std::vector<int> &curIndices, std::vector<int> &nextIndices,bool indicesOn);
private:
    double pyrScale;
    int levels;
    int winsize;
    int iter;
    int polyN;
    double polySigma;
    int flags;
};

#endif // OPTICALFLOWFARNEBACK_H
