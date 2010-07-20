#ifndef OPTICALFLOWENGINE_H
#define OPTICALFLOWENGINE_H

#include <cv.h>
#include <highgui.h>
#include <vector>
#include <iostream>


class OpticalFlowEngine
{
public:
    OpticalFlowEngine();
    virtual void computeFlow(const cv::Mat& prevImg, const cv::Mat& nextImg,
                             const std::vector<cv::Point2f>& prevPoints, std::vector<cv::Point2f>& nextPoints);
};

#endif // OPTICALFLOWENGINE_H
