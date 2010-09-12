#ifndef OPTICALFLOWENGINE_H
#define OPTICALFLOWENGINE_H

#include <cv.h>
#include <highgui.h>
#include <vector>
#include <iostream>
#include <cstdlib>

class OpticalFlowEngine
{
public:
    OpticalFlowEngine();
    virtual void computeFlow(const cv::Mat& prevImg, const cv::Mat& nextImg,
                             const std::vector<cv::Point2f>& prevPoints, std::vector<cv::Point2f>& nextPoints);
    virtual void computeFlow(const cv::Mat& prevImg, const cv::Mat& nextImg,
                             const std::vector<cv::Point2f>& prevPoints, std::vector<cv::Point2f>& nextPoints,
                             const std::vector<int> &curIndices,
                             std::vector<int> &nextIndices);
protected:
    virtual void computeFlow(const cv::Mat& prevImg, const cv::Mat& nextImg,
                             const std::vector<cv::Point2f>& prevPoints, std::vector<cv::Point2f>& nextPoints,
                             const std::vector<int> &curIndices,
                             std::vector<int> &nextIndices, bool indicesOn);

};

#endif // OPTICALFLOWENGINE_H
