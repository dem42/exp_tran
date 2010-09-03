#ifndef VIDEOPROCESSOR_H
#define VIDEOPROCESSOR_H

#include "opticalflowengine.h"
#include "optimizer.h"
#include "poseestimator.h"
#include <cv.h>
#include <vector>

class VideoProcessor
{
public:
    VideoProcessor();
    VideoProcessor(Optimizer *paramOptimizer, OpticalFlowEngine *flowEngine);
    void processVideo(const vector<cv::Point2f> &featurePoints, const vector<cv::Mat> &frameData,
                      const Mat &cameraMatrix, const Mat &lensDist,
                      vector<cv::Mat> &frameTranslation, vector<cv::Mat> &frameRotation,
                      vector<vector<cv::Point2f> > &generatedPoints,
                      vector<vector<double> >&vector_weights_exp,
                      vector<vector<double> >&vector_weights_id );

    void setFlowEngine(OpticalFlowEngine *flowEngine);
    void setOptimizer(Optimizer *paramOptimizer);

private:    
    const unsigned int FRAME_MAX;
    Optimizer *paramOptimizer;
    OpticalFlowEngine *flowEngine;
    PoseEstimator *poseEstimator;
};

#endif // VIDEOPROCESSOR_H
