#ifndef VIDEOPROCESSOR_H
#define VIDEOPROCESSOR_H

#include "opticalflowengine.h"
#include "optimizer.h"
#include "nnlsoptimizer.h"
#include "poseestimator.h"
#include <cv.h>
#include <vector>
#include <QThread>

class VideoProcessor : public QThread
{
public:
    VideoProcessor();
    VideoProcessor(const vector<cv::Point2f> &featurePoints, const vector<cv::Mat> &frameData,
                   const cv::Mat &cameraMatrix, const cv::Mat &lensDist);
    VideoProcessor(NNLSOptimizer *paramOptimizer, OpticalFlowEngine *flowEngine);
    void processVideo2(const vector<cv::Point2f> &featurePoints, const vector<cv::Mat> &frameData,
                      const Mat &cameraMatrix, const Mat &lensDist,
                      vector<cv::Mat> &frameTranslation, vector<cv::Mat> &frameRotation,
                      vector<vector<cv::Point2f> > &generatedPoints,
                      vector<vector<double> >&vector_weights_exp,
                      vector<vector<double> >&vector_weights_id );
    void processVideo3(const vector<cv::Point2f> &featurePoints, const vector<cv::Mat> &frameData,
                      const Mat &cameraMatrix, const Mat &lensDist,
                      vector<cv::Mat> &frameTranslation, vector<cv::Mat> &frameRotation,
                      vector<vector<cv::Point2f> > &generatedPoints,
                      vector<vector<double> >&vector_weights_exp,
                      vector<vector<double> >&vector_weights_id );
    void processVideo(const vector<cv::Point2f> &featurePoints, const vector<cv::Mat> &frameData,
                      const Mat &cameraMatrix, const Mat &lensDist,
                      vector<cv::Mat> &frameTranslation, vector<cv::Mat> &frameRotation,
                      vector<vector<cv::Point2f> > &generatedPoints,
                      vector<vector<double> >&vector_weights_exp,
                      vector<vector<double> >&vector_weights_id );

    void setFlowEngine(OpticalFlowEngine *flowEngine);
    void setOptimizer(NNLSOptimizer *paramOptimizer);

    void run();

private:    
    const unsigned int FRAME_MAX;
    NNLSOptimizer *paramOptimizer;
    OpticalFlowEngine *flowEngine;
    PoseEstimator *poseEstimator;

    vector<cv::Mat> fData;
    vector<cv::Point2f> fPoints;
    cv::Mat cameraMatrix;
    cv::Mat lensDist;

    //pose data for every frame
    vector<cv::Mat> frameTranslation;
    vector<cv::Mat> frameRotation;
    //points used to calculate model parameters
    vector<vector<cv::Point2f> > generatedPoints;
    //recalculated weights
    vector<vector<double> >vector_weights_exp;
    vector<vector<double> >vector_weights_id;
};

#endif // VIDEOPROCESSOR_H
