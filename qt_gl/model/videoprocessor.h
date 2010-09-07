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
    enum OptType { OptType_INTERPOLATE, OptType_LIN_COMB, OptType_NELDER_INT, OptType_NELDER_LIN };

    //constructors with default frame max and iteration max values
    VideoProcessor(const unsigned int fmax=5, const unsigned int imax=3);
    VideoProcessor(const vector<cv::Point2f> &featurePoints, const vector<cv::Mat> &frameData,
                   const cv::Mat &cameraMatrix, const cv::Mat &lensDist,
                   VideoProcessor::OptType type = OptType_INTERPOLATE, double regParam = 2000.0,
                   const unsigned int fmax=5, const unsigned int imax=3);
    VideoProcessor(Optimizer *paramOptimizer, OpticalFlowEngine *flowEngine,
                   const unsigned int fmax=5, const unsigned int imax=3);

    //process video
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
    void setOptimizer(Optimizer *paramOptimizer);

    //interpolates face with the parameters and returns the 3d pose parameters
    void getFaceForFrame(unsigned int frameIndex, Face *face_ptr) const;
    void getFaceAndPoseForFrame(unsigned int frameIndex, Face *face_ptr, Mat &rot, Mat &tran) const;
    void getGeneratedPointsForFrame(unsigned int frameIndex, vector<Point2f> &points) const;
    unsigned int getFrameNum() const;

    void run();

private:    
    const unsigned int FRAME_MAX;
    const unsigned int ITER_MAX;

    Optimizer *paramOptimizer;
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
