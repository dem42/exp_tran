#ifndef VIDEOPROCESSOR_H
#define VIDEOPROCESSOR_H

#include "opticalflowengine.h"
#include "optimizer.h"
#include "nnlsoptimizer.h"
#include "poseestimator.h"
#include <cv.h>
#include <vector>
#include <QThread>
#include "exptranexception.h"

class VideoProcessor : public QThread
{
public:
    enum OptType { OptType_INTERPOLATE, OptType_LIN_COMB, OptType_NELDER_INT, OptType_NELDER_LIN };
    enum IdConstraintType { IdConstraintType_NONE, IdConstraintType_CONST};
    enum PointGenerationType {PointGenerationType_3D, PointGenerationType_2D, PointGenerationType_HYBRID, PointGenerationType_NONE};

    //constructors with default frame max and iteration max values
    VideoProcessor(const unsigned int fmax=5, const unsigned int imax=3);
    VideoProcessor(const vector<cv::Point2f> &featurePoints, const vector<cv::Mat> &frameData,
                   const cv::Mat &cameraMatrix, const cv::Mat &lensDist,
                   VideoProcessor::OptType type = OptType_INTERPOLATE, double regParam = 2000.0,
                   VideoProcessor::IdConstraintType idconst = IdConstraintType_CONST,
                   VideoProcessor::PointGenerationType pgtype = PointGenerationType_3D,
                   const unsigned int fmax=5, const unsigned int imax=3,
                   bool withFirstFrame = true, unsigned int gen_point_num=400);
    VideoProcessor(Optimizer *paramOptimizer, OpticalFlowEngine *flowEngine,
                   const unsigned int fmax=5, const unsigned int imax=3);
    ~VideoProcessor();

    //process video
    void processVideo(const vector<cv::Point2f> &featurePoints, const vector<cv::Mat> &frameData,                      
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
    bool getCrashed() const;

    void run();

private:    
    bool termination(const vector<vector<double> >&prevExp, const vector<vector<double> >&exp,
                     const vector<vector<double> >&prevId, const vector<vector<double> >&id);
    void identityExpressionUpdate(const vector<vector<Point2f> >&estimationPoints,
                                  const vector<vector<int> >&estimation_point_indices,
                                  const vector<cv::Mat> &frameTranslation, const vector<cv::Mat> &frameRotation,
                                  Face *face_ptr,vector<vector<double> >&vector_weights_exp,
                                  vector<vector<double> >&vector_weights_id);
    void generateNewFeaturePoints(const Mat& rotation, const Mat &translation,  Face *face_ptr,
                                  vector<Point2f> &currentPoints, vector<int> &indices);


    bool threadCrashed;
    bool withFirstFrame;

    const unsigned int FRAME_MAX;
    const unsigned int ITER_MAX;
    const unsigned int NEW_POINT_SIZE;

    Optimizer *paramOptimizer;
    OpticalFlowEngine *flowEngine;
    PoseEstimator *poseEstimator;

    VideoProcessor::PointGenerationType pgt;
    VideoProcessor::IdConstraintType idct;

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
