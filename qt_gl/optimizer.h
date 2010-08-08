#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <vector>
#include <cv.h>
#include "Face.h"

using namespace std;
using namespace cv;

class Optimizer
{
public:
    virtual void estimateParametersAndPose(const vector<Mat> &frames, const vector<vector<Point2f> > &featurePoints,
                                   const Mat &cameraMatrix, const Mat& lensDist, vector<Mat> &rotations,
                                   vector<Mat> &translations, vector<vector<double> >&w_id, vector<vector<double> >&w_ex,
                                   vector<vector<Point2f> > &generatedPoints) = 0;
    void calculateTransformation(const vector<Point2f> &imagePoints, Face *face_ptr,
                                 const Mat &cameraMatrix, const Mat &lensDist,
                                 Mat &rvec,Mat &tvec,bool useExt=true);
protected:
    void estimatePose(const vector<vector<Point2f> > &featurePoints, Face *face_ptr,
                      const Mat &cameraMatrix, const Mat &lensDist,
                      vector<Mat> &rotations, vector<Mat> &translations);
    //static here could be read from an xml config
    static const int fPoints[20];
    static const int fPoints_size;
};


#endif // OPTIMIZER_H
