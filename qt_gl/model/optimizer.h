#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <vector>
#include <cv.h>
#include "model/Face.h"

using namespace std;
using namespace cv;

class Optimizer
{
public:
    virtual void estimateModelParameters(const Mat &frame, const vector<Point2f> &featurePoints,
                                         const Mat &cameraMatrix, const Mat& lensDist,
                                         Face* face_ptr,const vector<int> &point_indices,
                                         const Mat &rotation, const Mat &translation,
                                         vector<double> &weights_id, vector<double> &weights_ex) = 0;

    virtual void estimateExpressionParameters(const vector<Point2f> &featurePoints,
                                      const Mat &cameraMatrix, const Mat& lensDist,
                                      Face* face_ptr,const vector<int> &point_indices,
                                      const Mat &rotation, const Mat &translation,
                                      vector<double> &weights_ex) = 0;

    virtual void estimateIdentityParameters(const vector<vector<Point2f> >&featurePointsVector,
                                    const Mat &cameraMatrix, const Mat& lensDist,
                                    Face* face_ptr,const vector<vector<int> >&point_indices_vector,
                                    const vector<Mat> &rotation, const vector<Mat> &translation,
                                    const vector<vector<double> > &weights_ex,
                                    vector<double> &weights_id) = 0;

};


#endif // OPTIMIZER_H
