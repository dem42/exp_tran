#ifndef CLOSEDFORMOPTIMIZER_H
#define CLOSEDFORMOPTIMIZER_H

#include "optimizer.h"
#include <vector>
#include <cv.h>
#include "model/Face.h"

using namespace std;
using namespace cv;

class ClosedFormOptimizer : public Optimizer
{
public:
    ClosedFormOptimizer();
    void estimateModelParameters(const Mat &frame, const vector<Point2f> &featurePoints,
                                 const Mat &cameraMatrix, const Mat& lensDist,
                                 Face* face_ptr,const vector<int> &point_indices,
                                 const Mat &rotation, const Mat &translation,
                                 vector<double> &weights_id, vector<double> &weights_ex);

    void estimateExpressionParameters(const vector<Point2f> &featurePoints,
                                      const Mat &cameraMatrix, const Mat& lensDist,
                                      Face* face_ptr,const vector<int> &point_indices,
                                      const Mat &rotation, const Mat &translation,
                                      vector<double> &weights_ex);

    void estimateIdentityParameters(const vector<vector<Point2f> >&featurePointsVector,
                                    const Mat &cameraMatrix, const Mat& lensDist,
                                    Face* face_ptr,const vector<vector<int> >&point_indices_vector,
                                    const vector<Mat> &rotation, const vector<Mat> &translation,
                                    const vector<vector<double> > &weights_ex,
                                    vector<double> &weights_id);
private:
    const int max_iterations;
};

#endif // CLOSEDFORMOPTIMIZER_H
