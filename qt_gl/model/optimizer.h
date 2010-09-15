#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <vector>
#include <map>
#include <cv.h>
#include "model/Face.h"
#include "model/FaceModel.h"

using namespace std;
using namespace cv;

class Optimizer
{
public:
    Optimizer();
    virtual ~Optimizer();
    virtual void estimateModelParameters(const vector<Point2f> &featurePoints,
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

    void setPointIndices(const vector<int>&point_indices_vector);

protected:
    map<int,Mat> M;
    FaceModel *model;
    Matrix core;
    Mat_<double> u_ex;
    Mat_<double> u_id;

};


#endif // OPTIMIZER_H
