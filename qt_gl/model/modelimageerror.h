#ifndef MODELIMAGEERROR_H
#define MODELIMAGEERROR_H

#include "errorfunction.h"
#include <vector>
#include <cv.h>
#include "model/FaceModel.h"

using namespace std;
using namespace cv;

class ModelImageError : public ErrorFunction
{
public:
    ModelImageError();
    ModelImageError(Mat P,Mat R,Mat t);
    void setWeights(const vector<double> &);
    void setPoints(const vector<Point2f> &, const vector<int> &);

    double operator()(vector<double>&);
private:
    //projection
    Mat_<double> P;
    //rotation
    Mat_<double> R;
    //translation
    Mat_<double> t;


    Matrix core;
    Matrix u_id;
    Matrix u_ex;

    //face model
    FaceModel *model;
    //other weights
    vector<double> weights;
    Matrix linear_combination_id;
    Matrix linear_combination_exp;
    //points that we sum the error over
    //the index points to one of the 1-5090 points
    vector<int> indices;
    vector<Point2f> points;
   //the part of the core for each point times P and R
    vector<Mat_<double> >Z;
};

#endif // MODELIMAGEERROR_H
