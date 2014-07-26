#ifndef MODELIDENTITYERROR_H
#define MODELIDENTITYERROR_H

#include "errorfunction.h"
#include <vector>
#include <opencv/cv.hpp>
#include "model/FaceModel.h"

using namespace std;
using namespace cv;

class ModelIdentityError : public ErrorFunction
{
public:

    ModelIdentityError(Mat P);
    void setWeights(const vector<vector<double> > &);
    void setPoints(const vector<vector<Point2f> > &, const vector<vector<int> > &);
    void setTransformations(const vector<Mat>&rot, const vector<Mat>&tran);

    double operator()(vector<double>&);
private:
    //projection
    Mat_<double> P;
    //rotation

    Mat_<double> u_id;
    Mat_<double> u_ex;

    //face model
    FaceModel *model;
    //other weights
    vector<vector<double> > weights;
    vector<Mat_<double> > rotations;
    vector<Mat_<double> > translations;

    Mat_<double> linear_combination_id;
    Mat_<double> linear_combination_exp;
    //points that we sum the error over
    //the index points to one of the 1-5090 points
    vector<vector<int> > indices;
    vector<vector<Point2f> > points;
   //the part of the core for each point times P and R
    map<uint,Mat_<double> >Z;
};


#endif // MODELIDENTITYERROR_H
