#include "modelimageerror.h"
#include <limits>

using namespace std;
using namespace cv;

ModelImageError::ModelImageError()
{
    model = FaceModel::getInstance();    
    u_id = model->getUIdentity();
    u_ex = model->getUExpression();
}

ModelImageError::ModelImageError(Mat P,Mat R,Mat t) : P(P), R(R), t(t)
{    
    model = FaceModel::getInstance();
    u_id = model->getUIdentity();
    u_ex = model->getUExpression(); 
}

void ModelImageError::setWeights(const vector<double>& w)
{   
    weights.assign(w.begin(),w.end());

    linear_combination_id = Mat_<double>(weights)*u_id;

}

void ModelImageError::setPoints(const vector<Point2f> &p,const vector<int> &i)
{
    points = p;
    indices = i;

    Mat_<double> core_extracted;
    Mat_<double> projected;
    int index = 0;

    //preprocess the projections
    for(unsigned int i=0; i<points.size(); ++i)
    {
        index = indices[i];       
        core_extracted = model->coreSubmatrix(index*3,index*3+2);
        projected = P*R*core_extracted;
        Z.push_back(projected.clone());
    }
}

double ModelImageError::operator()(vector<double>& x)
{
    //the part of core corresponding to the point i    
    Mat_<double> projection(3,1);
    Mat_<double> K;
    Mat_<double> point2d(3,1);
    Mat_<double> projectedT;

    double sum = 0;

    double x_error_sqrt;
    double y_error_sqrt;

    Matrix w(x);

    //implement non-negativity constraints
    for(unsigned int i=0;i<x.size();i++)
    {
        if(x[i] < 0) return numeric_limits<double>::max();    
    }

    //here inside of operator() if error is identity then the parameter w is identity weights

    linear_combination_exp = w*u_ex;

    K = Matrix::kronecker(linear_combination_id,linear_combination_exp);
    K = K.t();

    projectedT = P*t;


   for(unsigned int i=0; i<points.size(); ++i)
    {        
       projection = projectedT + Z[i]*K;

        //convert from homogenous coordinates
        projection.at<double>(0,0) /= projection.at<double>(2,0);
        projection.at<double>(1,0) /= projection.at<double>(2,0);

//        cout << "p2d : " << points[i].x << "  " << points[i].y << endl;
//        cout << "guess p2d : " << projection(0,0) << "  " << projection(1,0) << endl;

        x_error_sqrt = (points[i].x - projection(0,0))*(points[i].x - projection(0,0));
        y_error_sqrt = (points[i].y - projection(1,0))*(points[i].y - projection(1,0));
        sum += x_error_sqrt + y_error_sqrt;
        //cout << "sum : " << sum << endl;
    }

    return sum;
}
