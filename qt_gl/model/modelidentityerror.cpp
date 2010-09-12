#include "modelidentityerror.h"
#include <limits>

using namespace std;
using namespace cv;


ModelIdentityError::ModelIdentityError(Mat P) : P(P)
{
    model = FaceModel::getInstance();
    core = model->getCoreTensor();
    u_id = model->getUIdentity();
    u_ex = model->getUExpression();
}

void ModelIdentityError::setWeights(const vector<vector<double> >& w)
{
    weights.assign(w.begin(),w.end());
}

void ModelIdentityError::setTransformations(const vector<Mat>&rot, const vector<Mat>&tran)
{
    Mat_<double> rmatrix;
    for(int i=0;i<rot.size();i++)
    {
        Rodrigues(rot[i],rmatrix);
        rotations.push_back(rmatrix.clone());
        translations.push_back(tran[i].clone());
    }
}

void ModelIdentityError::setPoints(const vector<vector<Point2f> >&p,const vector<vector<int> >&i)
{
    points.assign(p.begin(),p.end());
    indices.assign(i.begin(),i.end());

    Mat_<double> core_extracted;
    Mat_<double> projected;
    int index = 0;

    //preprocess the projections
    for(unsigned int j=0;j<p.size();j++)
    {
        for(unsigned int i=0; i<points[j].size(); ++i)
        {
            index = indices[j][i];
            if(Z.find(index) == Z.end())
            {
                core_extracted = core.submatrix(index*3,index*3+2);
                projected = P*rotations[j]*core_extracted;
                Z[index] = projected.clone();
            }
        }
    }
}

double ModelIdentityError::operator()(vector<double>& x)
{
    //the part of core corresponding to the point i
    Mat_<double> projection(3,1);
    Mat_<double> K;
    Mat_<double> point2d(3,1);
    Mat_<double> projectedT;

    double sum = 0;

    double x_error_sqrt;
    double y_error_sqrt;


    //implement non-negativity constraints
    for(unsigned int i=0;i<x.size();i++)
    {
        if(x[i] < 0) return numeric_limits<double>::max();
    }

    //here inside of operator() if error is identity then the parameter w is identity weights
    linear_combination_exp = Matrix::matrix_mult(Matrix(x),u_ex);
    for(int j=0;j<indices.size();j++)
    {

        linear_combination_id = Matrix::matrix_mult(weights[j],u_id);

        K = Matrix::kron(linear_combination_id,linear_combination_exp);
        K = K.t();

        projectedT = P*translations[j];


        for(unsigned int i=0; i<indices[j].size(); ++i)
        {
            projection = projectedT + Z[indices[j][i]]*K;

            //convert from homogenous coordinates
            projection.at<double>(0,0) /= projection.at<double>(2,0);
            projection.at<double>(1,0) /= projection.at<double>(2,0);

            //        cout << "p2d : " << points[i].x << "  " << points[i].y << endl;
            //        cout << "guess p2d : " << projection(0,0) << "  " << projection(1,0) << endl;

            x_error_sqrt = (points[j][i].x - projection(0,0))*(points[j][i].x - projection(0,0));
            y_error_sqrt = (points[j][i].y - projection(1,0))*(points[j][i].y - projection(1,0));
            sum += x_error_sqrt + y_error_sqrt;
            //cout << "sum : " << sum << endl;
        }
    }
    return sum;
}
