#include "modelimageerror.h"

using namespace std;
using namespace cv;

ModelImageError::ModelImageError()
{
    model = FaceModel::getInstance();
    core = model->getCoreTensor();
    u_id = model->getUIdentity();
    u_ex = model->getUExpression();
}

ModelImageError::ModelImageError(Mat P,Mat R,Mat t) : P(P), R(R), t(t)
{    
    model = FaceModel::getInstance();
    core = model->getCoreTensor();
    u_id = model->getUIdentity();
    u_ex = model->getUExpression(); 
}

void ModelImageError::setWeights(vector<double>& w)
{   
    weights = w;
    linear_combination_id = Matrix::matrix_mult(Matrix(w),u_id);
}

void ModelImageError::setPoints(vector<Point2f> &p, vector<int> &i)
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
        core_extracted = core.submatrix(index*3,index*3+2);
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

    Matrix weights_exp(x);
    Matrix linear_combination_exp = Matrix::matrix_mult(weights_exp,u_ex);

    K = Matrix::kron(linear_combination_id,linear_combination_exp);
    K = K.t();

    projection = P*t;

    double sum = 0;
    double x_error_sqrt;
    double y_error_sqrt;

   for(unsigned int i=0; i<points.size(); ++i)
    {        
//        cout << "in error " << endl;
//        cout << P.size().height << " " << P.size().width << endl;
//        cout << R.size().height << " " << R.size().width << endl;
//        cout << core_extracted.size().height << " " << core_extracted.size().width << endl;
//        cout << K.size().height << " " << K.size().width << endl;
//        cout << t.size().height << " " << t.size().width << endl;

       projection = projection + Z[i]*K;

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
