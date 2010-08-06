#include "modelimageerror.h"

using namespace std;
using namespace cv;

ModelImageError::ModelImageError()
{
    model = FaceModel::getInstance();
}

ModelImageError::ModelImageError(Mat P,Mat R,Mat t) : P(P), R(R), t(t)
{
    model = FaceModel::getInstance();
}

void ModelImageError::setWeights(vector<double>& w)
{
    weights = w;
}

void ModelImageError::setPointIndices(vector<int> &p)
{
     indices = p;
}

void ModelImageError::setPoints(vector<Point2f> &p)
{
    points = p;
}

double ModelImageError::operator()(vector<double>& x)
{

    Matrix core = model->getCoreTensor();
    Matrix u_id = model->getUIdentity();
    Matrix u_ex = model->getUExpression();

    //the part of core corresponding to the point i
    Mat_<double> core_extracted;
    Mat_<double> projection(3,1);
    Mat_<double> K;
    Mat_<double> point2d(3,1);

    double sum = 0;
    double x_error_sqrt;
    double y_error_sqrt;

    for(unsigned int i=0; i<points.size(); ++i)
    {
        core_extracted = core.submatrix(i*3,i*3+2);
        K = Matrix::kron(Matrix(weights),Matrix(x));
        K = K.t();
//        cout << "in error " << endl;
//        cout << P.size().height << " " << P.size().width << endl;
//        cout << R.size().height << " " << R.size().width << endl;
//        cout << core_extracted.size().height << " " << core_extracted.size().width << endl;
//        cout << K.size().height << " " << K.size().width << endl;
//        cout << t.size().height << " " << t.size().width << endl;
        projection = (P*R*core_extracted*K);
        projection = projection + P*t;
        //convert from homogenous coordinates
        projection.at<double>(0,0) /= projection.at<double>(2,0);
        projection.at<double>(1,0) /= projection.at<double>(2,0);

        x_error_sqrt = (points[i].x - projection(0,0))*(points[i].x - projection(0,0));
        y_error_sqrt = (points[i].x - projection(1,0))*(points[i].x - projection(1,0));
        sum += x_error_sqrt + y_error_sqrt;
    }

    return sum;
}
