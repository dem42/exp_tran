#ifndef NNLSOPTIMIZER_H
#define NNLSOPTIMIZER_H

#include "optimizer.h"
#include "errorfunction.h"
#include <vector>
#include <cv.h>
#include "model/Face.h"

using namespace std;
using namespace cv;

class NNLSOptimizer : public Optimizer
{
public:
    NNLSOptimizer();
    void estimateModelParameters(const Mat &frame, const vector<Point2f> &featurePoints,
                                 const Mat &cameraMatrix, const Mat& lensDist,
                                 Face* face_ptr,const vector<int> &point_indices,
                                 const Mat &rotation, const Mat &translation,
                                 vector<double> &weights_id, vector<double> &weights_ex);

    /**
      * Sequential coordinate descent non negative least squares
      * min 1/2*norm(b - A*x)^2  .. so that x >= 0
      */
    void scannls(const Mat& A, const Mat& b,Mat &x);
    void test();
private:
    //e-KKT conditions used for stopping condition
    bool eKKT(const Mat& H,const Mat& F,const Mat& x,double e);
    const int NNLS_MAX_ITER;
    const int max_iterations;
};

#endif // NNLSOPTIMIZER_H
