#ifndef NELDERMEADOPTIMIZER_H
#define NELDERMEADOPTIMIZER_H

#include "optimizer.h"
#include "errorfunction.h"
#include <vector>
#include <cv.h>
#include "model/Face.h"

using namespace std;
using namespace cv;

class NelderMeadOptimizer : public Optimizer
{
public:
    NelderMeadOptimizer();
    void estimateModelParameters(const Mat &frame, const vector<Point2f> &featurePoints,
                                 const Mat &cameraMatrix, const Mat& lensDist,
                                 Face* face_ptr,const vector<int> &point_indices,
                                 const Mat &rotation, const Mat &translation,
                                 vector<double> &weights_id, vector<double> &weights_ex);
    /*
     * nelder mead downhill simplex (numerical optimization that doesnt need gradients)
     * @param func     function to optimize
     * @param start    initial vector
     * @param n        size of initial vector (dimension of the problem )
     * @param scale    used for calculating the vertices of the simplex
     * @return min     function minimum
     */
    double mysimplex(ErrorFunction &func, std::vector<double>& start, int n, double scale);
};

#endif // NELDERMEADOPTIMIZER_H
