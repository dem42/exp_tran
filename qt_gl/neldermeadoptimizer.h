#ifndef NELDERMEADOPTIMIZER_H
#define NELDERMEADOPTIMIZER_H

#include "optimizer.h"
#include "errorfunction.h"
#include <vector>
#include <cv.h>
#include "Face.h"

using namespace std;
using namespace cv;

class NelderMeadOptimizer : public Optimizer
{
public:
    NelderMeadOptimizer();
    void estimateParametersAndPose(const vector<Mat> &frames, const vector<vector<Point2f> > &featurePoints,
                                   const Mat &cameraMatrix, const Mat& lensDist, vector<Mat> &rotations, vector<Mat> &translations,
                                   vector<vector<double> >&w_id, vector<vector<double> >&w_ex,
                                   vector<vector<Point2f> > &generatedPoints);
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
