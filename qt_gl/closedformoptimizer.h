#ifndef CLOSEDFORMOPTIMIZER_H
#define CLOSEDFORMOPTIMIZER_H

#include "optimizer.h"
#include <vector>
#include <cv.h>
#include "Face.h"

using namespace std;
using namespace cv;

class ClosedFormOptimizer : public Optimizer
{
public:
    ClosedFormOptimizer();
    void estimateParametersAndPose(const vector<Mat> &frames, const vector<vector<Point2f> > &featurePoints,
                                   const Mat &cameraMatrix, const Mat& lensDist, vector<Mat> &rotations, vector<Mat> &translations,
                                   vector<vector<double> >&w_id, vector<vector<double> >&w_ex,
                                   vector<vector<Point2f> > &generatedPoints);
private:
    const int max_iterations;
};

#endif // CLOSEDFORMOPTIMIZER_H
