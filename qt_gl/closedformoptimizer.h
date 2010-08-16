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
    void estimateModelParameters(const Mat &frame, const vector<Point2f> &featurePoints,
                                 const Mat &cameraMatrix, const Mat& lensDist,
                                 Face* face_ptr,const vector<int> &point_indices,
                                 const Mat &rotation, const Mat &translation,
                                 vector<double> &weights_id, vector<double> &weights_ex);
private:
    const int max_iterations;
};

#endif // CLOSEDFORMOPTIMIZER_H
