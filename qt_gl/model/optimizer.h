#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <vector>
#include <cv.h>
#include "model/Face.h"

using namespace std;
using namespace cv;

class Optimizer
{
public:
    virtual void estimateModelParameters(const Mat &frame, const vector<Point2f> &featurePoints,
                                         const Mat &cameraMatrix, const Mat& lensDist,
                                         Face* face_ptr,const vector<int> &point_indices,
                                         const Mat &rotation, const Mat &translation,
                                         vector<double> &weights_id, vector<double> &weights_ex) = 0;

};


#endif // OPTIMIZER_H
