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

    void calculateTransformation(const vector<Point2f> &imagePoints, Face *face_ptr,
                                 const Mat &cameraMatrix, const Mat &lensDist,
                                 const vector<int> &indices,
                                 Mat &rvec,Mat &tvec,bool useExt=true);

    void generatePoints(const Mat &rotation, const Mat &translation,
                        const Mat& cameraMatrix, const Mat& lensDist,
                        int point_number, Face *face_ptr,
                        vector<Point2f> &generatedPoints,
                        vector<int> &point_indices_for_frame);
    void weakPerspectiveProjectPoints(const Mat &rotation, const Mat &translation,
                                      const Mat& cameraMatrix, const Mat& lensDist,
                                      const vector<int> &point_indices, Face *face_ptr,
                                      vector<Point2f> &projectedPoints);
};


#endif // OPTIMIZER_H