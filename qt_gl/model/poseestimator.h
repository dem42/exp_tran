#ifndef POSEESTIMATOR_H
#define POSEESTIMATOR_H

#include <vector>
#include <cv.h>
#include "model/Face.h"

using namespace std;
using namespace cv;


class PoseEstimator
{
public:
    PoseEstimator();
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
    void reprojectInto3DUsingWeak(const vector<Point2f> &imagePoints,
                                  const Mat &rotation, const Mat &translation,
                                  const Mat& cameraMatrix, const Mat& lensDist,
                                  Face *face_ptr, vector<int> &correspondence3d);
};

#endif // POSEESTIMATOR_H
