#ifndef UTILITY_H
#define UTILITY_H


#include <highgui.h>
#include <QWidget>
#include <QString>
#include <QLabel>
#include <QPixmap>

#include <cv.h>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

class Utility
{
public:
    static QImage mat2QImage(const Mat mat_rgb);
    static void computeEulerAnglesFromRmatrix(const Mat &rmatrix,double &euler_x, double &euler_y, double &euler_z);
    static void grabThumbnailForVideo(string videoName, Mat& thumbnail);
    static QPixmap mat2QPixmap(const Mat &m);
    static int closestLargetPowerOf2(int x);
    static QPixmap composePixmaps(const QPixmap &src, const QPixmap &dest);
    static void pointSampling(const Point2f& a, const Point2f& b, const int num,vector<Point2f> &points);
    static void sampleGoodPoints(const vector<Point2f> &points, vector<Point2f> &sampledPoints);
};

#endif // UTILITY_H
