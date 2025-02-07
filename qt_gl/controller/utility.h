#ifndef UTILITY_H
#define UTILITY_H


#include <opencv/highgui.h>
#include <QWidget>
#include <QString>
#include <QLabel>
#include <QPixmap>
#include "view/clickableqlabel.h"

#include <opencv/cv.hpp>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

class Utility
{
public:
    static QImage mat2QImage(const Mat mat_rgb, QImage::Format f = QImage::Format_RGB888);
    static void computeEulerAnglesFromRmatrix(const Mat &rmatrix,double &euler_x, double &euler_y, double &euler_z);
    static void grabThumbnailForVideo(string videoName, Mat& thumbnail);
    static QPixmap mat2QPixmap(const Mat &m, QImage::Format f = QImage::Format_RGB888);
    static int closestLargetPowerOf2(int x);
    static QPixmap composePixmaps(const QPixmap &src, const QPixmap &dest);
    static void pointSampling(const Point2f& a, const Point2f& b, const int num,vector<Point2f> &points);
    static void pointSamplingNormal(const Point2f&a, const int num, const double var1, const double var2,
                                  vector<Point2f> &points);
    static void sampleGoodPoints(const vector<Point2f> &points, vector<Point2f> &sampledPoints);

    static void filterForGradient(const Mat &img, Mat &dest);
    static void poissonClone(const Mat &src, const Mat &mask, Mat &target,int o_x=0, int o_y=0);

    static void selectGoodFeaturePoints(ClickableQLabel *label);
};

#endif // UTILITY_H
