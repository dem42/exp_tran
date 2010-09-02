#include "utility.h"
#include <cmath>
#include <iostream>
#include <QPainter>
using namespace std;


QPixmap Utility::composePixmaps(const QPixmap &src, const QPixmap &dest)
{
    QPixmap result;
    result = src.copy();
    QPainter painter(&result);
    painter.setCompositionMode(QPainter::CompositionMode_Source);
    painter.drawPixmap(0,0,src);
    painter.setCompositionMode(QPainter::CompositionMode_DestinationOver);
    painter.drawPixmap(0,0,dest);
    painter.end();
    return result;
}

int Utility::closestLargetPowerOf2(int x)
{
    double ldx = std::log(x) / std::log(2.0);
    double powldx = std::pow(2,std::ceil(ldx));

    return (int)powldx;
}
QImage Utility::mat2QImage(const Mat mat_rgb)
{
    QImage qframe;
    //converting to rgb not necessary it seems
    //cvtColor(mat_bgr, mat_rgb, CV_BGR2RGB);

    qframe = QImage((const unsigned char*)(mat_rgb.data), mat_rgb.cols,
    mat_rgb.rows, QImage::Format_RGB888);
    //rgb QImage::Format_RGB888
    return qframe;
}

QPixmap Utility::mat2QPixmap(const Mat &m)
{
    QImage img_q = Utility::mat2QImage(m);
    QPixmap p_map;
    p_map = QPixmap::fromImage(img_q);
    return p_map;
}

void Utility::computeEulerAnglesFromRmatrix(const Mat &rmatrix,double &euler_x, double &euler_y, double &euler_z)
{
    const double PI = 3.141593;


    double rot_y, rot_x, rot_z,cy, cx,sx,cz,sz;
    rot_y = asin( rmatrix.at<double>(2,0));        /* Calculate Y-axis angle */
    cy           =  cos( rot_y );

    if ( fabs( cy ) > 0.005 )             /* Gimball lock? */
    {
        cx      =  rmatrix.at<double>(2,2) / cy;           /* No, so get X-axis angle */
        sx      = rmatrix.at<double>(2,1)  / cy;

        rot_x  = atan2( sx, cx );

        cz      =  rmatrix.at<double>(0,0) / cy;            /* Get Z-axis angle */
        sz      = rmatrix.at<double>(1,0) / cy;

        rot_z  = atan2( sz, cz );
    }

    euler_x = rot_x * 180./PI;
    euler_y = rot_y * 180./PI;
    euler_z = rot_z * 180./PI;
}

void Utility::grabThumbnailForVideo(string videoName, Mat& thumbnail)
{
    Mat frame;

    VideoCapture cap(videoName);
    cap >> frame;
    cvtColor(frame, thumbnail, CV_BGR2RGB);

    cap.release();
}
