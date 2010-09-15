#include "utility.h"
#include "model/Matrix.h"
#include "model/Face.h"
#include <cmath>
#include <iostream>
#include <QPainter>
#include <cstdio>
#include <cstdlib>
using namespace std;


void Utility::filterForGradient(const Mat &img, Mat &dest)
{
    Mat d1, d2;

    float kernel[3][3] = {{-3,0,3},{-10,0,10},{-3,0,3}};
    Mat filter(3,3,CV_32FC1,kernel);

    cv::filter2D(img,dest,CV_8UC1,filter);
}

double static inline clamp(double f) {
        if (f < 0.0) return 0.0;
        else if (f > 1.0) return 1.0;
        else return f;
}

void Utility::poissonClone(const Mat &src, const Mat &mask, Mat &target, int o_x, int o_y)
{
    Mat laplacian;
    //mapping of variables
    map<uint,uint> mp;
    int N = 0;

    int step = src.step;
    int w = src.cols;
    int h = src.rows;
    int channel = src.channels();


    for(int y=1;y<h-1;y++)
        for(int x=1;x<w-1;x++)
            if(mask.at<double>(y,x) != 0.0)
            {
                mp[y*w+x] = N;
                N++;
            }


    Mat_<double> A = Mat_<double>::zeros(N,N);
    Mat_<double> b1(N,1),b2(N,1),b3(N,1);
    Mat_<double> u1(N,1), u2(N,1), u3(N,1);

    bool o_x_is_neg = o_x < 0;
    bool o_y_is_neg = o_y < 0;
    o_x = ::abs(o_x);
    o_y = ::abs(o_y);

    Mat src_copy = Mat::zeros(src.rows,src.cols,CV_64FC3);
    for(int i=0;i<src_copy.rows-o_y;i++)
    {
        for(int j=0;j<src_copy.cols-o_x;j++)
        {
            if(!o_x_is_neg && !o_y_is_neg)
            {
                src_copy.at<double[3]>(i,j)[0] = (double)src.at<uchar[3]>(i+o_y,j+o_x)[0] / 255.0;
                src_copy.at<double[3]>(i,j)[1] = (double)src.at<uchar[3]>(i+o_y,j+o_x)[1] / 255.0;
                src_copy.at<double[3]>(i,j)[2] = (double)src.at<uchar[3]>(i+o_y,j+o_x)[2] / 255.0;
            } else if(o_y_is_neg && !o_x_is_neg)
            {
                src_copy.at<double[3]>(i+o_y,j)[0] = (double)src.at<uchar[3]>(i,j+o_x)[0] / 255.0;
                src_copy.at<double[3]>(i+o_y,j)[1] = (double)src.at<uchar[3]>(i,j+o_x)[1] / 255.0;
                src_copy.at<double[3]>(i+o_y,j)[2] = (double)src.at<uchar[3]>(i,j+o_x)[2] / 255.0;
            }else if(!o_y_is_neg && o_x_is_neg)
            {
                src_copy.at<double[3]>(i,j+o_x)[0] = (double)src.at<uchar[3]>(i+o_y,j)[0] / 255.0;
                src_copy.at<double[3]>(i,j+o_x)[1] = (double)src.at<uchar[3]>(i+o_y,j)[1] / 255.0;
                src_copy.at<double[3]>(i,j+o_x)[2] = (double)src.at<uchar[3]>(i+o_y,j)[2] / 255.0;
            } else {
                src_copy.at<double[3]>(i+o_y,j+o_x)[0] = (double)src.at<uchar[3]>(i,j)[0] / 255.0;
                src_copy.at<double[3]>(i+o_y,j+o_x)[1] = (double)src.at<uchar[3]>(i,j)[1] / 255.0;
                src_copy.at<double[3]>(i+o_y,j+o_x)[2] = (double)src.at<uchar[3]>(i,j)[2] / 255.0;
            }
        }
    }

    cv::Laplacian(src_copy,laplacian,-1);
    IplImage ipl = laplacian;
    cvShowImage("ugh",&ipl);

    int count = 0;
    uint id = 0;
    double bb[3];

    for(int y=1;y<h-1;y++)
    {
        for(int x=1;x<w-1;x++)
        {
            if(mask.at<double>(y,x) != 0.0)
            {
                id = y*w + x;
                bb[0] = (laplacian.at<double[3]>(y,x)[0]);
                bb[1] = (laplacian.at<double[3]>(y,x)[1]);
                bb[2] = (laplacian.at<double[3]>(y,x)[2]);

//                bb[0] = 0.0;
//                bb[1] = 0.0;
//                bb[2] = 0.0;
                if(mask.at<double>(y-1,x) != 0.0)
                {
                    A.at<double>(count,mp[id-w]) = 1.0;
                }
                else
                {                    
                    bb[0] -= (double)target.at<uchar[3]>(y-1,x)[0]/255.;
                    bb[1] -= (double)target.at<uchar[3]>(y-1,x)[1]/255.;
                    bb[2] -= (double)target.at<uchar[3]>(y-1,x)[2]/255.;
                }
                if(mask.at<double>(y,x-1) != 0.0)
                {
                    A.at<double>(count,mp[id-1]) = 1.0;
                }
                else
                {                    
                    bb[0] -= (double)target.at<uchar[3]>(y,x-1)[0]/255.;
                    bb[1] -= (double)target.at<uchar[3]>(y,x-1)[1]/255.;
                    bb[2] -= (double)target.at<uchar[3]>(y,x-1)[2]/255.;
                }

                A.at<double>(count,mp[id]) = -4.0;

                if(mask.at<double>(y,x+1) != 0.0)
                {
                    A.at<double>(count,mp[id+1]) = 1.0;
                }
                else
                {                    
                    bb[0] -= (double)target.at<uchar[3]>(y,x+1)[0]/255.;
                    bb[1] -= (double)target.at<uchar[3]>(y,x+1)[1]/255.;
                    bb[2] -= (double)target.at<uchar[3]>(y,x+1)[2]/255.;
                }
                if(mask.at<double>(y+1,x) != 0.0)
                {
                    A.at<double>(count,mp[id+w]) = 1.0;
                }
                else
                {                    
                    bb[0] -= (double)target.at<uchar[3]>(y+1,x)[0]/255.;
                    bb[1] -= (double)target.at<uchar[3]>(y+1,x)[1]/255.;
                    bb[2] -= (double)target.at<uchar[3]>(y+1,x)[2]/255.;
                }

                b1.at<double>(count,0) = (double)bb[0];
                b2.at<double>(count,0) = (double)bb[1];
                b3.at<double>(count,0) = (double)bb[2];
                count++;
            }
        }
    }

    solve(A,b1,u1);    
    solve(A,b2,u2);
    solve(A,b3,u3);

    count=0;
    for(int y=1;y<h-1;y++)
        for(int x=1;x<w-1;x++)
            if(mask.at<double>(y,x) != 0.0)
            {
                id = y*w + x;
                target.at<uchar[3]>(y,x)[0] = (uchar)(255.*clamp(u1(mp[id],0)));
                target.at<uchar[3]>(y,x)[1] = (uchar)(255.*clamp(u2(mp[id],0)));
                target.at<uchar[3]>(y,x)[2] = (uchar)(255.*clamp(u3(mp[id],0)));
                count++;
            }
}

void Utility::sampleGoodPoints(const vector<Point2f> &points, vector<Point2f> &sampledPoints)
{
    int lmci = Face::leftMouthCornerIndex;
    int rmci = Face::rightMouthCornerIndex;
    int tl = Face::topLipIndex;
    int bl = Face::bottomLipIndex;

    int lelc = Face::leftEyeBrow;
    int lerc = lelc+1;
    int relc = Face::rightEyeBrow;
    int rerc = relc+1;

    Utility::pointSampling(points[lmci],points[tl],10,sampledPoints);
    Utility::pointSampling(points[lmci],points[bl],10,sampledPoints);
    Utility::pointSampling(points[tl],points[rmci],10,sampledPoints);
    Utility::pointSampling(points[bl],points[rmci],10,sampledPoints);

    //Utility::pointSamplingNormal(points[tl],40,100,100,sampledPoints);

//    Utility::pointSampling(points[lelc],points[lerc],10,sampledPoints);
//    Utility::pointSampling(points[relc],points[rerc],10,sampledPoints);

    //Utility::pointSampling(points[17],points[18],10,sampledPoints);
    //Utility::pointSampling(points[17],points[19],10,sampledPoints);
}

static double random_0_1_unifrom(void)
{
    return double(rand()/double(RAND_MAX));
}

static double random_normal(void)
{
    double U1,U2,V1,V2;
    double S = 2;
    while(S>=1)
    {
        U1 = random_0_1_unifrom();
        U2 = random_0_1_unifrom();
        V1 = 2.0*U1 - 1.0;
        V2 = 2.0*U2 - 1.0;
        S = ::pow(V1,2) + pow(V2,2);
    }
    double X1 = V1*::sqrt((-2.0*::log(S))/S);
    return X1;
}

void Utility::pointSamplingNormal(const Point2f&a, const int num, const double var1, const double var2,
                                  vector<Point2f> &points)
{
    srand( (unsigned int)time(0) );

    Point2f newPoint;
    double n1,n2;
    for(int i=0;i<num;i++)
    {
        n1 = random_normal();
        n2 = random_normal();
        newPoint.x = a.x + n1*::sqrt(var1);
        newPoint.y = a.y + n2*::sqrt(var2);
        points.push_back(newPoint);
    }    
}


void Utility::pointSampling(const Point2f& a, const Point2f& b, const int num,
                                   vector<Point2f> &points)
{
    double random;
    double low = 0;
    double high = 100;

    srand( (unsigned int)time(0) );

    //now generate random numbers from range based on unifrom sampling dist formula
    //we look at it as a parametrization of the interval a,b
    //with a param based on rand() from interval 0 to 1
    //with the +1 coz it will never go to 1 .. coz it will never be rand_max
    //U = a + (b-a+1)*rand()/(1+RAND_MAX);

    Point2f newPoint;
    for(int i=0;i<num;i++)
    {
        random = low + (high - low + 1.0)*rand()/(1.0 + RAND_MAX);
        random /= 100;
        newPoint.x = (1-random)*a.x + random*b.x;
        newPoint.y = (1-random)*a.y + random*b.y;
        points.push_back(newPoint);
    }    
}

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
QImage Utility::mat2QImage(const Mat mat_rgb, QImage::Format f)
{
    QImage qframe;
    //converting to rgb not necessary it seems
    //cvtColor(mat_bgr, mat_rgb, CV_BGR2RGB);

    qframe = QImage((const unsigned char*)(mat_rgb.data), mat_rgb.cols,
    mat_rgb.rows, f);
    //rgb QImage::Format_RGB888
    return qframe;
}

QPixmap Utility::mat2QPixmap(const Mat &m, QImage::Format f)
{
    QImage img_q = Utility::mat2QImage(m,f);
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
