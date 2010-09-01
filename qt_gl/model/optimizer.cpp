#include "optimizer.h"
#include <cstdio>
#include <cstdlib>
#include "model/FaceModel.h"

void Optimizer::generatePoints(const Mat &rotation, const Mat &translation,
                               const Mat& cameraMatrix, const Mat& lensDist,
                               int pnumber, Face *face_ptr,
                               vector<Point2f> &generatedPoints,
                               vector<int> &point_indices)
{
    int random;
    double low = 0;
    double high = face_ptr->getPointNum();
    cout << "HIGH is : " << high << endl;

    vector<Point3f> objectPoints;

    Point3f p;
    Mat_<double> point3dMat(3,1);
    Mat_<double> point2dMat(3,1);
    Mat_<double> transpose;
    Mat_<double> rmatrix;

    //seed our pseudorandom number generator to give different numbers each time
    srand( (unsigned int)time(0) );

    double zavg = 0;
    int MAX = pnumber;
    for(int i=0;i<MAX;i++)
    {
        //now generate random numbers from range based on unifrom sampling dist formula
        //we look at it as a parametrization of the interval a,b
        //with a param based on rand() from interval 0 to 1
        //with the +1 coz it will never go to 1 .. coz it will never be rand_max
        //U = a + (b-a+1)*rand()/(1+RAND_MAX);
        random = low + (high - low + 1.0)*rand()/(1.0 + RAND_MAX);

        p = face_ptr->getPoint(random);
        point_indices.push_back(random);

        point3dMat(0,0) = p.x;
        point3dMat(1,0) = p.y;
        point3dMat(2,0) = face_ptr->getAverageDepth();

        Rodrigues(rotation,rmatrix);
        transpose = translation;

        point2dMat = cameraMatrix*((rmatrix * point3dMat) + transpose);

        //homogenous coord
        if(zavg == 0)
            zavg = point2dMat(2,0);
        cout << "zavg : " << zavg << " point z : " << point2dMat(2,0) << endl;
        point2dMat  = (1./zavg)*point2dMat;

        cout << "feature point : " << point3dMat(0,0) << " " << point3dMat(1,0) << " " << point3dMat(2,0) << endl;
        cout << "generated new point : " << point2dMat(0,0) << " " << point2dMat(1,0) << endl;

        generatedPoints.push_back(Point2f(point2dMat(0,0) , point2dMat(1,0)));
    }
}

void Optimizer::weakPerspectiveProjectPoints(const Mat &rotation, const Mat &translation,
                                             const Mat& cameraMatrix, const Mat& lensDist,
                                             const vector<int> &point_indices, Face *face_ptr,
                                             vector<Point2f> &projectedPoints)
{
    Point3f p;
    Mat_<double> point3dMat(3,1);
    Mat_<double> point2dMat(3,1);
    Mat_<double> transpose;
    Mat_<double> rmatrix;

    for(unsigned int i=0;i<point_indices.size();i++)
    {        
        p = face_ptr->getPoint(point_indices[i]);

        point3dMat(0,0) = p.x;
        point3dMat(1,0) = p.y;
        point3dMat(2,0) = face_ptr->getAverageDepth();

        Rodrigues(rotation,rmatrix);
        transpose = translation;

        //homogenous and dehomogenized with zavg
        point2dMat = cameraMatrix*((rmatrix * point3dMat) + transpose);
        point2dMat  = (1./point2dMat(2,0))*point2dMat;

        projectedPoints.push_back(Point2f(point2dMat(0,0) , point2dMat(1,0)));
    }
}

void Optimizer::calculateTransformation(const vector<Point2f> &imagePoints, Face *face_ptr,
                                        const Mat &cameraMatrix, const Mat &lensDist,
                                        const vector<int> &indices,
                                        Mat &rvec,Mat &tvec,bool useExt)
{ 
    vector<Point3f> objectPoints;

    cout << " size of image points " << imagePoints.size() << endl;

    Point3f p3;
    for(int i=0;i<indices.size();i++)
    {
        p3 = face_ptr->getPoint(indices[i]);

        //cout << "point at polygon " << face_ptr->getPointIndexFromPolygon(fPoints[i]) << endl;

        //cout << "point " << p3.x << " " << p3.y << " " << p3.z << endl;
        objectPoints.push_back(p3);
    }
    MatConstIterator_<double> it = cameraMatrix.begin<double>(), it_end = cameraMatrix.end<double>();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;
    cout << "before solve pnp" << endl;
    //transform vectors into Mats with 3 (2) channels
    cv::solvePnP(Mat(objectPoints),Mat(imagePoints),cameraMatrix,lensDist,rvec,tvec,useExt);
    cout << "after solve pnp" << endl;
}
