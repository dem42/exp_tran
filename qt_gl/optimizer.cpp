#include "optimizer.h"
#include <cstdio>
#include <cstdlib>
#include "FaceModel.h"

void Optimizer::generatePoints(const Mat &rotation, const Mat &translation,
                               const Mat& cameraMatrix, const Mat& lensDist,
                               int frame_number, Face *face_ptr,
                               vector<Point2f> &generatedPoints,
                               vector<int> &point_indices, bool newPoints ,bool test)
{
    int random;
    double low = 0;
    double high = face_ptr->getPolyNum();
    cout << "HIGH is : " << high << endl;

    vector<Point3f> objectPoints;

    Point3f p;
    Mat_<double> point3dMat(3,1);
    Mat_<double> point2dMat(3,1);
    Mat_<double> transpose;
    Mat_<double> rmatrix;

    //seed our pseudorandom number generator to give different numbers each time
    srand( (unsigned int)time(0) );

    int MAX = Face::fPoints_size;
    if(newPoints == true)
        MAX = 500;
    for(int i=0;i<MAX;i++)
    {
        //now generate random numbers from range based on unifrom sampling dist formula
        //we look at it as a parametrization of the interval a,b
        //with a param based on rand() from interval 0 to 1
        //with the +1 coz it will never go to 1 .. coz it will never be rand_max
        //U = a + (b-a+1)*rand()/(1+RAND_MAX);
        random = low + (high - low + 1.0)*rand()/(1.0 + RAND_MAX);

        if(newPoints == false)
        {
            cout << "omg nice bday" << endl;
            p = face_ptr->getPointFromPolygon(Face::fPoints[i]);
            point_indices.push_back(face_ptr->getPointIndexFromPolygon(Face::fPoints[i]));
        }
        else
        {
            cout << " yeah thnx " << endl;
            p = face_ptr->getPointFromPolygon(random);
            point_indices.push_back(face_ptr->getPointIndexFromPolygon(random));
        }

        point3dMat(0,0) = p.x;
        point3dMat(1,0) = p.y;
        point3dMat(2,0) = p.z;

        Rodrigues(rotation,rmatrix);
        transpose = translation;

        if(test == false)
        {

            point2dMat = cameraMatrix*((rmatrix * point3dMat) + transpose);

            //homogenous coord

            point2dMat(0,0) /= point2dMat(2,0);
            point2dMat(1,0) /= point2dMat(2,0);
        }
        else
        {
            Mat_<double> Pt = (1./transpose(0,2))*(cameraMatrix*transpose);
            point2dMat = cameraMatrix*(rmatrix * point3dMat) + Pt;
        }


        cout << "feature point : " << point3dMat(0,0) << " " << point3dMat(1,0) << " " << point3dMat(2,0) << endl;
        cout << "generated new point : " << point2dMat(0,0) << " " << point2dMat(1,0) << endl;

        generatedPoints.push_back(Point2f(point2dMat(0,0) , point2dMat(1,0)));
    }
}


void Optimizer::calculateTransformation(const vector<Point2f> &imagePoints, Face *face_ptr,
                                        const Mat &cameraMatrix, const Mat &lensDist,
                                        Mat &rvec,Mat &tvec,bool useExt)
{ 
    vector<Point3f> objectPoints;

    cout << " size of image points " << imagePoints.size() << endl;

    Point3f p3;
    for(int i=0;i<Face::fPoints_size;i++)
    {
        p3 = face_ptr->getPointFromPolygon(Face::fPoints[i]);

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
