#include "poseestimator.h"
#include <cstdio>
#include <cstdlib>
#include "model/FaceModel.h"

PoseEstimator::PoseEstimator()
{
}

void PoseEstimator::projectModelPointsInto2D(const Mat &rotation, const Mat &translation,
                                             const Mat& cameraMatrix, const Mat& lensDist,
                                             Face *face_ptr, vector<int> &correspondence3d,
                                             vector<Point2f> &imagePoints)
{
    vector<Point3f> obj;
    /*mouth*/
    for(int i=0;i<Face::mouth_size;i++)
    {
        correspondence3d.push_back(face_ptr->getPointIndexFromPolygon(Face::mouth[i]));
        obj.push_back(face_ptr->getPointFromPolygon(Face::mouth[i]));
    }
    for(int i=0;i<Face::brow;i++)
    {
        correspondence3d.push_back(Face::lEyeB[i]);
        obj.push_back(face_ptr->getPoint(Face::lEyeB[i]));
        correspondence3d.push_back(Face::rEyeB[i]);
        obj.push_back(face_ptr->getPoint(Face::rEyeB[i]));
    }

    projectPoints(Mat(obj),rotation,translation,cameraMatrix,lensDist,imagePoints);
}



void PoseEstimator::reprojectInto3DUsingWeak(const vector<Point2f> &imagePoints,
                                             const Mat &rotation, const Mat &translation,
                                             const Mat& cameraMatrix, const Mat& lensDist,
                                             Face *face_ptr, vector<int> &correspondence3d)
{
    Point3 p;
    Mat_<double> point3dMat(3,1);
    Mat_<double> point2dMat(3,1);
    Mat_<double> principalPoint(3,1);
    Mat_<double> tranM;
    Mat_<double> rmatrix;
    Mat_<double> proP(3,1), pM(3,1);
    double zavg = face_ptr->getAverageDepth();
    double Z_avg;
    int index;

    Rodrigues(rotation,rmatrix);
    tranM = translation;

    principalPoint(0,0) = cameraMatrix.at<double>(0,2);
    principalPoint(1,0) = cameraMatrix.at<double>(1,2);
    principalPoint(2,0) = 0.0;

    //obtain the zavg by projecting any point with avg coord
    pM(0,0) = 1;
    pM(1,0) = 1;
    pM(2,0) = zavg;    
    proP = cameraMatrix*rmatrix*pM;
    proP = proP + cameraMatrix*tranM;
    Z_avg = proP(0,2);

    //reverse weak perspective (x,y,1) = (1/zavg)*(fx*X,fy*Y,zavg) + (c_x,c_y,0)
    //to get (X,Y,zavg) = (  zavg*[x-c_x]/fx, zavg*[y-c_y]/fy , zavg)
    //the entire projection to reverse is x = PwRX + Pt
    for(unsigned int i=0;i<imagePoints.size();i++)
    {

        point3dMat(0,0) = imagePoints[i].x - principalPoint(0,0);
        point3dMat(1,0) = imagePoints[i].y - principalPoint(1,0);

        point3dMat(0,0) *= (Z_avg / cameraMatrix.at<double>(0,0));
        point3dMat(1,0) *= (Z_avg / cameraMatrix.at<double>(1,1));
        point3dMat(2,0) = Z_avg;

        //cout << "after reverse persp " << point3dMat(0,0) << " " << point3dMat(1,0) << " " << point3dMat(2,0) << endl;

        //subtract translation
        point3dMat = point3dMat - tranM;

        //cout << "after trans " << point3dMat(0,0) << " " << point3dMat(1,0) << " " << point3dMat(2,0) << endl;
        //reverse rotation which is orthogonal
        point3dMat = rmatrix.t()*point3dMat;


        p.x = point3dMat(0,0);
        p.y = point3dMat(1,0);
        p.z = point3dMat(2,0);

        index = face_ptr->closestPointIndexForPoint(p);

        correspondence3d.push_back(index);
    }
}


void PoseEstimator::generatePoints(const Mat &rotation, const Mat &translation,
                               const Mat& cameraMatrix, const Mat& lensDist,
                               int pnumber, Face *face_ptr,
                               vector<Point2f> &generatedPoints,
                               vector<int> &point_indices)
{
    int random;
    double low = 0;
    double high = face_ptr->getPointNum();

    vector<Point3f> objectPoints;

    Point3f p;
    Mat_<double> point3dMat(3,1);
    Mat_<double> point2dMat(3,1);
    Mat_<double> tranM;
    Mat_<double> rmatrix;

    Rodrigues(rotation,rmatrix);
    tranM = translation;
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

        point2dMat = cameraMatrix*((rmatrix * point3dMat) + tranM);

        //homogenous coord
        if(zavg == 0)
            zavg = point2dMat(2,0);

        point2dMat  = (1./zavg)*point2dMat;

        generatedPoints.push_back(Point2f(point2dMat(0,0) , point2dMat(1,0)));
    }
}

void PoseEstimator::weakPerspectiveProjectPoints(const Mat &rotation, const Mat &translation,
                                             const Mat& cameraMatrix, const Mat& lensDist,
                                             const vector<int> &point_indices, Face *face_ptr,
                                             vector<Point2f> &projectedPoints)
{
    Point3f p;
    Mat_<double> point3dMat(3,1);
    Mat_<double> point2dMat(3,1);
    Mat_<double> tranM;
    Mat_<double> rmatrix;

    Rodrigues(rotation,rmatrix);
    tranM = translation;

    for(unsigned int i=0;i<point_indices.size();i++)
    {
        p = face_ptr->getPoint(point_indices[i]);

        point3dMat(0,0) = p.x;
        point3dMat(1,0) = p.y;
        point3dMat(2,0) = face_ptr->getAverageDepth();

        //homogenous and dehomogenized with zavg
        point2dMat = cameraMatrix*((rmatrix * point3dMat) + tranM);
        point2dMat  = (1./point2dMat(2,0))*point2dMat;

        projectedPoints.push_back(Point2f(point2dMat(0,0) , point2dMat(1,0)));
    }
}

void PoseEstimator::calculateTransformation(const vector<Point2f> &imagePoints, Face *face_ptr,
                                        const Mat &cameraMatrix, const Mat &lensDist,
                                        const vector<int> &indices,
                                        Mat &rvec,Mat &tvec,bool useExt)
{
    vector<Point3f> objectPoints;

    Point3f p3;
    for(unsigned int i=0;i<indices.size();i++)
    {
        p3 = face_ptr->getPoint(indices[i]);

        objectPoints.push_back(p3);
    }

    //transform vectors into Mats with 3 (2) channels
    cv::solvePnP(Mat(objectPoints),Mat(imagePoints),cameraMatrix,lensDist,rvec,tvec,useExt);
}
