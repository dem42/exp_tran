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
    for(int i=0;i<Face::mouth_size;i++)
    {
        correspondence3d.push_back(face_ptr->getPointIndexFromPolygon(Face::mouth[i]));
        obj.push_back(face_ptr->getPointFromPolygon(Face::mouth[i]));
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
    cout << " with a z_avg after projecting " << Z_avg << endl;

    //reverse weak perspective (x,y,1) = (1/zavg)*(fx*X,fy*Y,zavg) + (c_x,c_y,0)
    //to get (X,Y,zavg) = (  zavg*[x-c_x]/fx, zavg*[y-c_y]/fy , zavg)
    //the entire projection to reverse is x = PwRX + Pt
    for(unsigned int i=0;i<imagePoints.size();i++)
    {
//        cout << "things " << zavg << " " << principalPoint(0,0) <<  " " << principalPoint(1,0)
//             << " " << cameraMatrix.at<double>(0,0) << " " << cameraMatrix.at<double>(1,1) << endl;
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

        cout << "2d point is " << imagePoints[i].x << " " << imagePoints[i].y << endl;
        cout << "3d point is " << point3dMat(0,0) << " " << point3dMat(1,0) << " " << point3dMat(2,0) << endl;

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
        cout << "zavg : " << zavg << " point z : " << point2dMat(2,0) << endl;
        point2dMat  = (1./zavg)*point2dMat;

        cout << "feature point : " << point3dMat(0,0) << " " << point3dMat(1,0) << " " << point3dMat(2,0) << endl;
        cout << "generated new point : " << point2dMat(0,0) << " " << point2dMat(1,0) << endl;

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

    cout << " size of image points " << imagePoints.size() << endl;

    Point3f p3;
    for(unsigned int i=0;i<indices.size();i++)
    {
        p3 = face_ptr->getPoint(indices[i]);

        //cout << "point at polygon " << face_ptr->getPointIndexFromPolygon(fPoints[i]) << endl;

        cout << "point at " << indices[i] << " is " <<  p3.x << " " << p3.y << " " << p3.z << endl;
        objectPoints.push_back(p3);
    }
    MatConstIterator_<double> it = cameraMatrix.begin<double>(), it_end = cameraMatrix.end<double>();
    for(; it != it_end; ++it)
        cout << *it << " ";
    cout << endl;
    vector<Point2f>::const_iterator itp = imagePoints.begin(), itp_end = imagePoints.end();
    for(; itp != itp_end; ++itp)
        cout << (*itp).x << " , " << (*itp).y << " ";
    cout << endl;

    cout << "use ext is : " << useExt << endl;
    cout << "before solve pnp" << endl;
    //transform vectors into Mats with 3 (2) channels
    cv::solvePnP(Mat(objectPoints),Mat(imagePoints),cameraMatrix,lensDist,rvec,tvec,useExt);
    cout << "after solve pnp" << endl;
}
