#include "optimizer.h"
#include <cstdio>
#include <cstdlib>

const int Optimizer::fPoints[20] = {9521,8899,310,7240,1183,8934,8945,6284,7140,8197,
                                    2080,2851,3580,6058,8825,1680,3907,8144,6540,2519};
const int Optimizer::fPoints_size = 20;

void Optimizer::generatePoints(const vector<Mat>&rotations, const vector<Mat>&translations,
                               const Mat& cameraMatrix, const Mat& lensDist,
                               int frame_number, Face *face_ptr,
                               vector<vector<Point2f> >&generatedPoints,
                               vector<vector<int> >&point_indices_for_frame,bool test)
{
    int random;
    double low = 0;
    double high = face_ptr->getPolyNum();
    vector<Point2f> imagePoints;
    vector<Point3f> objectPoints;
    vector<int> point_indices;

    Point3f p;
    Mat_<double> point3dMat(3,1);
    Mat_<double> point2dMat(3,1);
    Mat_<double> transpose;
    Mat_<double> rmatrix;

    //seed our pseudorandom number generator to give different numbers each time
    srand( (unsigned int)time(0) );

    //using the transformations generate a 1000 new points on the 2d image
    for(unsigned int j=0; j<frame_number; j++)
    {
        cout << "frame : " << j << endl;
        imagePoints.clear();
        objectPoints.clear();
        point_indices.clear();


        for(int i=0;i<this->fPoints_size;i++)
        {
            //now generate random numbers from range based on unifrom sampling dist formula
            //we look at it as a parametrization of the interval a,b
            //with a param based on rand() from interval 0 to 1
            //with the +1 coz it will never go to 1 .. coz it will never be rand_max
            //U = a + (b-a+1)*rand()/(1+RAND_MAX);
            random = low + (high - low + 1.0)*rand()/(1.0 + RAND_MAX);

            p = face_ptr->getPointFromPolygon(fPoints[i]);
            point_indices.push_back(face_ptr->getPointIndexFromPolygon(fPoints[i]));

            point3dMat(0,0) = p.x;
            point3dMat(1,0) = p.y;
            if(test == false)
                point3dMat(2,0) = p.z+1500.0;
            else
                point3dMat(2,0) = p.z;

            Rodrigues(rotations[j],rmatrix);
            transpose = translations[j];

            point2dMat = cameraMatrix*((rmatrix * point3dMat) + transpose);

            //homogenous coord
            if(test == false)
            {
                point2dMat(0,0) /= point2dMat(2,0);
                point2dMat(1,0) /= point2dMat(2,0);
            }

            //objectPoints.push_back(Point3f(p.x,p.y,p.z+1500.0));

            //cout << "generated new point : " << point2dMat(0,0) << " " << point2dMat(1,0) << endl;

            imagePoints.push_back(Point2f(point2dMat(0,0) , point2dMat(1,0)));
        }
        //projectPoints(Mat(objectPoints),frameRotation[j],frameTranslation[j],camera,lens,imagePoints);

        point_indices_for_frame.push_back(point_indices);
        generatedPoints.push_back(imagePoints);
    }
}

void Optimizer::estimatePose(const vector<vector<Point2f> > &featurePoints, Face *face_ptr,
                             const Mat &cameraMatrix, const Mat &lensDist,
                             vector<Mat> &rotations, vector<Mat> &translations)
{    
    bool useExt = false;
    Mat rvec, tvec, rvec_clone, tvec_clone;
    /**********************/
    /*estimate model parameters + pose*/
    /**********************/
    vector<vector<Point2f> >::const_iterator it2 = featurePoints.begin(),
                                       it2_end = featurePoints.end();
    //do not use the guess in the first frame but do for every following frame
    for(; it2 != it2_end; ++it2)
    {
        //determine pose based on the preselected feature points (fPoints)
        //the function also uses the current values in rvec and tvec as a guess
        //for the extrinsic parameters (if it isnt the first frame
        this->calculateTransformation( *it2, face_ptr, cameraMatrix, lensDist, rvec, tvec, useExt);
        rvec_clone = rvec.clone();
        tvec_clone = tvec.clone();
        rotations.push_back(rvec_clone);
        translations.push_back(tvec_clone);

        //useExt or true = useExt
        useExt |= true;
    }    
}

void Optimizer::calculateTransformation(const vector<Point2f> &imagePoints, Face *face_ptr,
                                        const Mat &cameraMatrix, const Mat &lensDist,
                                        Mat &rvec,Mat &tvec,bool useExt)
{ 
    vector<Point3f> objectPoints;

    cout << " size of image points " << imagePoints.size() << endl;

    Point3f p3;
    for(int i=0;i<this->fPoints_size;i++)
    {
        p3 = face_ptr->getPointFromPolygon(fPoints[i]);

        //cout << "point at polygon " << face_ptr->getPointIndexFromPolygon(fPoints[i]) << endl;
        p3.z += 1500.0;
        //cout << "point " << p3.x << " " << p3.y << " " << p3.z << endl;
        objectPoints.push_back(p3);
    }

    cout << "before solve pnp" << endl;
    //transform vectors into Mats with 3 (2) channels
    cv::solvePnP(Mat(objectPoints),Mat(imagePoints),cameraMatrix,lensDist,rvec,tvec,useExt);
    cout << "after solve pnp" << endl;
}
