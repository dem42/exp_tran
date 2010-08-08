#include "optimizer.h"


const int Optimizer::fPoints[20] = {9521,8899,310,7240,1183,8934,8945,6284,7140,8197,
                                    2080,2851,3580,6058,8825,1680,3907,8144,6540,2519};
const int Optimizer::fPoints_size = 20;


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

        cout << "point at polygon " << face_ptr->getPointIndexFromPolygon(fPoints[i]) << endl;
        p3.z += 1500.0;
        cout << "point " << p3.x << " " << p3.y << " " << p3.z << endl;
        objectPoints.push_back(p3);
    }

    cout << "before solve pnp" << endl;
    //transform vectors into Mats with 3 (2) channels
    cv::solvePnP(Mat(objectPoints),Mat(imagePoints),cameraMatrix,lensDist,rvec,tvec,useExt);
    cout << "after solve pnp" << endl;
}
