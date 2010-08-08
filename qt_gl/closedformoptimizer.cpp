#include "closedformoptimizer.h"

ClosedFormOptimizer::ClosedFormOptimizer()
{
}


void ClosedFormOptimizer::estimateParametersAndPose(const vector<Mat> &frames, const vector<vector<Point2f> > &featurePoints,
                                   const Mat &cameraMatrix, const Mat& lensDist, vector<Mat> &rotations, vector<Mat> &translations,
                                   vector<vector<double> >&w_id, vector<vector<double> >&w_ex,
                                   vector<vector<Point2f> > &generatedPoints)
{
    Mat_<double> weakCamera(2,3);
    //matrices for A_ex * w_ex = B_ex and A_id * w_id = B_id
    Mat_<double> A_ex , A_id;
    Mat_<double> B_ex, B_ex;
    //matrix Z = kron(weights, Identity_matrix)
    Mat_<double> Z_ex, Z_id;

    const unsigned int FRAME_NUMBER = frames.size();
    Face *face_ptr = new Face();
    double *w_id = new double[56];
    double *w_exp = new double[7];


    //make a face guess
    for(int i=0;i<56;i++)
    {
        if(i==33)w_id[i] = 0.1;
        else if(i==7)w_id[i] = 0.8;
        else if(i==20)w_id[i] = 0.1;
        else w_id[i] = 0;
    }
    w_exp[0] = 0.0;
    w_exp[1] = 0.0;
    w_exp[2] = 0.0;
    w_exp[3] = 0.2;
    w_exp[4] = 0.0;
    w_exp[5] = 0.8;
    w_exp[6] = 0.0;

    face_ptr->interpolate(w_id,w_exp);

    //estimate the pose parameters and place estimations into vectors rotations and translations
    //the rotations vector holds the rodrigues rotation vectors which can be converted to a rotation matrix
    this->estimatePose(featurePoints,face_ptr,cameraMatrix,lensDist,rotations,translations);

    /***************************************************************/
    /*create weak-perspecitve camera matrix from camera matrix*/
    /***************************************************************/    
    for(int i=0;i<2;i++)
        for(int j=0;j<3;j++)
            weakCamera(i,j) = cameraMatrix.at<double>(i,j);


    id.assign(w_id,w_id+56);
    ex.assign(w_exp,w_exp+7);
    //sizeof(w_exp)/sizeof(double)
    //first expression
    //TODO smaller coz its too slow
    for(unsigned int i=0; i<1; i++)
    {
        Rodrigues(rotations[i],rmatrix);
        Z_ex = Matrix::kron(Matrix(id).transpose(),Matrix::eye(7));

    }
    //then identity using the expression guess
    for(unsigned int i=0; i<1; i++)
    {
        Rodrigues(rotations[i],rmatrix);
        Z_id = Matrix::kron(Matrix::eye(56),Matrix(ex).transpose());

    }

    delete face_ptr;
    delete[] w_id;
    delete[] w_exp;



}
