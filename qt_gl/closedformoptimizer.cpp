#include "closedformoptimizer.h"
#include "FaceModel.h"

ClosedFormOptimizer::ClosedFormOptimizer() : max_iterations(1)
{    
}

void ClosedFormOptimizer::estimateParametersAndPose(const vector<Mat> &frames, const vector<vector<Point2f> > &featurePoints,
                                   const Mat &cameraMatrix, const Mat& lensDist, vector<Mat> &rotations, vector<Mat> &translations,
                                   vector<vector<double> >&weights_id, vector<vector<double> >&weights_ex,
                                   vector<vector<Point2f> > &generatedPoints)
{
    Mat_<double> weakCamera(2,3);
    //matrices for A_ex * w_ex = B_ex and A_id * w_id = B_id
    Mat_<double> A_ex , A_id;
    Mat_<double> B_ex, B_id;
    //matrix Z = kron(weights, Identity_matrix)
    Mat_<double> Z_ex, Z_id;
    Mat_<double> ZU;
    Mat_<double> PR, Pt;
    Mat_<double> W, WT;

    //core tensor rows
    vector<vector<Mat_<double> > > Mi;

    //featurePoints converted to 2x1 matrices
    vector<vector<Mat_<double> > >featurePointsMat;
    vector<Mat_<double> >helpVector;
    Mat_<double> helpMat;

    //initialize model-morph bases
    FaceModel *model = FaceModel::getInstance();
    Matrix core = model->getCoreTensor();
    Mat_<double> u_id = model->getUIdentity();
    Mat_<double> u_ex = model->getUExpression();
    //point indices
    vector<vector<int> >point_indices;

    int index = 0;

    const int exr_size = 7;
    const int id_size = 56;

    Mat_<double> rmatrix;
    Matrix x_t(1,exr_size);
    Matrix y_t(1,id_size);
    Matrix x(exr_size,1);
    Matrix y(id_size,1);
    //used for x_t*U_ex and y_t*U_id
    Matrix lin_comb_x(exr_size,1);
    Matrix lin_comb_y(id_size,1);

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
    estimatePose(featurePoints,face_ptr,cameraMatrix,lensDist,rotations,translations);

    //generate points to improve tracking
    generatePoints(rotations,translations,cameraMatrix,lensDist,FRAME_NUMBER,face_ptr,generatedPoints,point_indices);

    /***************************************************************/
    /*create weak-perspecitve camera matrix from camera matrix*/
    /***************************************************************/    
    for(int i=0;i<2;i++)
        for(int j=0;j<3;j++)
            weakCamera(i,j) = cameraMatrix.at<double>(i,j);

    //preprocess points and core tensor rows
    vector<vector<Point2f> >::const_iterator it = featurePoints.begin(),
                                       it_end = featurePoints.end();
    vector<Point2f>::const_iterator it_inside, it_inside_end;

    for(; it != it_end; ++it)
    {
        it_inside = (*it).begin();
        it_inside_end = (*it).end();
        helpVector.clear();
        for(; it_inside != it_inside_end; ++it_inside)
        {
            helpMat = Mat_<double>(2,1);
            helpMat(0,0) = (*it_inside).x;
            helpMat(1,0) = (*it_inside).y;
            helpVector.push_back(helpMat);
        }
        featurePointsMat.push_back(helpVector);
    }
    vector<vector<int> >::const_iterator it2 = point_indices.begin(),
                                        it2_end = point_indices.end();
    vector<int>::const_iterator it2_inside, it2_inside_end;

    for(; it2 != it2_end; ++it2)
    {
        it2_inside = (*it2).begin();
        it2_inside_end = (*it2).end();
        helpVector.clear();
        for(; it2_inside != it2_inside_end; ++it2_inside)
        {
            helpMat = Mat_<double>(3,exr_size*id_size);
            index = *it2_inside;
            helpMat = core.submatrix( index*3 , index*3 + 2 );
            helpVector.push_back(helpMat);
        }
        Mi.push_back(helpVector);
    }


    //do not use the guess in the first frame but do for every following frame
    for(int i=0;i<exr_size;i++)
        x_t[0][i] = x[i][0] = w_exp[i];
    for(int i=0;i<id_size;i++)
        y_t[0][i] = y[i][0] = w_id[i];


    for(int count = 0;count < max_iterations; count++)
    {

        //put the guesses into matrix y and x
        x.transpose(x_t);
        Matrix::matrix_mult(x_t,u_ex).transpose(lin_comb_x);
        y_t.transpose(y);

        for(unsigned int i=0; i<1; i++)
        {
            Rodrigues(rotations[i],rmatrix);
            Z_id = Matrix::kron(Matrix::eye(id_size),lin_comb_x);
            ZU = Z_id*(u_id.t());

            PR = weakCamera*rmatrix;
            Pt = (1/translations[i].at<double>(2,0)) * (weakCamera*translations[i]);

            A_id = Mat_<double>::zeros(Size(id_size,id_size));
            B_id = Mat_<double>::zeros(Size(1,id_size));

            /* compute the formula :
         * Sum( U*Z'*Mi'*R'*PW'*PW*R*Mi*Z*U' )*y = Sum( U*Z'*Mi'*R'*PW'*fi - (1/tz)*U*Z'*Mi'*R'*PW'*PW'*t )
         */
            for(unsigned int j=0;j<point_indices[i].size();j++)
            {
                W = PR*(Mi[i][j])*ZU;
                WT = W.t();
                A_id = A_id + WT*W;
                B_id = B_id + WT*(featurePointsMat[i][j] - Pt);
            }

            y = Matrix::solveLinSysSvd(A_id,B_id);

            cout << "this should work ... O_O : " << endl << x;
            cout << "in ID optim" << endl;
            Mat_<double> yM = y;

            for(unsigned int j=0;j<point_indices[i].size();j++)
            {
                W = PR*(Mi[i][j])*ZU*yM + Pt;
                Mat_<double> pp =  (Mi[i][j])*ZU*yM;
                cout << pp(0,0) << " " << pp(1,0) << " " << pp(2,0) << endl;
                cout << W(0,0) << " " << W(1,0) << " vs " << featurePointsMat[i][j](0,0) << " " << featurePointsMat[i][j](1,0) << endl;

            }
        }

        //put the guesses into matrix y and x
        y.transpose(y_t);
        Matrix::matrix_mult(y_t,u_id).transpose(lin_comb_y);
        x_t.transpose(x);

        //TODO smaller coz its too slow
        for(unsigned int i=0; i<1; i++)
        {
            Rodrigues(rotations[i],rmatrix);
            Z_ex = Matrix::kron(lin_comb_y,Matrix::eye(exr_size));
            ZU = Z_ex*(u_ex.t());

            PR = weakCamera*rmatrix;
            Pt = (1/translations[i].at<double>(2,0)) * (weakCamera*translations[i]);

            A_ex = Mat_<double>::zeros(Size(exr_size,exr_size));
            B_ex = Mat_<double>::zeros(Size(1,exr_size));

            /* compute the formula :
         * Sum( U*Z'*Mi'*R'*PW'*PW*R*Mi*Z*U' )*x = Sum( U*Z'*Mi'*R'*PW'*fi - (1/tz)*U*Z'*Mi'*R'*PW'*PW'*t )
         */
            for(unsigned int j=0;j<point_indices[i].size();j++)
            {
                W = PR*(Mi[i][j])*ZU;
                WT = W.t();
                A_ex = A_ex + WT*W;
                B_ex = B_ex + WT*(featurePointsMat[i][j] - Pt);
            }

            x = Matrix::solveLinSysSvd(A_ex,B_ex);
            cout << "in EX optim" << endl;
            Mat_<double> xM = x;

            for(unsigned int j=0;j<point_indices[i].size();j++)
            {
                W = PR*(Mi[i][j])*ZU*xM + Pt;
                Mat_<double> pp =  (Mi[i][j])*ZU*xM;
                cout << pp(0,0) << " " << pp(1,0) << " " << pp(2,0) << endl;
                cout << W(0,0) << " " << W(1,0) << " vs " << featurePointsMat[i][j](0,0) << " " << featurePointsMat[i][j](1,0) << endl;

            }

        }        
    }

    for(int i=0;i<exr_size;i++){
        w_exp[i] = x[i][0];
        cout << w_exp[i] << " ";
    }
    cout << endl;

    for(int i=0;i<id_size;i++){
        w_id[i] = y[i][0];
        cout << w_id[i] << " ";
    }
    cout << endl;

    face_ptr->interpolate(w_id,w_exp);

    //here you should clear generated points
    generatedPoints.clear();
    generatePoints(rotations,translations,weakCamera,lensDist,FRAME_NUMBER,face_ptr,generatedPoints,point_indices,true);

    delete face_ptr;
    delete[] w_id;
    delete[] w_exp;
}
