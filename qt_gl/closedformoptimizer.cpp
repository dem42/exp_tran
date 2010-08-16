#include "closedformoptimizer.h"
#include "FaceModel.h"

ClosedFormOptimizer::ClosedFormOptimizer() : max_iterations(1)
{    
}

void ClosedFormOptimizer::estimateModelParameters(const Mat &frame, const vector<Point2f> &featurePoints,
                                                  const Mat &cameraMatrix, const Mat& lensDist,
                                                  Face* face_ptr,const vector<int> &point_indices,
                                                  const Mat &rotation, const Mat &translation,
                                                  vector<double> &weights_id, vector<double> &weights_ex)
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
    vector<Mat_<double> > Mi;

    //featurePoints converted to 2x1 matrices
    vector<Mat_<double> >featurePointsMat;    
    Mat_<double> helpMat;

    //initialize model-morph bases
    FaceModel *model = FaceModel::getInstance();
    Matrix core = model->getCoreTensor();
    Mat_<double> u_id = model->getUIdentity();
    Mat_<double> u_ex = model->getUExpression();

    int index = 0;

    const int exr_size = 7;
    const int id_size = 56;

    double *w_id = new double[id_size];
    double *w_exp = new double[exr_size];

    Mat_<double> rmatrix;
    Matrix x_t(1,exr_size);
    Matrix y_t(1,id_size);
    Matrix x(exr_size,1);
    Matrix y(id_size,1);
    //used for x_t*U_ex and y_t*U_id
    Matrix lin_comb_x(exr_size,1);
    Matrix lin_comb_y(id_size,1);

    //get weights from the current face instance
    face_ptr->getWeights(w_id,id_size,w_exp,exr_size);

    /***************************************************************/
    /*create weak-perspecitve camera matrix from camera matrix*/
    /***************************************************************/    
    for(int i=0;i<2;i++)
        for(int j=0;j<3;j++)
            weakCamera(i,j) = cameraMatrix.at<double>(i,j);

    //preprocess points and core tensor rows

    vector<Point2f>::const_iterator it_inside = featurePoints.begin(),
                                    it_inside_end = featurePoints.end();

    for(; it_inside != it_inside_end; ++it_inside)
    {
        helpMat = Mat_<double>(2,1);
        helpMat(0,0) = (*it_inside).x;
        helpMat(1,0) = (*it_inside).y;        
        featurePointsMat.push_back(helpMat);
    }


    vector<int>::const_iterator it2_inside = point_indices.begin(),
                                it2_inside_end = point_indices.end();


    for(; it2_inside != it2_inside_end; ++it2_inside)
    {
        helpMat = Mat_<double>(3,exr_size*id_size);
        index = *it2_inside;
        cout << index << endl;
        helpMat = core.submatrix( index*3 , index*3 + 2 );
        Mi.push_back(helpMat);
    }

    //do not use the guess in the first frame but do for every following frame
    for(int i=0;i<exr_size;i++)
        x_t[0][i] = x[i][0] = w_exp[i];
    for(int i=0;i<id_size;i++)
        y_t[0][i] = y[i][0] = w_id[i];


    for(int count = 0;count < max_iterations; count++)
    {        
        //put the guesses into matrix y and x
        y.transpose(y_t);
        Matrix::matrix_mult(y_t,u_id).transpose(lin_comb_y);
        x_t.transpose(x);


        Rodrigues(rotation,rmatrix);
        Z_ex = Matrix::kron(lin_comb_y,Matrix::eye(exr_size));
        ZU = Z_ex*(u_ex.t());

        PR = weakCamera*rmatrix;
        Pt = (1/translation.at<double>(2,0)) * (weakCamera*translation);

        A_ex = Mat_<double>::zeros(Size(exr_size,exr_size));
        B_ex = Mat_<double>::zeros(Size(1,exr_size));

        /* compute the formula :
         * Sum( U*Z'*Mi'*R'*PW'*PW*R*Mi*Z*U' )*x = Sum( U*Z'*Mi'*R'*PW'*fi - (1/tz)*U*Z'*Mi'*R'*PW'*PW'*t )
         */
        for(unsigned int j=0;j<point_indices.size();j++)
        {
            W = PR*(Mi[j])*ZU;
            WT = W.t();
            A_ex = A_ex + WT*W;
            B_ex = B_ex + WT*(featurePointsMat[j] - Pt);
        }

        x = Matrix::solveLinSysSvd(A_ex,B_ex);
        cout << "in EX optim" << endl;
        Mat_<double> xM = x;

        for(unsigned int j=0;j<point_indices.size();j++)
        {
            W = PR*(Mi[j])*ZU*xM + Pt;
            Mat_<double> pp =  (Mi[j])*ZU*xM;
            cout << pp(0,0) << " " << pp(1,0) << " " << pp(2,0) << endl;
            cout << W(0,0) << " " << W(1,0) << " vs " << featurePointsMat[j](0,0) << " " << featurePointsMat[j](1,0) << endl;

        }
        //put the guesses into matrix y and x
        x.transpose(x_t);
        Matrix::matrix_mult(x_t,u_ex).transpose(lin_comb_x);
        y_t.transpose(y);


        Rodrigues(rotation,rmatrix);
        Z_id = Matrix::kron(Matrix::eye(id_size),lin_comb_x);
        ZU = Z_id*(u_id.t());

        PR = weakCamera*rmatrix;
        Pt = (1/translation.at<double>(2,0)) * (weakCamera*translation);

        A_id = Mat_<double>::zeros(Size(id_size,id_size));
        B_id = Mat_<double>::zeros(Size(1,id_size));

        /* compute the formula :
         * Sum( U*Z'*Mi'*R'*PW'*PW*R*Mi*Z*U' )*y = Sum( U*Z'*Mi'*R'*PW'*fi - (1/tz)*U*Z'*Mi'*R'*PW'*PW'*t )
         */
        cout << "here " << point_indices.size() << endl;
        for(unsigned int j=0;j<point_indices.size();j++)
        {
            W = PR*(Mi[j])*ZU;
            WT = W.t();
            A_id = A_id + WT*W;
            cout << featurePointsMat[j](0,0) << " " << featurePointsMat[j](1,0)  << endl;
            B_id = B_id + WT*(featurePointsMat[j] - Pt);
        }

        y = Matrix::solveLinSysSvd(A_id,B_id);

        cout << "this should work ... O_O : " << endl << x;
        cout << "in ID optim" << endl;
        Mat_<double> yM = y;

        for(unsigned int j=0;j<point_indices.size();j++)
        {
            W = PR*(Mi[j])*ZU*yM + Pt;
            Mat_<double> pp =  (Mi[j])*ZU*yM;
            cout << pp(0,0) << " " << pp(1,0) << " " << pp(2,0) << endl;
            cout << W(0,0) << " " << W(1,0) << " vs " << featurePointsMat[j](0,0) << " " << featurePointsMat[j](1,0) << endl;

        }
    }

    Vector3::normalize(w_exp, exr_size);
    Vector3::normalize(w_id, id_size);

    for(int i=0;i<exr_size;i++){
        w_exp[i] = x[i][0];
        weights_ex.push_back(w_exp[i]);
        cout << w_exp[i] << " == " << weights_ex[i] << " ";
    }
    cout << endl;

    for(int i=0;i<id_size;i++){
        w_id[i] = y[i][0];
        weights_id.push_back(w_id[i]);
        cout << w_id[i] << " ";
    }
    cout << endl;

    face_ptr->interpolate(w_id,w_exp);

    delete[] w_id;
    delete[] w_exp;
}
