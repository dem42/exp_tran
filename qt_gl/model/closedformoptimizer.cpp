#include "closedformoptimizer.h"
#include "model/FaceModel.h"

ClosedFormOptimizer::ClosedFormOptimizer() : max_iterations(4)
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
    Mat_<double> PR, PRT, pr, Pt, PRM;
    Mat_<double> W, WT;

    //core tensor rows
    Mat_<double> Mi;
    Mat_<double> M;

    //featurePoints converted to 2x1 matrices    
    Mat_<double> fi;
    Mat_<double> f;
    Mat_<double> O;

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

    //regularization term
    Mat_<double> regTerm_id, regTerm_exp, leftTerm_id, leftTerm_exp;
    const double regParam = 2000.8;

    //size is Size(width,height)
    regTerm_exp = Mat_<double>::eye(Size(exr_size,exr_size));
    regTerm_id = Mat_<double>::eye(Size(id_size,id_size));
    leftTerm_exp = Mat_<double>::eye(Size(1,exr_size));
    leftTerm_id = Mat_<double>::eye(Size(1,id_size));

    //init the left terms which are made up of regParam*mean
    //mean is 1./size for both since theres exactly same amount of
    //faces with 1s at those spots and mean is in between all of them (? not sure)
    for(int i=0;i<exr_size;i++)
        leftTerm_exp(0,i) = 1./exr_size;
    for(int i=0;i<id_size;i++)
        leftTerm_id(0,i) = 1./id_size;
    leftTerm_exp = regParam*leftTerm_exp;
    leftTerm_id = regParam*leftTerm_id;

    regTerm_exp = regParam*regTerm_exp;
    regTerm_id = regParam*regTerm_id;

    double Z_avg;
    double average_depth;
    Point3f p;
    Mat_<double> proP;
    Mat_<double> pM(3,1);

    cout << "huh" << endl;
    Rodrigues(rotation,rmatrix);

    //get weights from the current face instance
    face_ptr->getWeights(w_id,id_size,w_exp,exr_size);
    p = face_ptr->getPointFromPolygon(7403);
    average_depth = face_ptr->getAverageDepth();
    cout << "huh2 avg depth : " << face_ptr->getAverageDepth() << endl;
    pM(0,0) = p.x;
    pM(1,0) = p.y;
    pM(2,0) = face_ptr->getAverageDepth();
    cout << "face is at a distance " << p.z << endl;
    proP = cameraMatrix*rmatrix*pM;
    proP = proP + cameraMatrix*translation;
    Z_avg = proP(0,2);


    //get weights from the current face instance
    face_ptr->getWeights(w_id,id_size,w_exp,exr_size);

    /***************************************************************/
    /*create weak-perspecitve camera matrix from camera matrix*/
    /***************************************************************/    
    for(int i=0;i<2;i++)
        for(int j=0;j<3;j++)
            weakCamera(i,j) = cameraMatrix.at<double>(i,j);

    f = Mat_<double>(featurePoints.size()*2,1);
    fi = Mat_<double>(2,1);

    //precompute translation
    Pt = (1/translation.at<double>(2,0)) * (weakCamera*translation);

    //preprocess points and core tensor rows
    //for points subtract translation too
    for(unsigned int i=0;i<featurePoints.size();i++)
    {        
        fi = Mat_<double>(2,1);
        fi(0,0) = featurePoints[i].x;
        fi(1,0) = featurePoints[i].y;
        fi = fi - Pt;     
        f.row(2*i) = fi.row(0) + 0;
        f.row(2*i+1) = fi.row(1) + 0;     
    }

    M = Mat_<double>(3*featurePoints.size(),exr_size*id_size);
    Mi = Mat_<double>(3,exr_size,id_size);

    for(unsigned int i=0;i<point_indices.size();++i)
    {
        index = point_indices[i];
        Mi = core.submatrix( index*3 , index*3 + 2 );        
        M.row(3*i) = Mi.row(0) + 0;
        M.row(3*i+1) = Mi.row(1) + 0;
        M.row(3*i+2) = Mi.row(2) + 0;        
    }


    //do not use the guess in the first frame but do for every following frame
    for(int i=0;i<exr_size;i++)
        x_t[0][i] = x[i][0] = w_exp[i];
    for(int i=0;i<id_size;i++)
        y_t[0][i] = y[i][0] = w_id[i];


    //initialize variables which dont need to be computed at every step
    Rodrigues(rotation,rmatrix);
    pr = (1.0/Z_avg)*weakCamera*rmatrix;
    PR = Mat_<double>::zeros(2*featurePoints.size(),3*featurePoints.size());
    PRT = Mat_<double>(3*featurePoints.size(),2*featurePoints.size());
    for(unsigned int i=0;i<featurePoints.size();i++)
    {
        PR(Range(2*i,2*i+2),Range(3*i,3*i+3)) = pr + 0;
    }
    PRT = PR.t();    

    PRM = PR*M;

    //size is Size(width,height)
    A_ex = Mat_<double>::zeros(Size(exr_size,exr_size));
    B_ex = Mat_<double>::zeros(Size(1,exr_size));

    A_id = Mat_<double>::zeros(Size(id_size,id_size));
    B_id = Mat_<double>::zeros(Size(1,id_size));


    for(int count = 0;count < max_iterations; count++)
    {        
        //put the guesses into matrix y and x
        y.transpose(y_t);
        //Matrix::matrix_mult(y_t,u_id).transpose(lin_comb_y);
        x_t.transpose(x);

        //Z_ex = Matrix::kron(lin_comb_y,Matrix::eye(exr_size));        
        //ZU = Z_ex*(u_ex.t());
        ZU = Matrix::kron(y,Matrix::eye(exr_size));

        /* compute the formula :
         * compute by stacking rather then summing since with summing its possible to lose the solution
         * despite the fact that here we have a square matrix
         * Sum( [U*Z'*Mi'*R'*PW'*PW*R*Mi*Z*U'] *x ) = Sum( U*Z'*Mi'*R'*PW'*fi - (1/tz)*U*Z'*Mi'*R'*PW'*PW'*t )
         */
        W = PRM*ZU;
        WT = W.t();
        A_ex = WT*W;
        B_ex = WT*f;

        cout << "sizes " << A_ex.size().width << " " << A_ex.size().height << " "
                << regTerm_exp.size().width << " " << regTerm_exp.size().height << " "
                << B_ex.size().width << " " << B_ex.size().height << " "
                << leftTerm_exp.size().width << " " << leftTerm_exp.size().height << endl;
        cout << "types " << A_ex.type() << " " << regTerm_exp.type() << " " << B_ex.type() << " " << leftTerm_exp.type() << endl;
        A_ex = A_ex + regTerm_exp;
        B_ex = B_ex + leftTerm_exp;
        x = Matrix::solveLinSysSvd(A_ex,B_ex);
        cout << "in EX optim" << endl;
        Mat_<double> xM = x;

//        for(unsigned int j=0;j<point_indices.size();j++)
//        {
//            O = PRM*ZU*xM;
//            Mat_<double> pp =  M*ZU*xM;
//            cout << pp(0,0) << " " << pp(1,0) << " " << pp(2,0) << endl;
//            Mi = O.rowRange(Range(2*j,2*j+2));
//            fi = f.rowRange(Range(2*j,2*j+2));
//            cout << "o.rowrange size : " << Mi.size().height << " " << Mi.size().width << endl;
//            Mi(0,0) = Mi(0,0);
//            Mi(1,0) = Mi(1,0);
//            cout << Mi(0,0) << " " << Mi(1,0) << " vs "
//                 << fi(0,0) << " " << fi(1,0) << endl;
//        }
        //put the guesses into matrix y and x
        x.transpose(x_t);
        //Matrix::matrix_mult(x_t,u_ex).transpose(lin_comb_x);
        y_t.transpose(y);

//        Z_id = Matrix::kron(Matrix::eye(id_size),lin_comb_x);        
        //ZU = Z_id*(u_id.t());
        ZU = Matrix::kron(Matrix::eye(id_size),x);

        /* compute the formula :
         * Sum( U*Z'*Mi'*R'*PW'*PW*R*Mi*Z*U' )*y = Sum( U*Z'*Mi'*R'*PW'*fi - (1/tz)*U*Z'*Mi'*R'*PW'*PW'*t )
         */
        cout << "here " << point_indices.size() << endl;

        W = PRM*ZU;
        WT = W.t();
        A_id = WT*W;        
        B_id = WT*f;

        A_id = A_id + regTerm_id;
        B_id = B_id + leftTerm_id;
        y = Matrix::solveLinSysSvd(A_id,B_id);

        cout << "this should work ... O_O : " << endl << x;
        cout << "in ID optim" << endl;
        Mat_<double> yM = y;

//        for(unsigned int j=0;j<point_indices.size();j++)
//        {
//            O = PRM*ZU*yM;
//            Mat_<double> pp =  M*ZU*yM;
//            cout << pp(0,0) << " " << pp(1,0) << " " << pp(2,0) << endl;
//            Mi = O.rowRange(Range(2*j,2*j+2));
//            fi = f.rowRange(Range(2*j,2*j+2));
//            Mi(0,0) = Mi(0,0);
//            Mi(1,0) = Mi(1,0);
//            cout << Mi(0,0) << " " << Mi(1,0) << " vs "
//                 << fi(0,0) << " " << fi(1,0) << endl;
//        }
    }

    for(int i=0;i<exr_size;i++){
        w_exp[i] = x[i][0];
    }
    for(int i=0;i<id_size;i++){
        w_id[i] = y[i][0];
    }

    //we cannot normalize because then we lose 1/z_avg which is the scale
//    Vector3::normalize(w_exp, exr_size);
//    Vector3::normalize(w_id, id_size);

    for(int i=0;i<exr_size;i++){        
        weights_ex.push_back(w_exp[i]);
        //cout << w_exp[i] << " == " << weights_ex[i] << " ";
    }
    cout << endl;

    for(int i=0;i<id_size;i++){        
        weights_id.push_back(w_id[i]);
        //cout << w_id[i] << " ";
    }
    cout << endl;

    face_ptr->interpolate(w_id,w_exp,true,true);
    cout << " avg depth after : " << face_ptr->getAverageDepth() << "avg depth before : " << average_depth << endl;
    face_ptr->setAverageDepth(average_depth);

    delete[] w_id;
    delete[] w_exp;
}
