#include "nnlsoptimizer.h"
#include "model/FaceModel.h"

NNLSOptimizer::NNLSOptimizer() : NNLS_MAX_ITER(1000), max_iterations(3)
{

}

void NNLSOptimizer::test()
{

    Mat_<double> M(5,4);
    Mat_<double> b(5,1);
    Mat_<double> x(4,1);

    M(0,0) = 1;
    M(0,1) = 0.5;
    M(0,2) = 0.2;
    M(0,3) = 41;

    M(1,0) = 1;
    M(1,1) = 11;
    M(1,2) = -4;
    M(1,3) = 2;

    M(2,0) = 0;
    M(2,1) = 0;
    M(2,2) = 9;
    M(2,3) = 0;

    M(3,0) = 11;
    M(3,1) = 1;
    M(3,2) = 0.5;
    M(3,3) = -5;

    M(4,0) = 66;
    M(4,1) = 1;
    M(4,2) = -11;
    M(4,3) = 2;


    b(0,0) = 125.9;
    b(1,0) = 11;
    b(2,0) = 18;
    b(3,0) = 9;
    b(4,0) = 117;

    x(0,0) = 0;
    x(1,0) = 0;
    x(2,0) = 0;
    x(3,0) = 0;

    this->scannls(M,b,x);

    Matrix X = x;

    cout << "RESULT OF SCA NNLS " << endl << X;
}

void NNLSOptimizer::estimateExpressionParameters(const vector<Point2f> &featurePoints,
                                 const Mat &cameraMatrix, const Mat& lensDist,
                                 Face* face_ptr,const vector<int> &point_indices,
                                 const Mat &rotation, const Mat &translation,
                                 vector<double> &weights_ex)
{
    Mat_<double> weakCamera(2,3);
    //matrices for A_ex * w_ex = f and A_id * w_id = f
    Mat_<double> A_ex, seg_A_ex;
    //matrix Z = kron(weights, Identity_matrix)
    Mat_<double> Z_ex;
    Mat_<double> ZU;
    Mat_<double> pr, Pt;

    //core tensor rows
    Mat_<double> Mi;

    //featurePoints converted to 2x1 matrices
    Mat_<double> fi;
    Mat_<double> f;

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
    Mat_<double> x_t(1,exr_size);
    Mat_<double> y_t(1,id_size);
    Mat_<double> x(exr_size,1);
    Mat_<double> y(id_size,1);
    //used for x_t*U_ex and y_t*U_id
    Matrix lin_comb_x(exr_size,1);
    Matrix lin_comb_y(id_size,1);

    double Z_avg;
    double average_depth;
    Point3f p;
    Mat_<double> proP;
    Mat_<double> pM(3,1);

    Rodrigues(rotation,rmatrix);

    //get weights from the current face instance
    face_ptr->getWeights(w_id,id_size,w_exp,exr_size);

    //find average depth
    average_depth = face_ptr->getAverageDepth();
    cout << "is the average depth 0 .. i think it is always the same" << average_depth << endl;
    pM(0,0) = 1;
    pM(1,0) = 1;
    pM(2,0) = average_depth;
    proP = cameraMatrix*rmatrix*pM;
    proP = proP + cameraMatrix*translation;
    Z_avg = proP(0,2);
    cout << " with a z_avg after projecting " << Z_avg << endl;

    /***************************************************************/
    /*create weak-perspecitve camera matrix from camera matrix*/
    /***************************************************************/
    for(int i=0;i<2;i++)
        for(int j=0;j<3;j++)
            weakCamera(i,j) = cameraMatrix.at<double>(i,j);

    f = Mat_<double>(featurePoints.size()*2,1);
    fi = Mat_<double>(2,1);

    //precompute translation
    Pt = (1./translation.at<double>(2,0)) * (weakCamera*translation);

    cout << "here " << featurePoints.size() << " &  " << point_indices.size() << endl;
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

    pr = (1.0/Z_avg)*weakCamera*rmatrix;
    A_ex = Mat_<double>::zeros(2*featurePoints.size(),exr_size);
    seg_A_ex = Mat_<double>::zeros(2*featurePoints.size(),exr_size);
    Mi = Mat_<double>(3,exr_size*id_size);

    for(int i=0;i<id_size;i++)
        y_t(0,i) = w_id[i];
    Matrix::matrix_mult(y_t,u_id).transpose(lin_comb_y);
    Z_ex = Matrix::kron(lin_comb_y,Matrix::eye(exr_size));
    ZU = Z_ex*(u_ex.t());

    for(unsigned int i=0;i<point_indices.size();++i)
    {
        index = point_indices[i];        
        Mi = core.submatrix( index*3 , index*3 + 2 );

        seg_A_ex = pr*Mi*ZU;
        A_ex.row(2*i) = seg_A_ex.row(0) + 0;
        A_ex.row(2*i+1) = seg_A_ex.row(1) + 0;
    }


    //do not use the guess in the first frame but do for every following frame
    for(int i=0;i<exr_size;i++)
        x_t(0,i) = x(i,0) = w_exp[i];

    //op .. lets see if we get here
    this->scannls(A_ex,f,x);
    cout << " OPT EXP " << Matrix(x);

    for(int i=0;i<exr_size;i++){
        w_exp[i] = x(i,0);
    }

    for(int i=0;i<exr_size;i++){
        weights_ex.push_back(w_exp[i]);
    }

    face_ptr->interpolate(w_id,w_exp);
    face_ptr->setAverageDepth(average_depth);

    delete[] w_id;
    delete[] w_exp;
}


void NNLSOptimizer::estimateIdentityParameters(const vector<vector<Point2f> >&featurePointsVector,
                                 const Mat &cameraMatrix, const Mat& lensDist,
                                 Face* face_ptr,const vector<vector<int> >&point_indices_vector,
                                 const vector<Mat> &rotation, const vector<Mat> &translation,
                                 const vector<vector<double> > &weights_ex,
                                 vector<double> &weights_id)
{
    Mat_<double> weakCamera(2,3);
    //matrices for A_id * w_id = f
    Mat_<double> A_id, seg_A_id;
    //matrix Z = kron(weights, Identity_matrix)
    Mat_<double> Z_id;
    Mat_<double> ZU;
    Mat_<double> pr, Pt;

    //core tensor rows
    Mat_<double> Mi;

    //featurePoints converted to 2x1 matrices
    Mat_<double> fi;
    Mat_<double> f;

    //initialize model-morph bases
    FaceModel *model = FaceModel::getInstance();
    Matrix core = model->getCoreTensor();
    Mat_<double> u_id = model->getUIdentity();
    Mat_<double> u_ex = model->getUExpression();

    int index = 0;
    //total number of feature points
    int featurePointNum = 0;

    const int exr_size = 7;
    const int id_size = 56;

    double *w_id = new double[id_size];
    double *w_exp = new double[exr_size];

    Mat_<double> rmatrix;
    Mat_<double> x_t(1,exr_size);
    Mat_<double> y_t(1,id_size);
    Mat_<double> x(exr_size,1);
    Mat_<double> y(id_size,1);
    //used for x_t*U_ex and y_t*U_id
    Matrix lin_comb_x(exr_size,1);

    double Z_avg;
    double average_depth;    
    Mat_<double> proP;
    Mat_<double> pM(3,1);


    //get weights from the current face instance
    face_ptr->getWeights(w_id,id_size,w_exp,exr_size);


    /***************************************************************/
    /*create weak-perspecitve camera matrix from camera matrix*/
    /***************************************************************/
    for(int i=0;i<2;i++)
        for(int j=0;j<3;j++)
            weakCamera(i,j) = cameraMatrix.at<double>(i,j);

    for(unsigned int i=0;i<featurePointsVector.size();i++)
        featurePointNum += featurePointsVector[i].size();
    cout << "total number of points " << featurePointNum << endl;

    f = Mat_<double>(featurePointNum*2,1);
    fi = Mat_<double>(2,1);


    //preprocess points and core tensor rows
    //for points subtract translation too
    for(unsigned int i=0, count=0;i<featurePointsVector.size();i++)
    {
        //precompute translation
        Pt = (1./translation[i].at<double>(2,0)) * (weakCamera*translation[i]);

        for(unsigned int j=0;j<featurePointsVector[i].size();j++)
        {
            fi = Mat_<double>(2,1);
            fi(0,0) = featurePointsVector[i][j].x;
            fi(1,0) = featurePointsVector[i][j].y;

            fi = fi - Pt;
            f.row(count + 2*j) = fi.row(0) + 0;
            f.row(count + 2*j+1) = fi.row(1) + 0;
        }
        count += featurePointsVector[i].size();
    }


    A_id = Mat_<double>(2*featurePointNum,id_size);
    seg_A_id = Mat_<double>(2,id_size);
    Mi = Mat_<double>(3,exr_size*id_size);

    average_depth = face_ptr->getAverageDepth();

    pM(0,0) = 1;
    pM(1,0) = 1;
    pM(2,0) = average_depth;

    cout << "featurePointsVector size : " << featurePointsVector.size() << endl;
    cout << "point indices vector size : " << point_indices_vector.size() << endl;

    for(unsigned int i=0, count = 0;i<point_indices_vector.size();++i)
    {
        Rodrigues(rotation[i],rmatrix);
        proP = cameraMatrix*rmatrix*pM;
        proP = proP + cameraMatrix*translation[i];
        Z_avg = proP(0,2);
        cout << " with a z_avg after projecting in frame : " << i << " " << Z_avg << endl;

        pr = (1.0/Z_avg)*weakCamera*rmatrix;


        //load the appropriate weights for the i-th frame
        for(unsigned int k=0;k<weights_ex[i].size();k++)
            x_t(0,k) = weights_ex[i][k];
        Matrix::matrix_mult(x_t,u_ex).transpose(lin_comb_x);
        Z_id = Matrix::kron(Matrix::eye(id_size),lin_comb_x);
        ZU = Z_id*(u_id.t());

        for(unsigned int j=0;j<point_indices_vector[i].size();++j)
        {
            index = point_indices_vector[i][j];
            Mi = core.submatrix( index*3 , index*3 + 2 );

            seg_A_id = pr*Mi*ZU;
            A_id.row(count + 2*j) = seg_A_id.row(0) + 0;
            A_id.row(count + 2*j+1) = seg_A_id.row(1) + 0;
        }
        count += point_indices_vector[i].size();
    }


    //do not use the guess in the first frame but do for every following frame
    for(int i=0;i<id_size;i++)
        y_t(0,i) = y(i,0) = w_id[i];

    //optimize using sequential coordinate descent
    this->scannls(A_id,f,y);

    cout << "OPT id .. not that well ever get here .. " << Matrix(y);


    for(int i=0;i<id_size;i++){
        w_id[i] = y(i,0);
    }

    for(int i=0;i<id_size;i++){
        weights_id.push_back(w_id[i]);
    }

    delete[] w_id;
    delete[] w_exp;
}



void NNLSOptimizer::estimateModelParameters(const Mat &frame, const vector<Point2f> &featurePoints,
                                 const Mat &cameraMatrix, const Mat& lensDist,
                                 Face* face_ptr,const vector<int> &point_indices,
                                 const Mat &rotation, const Mat &translation,
                                 vector<double> &weights_id, vector<double> &weights_ex)
{
    Mat_<double> weakCamera(2,3);
    //matrices for A_ex * w_ex = f and A_id * w_id = f
    Mat_<double> A_ex , A_id;
    //matrix Z = kron(weights, Identity_matrix)
    Mat_<double> Z_ex, Z_id;
    Mat_<double> ZU;
    Mat_<double> pr, Pt, PRM, prm;

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
    Mat_<double> x_t(1,exr_size);
    Mat_<double> y_t(1,id_size);
    Mat_<double> x(exr_size,1);
    Mat_<double> y(id_size,1);
    //used for x_t*U_ex and y_t*U_id
    Matrix lin_comb_x(exr_size,1);
    Matrix lin_comb_y(id_size,1);

    double Z_avg;
    double average_depth;
    Point3f p;
    Mat_<double> proP;
    Mat_<double> pM(3,1);

    cout << "huh" << endl;
    Rodrigues(rotation,rmatrix);

    //get weights from the current face instance
    face_ptr->getWeights(w_id,id_size,w_exp,exr_size);

    average_depth = face_ptr->getAverageDepth();
    pM(0,0) = 1;
    pM(1,0) = 1;
    pM(2,0) = average_depth;
    cout << "face is at a distance " << p.z << endl;
    proP = cameraMatrix*rmatrix*pM;
    proP = proP + cameraMatrix*translation;
    Z_avg = proP(0,2);
    cout << " with a z_avg after projecting " << Z_avg << endl;

    /***************************************************************/
    /*create weak-perspecitve camera matrix from camera matrix*/
    /***************************************************************/
    for(int i=0;i<2;i++)
        for(int j=0;j<3;j++)
            weakCamera(i,j) = cameraMatrix.at<double>(i,j);

    f = Mat_<double>(featurePoints.size()*2,1);
    fi = Mat_<double>(2,1);

    //precompute translation
    Pt = (1./translation.at<double>(2,0)) * (weakCamera*translation);

    cout << "here " << featurePoints.size() << " &  " << point_indices.size() << endl;
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

    pr = (1.0/Z_avg)*weakCamera*rmatrix;
    PRM = Mat_<double>(2*featurePoints.size(),exr_size*id_size);
    prm = Mat_<double>(2,exr_size*id_size);
    Mi = Mat_<double>(3,exr_size*id_size);

    for(unsigned int i=0;i<point_indices.size();++i)
    {
        index = point_indices[i];
        cout << "hey : " << index << endl;
        Mi = core.submatrix( index*3 , index*3 + 2 );
        prm = pr*Mi;
        PRM.row(2*i) = prm.row(0) + 0;
        PRM.row(2*i+1) = prm.row(1) + 0;
    }


    //do not use the guess in the first frame but do for every following frame
    for(int i=0;i<exr_size;i++)
        x_t(0,i) = x(i,0) = w_exp[i];
    for(int i=0;i<id_size;i++)
        y_t(0,i) = y(i,0) = w_id[i];

    cout << "and here! " << endl;


    A_ex = Mat_<double>::zeros(2*featurePoints.size(),exr_size);
    A_id = Mat_<double>::zeros(2*featurePoints.size(),id_size);

    for(int count = 0;count < max_iterations; count++)
    {
        //put the guesses into matrix y and x
        y_t = y.t();
        Matrix::matrix_mult(y_t,u_id).transpose(lin_comb_y);

        Z_ex = Matrix::kron(lin_comb_y,Matrix::eye(exr_size));
        ZU = Z_ex*(u_ex.t());

        /* compute the formula :
         * compute by stacking rather then summing since with summing its possible to lose the solution
         * despite the fact that here we have a square matrix
         * Pw*R*M*Z*U*x = f - (1/tz)*Pw*t
         */
        A_ex = PRM*ZU;
        // Mat_<double> p = (f - A_ex*x);
        //Mat_<double> err = p.t()*p;
        //cout << "the error before is : " << err(0,0) << endl;
//        for(unsigned int j=0;j<point_indices.size();j++)
//        {
//            O = PRM*ZU*x;
//            Mat_<double> pp =  M*ZU*x;
//            cout << pp(0,0) << " " << pp(1,0) << " " << pp(2,0) << endl;
//            Mi = O.rowRange(Range(2*j,2*j+2));
//            fi = f.rowRange(Range(2*j,2*j+2));
//            cout << "o.rowrange size : " << Mi.size().height << " " << Mi.size().width << endl;
//            Mi(0,0) = Mi(0,0);
//            Mi(1,0) = Mi(1,0);
//            cout << Mi(0,0) << " " << Mi(1,0) << " vs "
//                 << fi(0,0) << " " << fi(1,0) << endl;
//        }


        this->scannls(A_ex,f,x);
        //p = (f - A_ex*x);
//        err = p.t()*p;
//        cout << "the error after is : " << err(0,0) << endl;
//
//        cout << "in EX optim" << endl;
//
//        for(unsigned int j=0;j<point_indices.size();j++)
//        {
//            O = PRM*ZU*x;
//            Mat_<double> pp =  M*ZU*x;
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
        x_t = x.t();
        Matrix::matrix_mult(x_t,u_ex).transpose(lin_comb_x);

        Z_id = Matrix::kron(Matrix::eye(id_size),lin_comb_x);
        ZU = Z_id*(u_id.t());

        /* compute the formula :
         * Sum( U*Z'*Mi'*R'*PW'*PW*R*Mi*Z*U' )*y = Sum( U*Z'*Mi'*R'*PW'*fi - (1/tz)*U*Z'*Mi'*R'*PW'*PW'*t )
         */
//        cout << "here " << point_indices.size() << endl;

        A_id = PRM*ZU;
//        p = (f - A_id*y);
//        err = p.t()*p;
//        cout << "the error before is : " << err(0,0) << endl;
//                for(unsigned int j=0;j<point_indices.size();j++)
//        {
//            O = PRM*ZU*y;
//            Mat_<double> pp =  M*ZU*y;
//            cout << pp(0,0) << " " << pp(1,0) << " " << pp(2,0) << endl;
//            Mi = O.rowRange(Range(2*j,2*j+2));
//            fi = f.rowRange(Range(2*j,2*j+2));
//            Mi(0,0) = Mi(0,0);
//            Mi(1,0) = Mi(1,0);
//            cout << Mi(0,0) << " " << Mi(1,0) << " vs "
//                 << fi(0,0) << " " << fi(1,0) << endl;
//        }


        this->scannls(A_id,f,y);
//        p = (f - A_id*y);
//        err = p.t()*p;
//        cout << "the error after is : " << err(0,0) << endl;
//
//        cout << "this should work ... O_O : " << endl << Matrix(x);
//        cout << "in ID optim" << endl;
//
//
//        for(unsigned int j=0;j<point_indices.size();j++)
//        {
//            O = PRM*ZU*y;
//            Mat_<double> pp =  M*ZU*y;
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
        w_exp[i] = x(i,0);
    }
    for(int i=0;i<id_size;i++){
        w_id[i] = y(i,0);
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
    }
    cout << endl;

    face_ptr->interpolate(w_id,w_exp);
    cout << " avg depth after : " << face_ptr->getAverageDepth() << "avg depth before : " << average_depth << endl;
    face_ptr->setAverageDepth(average_depth);

    delete[] w_id;
    delete[] w_exp;
}

void NNLSOptimizer::scannls(const Mat& A, const Mat& b,Mat &x)
{
    int iter = 0;
    int m = A.size().height;
    int n = A.size().width;
    Mat_<double> AT = A.t();
    double error = 1e-10;
    Mat_<double> H = AT*A;
    Mat_<double> f = -AT*b;

    Mat_<double> x_old = Mat_<double>::zeros(n,1);
    Mat_<double> x_new = Mat_<double>::zeros(n,1);

    Mat_<double> mu_old = Mat_<double>::zeros(n,1);
    Mat_<double> mu_new = Mat_<double>::zeros(n,1);
    Mat_<double> r = Mat_<double>::zeros(n,1);
    f.copyTo(mu_old);

    while(iter < NNLS_MAX_ITER)
    {
        iter++;
        for(int k=0;k<n;k++)
        {
            x_old.copyTo(x_new);
            x_new(k,0) = std::max(0.0, x_old(k,0) - (mu_old(k,0)/H(k,k)) );

            if(x_new(k,0) != x_old(k,0))
            {
                r = mu_old + (x_new(k,0) - x_old(k,0))*H.col(k);
                r.copyTo(mu_new);
            }
            x_new.copyTo(x_old);
            mu_new.copyTo(mu_old);
        }

        if(eKKT(H,f,x_new,error) == true)
        {
            cout << "coz of this at " << iter << endl;
            break;
        }
    }
    x = x_new;
}


bool NNLSOptimizer::eKKT(const Mat& H,const Mat& f,const Mat& x,double e)
{
    int n = H.size().width;

    Mat_<double> hx_f = H*x + f;

    for(int i=0;i<n;++i)
    {
        if( x.at<double>(i,0) < 0 ) return false;
        else if( x.at<double>(i,0) > 0 )
        {
            if(hx_f(i,0) > e)
                return false;
        }
    }

    MatConstIterator_<double> cit = hx_f.begin(), cit_end = hx_f.end();
    for(;cit!=cit_end;++cit)
        if( (*cit) < -e ) return false;

    return true;
}
