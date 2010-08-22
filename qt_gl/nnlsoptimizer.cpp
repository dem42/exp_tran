#include "nnlsoptimizer.h"
#include "FaceModel.h"

NNLSOptimizer::NNLSOptimizer() : NNLS_MAX_ITER(1000), max_iterations(1)
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
    Mat_<double> PR, PRT, pr, Pt, PRM;

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
    Point3f p;
    Mat_<double> proP;
    Mat_<double> pM(3,1);

    cout << "huh" << endl;
    Rodrigues(rotation,rmatrix);

    //get weights from the current face instance
    face_ptr->getWeights(w_id,id_size,w_exp,exr_size);
    p = face_ptr->getPointFromPolygon(7403);
    cout << "huh2" << endl;
    pM(0,0) = p.x;
    pM(1,0) = p.y;
    pM(2,0) = p.z;
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
    Pt = (1/translation.at<double>(2,0)) * (weakCamera*translation);

    //preprocess points and core tensor rows
    //for points subtract translation too
    for(int i=0;i<featurePoints.size();i++)
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

    for(int i=0;i<point_indices.size();++i)
    {
        index = point_indices[i];
        Mi = core.submatrix( index*3 , index*3 + 2 );
        M.row(3*i) = Mi.row(0) + 0;
        M.row(3*i+1) = Mi.row(1) + 0;
        M.row(3*i+2) = Mi.row(2) + 0;
    }


    //do not use the guess in the first frame but do for every following frame
    for(int i=0;i<exr_size;i++)
        x_t(0,i) = x(i,0) = w_exp[i];
    for(int i=0;i<id_size;i++)
        y_t(0,i) = y(i,0) = w_id[i];


    //initialize variables which dont need to be computed at every step    
    pr = (1.0/Z_avg)*weakCamera*rmatrix;
    PR = Mat_<double>::zeros(2*featurePoints.size(),3*featurePoints.size());
    PRT = Mat_<double>(3*featurePoints.size(),2*featurePoints.size());
    for(int i=0;i<featurePoints.size();i++)
    {
        PR(Range(2*i,2*i+2),Range(3*i,3*i+3)) = pr + 0;
    }
    PRT = PR.t();

    PRM = PR*M;

    A_ex = Mat_<double>::zeros(Size(2*featurePoints.size(),exr_size));

    A_id = Mat_<double>::zeros(Size(2*featurePoints.size(),id_size));

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
         Mat_<double> p = (f - A_ex*x);
        Mat_<double> err = p.t()*p;
        cout << "the error before is : " << err(0,0) << endl;
        for(unsigned int j=0;j<point_indices.size();j++)
        {
            O = PRM*ZU*x;
            Mat_<double> pp =  M*ZU*x;
            cout << pp(0,0) << " " << pp(1,0) << " " << pp(2,0) << endl;
            Mi = O.rowRange(Range(2*j,2*j+2));
            fi = f.rowRange(Range(2*j,2*j+2));
            cout << "o.rowrange size : " << Mi.size().height << " " << Mi.size().width << endl;
            Mi(0,0) = Mi(0,0);
            Mi(1,0) = Mi(1,0);
            cout << Mi(0,0) << " " << Mi(1,0) << " vs "
                 << fi(0,0) << " " << fi(1,0) << endl;
        }


        this->scannls(A_ex,f,x);
        p = (f - A_ex*x);
        err = p.t()*p;
        cout << "the error after is : " << err(0,0) << endl;

        cout << "in EX optim" << endl;

        for(unsigned int j=0;j<point_indices.size();j++)
        {
            O = PRM*ZU*x;
            Mat_<double> pp =  M*ZU*x;
            cout << pp(0,0) << " " << pp(1,0) << " " << pp(2,0) << endl;
            Mi = O.rowRange(Range(2*j,2*j+2));
            fi = f.rowRange(Range(2*j,2*j+2));
            cout << "o.rowrange size : " << Mi.size().height << " " << Mi.size().width << endl;
            Mi(0,0) = Mi(0,0);
            Mi(1,0) = Mi(1,0);
            cout << Mi(0,0) << " " << Mi(1,0) << " vs "
                 << fi(0,0) << " " << fi(1,0) << endl;
        }
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
        p = (f - A_id*y);
        err = p.t()*p;
        cout << "the error before is : " << err(0,0) << endl;
                for(unsigned int j=0;j<point_indices.size();j++)
        {
            O = PRM*ZU*y;
            Mat_<double> pp =  M*ZU*y;
            cout << pp(0,0) << " " << pp(1,0) << " " << pp(2,0) << endl;
            Mi = O.rowRange(Range(2*j,2*j+2));
            fi = f.rowRange(Range(2*j,2*j+2));
            Mi(0,0) = Mi(0,0);
            Mi(1,0) = Mi(1,0);
            cout << Mi(0,0) << " " << Mi(1,0) << " vs "
                 << fi(0,0) << " " << fi(1,0) << endl;
        }


        this->scannls(A_id,f,y);
        p = (f - A_id*y);
        err = p.t()*p;
        cout << "the error after is : " << err(0,0) << endl;

        cout << "this should work ... O_O : " << endl << Matrix(x);
        cout << "in ID optim" << endl;


        for(unsigned int j=0;j<point_indices.size();j++)
        {
            O = PRM*ZU*y;
            Mat_<double> pp =  M*ZU*y;
            cout << pp(0,0) << " " << pp(1,0) << " " << pp(2,0) << endl;
            Mi = O.rowRange(Range(2*j,2*j+2));
            fi = f.rowRange(Range(2*j,2*j+2));
            Mi(0,0) = Mi(0,0);
            Mi(1,0) = Mi(1,0);
            cout << Mi(0,0) << " " << Mi(1,0) << " vs "
                 << fi(0,0) << " " << fi(1,0) << endl;
        }
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

    //face_ptr->interpolate(w_id,w_exp);

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
    bool result = true;
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
