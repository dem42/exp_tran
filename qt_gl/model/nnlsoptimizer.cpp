#include "nnlsoptimizer.h"
#include "model/FaceModel.h"

NNLSOptimizer::NNLSOptimizer() : Optimizer(), NNLS_MAX_ITER(3000), max_iterations(5)
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


    int index = 0;

    const int exr_size = model->getExpSize();
    const int id_size = model->getIdSize();

    double *w_id = new double[id_size];
    double *w_exp = new double[exr_size];

    Mat_<double> rmatrix;
    Mat_<double> x_t(1,exr_size);
    Mat_<double> y_t(1,id_size);
    Mat_<double> x(exr_size,1);
    Mat_<double> y(id_size,1);
    //used for x_t*U_ex and y_t*U_id
    Mat_<double> lin_comb_x(exr_size,1);
    Mat_<double> lin_comb_y(id_size,1);

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
    pM(0,0) = 1;
    pM(1,0) = 1;
    pM(2,0) = average_depth;
    proP = cameraMatrix*rmatrix*pM;
    proP = proP + cameraMatrix*translation;
    Z_avg = proP(0,2);

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

    for(int i=0;i<id_size;i++)
        y_t(0,i) = w_id[i];
    lin_comb_y = (y_t*u_id).t();
    Z_ex = Matrix::kronecker(lin_comb_y,Mat_<double>::eye(exr_size,exr_size));
    ZU = Z_ex*(u_ex.t());

    for(unsigned int i=0;i<point_indices.size();++i)
    {
        index = point_indices[i];
        seg_A_ex = pr*M[index]*ZU;
        A_ex.row(2*i) = seg_A_ex.row(0) + 0;
        A_ex.row(2*i+1) = seg_A_ex.row(1) + 0;
    }

    //do not use the guess in the first frame but do for every following frame
    for(int i=0;i<exr_size;i++)
        x_t(0,i) = x(i,0) = w_exp[i];

    Mat_<double> temp = (A_ex*x - f);
    Mat_<double> e = temp.t()*temp;
    cout << "exp, error before " << e(0,0)<<endl;
    //op .. lets see if we get here
    this->scannls(A_ex,f,x);
    temp = (A_ex*x - f);
    e = temp.t()*temp;
    cout << "exp, error after " << e(0,0)<<endl;


    for(int i=0;i<exr_size;i++){
        w_exp[i] = x(i,0);
    }

    for(int i=0;i<exr_size;i++){
        weights_ex.push_back(w_exp[i]);
    }

    face_ptr->setNewIdentityAndExpression(w_id,w_exp);
    //unecessary it seems 
    face_ptr->setAverageDepth(average_depth);



    ZU.release();    
    Z_ex.release();
    A_ex.release();
    f.release();
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

    //featurePoints converted to 2x1 matrices
    Mat_<double> fi;
    Mat_<double> f;

    int index = 0;
    //total number of feature points
    int featurePointNum = 0;

    const int exr_size = model->getExpSize();
    const int id_size = model->getIdSize();

    double *w_id = new double[id_size];
    double *w_exp = new double[exr_size];

    Mat_<double> rmatrix;
    Mat_<double> x_t(1,exr_size);
    Mat_<double> y_t(1,id_size);
    Mat_<double> x(exr_size,1);
    Mat_<double> y(id_size,1);
    //used for x_t*U_ex and y_t*U_id
    Mat_<double> lin_comb_x(exr_size,1);

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
        count += 2*featurePointsVector[i].size();
    }

    A_id = Mat_<double>(2*featurePointNum,id_size);
    seg_A_id = Mat_<double>(2,id_size);

    average_depth = face_ptr->getAverageDepth();

    pM(0,0) = 1;
    pM(1,0) = 1;
    pM(2,0) = average_depth;


    for(unsigned int i=0, count = 0;i<point_indices_vector.size();++i)
    {
        Rodrigues(rotation[i],rmatrix);
        proP = cameraMatrix*rmatrix*pM;
        proP = proP + cameraMatrix*translation[i];
        Z_avg = proP(0,2);

        pr = (1.0/Z_avg)*weakCamera*rmatrix;


        //load the appropriate weights for the i-th frame
        for(unsigned int k=0;k<weights_ex[i].size();k++)
            x_t(0,k) = weights_ex[i][k];
        lin_comb_x = (x_t*u_ex).t();
        Z_id = Matrix::kronecker(Mat_<double>::eye(id_size,id_size),lin_comb_x);
        ZU = Z_id*(u_id.t());

        for(unsigned int j=0;j<point_indices_vector[i].size();++j)
        {
            index = point_indices_vector[i][j];

            seg_A_id = pr*M[index]*ZU;
            A_id.row(count + 2*j) = seg_A_id.row(0) + 0;
            A_id.row(count + 2*j+1) = seg_A_id.row(1) + 0;
        }
        count += 2*point_indices_vector[i].size();
    }


    //do not use the guess in the first frame but do for every following frame
    for(int i=0;i<id_size;i++)
        y_t(0,i) = y(i,0) = w_id[i];

    Mat_<double> temp = (A_id*y - f);
    Mat_<double> e = temp.t()*temp;
    cout << "id, error before " << e(0,0)<<endl;
    //optimize using sequential coordinate descent
    this->scannls(A_id,f,y);    
    temp = (A_id*y - f);
    e = temp.t()*temp;
    cout << "id, error after " << e(0,0)<<endl;


    for(int i=0;i<id_size;i++){
        w_id[i] = y(i,0);
    }
    for(int i=0;i<exr_size;i++){
        w_exp[i] = weights_ex[0][i];
    }

    for(int i=0;i<id_size;i++){
        weights_id.push_back(w_id[i]);
    }

    face_ptr->setNewIdentityAndExpression(w_id,w_exp);
    face_ptr->setAverageDepth(average_depth);

    ZU.release();
    Z_id.release();
    A_id.release();
    f.release();
    delete[] w_id;
    delete[] w_exp;
}



void NNLSOptimizer::estimateModelParameters(const vector<Point2f> &featurePoints,
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

    //featurePoints converted to 2x1 matrices
    Mat_<double> fi;
    Mat_<double> f;
    Mat_<double> O;

    int index = 0;
    const int exr_size = model->getExpSize();
    const int id_size = model->getIdSize();

    double *w_id = new double[id_size];
    double *w_exp = new double[exr_size];

    Mat_<double> rmatrix;
    Mat_<double> x_t(1,exr_size);
    Mat_<double> y_t(1,id_size);
    Mat_<double> x(exr_size,1);
    Mat_<double> y(id_size,1);
    //used for x_t*U_ex and y_t*U_id
    Mat_<double> lin_comb_x(exr_size,1);
    Mat_<double> lin_comb_y(id_size,1);

    double Z_avg;
    double average_depth;
    Point3f p;
    Mat_<double> proP;
    Mat_<double> pM(3,1);

    Rodrigues(rotation,rmatrix);

    //get weights from the current face instance
    face_ptr->getWeights(w_id,id_size,w_exp,exr_size);

    average_depth = face_ptr->getAverageDepth();
    pM(0,0) = 1;
    pM(1,0) = 1;
    pM(2,0) = average_depth;    
    proP = cameraMatrix*rmatrix*pM;
    proP = proP + cameraMatrix*translation;
    Z_avg = proP(0,2);

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

    for(unsigned int i=0;i<point_indices.size();++i)
    {
        index = point_indices[i];        
        prm = pr*M[index];
        PRM.row(2*i) = prm.row(0) + 0;
        PRM.row(2*i+1) = prm.row(1) + 0;
    }


    //do not use the guess in the first frame but do for every following frame
    for(int i=0;i<exr_size;i++)
        x_t(0,i) = x(i,0) = w_exp[i];
    for(int i=0;i<id_size;i++)
        y_t(0,i) = y(i,0) = w_id[i];

    A_ex = Mat_<double>::zeros(2*featurePoints.size(),exr_size);
    A_id = Mat_<double>::zeros(2*featurePoints.size(),id_size);

    for(int count = 0;count < max_iterations; count++)
    {
        //put the guesses into matrix y and x
        y_t = y.t();

        lin_comb_y = (y_t*u_id).t();

        Z_ex = Matrix::kronecker(lin_comb_y,Mat_<double>::eye(exr_size,exr_size));
        ZU = Z_ex*(u_ex.t());


        A_ex = PRM*ZU;


        this->scannls(A_ex,f,x);

        x_t = x.t();
        lin_comb_x = (x_t*u_ex).t();

        Z_id = Matrix::kronecker(Mat_<double>::eye(id_size,id_size),lin_comb_x);
        ZU = Z_id*(u_id.t());

        /* compute the formula :
         * Sum( U*Z'*Mi'*R'*PW'*PW*R*Mi*Z*U' )*y = Sum( U*Z'*Mi'*R'*PW'*fi - (1/tz)*U*Z'*Mi'*R'*PW'*PW'*t )
         */


        A_id = PRM*ZU;

        Mat_<double> temp = (A_id*y - f);
        Mat_<double> e = temp.t()*temp;
        cout << "id, error before " << e(0,0)<<endl;
        this->scannls(A_id,f,y);
        temp = (A_id*y - f);
        e = temp.t()*temp;
        cout << "id, error after " << e(0,0)<<endl;
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
    }


    for(int i=0;i<id_size;i++){
        weights_id.push_back(w_id[i]);        
    }

    face_ptr->setNewIdentityAndExpression(w_id,w_exp);    
    face_ptr->setAverageDepth(average_depth);


    prm.release();
    PRM.release();
    ZU.release();
    Z_id.release();
    A_id.release();
    Z_ex.release();
    A_ex.release();
    f.release();
    delete[] w_id;
    delete[] w_exp;
}

void NNLSOptimizer::scannls(const Mat& A, const Mat& b,Mat &x)
{
    int iter = 0;
    int m = A.size().height;
    int n = A.size().width;
    Mat_<double> AT = A.t();
    double error = 1e-8;
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
            break;
        }
    }
    x_new.copyTo(x);
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
