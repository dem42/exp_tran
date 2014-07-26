#include "closedformoptimizer.h"
#include "model/FaceModel.h"

ClosedFormOptimizer::ClosedFormOptimizer(double regParam) : Optimizer(), max_iterations(4)
{    
    this->regParam = regParam;

    const int exr_size = model->getExpSize();
    const int id_size = model->getIdSize();

    //size is Size(width,height)
    regTerm_exp = Mat_<double>::eye(exr_size,exr_size);
    regTerm_id = Mat_<double>::eye(id_size,id_size);
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
}

void ClosedFormOptimizer::estimateModelParameters(const vector<Point2f> &featurePoints,
                                                  const Mat &cameraMatrix, const Mat& lensDist,
                                                  Face* face_ptr,const vector<int> &point_indices,
                                                  const Mat &rotation, const Mat &translation,
                                                  vector<double> &weights_id, vector<double> &weights_ex)
{
    Mat_<double> weakCamera(2,3);
    //matrices for A_ex * w_ex = B_ex and A_id * w_id = B_id
    Mat_<double> A_ex , A_id, A_ext , A_idt;
    Mat_<double> B_ex, B_id;
    //matrix Z = kron(weights, Identity_matrix)
    Mat_<double> Z_ex, Z_id;
    Mat_<double> ZU;
    Mat_<double> PR, prm, pr, Pt, PRM;
    Mat_<double> W;

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

    average_depth = face_ptr->getAverageDepth();    
    pM(0,0) = 1;
    pM(1,0) = 1;
    pM(2,0) = average_depth;
    proP = cameraMatrix*rmatrix*pM;
    proP = proP + cameraMatrix*translation;
    Z_avg = proP(0,2);
    cout << "z " << Z_avg << endl;

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
        f.row(2*i) = fi.row(0) + 0.0;
        f.row(2*i+1) = fi.row(1) + 0.0;
    }

    pr = (1.0/Z_avg)*weakCamera*rmatrix;
    PRM = Mat_<double>(2*featurePoints.size(),exr_size*id_size);
    prm = Mat_<double>(2,exr_size*id_size);

    for(unsigned int i=0;i<point_indices.size();++i)
    {
        index = point_indices[i];        
        prm = pr*M[index];
        PRM.row(2*i) = prm.row(0) + 0.0;
        PRM.row(2*i+1) = prm.row(1) + 0.0;
    }


    //size is Size(width,height)
    A_ex = Mat_<double>::zeros(Size(exr_size,exr_size));
    B_ex = Mat_<double>::zeros(Size(1,exr_size));

    A_id = Mat_<double>::zeros(Size(id_size,id_size));
    B_id = Mat_<double>::zeros(Size(1,id_size));


    //do not use the guess in the first frame but do for every following frame
    for(int i=0;i<exr_size;i++)
        x_t(0,i) = x(i,0) = w_exp[i];
    for(int i=0;i<id_size;i++)
        y_t(0,i) = y(i,0) = w_id[i];


    for(int count = 0;count < max_iterations; count++)
    {        /* compute the formula :
         * compute by stacking rather then summing since with summing its possible to lose the solution
         * despite the fact that here we have a square matrix
         * Sum( [U*Z'*Mi'*R'*PW'*PW*R*Mi*Z*U'] *x ) = Sum( U*Z'*Mi'*R'*PW'*fi - (1/tz)*U*Z'*Mi'*R'*PW'*PW'*t )
         */
        for(int i=0;i<id_size;i++)
            y_t(0,i) = y(i,0);        
        lin_comb_y = (y_t*u_id).t();
        Z_ex = Matrix::kronecker(lin_comb_y,Mat_<double>::eye(exr_size,exr_size));
        ZU = Z_ex*(u_ex.t());

        A_ex = PRM*ZU;
        A_ext = A_ex.t();
        W = A_ext*A_ex;
        W = W + regTerm_exp;
        
        B_ex = A_ext*f;
        B_ex = B_ex + leftTerm_exp;
        
        cv::solve(W,B_ex,x);

        for(int k=0;k<exr_size;k++)
            x_t(0,k) = x(k,0);
        lin_comb_x = (x_t*u_ex).t();
        Z_id = Matrix::kronecker(Mat_<double>::eye(id_size,id_size),lin_comb_x);
        ZU = Z_id*(u_id.t());

        A_id = PRM*ZU;
        A_idt = A_id.t();
        W = A_idt*A_id;

        W = W + regTerm_id;
        
        B_id = A_idt*f;
        B_id = B_id + leftTerm_id;
        
        cv::solve(W,B_id,y);
    }

    for(int i=0;i<exr_size;i++){
        w_exp[i] = x(i,0);
    }
    for(int i=0;i<id_size;i++){
        w_id[i] = y(i,0);
    }

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
    A_idt.release();
    W.release();
    B_ex.release();
    B_id.release();    
    Z_ex.release();
    A_ex.release();
    A_ext.release();
    f.release();
    delete[] w_id;
    delete[] w_exp;
}


void ClosedFormOptimizer::estimateExpressionParameters(const vector<Point2f> &featurePoints,
                                 const Mat &cameraMatrix, const Mat& lensDist,
                                 Face* face_ptr,const vector<int> &point_indices,
                                 const Mat &rotation, const Mat &translation,
                                 vector<double> &weights_ex)
{
    Mat_<double> weakCamera(2,3);
    //matrices for A_ex * w_ex = f and A_id * w_id = f
    Mat_<double> A_ex, seg_A_ex, A_ext;
    //matrix Z = kron(weights, Identity_matrix)
    Mat_<double> Z_ex;
    Mat_<double> ZU;
    Mat_<double> pr, Pt;
    Mat_<double> W, B;


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
        f.row(2*i) = fi.row(0) + 0.0;
        f.row(2*i+1) = fi.row(1) + 0.0;
    }

    pr = weakCamera*rmatrix;
    A_ex = Mat_<double>::zeros(2*featurePoints.size(),exr_size);
    seg_A_ex = Mat_<double>::zeros(2*featurePoints.size(),exr_size);

    for(int i=0;i<id_size;i++)
        y_t(0,i) = w_id[i];

    lin_comb_y = (y_t*u_id).t();
    Z_ex = Matrix::kronecker(lin_comb_y,Mat_<double>::eye(exr_size,exr_size));

    //without brute ->
    ZU = Z_ex*(u_ex.t());

    //with brute ->
    //ZU = Z_ex;


    for(unsigned int i=0;i<point_indices.size();++i)
    {
        index = point_indices[i];

        seg_A_ex = pr*M[index]*ZU;
        A_ex.row(2*i) = seg_A_ex.row(0) + 0.0;
        A_ex.row(2*i+1) = seg_A_ex.row(1) + 0.0;
    }


    //do not use the guess in the first frame but do for every following frame
    for(int i=0;i<exr_size;i++)
        x_t(0,i) = x(i,0) = w_exp[i];


    A_ext = A_ex.t();
    W = A_ext*A_ex;
    W = (1.0/Z_avg)*W;
    W = W + Z_avg*regTerm_exp;

    B = A_ext*f;
    B = B + Z_avg*leftTerm_exp;
//
//    Mat_<double> temp = ((1.0/Z_avg)*A_ex*x - f);
//    Mat_<double> e = temp.t()*temp;
//    cout << "exp, error before " << e(0,0)<<endl;
    cv::solve(W,B,x);
    //x = Matrix::solveLinSysSvd(W,B);
//    temp = ((1.0/Z_avg)*A_ex*x - f);
//    e = temp.t()*temp;
//    cout << "exp, error after " << e(0,0)<<endl;



    for(int i=0;i<exr_size;i++){
        w_exp[i] = x(i,0);
    }

    for(int i=0;i<exr_size;i++){
        weights_ex.push_back(w_exp[i]);
    }

    face_ptr->setNewIdentityAndExpression(w_id,w_exp);
    // unecessary it seems 
    face_ptr->setAverageDepth(average_depth);


    ZU.release();
    W.release();
    B.release();
    Z_ex.release();
    A_ex.release();
    A_ext.release();
    f.release();
    delete[] w_id;
    delete[] w_exp;
}


void ClosedFormOptimizer::estimateIdentityParameters(const vector<vector<Point2f> >&featurePointsVector,
                                 const Mat &cameraMatrix, const Mat& lensDist,
                                 Face* face_ptr,const vector<vector<int> >&point_indices_vector,
                                 const vector<Mat> &rotation, const vector<Mat> &translation,
                                 const vector<vector<double> > &weights_ex,
                                 vector<double> &weights_id)
{
    Mat_<double> weakCamera(2,3);
    //matrices for A_id * w_id = f
    Mat_<double> A_id, seg_A_id, A_idt;
    //matrix Z = kron(weights, Identity_matrix)
    Mat_<double> Z_id;
    Mat_<double> ZU;
    Mat_<double> pr, Pt;
    Mat_<double> W, B;

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
            f.row(count + 2*j) = fi.row(0) + 0.0;
            f.row(count + 2*j+1) = fi.row(1) + 0.0;
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
        pr = weakCamera*rmatrix;


        //load the appropriate weights for the i-th frame
        for(unsigned int k=0;k<weights_ex[i].size();k++)
            x(k,0) = x_t(0,k) = weights_ex[i][k];

        //without brute
        lin_comb_x = (x_t*u_ex).t();
        Z_id = Matrix::kronecker(Mat_<double>::eye(id_size,id_size),lin_comb_x);
        ZU = Z_id*(u_id.t());

        //with brute
        //Z_id = Matrix::kron(Matrix::eye(id_size),x);
        //with brute ->
        //ZU = Z_id;

        for(unsigned int j=0;j<point_indices_vector[i].size();++j)
        {
            index = point_indices_vector[i][j];

            seg_A_id = pr*M[index]*ZU;
            A_id.row(count + 2*j) = seg_A_id.row(0) + 0.0;
            A_id.row(count + 2*j+1) = seg_A_id.row(1) + 0.0;
        }
        count += 2*point_indices_vector[i].size();
    }


    //do not use the guess in the first frame but do for every following frame
    for(int i=0;i<id_size;i++)
        y_t(0,i) = y(i,0) = w_id[i];

    A_idt = A_id.t();
    W = A_idt*A_id;
    W = (1.0/Z_avg)*W;
    W = W + Z_avg*regTerm_id;

    B = A_idt*f;
    B = B + Z_avg*leftTerm_id;
//
//    Mat_<double> temp = ((1./Z_avg)*A_id*y - f);
//    Mat_<double> e1 = temp.t()*temp ,e2;
//    cout << "id, error before " << e1(0,0)<<endl;
    cv::solve(W,B,y);
//    temp = ((1./Z_avg)*A_id*y - f);
//    e2 = temp.t()*temp;
//    cout << "id, error after " << e2(0,0)<<endl;
//    if(e1(0,0) < e2(0,0))
//        cout << "!!!! " << Matrix(y);


    for(int i=0;i<id_size;i++){
        w_id[i] = y(i,0);
    }

    for(int i=0;i<exr_size;i++)
    {
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
    A_idt.release();
    W.release();
    B.release();
    f.release();
    delete[] w_id;
    delete[] w_exp;
}

