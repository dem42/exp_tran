#include "closedformoptimizer.h"
#include "model/FaceModel.h"

ClosedFormOptimizer::ClosedFormOptimizer(double regParam) : max_iterations(4)
{    
    this->regParam = regParam;
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
    Matrix lin_comb_x(exr_size,1);
    Matrix lin_comb_y(id_size,1);

    //regularization term
    Mat_<double> regTerm_id, regTerm_exp, leftTerm_id, leftTerm_exp;

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
        Matrix::matrix_mult(y_t,u_id).transpose(lin_comb_y);
        Z_ex = Matrix::kron(lin_comb_y,Matrix::eye(exr_size));
        ZU = Z_ex*(u_ex.t());

        A_ex = PRM*ZU;
        A_ext = A_ex.t();
        W = A_ext*A_ex;
        W = W + regTerm_exp;
        
        B_ex = A_ext*f;
        B_ex = B_ex + leftTerm_exp;
        
        cv::solve(W,B_ex,x,DECOMP_SVD);

        for(unsigned int k=0;k<exr_size;k++)
            x_t(0,k) = x(k,0);
        Matrix::matrix_mult(x_t,u_ex).transpose(lin_comb_x);
        Z_id = Matrix::kron(Matrix::eye(id_size),lin_comb_x);
        ZU = Z_id*(u_id.t());

        A_id = PRM*ZU;
        A_idt = A_id.t();
        W = A_idt*A_id;
        W = W + regTerm_id;
        
        B_id = A_idt*f;
        B_id = B_id + leftTerm_id;
        
        cv::solve(W,B_id,y,DECOMP_SVD);
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
    cout << endl;

    for(int i=0;i<id_size;i++){        
        weights_id.push_back(w_id[i]);        
    }
    cout << endl;

    face_ptr->interpolate(w_id,w_exp);
    face_ptr->setAverageDepth(average_depth);

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
    Matrix lin_comb_x(exr_size,1);
    Matrix lin_comb_y(id_size,1);

    double Z_avg;
    double average_depth;
    Point3f p;
    Mat_<double> proP;
    Mat_<double> pM(3,1);    

        //regularization term
    Mat_<double> regTerm_exp, leftTerm_exp;

    //size is Size(width,height)
    regTerm_exp = Mat_<double>::eye(Size(exr_size,exr_size));
    leftTerm_exp = Mat_<double>::eye(Size(1,exr_size));

    //init the left terms which are made up of regParam*mean
    //mean is 1./size for both since theres exactly same amount of
    //faces with 1s at those spots and mean is in between all of them (? not sure)
    for(int i=0;i<exr_size;i++)
        leftTerm_exp(0,i) = 1./exr_size;

    leftTerm_exp = regParam*leftTerm_exp;

    regTerm_exp = regParam*regTerm_exp;


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

    //without brute ->
    ZU = Z_ex*(u_ex.t());

    //with brute ->
    //ZU = Z_ex;


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


    A_ext = A_ex.t();
    W = A_ext*A_ex;
    W = W + regTerm_exp;

    B = A_ext*f;
    B = B + leftTerm_exp;

    cv::solve(W,B,x,DECOMP_SVD);
    //x = Matrix::solveLinSysSvd(W,B);

    //cout << " OPT EXP " << Matrix(x);

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
    Matrix lin_comb_x(exr_size,1);

    double Z_avg;
    double average_depth;
    Mat_<double> proP;
    Mat_<double> pM(3,1);

        //regularization term
    Mat_<double> regTerm_id, leftTerm_id;

    //size is Size(width,height)
    regTerm_id = Mat_<double>::eye(Size(id_size,id_size));

    leftTerm_id = Mat_<double>::eye(Size(1,id_size));

    //init the left terms which are made up of regParam*mean
    //mean is 1./size for both since theres exactly same amount of
    //faces with 1s at those spots and mean is in between all of them (? not sure)
    for(int i=0;i<id_size;i++)
        leftTerm_id(0,i) = 1./id_size;
    leftTerm_id = regParam*leftTerm_id;

    regTerm_id = regParam*regTerm_id;


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
            x(k,0) = x_t(0,k) = weights_ex[i][k];

        //without brute
        Matrix::matrix_mult(x_t,u_ex).transpose(lin_comb_x);
        Z_id = Matrix::kron(Matrix::eye(id_size),lin_comb_x);
        ZU = Z_id*(u_id.t());

        //with brute
        //Z_id = Matrix::kron(Matrix::eye(id_size),x);
        //with brute ->
        //ZU = Z_id;

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

    A_idt = A_id.t();
    W = A_idt*A_id;
    W = W + regTerm_id;

    B = A_idt*f;
    B = B + leftTerm_id;

    cv::solve(W,B,y,DECOMP_SVD);


    for(int i=0;i<id_size;i++){
        w_id[i] = y(i,0);
    }

    for(int i=0;i<id_size;i++){
        weights_id.push_back(w_id[i]);
    }

    delete[] w_id;
    delete[] w_exp;
}

