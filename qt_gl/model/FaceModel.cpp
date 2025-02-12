#include "FaceModel.h"
#include "Matrix.h"
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <iostream>
#include <fstream>

using namespace std;

FaceModel *FaceModel::instance = 0;

FaceModel * FaceModel::getInstance()
{
    //lazy initialization
    if(instance == NULL)
    {        
        cout << "here" << endl;
        ifstream fstr("model.properties",ifstream::in);
        string result_object;
        string database_location;
        string db_out_listing;
        int id;
        int exp;
        int pts;
        if(fstr.fail() == true)
        {
            cerr << "please provide a model.properties file" << endl;
            abort();
        }
        fstr >> result_object >> database_location >> db_out_listing >> id >> exp >> pts;

        instance = new FaceModel(result_object.c_str(),
                                 database_location,
                                 db_out_listing,
                                  id,exp,pts);
    }
    return instance;
}

//chain constructors of U2,U3,core
FaceModel::FaceModel(string filename,string dir,string db_list,int f,int e,int v) :
        filename(filename), dir_name(dir), db_list(db_list),
        n_f(f), n_e(e), n_v(v), U_id(f,f), U_ex(e,e), core(3*v,f*e)
{
    //initialize the sigmas
    sigma_id = new double[f];
    sigma_exp = new double[e];

    bool loaded = load();

    if(!loaded)
    {
        strs = new string*[f];
        for(int i=0;i<f;i++)
            strs[i] = new string[e];        

        initializeDbStrings();

        compute_core_tensor();

        loadFacePolygon("/home/martin/project/JaceyBinghamtonVTKFiles/M0014_HA01WH.vtk");


        /*persist core, u2, u3, point_num, poly_num, triangles*/
        persist();
    } else
        cout << "the u2,u3,core have been loaded from: " << filename << endl;

    cout << "model : point_num and poly_num : " << point_num << " " << poly_num << endl;


    uex = Mat_<double>(U_ex.getM(),U_ex.getN());
    uid = Mat_<double>(U_id.getM(),U_id.getN());
    coreMat = Mat_<double>(core.getM(),core.getN());

    for(int i=0;i<core.getM();i++)
        for(int j=0;j<core.getN();j++)
            coreMat(i,j) = core.mat[i][j];
    for(int i=0;i<uid.rows;i++)
        for(int j=0;j<uid.cols;j++)
            uid(i,j) = U_id.mat[i][j];    
    for(int i=0;i<uex.rows;i++)
        for(int j=0;j<uex.cols;j++)
            uex(i,j) = U_ex.mat[i][j];
    //test of matrix functions
    //core.test();
}

//we do not explicitly call this .. rather we rely on the OS
//to clean up the process .. possible memory leak?
FaceModel::~FaceModel()        
{
    std::cout << "destructor FaceModel" << std::endl;
    delete[] strs;
    delete[] triangles;
}

Mat FaceModel::getCoreTensor() const
{
    cout << "core" << endl;
    return coreMat;
}


cv::Mat FaceModel::coreSubmatrix(int rowstart,int rowend)
{
    //from rowstart to rowend including the row with index rowend
    int size = rowend - rowstart + 1;
    cv::Mat_<double> sub(size,core.getN());
    for(int i=0, ri = rowstart; i<size; ++i, ++ri)
        for(int j=0; j<core.getN(); ++j)
            sub(i,j) = core.mat[ri][j];
    return sub;
}

int FaceModel::getIdSize() const
{
    return n_f;
}

int FaceModel::getExpSize() const
{
    return n_e;
}

cv::Mat FaceModel::getUIdentity() const
{ 
    return uid;
}
cv::Mat FaceModel::getUExpression() const
{    
    return uex;
}

double FaceModel::getSigmaIdAt(int i) const
{
    if(i >= 0 && i < n_f)
        return sigma_id[i];
    else
        cerr << "incorrect index of sigma_id" << endl;
    return 0;
}
double FaceModel::getSigmaExpAt(int i) const
{
    if(i >= 0 && i < n_e)
        return sigma_exp[i];
    else
        cerr << "incorrect index of sigma_exp" << endl;
    return 0;
}
void FaceModel::initializeDbStrings()
{
    FILE *fid;
    char file[50];
    int num, i, j;
    num = i = j = 0;

    const char *file_list = db_list.c_str();
    const char *dir = dir_name.c_str();

    fid = fopen(file_list,"r");
    if(fid == NULL)
    {
        cerr << "critical error file list : " << file_list << " does not exist" << endl;
        abort();
    }

    fscanf(fid,"%d",&num);
    if(num != n_f * n_e) printf("problem\n");

    for(i=0;i<n_f;i++)
        for(j=0;j<n_e;j++)
        {
            fscanf(fid,"%s",file);
            strs[i][j].assign(file);
        }

    for(i=0;i<n_f;i++)
        for(j=0;j<n_e;j++)
            std::cout << strs[i][j] << std::endl;
    fclose(fid);
}


//@param brute .. whether we interpolate using U2 and U3 or just the weights right away
void FaceModel::generateFace(Point3 *face,double *w_id,double *w_ex,bool brute_exp, bool brute_id)
{
//    Matrix m_wid(1,n_f);
//    Matrix m_wex(1,n_e);
    cv::Mat_<double> m_wid(1,n_f);
    cv::Mat_<double> m_wex(1,n_e);

    //linear combinations
//    Matrix row_id(1,n_f);
//    Matrix row_ex(1,n_e);
    cv::Mat_<double> row_id(1,n_f);
    cv::Mat_<double> row_ex(1,n_e);

    int i = 0,j=0;

    for(i=0;i<n_f;i++)
        m_wid(0,i) = w_id[i];
    for(i=0;i<n_e;i++)
        m_wex(0,i) = w_ex[i];

//    cout << "wex " << m_wex;
//    cout << "wid " << m_wid;

    if(brute_exp == false)
    {
        //multiply with u2 and u3     
        row_ex = m_wex*uex;
        //Vector3::normalize(row_ex.mat[0],n_e);
    }
    else
    {
        //brute == true so
        //just copy w_exp over from the output
        //brute means we are turning the dials that correspond to the basis vectors
        for(i=0;i<n_e;i++)
            row_ex(0,i) = w_ex[i];
        //Vector3::normalize(row_ex.mat[0],n_e);
    }
    if(brute_id == false)
    {
        //multiply with u2 and u3
        row_id = m_wid*uid;
        //Vector3::normalize(row_id.mat[0],n_f);
    }
    else
    {        
        for(i=0;i<n_f;i++)
            row_id(0,i) = w_id[i];
        //Vector3::normalize(row_id.mat[0],n_f);
    }


    cv::Mat_<double> K(1,n_f*n_e), KT(n_f*n_e,1), f(3*n_v,1);

    K = Matrix::kronecker(row_id,row_ex);
    KT = K.t();

    //core * kron( (w2*u2), (w3*u3) )' = f
    f = coreMat*KT;

    for(i=0,j=0;i<3*n_v;i=i+3,j++)
    {
        face[j].x = (float)(f(i,0));
        face[j].y = (float)(f(i+1,0));
        face[j].z = (float)(f(i+2,0));
    }
}


void FaceModel::computeMean(int first_dim, int second_dim, int n, double &m_x, double &m_y, double &m_z)
{
    double sum_x = 0;
    double sum_y = 0;
    double sum_z = 0;

    FILE *fid;
    int i,j,k;
    int num_dont_care;
    double number;
    char filename[256];
    char str_dont_care[50];


    for(i=0;i<first_dim;i++)
    {
        for(j=0;j<second_dim;j++)
        {
            sprintf(filename,"%s/%s",dir_name.c_str(),strs[i][j].c_str());
            cout << filename <<" " << i << " " << j <<  endl;

            fid = fopen(filename,"r");
            if(fid == NULL)
            {
                cerr << "critical error db file : " << filename << " does not exist" << endl;
                abort();
            }
            fgets(str_dont_care,50,fid);
            fgets(str_dont_care,50,fid);
            fgets(str_dont_care,50,fid);
            fgets(str_dont_care,50,fid);

            fscanf(fid,"%s %d",str_dont_care,&num_dont_care);
            fscanf(fid,"%s",str_dont_care);

            for(k=0;k<n;k+=3)
            {
                fscanf(fid,"%lf",&number);
                sum_x += number;
                fscanf(fid,"%lf",&number);
                sum_y += number;
                fscanf(fid,"%lf",&number);
                sum_z += number;
            }

            fclose(fid);
        }
    }
    m_x = 3*sum_x / (n*first_dim*second_dim);
    m_y = 3*sum_y / (n*first_dim*second_dim);
    m_z = 3*sum_z / (n*first_dim*second_dim);
}
void FaceModel::computeIdentitySingularVectors(int m,int n)
{
    Matrix A(m,n);
    Matrix AT(n,m);
    Matrix A2(m,m);
    read_flat(A.mat,m,3*n_v,n_f,n_e,IDENTITY);

    //make the numbers more managable for the svd
    //more precise
    A.scalar_mult(0.01);
    A.transpose(AT);
    A2 = Matrix::matrix_mult(A,AT);

    sigma_id = new double[n];

    Matrix::svd(m,m,1,0,0.000001,0.000001,A2.mat,sigma_id,U_id.mat,NULL);
    cout << "sigma id"<< endl;
    for(int i=0;i<m;i++)
        cout << 10000.*sigma_id[i] << " ";
    cout << endl;
}

void FaceModel::computeExpressionSingularVectors(int m,int n)
{
    Matrix A(m,n);
    Matrix AT(n,m);
    Matrix A2(m,m);
    //make the numbers more managable for the svd
    //more precise

    read_flat(A.mat,m,3*n_v,n_e,n_f,EXPRESSION);

    A.scalar_mult(0.01);
    A.transpose(AT);
    A2 = Matrix::matrix_mult(A,AT);

    Matrix::svd(m,m,1,0,0.000001,0.000001,A2.mat,sigma_exp,U_ex.mat,NULL);
    cout << "sigma exp"<< endl;
    for(int i=0;i<m;i++)
        cout << 10000.*sigma_exp[i] << " ";
    cout << endl;
}

void FaceModel::compute_core_tensor(void)
{    
    int m,n;

    m = n_f;
    n = n_e * 3 * n_v;
    computeMean(n_f,n_e,3*n_v,this->mean_x,this->mean_y,this->mean_z);

    /********************************/
    /*********first read indentity*/
    /********************************/
    computeIdentitySingularVectors(m,n);

    m = n_e;
    n = n_f * 3 * n_v;
    /********************************/
    /*********then read expression*/
    /********************************/
    computeExpressionSingularVectors(m,n);

    m = 3*n_v;
    n = n_e * n_f;

    Matrix a_flat(m,n);
    Matrix K(n,n);
    /********************************/
    /*********finally read flattened vertex tensor*/
    /********************************/
    //finally we may need the flattened a matrix .. hmm maybe we dont need it
    //we could probably do with out the kronecker product and just flatten along mode2 and do u2^T * mode2_flat * u3
    //to get the core tensor flattened along mode 2
    read_flat_vertex(a_flat.mat,m,3*n_v,n_f,n_e,VERTEX);

    /********************************/
    /*********calculate flat core tensor core = a1_flat*kron(u2,u3)*/
    /********************************/
    K = Matrix::kron_(U_id,U_ex);
    core = Matrix::matrix_mult(a_flat,K);

    /**** test
    at = new double*[1];
    at[0] = new double[n];

    v = new double*[n];
    for(i=0;i<n;i++)
        v[i] = new double[1];

    for(i=0;i<n;i++)
        at[0][i] = K[5][i];


    matrix_transpose(at,v,1,n);
    delete[] at;
    at = new double*[m];
    for(i=0;i<m;i++)
        at[i] = new double[1];

    matrix_mult(core,v,at,m,n,1);
    ***/    
}


void FaceModel::persist()
{
    fstream fstr(filename.c_str(),fstream::out);
    //persist U_id and sigma_id (diagonal sigma matrix)
    for(int i=0;i<n_f;i++)
        for(int j=0;j<n_f;j++)
            fstr << U_id[i][j] << " ";
    fstr << endl;
    for(int j=0;j<n_f;j++)
        fstr << sigma_id[j] << " ";
    fstr << endl;
    //persist U_ex and sigma_exp (diagonal sigma matrix)
    for(int i=0;i<n_e;i++)
        for(int j=0;j<n_e;j++)
            fstr << U_ex[i][j] << " ";
    fstr << endl;
    for(int j=0;j<n_e;j++)
        fstr << sigma_exp[j] << " ";
    fstr << endl;
    //persist the core tensor
    for(int i=0;i<3*n_v;i++)
        for(int j=0;j<n_f*n_e;j++)
            fstr << core[i][j] << " ";
    fstr << endl;
    //persist the topology data
    fstr << point_num << " " << poly_num << endl;
    for(int i=0; i<poly_num; i++)
    {
        fstr << triangles[i][0] << " " << triangles[i][1] << " " << triangles[i][2] << " ";
    }    
    fstr.close();
}

bool FaceModel::load()
{
    //open the file for reading, so if it doesnt exist we get a not is_open() == false
    //if it is false we set our state to
    fstream fstr(filename.c_str(),fstream::in);

    if(fstr.is_open() == false)
    {
        cerr << "file doesnt exist so we have to calculate .. this will take a minute" << endl;
        return false;
    }
    //set what should cause an fstream::failure exception so that we can catch it
    fstr.exceptions ( ifstream::eofbit | ifstream::failbit | ifstream::badbit );
    try
    {
        //matrix memory allocation happens in the constructor
        //.. throw bad_alloc if something goes awry
        //now read from file
        for(int i=0;i<n_f;i++)
            for(int j=0;j<n_f;j++)
                fstr >> U_id[i][j];
        for(int j=0;j<n_f;j++)
            fstr >> sigma_id[j];

        for(int i=0;i<n_e;i++)
            for(int j=0;j<n_e;j++)
                fstr >> U_ex[i][j];
        for(int j=0;j<n_e;j++)
            fstr >> sigma_exp[j];

        for(int i=0;i<3*n_v;i++)
            for(int j=0;j<n_f*n_e;j++)
                fstr >> core[i][j];

        fstr >> point_num >> poly_num;

        triangles = new (nothrow) float[poly_num][3];
        if(triangles == NULL)
        {
            cerr << "error allocating memory for triangles" << endl;
            return false;
        }

        for(int i=0; i<poly_num; i++)
        {
            fstr >> triangles[i][0] >> triangles[i][1] >> triangles[i][2];
        }
    }
    catch(bad_alloc &bae)
    {
        cerr << "bad alloc caught " << bae.what() << endl;
        fstr.close();
        return false;
    }
    catch(fstream::failure &fe)
    {
        cerr << "error reading from file " << fe.what() << endl;
        fstr.close();
        return false;
    }
    fstr.close();
    return true;
}

void FaceModel::read_flat(double **a, int m, int n,
                          const int first_dim, const int second_dim, Mode_space_t flag)
{
    FILE *fid;
    int i,j,k;
    int number;
    char filename[256];
    char str_dont_care[50];
    double num;

    for(i=0;i<first_dim;i++)
    {
        for(j=0;j<second_dim;j++)
        {
            switch(flag)
            {
             case IDENTITY : sprintf(filename,"%s/%s",dir_name.c_str(),strs[i][j].c_str()); break;
             case EXPRESSION : sprintf(filename,"%s/%s",dir_name.c_str(),strs[j][i].c_str()); break;             
             case VERTEX : std::cerr << "use other function for vertex" << std:: endl; exit(0);
             default: std::cerr << "unknown flag type" << std:: endl; exit(0);
             }
            fid = fopen(filename,"r");
            if(fid == NULL)
            {
                cerr << "critical error db file : " << filename << " does not exist" << endl;
                abort();
            }
            fgets(str_dont_care,50,fid);
            fgets(str_dont_care,50,fid);
            fgets(str_dont_care,50,fid);
            fgets(str_dont_care,50,fid);

            fscanf(fid,"%s %d",str_dont_care,&number);
            fscanf(fid,"%s",str_dont_care);

            for(k=0;k<n;k+=3)
            {
                fscanf(fid,"%lf",&num);
                a[i][j*n+k] = num - mean_x;
                fscanf(fid,"%lf",&num);
                a[i][j*n+k+1] = num - mean_y;
                fscanf(fid,"%lf",&num);
                a[i][j*n+k+2] = num - mean_z;
            }

            fclose(fid);
        }
    }
}


void FaceModel::read_flat_vertex(double **a, int m, int n,
                                 const int first_dim, const int second_dim, Mode_space_t flag)
{
    FILE *fid;
    int i,j,k;
    int number;
    char filename[256];
    char str_dont_care[50];
    double num;

    if(flag != VERTEX)
        std::cerr << "use other function " << std::endl;

    for(i=0;i<first_dim;i++)
    {
        for(j=0;j<second_dim;j++)
        {
            sprintf(filename,"%s/%s",dir_name.c_str(),strs[i][j].c_str());

            fid = fopen(filename,"r");
            if(fid == NULL)
            {
                cerr << "critical error db file : " << filename << " does not exist" << endl;
                abort();
            }
            fgets(str_dont_care,50,fid);
            fgets(str_dont_care,50,fid);
            fgets(str_dont_care,50,fid);
            fgets(str_dont_care,50,fid);

            fscanf(fid,"%s %d",str_dont_care,&number);
            fscanf(fid,"%s",str_dont_care);

            for(k=0;k<n;k+=3)
            {                
                fscanf(fid,"%lf",&num);
                a[k][i*second_dim+j] = num - mean_x;
                fscanf(fid,"%lf",&num);
                a[k+1][i*second_dim+j] = num - mean_y;
                fscanf(fid,"%lf",&num);
                a[k+2][i*second_dim+j] = num - mean_z;
            }

            fclose(fid);
        }
    }
}

int FaceModel::getPolyNum() const
{
    return poly_num;
}
int FaceModel::getPointNum() const
{
    return point_num;
}

void FaceModel::loadFacePolygon(string filename)
{
  string str;
  int dont_need_int;
  fstream file_op(filename.c_str(),ios::in);

  /*vtk files begin with several unimportant lines*/
  char line[256];
  file_op.getline(line,256);
  cerr << line << endl;
  file_op.getline(line,256);
  cerr << line << endl;
  file_op.getline(line,256);
  cerr << line << endl;
  file_op.getline(line,256);
  cerr << line << endl;

  /**************************************************/
  /** load POINTS ***/
  /**************************************************/
  file_op >> str >> point_num;
  //error if wrong label
  if(str.compare("POINTS") != 0)
  {
      cerr << "did not match the label POINTS got " << str << endl;
      return;
  }

  Point3 p;

  //one more line telling us its float
  file_op >> str;

  for(int i=0; i<point_num; i++)
    file_op >> p.x >> p.y >> p.z;

  /**************************************************/
  /** load POLYGONS ***/
  /**************************************************/
  file_op >> str >> poly_num >> dont_need_int;
  //error if wrong label
  if(str.compare("POLYGONS") != 0)
    {
      cerr << "did not match the label POLYGONS" << endl;
      return;
    }

  triangles = new (nothrow) float[poly_num][3];
  if(triangles == NULL)
    {
      cerr << "error allocating memory for triangles" << endl;
      return;
    }

  for(int i=0; i<poly_num; i++)
    {
      file_op >> dont_need_int;
      if(dont_need_int != 3)
        {
          cerr << "error .. not a triangle on line " << i << endl;
          return;
        }
      file_op >> triangles[i][0] >> triangles[i][1] >> triangles[i][2];
    }
  file_op.close();
}
