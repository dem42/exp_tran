#ifndef FACE_MODEL_H
#define FACE_MODEL_H

#include "Vector3.h"
#include "Matrix.h"
#include <string>
#include <cv.h>

using namespace cv;


//singleton persistable class
class FaceModel
{
 public:

    static FaceModel *getInstance();

    enum Mode_space_t {IDENTITY, EXPRESSION, VERTEX};

    void read_flat_vertex(double **a, int m, int n,
                           const int first_dim, const int second_dim, Mode_space_t flag);
    void read_flat(double **a, int m, int n,
                           const int first_dim, const int second_dim, Mode_space_t flag);
    void generateFace(Point3 *face,double *w_id,double *w_ex,bool brute_exp=false, bool brute_id=false);

    cv::Mat coreSubmatrix(int rowstart,int rowend);

    cv::Mat getCoreTensor() const;
    cv::Mat getUIdentity() const;
    cv::Mat getUExpression() const;
    double getSigmaIdAt(int i) const;
    double getSigmaExpAt(int i) const;

    //polygon information
    float (*triangles)[3];
    int getPolyNum() const;
    int getPointNum() const;

    int getIdSize() const;
    int getExpSize() const;

 protected:
    FaceModel(std::string filename,std::string dir,std::string db_list,int f,int e,int v);
    ~FaceModel();
    void persist();
    bool load();
private:
    void compute_core_tensor(void);
    void computeIdentitySingularVectors(int m,int n);
    void computeExpressionSingularVectors(int m,int n);
    void initializeDbStrings();
    void loadFacePolygon(string filename);
    void computeMean(int first_dim, int second_dim, int n, double &mean_x, double &mean_y, double &mean_z);

    static FaceModel *instance;

    std::string **strs;
    const std::string filename;
    const std::string dir_name;
    const std::string db_list;
    const int n_f;
    const int n_e;
    const int n_v;

    Matrix U_id;
    Matrix U_ex;
    Matrix core;

    Mat_<double> uid;
    Mat_<double> uex;
    Mat_<double> coreMat;

    double *sigma_id;
    double *sigma_exp;

    int poly_num;
    int point_num;

    double mean_x;
    double mean_y;
    double mean_z;
};

#endif // FACEMODEL_H
