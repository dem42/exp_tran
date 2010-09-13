#ifndef FACE_H
#define FACE_H

#include "Vector3.h"
#include <string>
#include "model/FaceModel.h"

class Face 
{
 public:
    enum InterpolType{NO_INTER, ID_INTER, EXP_INTER, ID_EXP_INTER};

    Face();
    ~Face();

    void test(void);
    void load(string filename, const char *tex_map_filename);
    Color3 interpolate_color(Color3 a,Color3 b,Color3 c,Color3 d,float r,float s);

    void setNewIdentityAndExpression(double *w_id,double *w_exp,Face::InterpolType type = Face::NO_INTER);

    void transferExpressionFromFace(Face *src_face);

    void getWeights(double *w_id, int w_id_size, double *w_exp, int w_exp_size);

    int getPolyNum() const;
    int getPointNum() const;
    int getIdNum() const;
    int getExpNum() const;
    string getEmotionString() const;

    Point3 getPointFromPolygon(int);
    Point3 getPoint(int index) const;
    int getPointIndexFromPolygon(int);
    int closestPointIndexForPoint(Point3 p);

    //the following functions allow us to use weak perspective projection
    //in conjunction with parameter estimation
    //we never really consider the depth from points generated from the parameters we estimated
    //instead we just use the average depth of the initial guess
    //BUT .. do we lose the ability to scale? yes i think we do
    double getAverageDepth() const;
    void setAverageDepth(double d);

    //used to position the frustum in opengl
    void calculateBoundingSphere(float&cx,float&cy,float&cz,float&diameter) const;


    //vertexes are sorted and the points in triangles therefore as well
    //so we do not need to worry about keeping cross product order consistent
    //as long as we always calculate the cross product the same way in all triangles
    //public to make the drawing happen in faceWidget but not lose efficient or cause a mem leak
    Point3 *vertexes;
    Vector3 *vertex_normals;
    float (*triangles)[3];
    static const int fPoints[20];
    static const int fPolygons[20];
    static const int fPoints_size;

    static const int leftMouthCornerIndex;
    static const int rightMouthCornerIndex;
    static const int topLipIndex;
    static const int bottomLipIndex;
    static const int leftEyeBrow;
    static const int rightEyeBrow;

    static const int mouth[43];
    static const int mouth_size;

private:
    void loadPolygonDataFromModel();
    //Vector3 *surface_normals;

    Color3 *vertex_texture;
    //here we are declaring pointers to an array of 3 floats (or 2 floats)

    //initialized either from face_model or file
    int poly_num;
    int texture_num;
    int point_num;

    //whether we display using (line loop)wireframe or polygon (default)
    int gl_display_style;

    //SVD *model;
    FaceModel *model;

    //weights corresponding to the face
    double *w_exp;
    double *w_id;

    int ID;
    int EXP;
    int VER;

protected:
    void generate_vertex_normals(void);

};

#endif
