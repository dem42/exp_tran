#ifndef FACE_H
#define FACE_H

#include "Vector3.h"
#include <string>
#include "FaceModel.h"
#include "svd.h"

class Face 
{
 public:
  void display(void);
  void test(void);
  void load(string filename, const char *tex_map_filename);
  Color3 interpolate_color(Color3 a,Color3 b,Color3 c,Color3 d,float r,float s);
  Face();
  ~Face();
  void interpolate(long double *w_id,long double *w_exp,bool brute=false);  
  void setWireFrame(bool);

  Point3 getPointFromPolygon(int);

  void setColor(int index);
  //draw should probably not happen here instead allow the user to iterate over the face i think  
  void draw();
  void calculateBoundingSphere() const;
private:
  Point3 *vertexes;
  Vector3 *vertex_normals;
  //Vector3 *surface_normals;
    
  //vertexes are sorted and the points in triangles therefore as well
  //so we do not need to worry about keeping cross product order consistent
  //as long as we always calculate the cross product the same way in all triangles
  float (*triangles)[3];
  float (*texture_2d_coord)[2];
  Color3 *vertex_texture;  
  //here we are declaring pointers to an array of 3 floats (or 2 floats)

  //initialized either from face_model or file
  int poly_num;
  int texture_num;
  int point_num;

  int index;
  //whether we display using (line loop)wireframe or polygon (default)
  int gl_display_style;

  SVD *model;
  //FaceModel *model;

 protected:
  void generate_vertex_normals(void);
  
};

#endif
