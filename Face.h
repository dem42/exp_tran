#include "Vector3.h"

class Face 
{
 public:
  void display(void);
  void test(void);
  void load(const char* filename, const char *tex_map_filename);
  Color3 interpolate_color(Color3 a,Color3 b,Color3 c,Color3 d,float r,float s);
  ~Face();
  
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

  int poly_num;
  int texture_num;
  int point_num;

 protected:
  void generate_vertex_normals(void);
  
};
