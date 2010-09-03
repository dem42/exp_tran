#include <iostream>
#include <fstream>
#include <string>
#include <limits>
#include <cmath>
#include <new>
#include "model/Face.h"
#include "model/FaceModel.h"
#include <QColor>

using namespace std;

//possibly add an xml config file for the stuff like exp and database dir
Face::Face()
{
    ID = 56, EXP = 7, VER = 5090;

    //model = new SVD("svd_result_object_2",expr,id,ver);
    model = FaceModel::getInstance();

    //we always need to load at least the topology data (which triangles are joined up)
    //this happens in the load method which also allocates the vertexes array
    loadPolygonDataFromModel();
    cout << " point num " << point_num << endl;
    cout << " poly num " << poly_num << endl;

    //after we load polygonal data we can allocate memory for vertexes
    vertexes = new (nothrow) Point3[point_num];
    if(vertexes == NULL)
    {
        cerr << "error allocating memory for vertexes" << endl;
        return;
    }

    //finally we use default weights to interpolate values for vertexes
    w_id = new double[ID];
    w_exp = new double[EXP];

    for(int i=0;i<ID;i++)
        w_id[i] = 0.0;
    for(int i=0;i<EXP;i++)
        w_exp[i] = 0;

//    for(int i=0;i<ID;i++)
//    {
//        if(i==33)w_id[i] = 0.2;
//        else if(i==10)w_id[i] = 0.2;
//        else if(i==20)w_id[i] = 0.2;
//        else if(i==1)w_id[i] = 0.2;
//        else if(i==50)w_id[i] = 0.2;
//        else w_id[i] = 0;
//    }
//    w_exp[0] = 0.0;
//    w_exp[1] = 0.0;
//    w_exp[2] = 0.0;
//    w_exp[3] = 0.0;
//    w_exp[4] = 1.0;
//    w_exp[5] = 0.0;
//    w_exp[6] = 0.0;
    interpolate(w_id,w_exp);
}

Face::~Face()
{
  delete[] vertexes;  
  delete[] vertex_normals;
  delete[] triangles;
  cout << "face destroyed" << endl;
  delete[] w_exp;
  delete[] w_id;
  //singleton so we do not explicitly call its destructor
  //delete model

  //delete[] vertex_texture;
  //delete[] texture_2d_coord;
}
  
const int Face::fPoints[20] = {4925,4609,261,3878,702,4733,4632,3285,3828,4147,
                                    1058,1451,1824,3278,4572,953,1992,4332,3196,1930};
const int Face::fPolygons[20] = {9521,8899,310,7455,1386,8934,8945,6284,7140,8197,
                                    2080,2851,3580,6058,8825,1680,3907,8144,6111,3786};
const int Face::fPoints_size = 20;


const int Face::mouth[43] = {975,769,768,561,352,558,349,141,142,9569,9572,9361,9360,9149,9152,8940,
                             8939,8726,8729,8514,8302,8515,8301,8087,8090,7877,7878,7669,7458,
                             7668,8088,8299,8517,8728,8942,9363,9151,9571,144,351,560,
                             8941,8731};
const int Face::mouth_size = 43;

double Face::getAverageDepth() const
{
    double zavg = 0;
    if(point_num == 0)
        return 0.0;

    for(int i=0;i<point_num;i++)
        zavg += vertexes[i].z;

    zavg /= point_num;

    return zavg;
}

//changes the average depth of the face
//compute the current average and moves it
void Face::setAverageDepth(double d)
{
    double avg = getAverageDepth();
    for(int i=0;i<point_num;i++)
        vertexes[i].z = vertexes[i].z - avg + d;    
}

int Face::getPolyNum() const
{
    return poly_num;
}

/**** this is how we interpolate
  c ---- d
  |      |
  a ---- b
r horizontal, s vertical weights ****/
Color3 Face::interpolate_color(Color3 a,Color3 b,Color3 c,Color3 d,float r,float s)
{
  Color3 result;
  result.r = r*s*a.r + (1-r)*s*b.r + r*(1-s)*c.r + (1-r)*(1-s)*d.r;
  result.g = r*s*a.g + (1-r)*s*b.g + r*(1-s)*c.g + (1-r)*(1-s)*d.g;
  result.b = r*s*a.b + (1-r)*s*b.b + r*(1-s)*c.b + (1-r)*(1-s)*d.b;  
  return result;
}

//brute interpolate doesnt try to create believable expressions
//the weights are not multiplied with the U2 and U3 singular value
//vectors before they are mode multiplied with the core tensor

//standard interpolate where w_id and w_exp are multiplied with
//the singular value matricies U2 and U3
void Face::interpolate(double *w_id,double *w_exp,bool brute_exp,bool brute_id)
{
//    double sum = 0;
//    for(int i=0;i<7;i++)
//    {
//        cout << "w_exp["<<i<<"] = " << w_exp[i] << endl;
//        sum += w_exp[i];
//    }
//    //normalize:
//    //if no exp set make it neutral
//    if(sum == 0)
//       w_exp[4] = 1;
//    else
//        for(int i=0;i<7;i++)
//            w_exp[i] = w_exp[i]/sum;
//
//    sum = 0;
//
//    for(int i=0;i<56;i++)
//    {
//        sum += w_id[i];
//    }
//    std::cerr << sum <<  " the sum" << std::endl;
//
//    if(sum == 0)
//       w_id[28] = 1;
//    else
//        for(int i=0;i<56;i++)
//            w_id[i] = w_id[i]/sum;

    //remember the new weights
    for(int i=0;i<ID;i++)
        this->w_id[i] = w_id[i];
    for(int i=0;i<EXP;i++)
        this->w_exp[i] = w_exp[i];

    model->interpolate_expression(vertexes,w_id,w_exp,brute_exp,brute_id);
    //now recalculate the vertex normals for the newly interpolated face
    generate_vertex_normals();
}

void Face::getWeights(double *w_id, int w_id_size, double *w_exp, int w_exp_size)
{
    assert(w_id_size == ID);
    assert(w_exp_size == EXP);
    for(int i=0;i<ID;i++)
        w_id[i] = this->w_id[i];
    for(int i=0;i<EXP;i++)
        w_exp[i] = this->w_exp[i];
}

//right now we are merely selecting the first point in the polygon
//perhaps it would be better to average?
//on one hand we perhaps want points we can generate .. from the model
//thats a con for the average
//on the other if we average we always have distinct points
Point3 Face::getPointFromPolygon(int index)
{
    int p_index = triangles[index][0];
    return vertexes[p_index];
}

Point3 Face::getPoint(int index) const
{
    return vertexes[index];
}

int Face::getPointNum() const
{
    return point_num;
}

int Face::getPointIndexFromPolygon(int index)
{
    cout << "points in polygon are" << triangles[index][0] << " "
         << triangles[index][1] << " " << triangles[index][2] << endl;
    return triangles[index][0];
}

int Face::closestPointIndexForPoint(Point3 p)
{    
    int min_index = -1;
    double min = numeric_limits<double>::max();
    double dist = 0.0;
    for(int i=0;i<point_num;i++)
    {
        Vector3 v(vertexes[i],p);
        dist = v.length();

        if(dist < min)
        {
            min = dist;
            min_index = i;
        }
    }
    return min_index;
}

void Face::calculateBoundingSphere(float &cx,float &cy,float &cz, float &diameter) const
{
    int v1,v2,v3;
    float v;
    float bx_u,bx_d,by_u,by_d,bz_u,bz_d;
    bx_u = by_u = bz_u = -5000;
    bx_d = by_d = bz_d = 5000;

    cout << poly_num << endl;
    for(int i=0; i<poly_num; i++)
    {
        v1 = triangles[i][0];
        v2 = triangles[i][1];
        v3 = triangles[i][2];

        v = vertexes[v1].x;
        bx_u = (v>bx_u)?v:bx_u;
        bx_d = (v<bx_d)?v:bx_d;
        v = vertexes[v1].y;
        by_u = (v>by_u)?v:by_u;
        by_d = (v<by_d)?v:by_d;
        v = vertexes[v1].z;
        bz_u = (v>bz_u)?v:bz_u;
        bz_d = (v<bz_d)?v:bz_d;

        v = vertexes[v2].x;
        bx_u = (v>bx_u)?v:bx_u;
        bx_d = (v<bx_d)?v:bx_d;
        v = vertexes[v2].y;
        by_u = (v>by_u)?v:by_u;
        by_d = (v<by_d)?v:by_d;
        v = vertexes[v2].z;
        bz_u = (v>bz_u)?v:bz_u;
        bz_d = (v<bz_d)?v:bz_d;

        v = vertexes[v3].x;
        bx_u = (v>bx_u)?v:bx_u;
        bx_d = (v<bx_d)?v:bx_d;
        v = vertexes[v3].y;
        by_u = (v>by_u)?v:by_u;
        by_d = (v<by_d)?v:by_d;
        v = vertexes[v3].z;
        bz_u = (v>bz_u)?v:bz_u;
        bz_d = (v<bz_d)?v:bz_d;
    }
    cx = (bx_u + bx_d)/2.0;
    cy = (by_u + by_d)/2.0;
    cz = (bz_u + bz_d)/2.0;

    diameter = std::max(fabs(bx_d - bx_u), fabs(by_d - by_u));
    diameter = std::max(diameter, fabs(bz_d - bz_u));
}

  
void Face::test(void) 
{
  cout << "in my test" << endl;
  int v0, v1, v2, v3;
  
  for(int i=0; i<poly_num; i++)
    {
      v0 = triangles[i][0];
      v1 = triangles[i][1];
      v2 = triangles[i][2];
      
      cout << "vertex coord" << endl;
      cout << vertexes[v0].x << " " << vertexes[v0].y << " " << vertexes[v0].z << endl;
      cout << vertexes[v1].x << " " << vertexes[v1].y << " " << vertexes[v1].z << endl;  
      cout << vertexes[v2].x << " " << vertexes[v2].y << " " << vertexes[v2].z << endl;


      cout << "vertex normals" << endl;
      cout << vertex_normals[v0].x << " " << vertex_normals[v0].y << " " << vertex_normals[v0].z << endl;
      cout << vertex_normals[v1].x << " " << vertex_normals[v1].y << " " << vertex_normals[v1].z << endl;  
      cout << vertex_normals[v2].x << " " << vertex_normals[v2].y << " " << vertex_normals[v2].z << endl;

    }

  cout << "test some vector math" << endl;
  v1 = triangles[0][0];
  v2 = triangles[0][1];
  v3 = triangles[0][2];
  
  Vector3 vec1(vertexes[v1], vertexes[v2]);
  Vector3 vec2(vertexes[v2], vertexes[v3]);
  
  Vector3 vec3 = vec1.add(vec2);
  cout << vec1 << " + " << vec2 << " = " << vec3 << endl;
  vec3 = vec1.sub(vec2);
  cout << vec1 << " - " << vec2 << " = " << vec3 << endl;
  vec3 = vec1.mult(0.2);
  cout << vec1 << " * " << "0.2" << " = " << vec3 << endl; 
  vec3 = vec1.cross_product(vec2);
  cout << vec1 << " x " << vec2 << " = " << vec3 << endl; 

  cout << vec1 << " length is = " << vec1.length() << endl;
  cout << vec1 << " normalized is = " << vec1.normalize() << endl;

}
//needs triangles AND the computed vertices
 void Face::generate_vertex_normals(void)
 {
   Vector3 *surface_normals = new (nothrow) Vector3[poly_num];
   if(surface_normals == NULL)cerr<<"error allocating vertex normals"<<endl;
   //array to tell us how many polygons share a vertex
   int contributes[point_num];
   Vector3 *vec1, *vec2;
   int v1,v2,v3;

   for(int i=0; i<point_num; i++)contributes[i] = 0;
   
   for(int i=0; i<poly_num; i++)
     {
       v1 = triangles[i][0];
       v2 = triangles[i][1];
       v3 = triangles[i][2];
       
       vec1 = new Vector3(vertexes[v1],vertexes[v2]);
       vec2 = new Vector3(vertexes[v2],vertexes[v3]);
       //no need to normalize now since i will be throwing them away anyway
       surface_normals[i] = vec1->cross_product(*vec2);
       delete vec1;
       delete vec2;
     }
   
   vertex_normals = new (nothrow) Vector3[point_num];
   if(vertex_normals == NULL)cerr<<"error allocating vertex normals"<<endl;
   
   for(int i=0; i<poly_num; i++)
     {
       v1 = triangles[i][0];
       v2 = triangles[i][1];
       v3 = triangles[i][2];
       
       contributes[v1]++;
       contributes[v2]++;
       contributes[v3]++;
       
       vertex_normals[v1].addTo(surface_normals[i]);
       vertex_normals[v2].addTo(surface_normals[i]);
       vertex_normals[v3].addTo(surface_normals[i]);
     }

   //now average
   for(int i=0; i<point_num; i++)
     {
       //if == 0 theres an isolated vertex
       if( contributes[i] ==  0) 
	 {
	   cerr << "we have an isolated vertex at " << i << endl;
	   continue;
	 }
       vertex_normals[i].divideBy(contributes[i]);
       //normalizing made a huge difference !!! from no lighting to lighting
       vertex_normals[i] = vertex_normals[i].normalize();
       
     }
   delete[] surface_normals;   
 }

void Face::loadPolygonDataFromModel()
{
    point_num = model->getPointNum();
    poly_num = model->getPolyNum();

    triangles = new (nothrow) float[poly_num][3];
    if(triangles == NULL)
    {
        cerr << "error allocating memory for triangles" << endl;
        return;
    }

    for(int i=0;i<poly_num;i++)
    {
        triangles[i][0] = model->triangles[i][0];
        triangles[i][1] = model->triangles[i][1];
        triangles[i][2] = model->triangles[i][2];
    }
}

void Face::load(string filename, const char *tex_map_filename)
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
  vertexes = new (nothrow) Point3[point_num];
  if(vertexes == NULL)
  {
      cerr << "error allocating memory for vertexes" << endl;
      return;
  }

  //one more line telling us its float
  file_op >> str;

  for(int i=0; i<point_num; i++)
    file_op >> vertexes[i].x >> vertexes[i].y >> vertexes[i].z;
    
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
    
  /**************************************************/
  /** load TEXTURE_COORD ***/
  /**************************************************/
  
  // fstream tex_map_op(tex_map_filename,ios::in);
  // int w,h,maxval;
  
  // tex_map_op.getline(line,256);
  // //tex_map_op.getline(line,256);
  // tex_map_op >> w >> h;
  // cout << w << " " << h << endl;
  // //c++ as java has row major ordering (rows in memory)
  // Color3 texture_map[h][w];
  // Color3 col;
  
  // tex_map_op >> maxval;
  // cout << maxval << endl;
  // for(int i=0; i<h; i++)
  //   {
  //     for(int j=0; j<w; j++)
  // 	{	  
  // 	  tex_map_op >> col.r >> col.g >> col.b;
  // 	  col.r = col.r/maxval;
  // 	  col.g = col.g/maxval;
  // 	  col.b = col.b/maxval;	  
  // 	  //does this copy or overwrite ??? POSSIBLE ERROR
  // 	  texture_map[h][w] = col;
  // 	}
  //   }
  
  // file_op.getline(line,256);
  // file_op.getline(line,256);
  // file_op.getline(line,256);
  // file_op >> str >> texture_num;  
  // //error if wrong label
  // if(str.compare("POINT_DATA") != 0) 
  //   {
  //     cerr << "did not match the label POINT_DATA" << endl;
  //     return;
  //   }
  // /*a line with 4 things we dont need */
  // file_op >> str;
  // file_op >> str;
  // file_op >> dont_need_int;
  // file_op >> str;
  // cerr << str << endl;
  
  // texture_2d_coord = new (nothrow) float[texture_num][2];
  // vertex_texture = new (nothrow) Color3[texture_num];
  
  // if(vertex_texture == NULL)
  //   {
  //     cerr << "error allocating memory for texture" << endl;
  //     return;
  //   }

  // float texture_2d_coord_x, texture_2d_coord_y;
  // float rest_a, rest_b;
  // Color3 a,b,c,d;
  
  // //bilinear interpolation of texture values .. why not
  // for(int i=0; i<texture_num; i++)
  //   {
  //     file_op >> texture_2d_coord_x >> texture_2d_coord_y;
  //     a = texture_map[(int)floor(texture_2d_coord_y*h)][(int)floor(texture_2d_coord_x*w)];
  //     b = texture_map[(int)floor(texture_2d_coord_y*h)][(int)ceil(texture_2d_coord_x*w)];
  //     c = texture_map[(int)ceil(texture_2d_coord_y*h)][(int)floor(texture_2d_coord_x*w)];
  //     d = texture_map[(int)ceil(texture_2d_coord_y*h)][(int)ceil(texture_2d_coord_x*w)];
      
  //     rest_a = texture_2d_coord_x*w - floor(texture_2d_coord_x*w);
  //     rest_b = texture_2d_coord_y*h - floor(texture_2d_coord_y*h);
      
  //     vertex_texture[i] = interpolate_color(a,b,c,d,rest_a,rest_b);
  //   }
        
  
  // // for(int i=0; i<texture_num; i++)
  // //   file_op >> texture_2d_coord[i][0] >> texture_2d_coord[i][1];



  /***** generate vertex normals ******/
  generate_vertex_normals();  
  file_op.close();
}
