#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <new>
#include "Face.h"

//not too happy about having glut here too :(
#include <GL/glut.h>

using namespace std;


Face::~Face()
{
  delete[] vertexes;
  delete[] vertex_normals;
  delete[] triangles;
  //delete[] vertex_texture;
  //delete[] texture_2d_coord;
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

void Face::display(void)
{
  cout << "in my display with " << poly_num << endl;
  int v1,v2,v3;
  

  for(int i=0; i<poly_num; i++)
    {
      v1 = triangles[i][0];
      v2 = triangles[i][1];
      v3 = triangles[i][2];
      glBegin(GL_POLYGON);      

      // glTexCoord2f(vertex_texture[v1],vertex_texture[v1]);
      //glColor3f(vertex_texture[v1].r,vertex_texture[v1].g,vertex_texture[v1].b);
      glNormal3f(vertex_normals[v1].x,vertex_normals[v1].y,vertex_normals[v1].z);      
      glVertex3f(vertexes[v1].x,vertexes[v1].y,vertexes[v1].z);
      
      //glTexCoord2f(vertex_texture[v2],vertex_texture[v2]);
      //glColor3f(vertex_texture[v2].r,vertex_texture[v2].g,vertex_texture[v2].b);
      glNormal3f(vertex_normals[v2].x,vertex_normals[v2].y,vertex_normals[v2].z);
      glVertex3f(vertexes[v2].x,vertexes[v2].y,vertexes[v2].z);
      
      //glTexCoord2f(vertex_texture[v3],vertex_texture[v3]);
      //glColor3f(vertex_texture[v3].r,vertex_texture[v3].g,vertex_texture[v3].b);
      glNormal3f(vertex_normals[v3].x,vertex_normals[v3].y,vertex_normals[v3].z);
      glVertex3f(vertexes[v3].x,vertexes[v3].y,vertexes[v3].z);
      glEnd();      
    }
  glFlush();
  
    /*
  for (all polygons)
    glBegin(GL_POLYGON);
    for (all vertices of polygon)
      // Define texture coordinates of vertex
      glTexCoord2f(...);
      // Define normal of vertex
      glNormal3f(...);
      // Define coordinates of vertex
      glVertex3f(...);
    }
    glEnd();
  }
  glFlush ();
  */

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


      cout << "texture coord" << endl;
      cout << texture_2d_coord[v0][0] << " " << texture_2d_coord[v0][1] << endl;
      cout << texture_2d_coord[v1][0] << " " << texture_2d_coord[v1][1] << endl;  
      cout << texture_2d_coord[v2][0] << " " << texture_2d_coord[v2][1] << endl;  
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


void Face::load(const char* filename, const char *tex_map_filename) 
{
  string str;
  int dont_need_int;  
  float f1,f2,f3;
  fstream file_op(filename,ios::in);

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

  //one more line telling us its float
  file_op >> str;  

  vertexes = new (nothrow) Point3[point_num];
  if(vertexes == NULL)
    {
      cerr << "error allocating memory for vertexes" << endl;
      return;
    }

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
}
