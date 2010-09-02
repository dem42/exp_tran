#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <QGLFormat>
#include <QFileDialog>

#include "model/Vector3.h"

#include "facewidget.h"

#define PI 3.14159265

using namespace std;

FaceWidget::FaceWidget(QGLWidget *parent) : QGLWidget(parent)
{
  cout << "building" << endl;
  setFormat(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer));

  polygonNumber = 0;

  gl_display_style = GL_POLYGON;

  face_index = -1;

  face_ptr = new Face();
  texture_coord = new Point2[face_ptr->getPointNum()];
  texture_id = -1;

  //initialize camera parameters
  rot_x = 0;
  rot_y = 0;
  rot_z = 0;
  trans_x = 0;
  trans_y = 0;
  trans_z = 0;

//  trans_x = -77.04;
//  trans_y = -14.157;
//  trans_z = +140.0 + 1500.0;
//-0.31989 -0.914053 -0.249355 -0.922929 0.360119 -0.136078 0.21418 0.186607 -0.958804
  //0.479993 0.0882419 -0.872801 0.00623718
  center_x = -2.8741;
  center_y = 19.915;
  center_z =  1518;
  diameter = 100;

  cameraDistance = 500;
  cameraZPosition = 200;
  upVector = 1;
}

void FaceWidget::refreshGL()
{
    updateGL();
}

void FaceWidget::setFace(Face* face_ptr)
{
    this->face_ptr = face_ptr;
    this->polygonNumber = face_ptr->getPolyNum();
    face_ptr->calculateBoundingSphere(center_x,center_y,center_z,diameter);
    cout << center_x << " " << center_y << " " << center_z << " " << diameter << endl;
    this->resizeGL(this->width(), this->height());
    this->updateGL();
}

void FaceWidget::setFace(Face* face_ptr, Point2 *texture_coord)
{
    cout << "in set face with text coord" << endl;
    this->face_ptr = face_ptr;
    this->polygonNumber = face_ptr->getPolyNum();

    for(int i=0;i<face_ptr->getPointNum();i++)
    {
        this->texture_coord[i].x = texture_coord[i].x;
        this->texture_coord[i].y = texture_coord[i].y;
    }

    face_ptr->calculateBoundingSphere(center_x,center_y,center_z,diameter);
    cout << center_x << " " << center_y << " " << center_z << " " << diameter << endl;
    this->resizeGL(this->width(), this->height());
    this->updateGL();
}

void FaceWidget::render()
{  
  int v1,v2,v3;
  GLUquadric *quad = gluNewQuadric();

  Point3 *vertexes = face_ptr->vertexes;
  Vector3 *vertex_normals = face_ptr->vertex_normals;
  float (*triangles)[3] = face_ptr->triangles;

  vector<int>::iterator result;
  vector<int> fPoly;
  vector<int> mouthPoly;
  fPoly.assign(Face::fPolygons,Face::fPolygons+Face::fPoints_size);
  mouthPoly.assign(Face::mouth,Face::mouth+Face::mouth_size);

  vector<int>fP;
  fP.assign(Face::fPoints,Face::fPoints+Face::fPoints_size);

  //nothing to render
  if(face_ptr == NULL)
      return;

  //if(texture_id != 0)
  //{
      glEnable(GL_TEXTURE_2D);
      //glBindTexture(GL_TEXTURE_2D, texture_id);
  //}

  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

  for(int i=0; i<polygonNumber; i++)
    {

      result = std::find(mouthPoly.begin(),mouthPoly.end(),i);
      //this is a mouth polygon so dont render it
      if(result != mouthPoly.end())
          continue;

      v1 = triangles[i][0];
      v2 = triangles[i][1];
      v3 = triangles[i][2];

      //try to find if polygon i is a feature point
      result = std::find(fPoly.begin(),fPoly.end(),i);


      result = std::find(fP.begin(),fP.end(),v1);
      if(result != fP.end())
      {
            glPushMatrix();
            glTranslatef(vertexes[v1].x,vertexes[v1].y,vertexes[v1].z);

            glColor3f(0.5f, 0.0f, 0.0f); // red reflective properties            
            //adjust to the correct diameter
            gluSphere(quad,diameter*(3./144.32) ,10,10);

            glPopMatrix();
      }


      //if the polygon i was double clicked or is a feature point .. highlight it
//      if(i == face_index || result != fPoly.end())
//      {
//          // enable color tracking
//          glEnable(GL_COLOR_MATERIAL);
//          // set material properties which will be assigned by glColor
//          glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
//
//          glColor3f(1.0f, 0.0f, 0.0f); // red reflective properties
//      }
//      else
//      {
//          glEnable(GL_COLOR_MATERIAL);
//          glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
//          glColor3f(0.0f, 0.2f, 0.4f);
//      }

      //calls to glLoadName are ignored if we arent in GL_SELECT render mode
      //its used to tell us what the user clicked on
      glLoadName(i);
      glColor3f(0.6f, 0.6f, 0.6f); // grey material properties
      glBegin(gl_display_style);

      glTexCoord2f(texture_coord[v1].x,texture_coord[v1].y);
      glNormal3f(vertex_normals[v1].x,vertex_normals[v1].y,vertex_normals[v1].z);
      glVertex3f(vertexes[v1].x,vertexes[v1].y,vertexes[v1].z);

      glTexCoord2f(texture_coord[v2].x,texture_coord[v2].y);
      glNormal3f(vertex_normals[v2].x,vertex_normals[v2].y,vertex_normals[v2].z);
      glVertex3f(vertexes[v2].x,vertexes[v2].y,vertexes[v2].z);

      glTexCoord2f(texture_coord[v3].x,texture_coord[v3].y);
      glNormal3f(vertex_normals[v3].x,vertex_normals[v3].y,vertex_normals[v3].z);
      glVertex3f(vertexes[v3].x,vertexes[v3].y,vertexes[v3].z);
      glEnd();

    }

  //if(texture_id != 0)
      glDisable(GL_TEXTURE_2D);
  glDisable(GL_COLOR_MATERIAL);
}

void FaceWidget::setTransParams(double r_x, double r_y, double r_z, double t_x, double t_y, double t_z)
{
    rot_x = r_x;
    rot_y = r_y;
    rot_z = r_z;
    cout << "ROTATION" << rot_x << " " << rot_y << " " << rot_z << endl;
    trans_x = t_x;
    trans_y = t_y;
    trans_z = t_z;
    cout << "TRANSLATION" << trans_x << " " << trans_y << " " << trans_z << endl;
}

void FaceWidget::mousePressEvent(QMouseEvent *event)
{
  lastPos = event->pos();  
}

void FaceWidget::mouseMoveEvent(QMouseEvent *event)
{
  GLfloat dx = GLfloat(event->x() - lastPos.x()) / width();
  GLfloat dy = GLfloat(event->y() - lastPos.y()) / height();

  //entire width and entire height is 180 degrees .. so the fraction that we move by is 180*dx (or dy)
  if (event->buttons() & Qt::LeftButton)
    {
      rot_x += 180 * dy;
      rot_y += 180 * dx;
      updateGL();
    }
  lastPos = event->pos();
}


void FaceWidget::initializeGL(void)
{

  //background color
  qglClearColor(Qt::white);

  //scary lighting
  //GLfloat LightPosition[] =  { 2.8741, -19.915, 1.0, 0.0 };
  //GLfloat LightPosition[] =  { 0.0, 0.0, cameraZPosition, 0.0 };
  GLfloat LightSpecular[] = { 1.0, 1.0, 1.0 };
  GLfloat LightAmbient[] = { 0.0, 0.0, 0.0 };
  GLfloat LightDiffuse[] = { 1.0, 1.0, 1.0 };

  //white material specular
//  GLfloat MaterialSpecular[] = { 0.0, 0.0, 1.0 };
  GLfloat MaterialSpecular[] = { 1.0, 1.0, 1.0 };
  //128 being not that shiny
  GLfloat MaterialShininess[] = {128};

  glShadeModel (GL_SMOOTH);

  // Enable lighting
  glEnable (GL_LIGHTING);
  glEnable (GL_LIGHT0);
  //light position is transformed by model view matrix
  //glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
  glLightfv(GL_LIGHT0, GL_AMBIENT,  LightAmbient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  LightDiffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);

  // Set material parameters
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  MaterialSpecular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, MaterialShininess);

  // Enable Z-buffering
  glEnable(GL_DEPTH_TEST);

}

void FaceWidget::setCameraParameters(double cameraZPosition, double upVector, double cameraDistance)
{
    this->cameraZPosition = cameraZPosition;
    this->cameraDistance = cameraDistance;
    this->upVector = upVector;
}

void FaceWidget::zoom(int step)
{
    double increment = std::abs(step) / 2.;
    if(step > 0)
        diameter *= increment;
    else
        diameter /= increment;
    this->resizeGL(this->width(),this->height());
    this->updateGL();
}

void FaceWidget::wheelEvent(QWheelEvent *event)
{
    cout << "IN WHEEL EVENT" << endl;
    //mouse work in steps of 15 degrees and delta is in eights of a degree
    int numDegrees = event->delta() / 8;
    int numSteps = numDegrees / 15;

    zoom(numSteps);

    event->accept();
}

void FaceWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    face_index = selectPoint(event->pos());
    cout << "FACE INDEX " << face_index << endl;    
    cout << "POINT INDEX " << face_ptr->getPointIndexFromPolygon(face_index) << endl;
    updateGL();
}

void FaceWidget::setupFrustumParameters(GLdouble &left, GLdouble &right, GLdouble &bottom,
                                         GLdouble &top, GLdouble &near, GLdouble &far)
{
    left = center_x - diameter/2.;
    right = center_x + diameter/2.;
    bottom = center_y - diameter/2.;
    top = center_y + diameter/2.;

    near = center_z + diameter;
    far = center_z + 5*diameter;

    GLdouble aspect = (GLdouble) this->width() / this->height();
    if ( aspect < 1.0 )
    {
        // window taller than wide
        bottom /= aspect;
        top /= aspect;
    }
    else
    {
        left *= aspect;
        right *= aspect;
    }
    cout <<  "frustum params are: " << left << " " << right << " " << top << " " << bottom << " " << near << " " << far << endl;
}


//standard opengl GL_SELECT render mode trickery to
//get which polygon gets rendered at pos
int FaceWidget::selectPoint(const QPoint &pos)
{
    const int MaxSize = 512;
    GLuint buffer[MaxSize];
    GLint viewport[4];
    GLdouble left,right,bottom,top,near,far;

    makeCurrent();

    glGetIntegerv(GL_VIEWPORT, viewport);
    glSelectBuffer(MaxSize, buffer);
    glRenderMode(GL_SELECT);

    glInitNames();
    glPushName(0);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluPickMatrix(GLdouble(pos.x()), GLdouble(viewport[3] - pos.y()),
                  5.0, 5.0, viewport);

    setupFrustumParameters(left,right,bottom,top,near,far);  
    glFrustum(left,right,bottom,top,near,far);

    paintGL();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    if (!glRenderMode(GL_RENDER))
        return -1;
    return buffer[3];
}


void FaceWidget::wireFrameChecked(bool checked)
{
    setWireFrame(checked);
}

void FaceWidget::setWireFrame(bool on)
{
    if(on == true)
        gl_display_style = GL_LINE_LOOP;
    else
        gl_display_style = GL_POLYGON;
}

void FaceWidget::resizeGL(int width, int height)
{
  cout << "in resize" << endl;
  GLdouble left,right,bottom,top,near,far;

  //coordinates of 2D plane (the one we're projecting to)
  glViewport(0, 0, (GLsizei)width, (GLsizei)height);
  //switch to projection matrix
  glMatrixMode(GL_PROJECTION);
  //reset to identity matrix
  glLoadIdentity();

  //use frustum instead of gluPerspective .. frustum more flexible
  //frustum specifies viewing volume
  //perspective just does frustum with focal = trigonometry(fov)
  setupFrustumParameters(left,right,bottom,top,near,far);
  glFrustum(left,right,bottom,top,near,far);

  //switch back to transformation matrix
  glMatrixMode(GL_MODELVIEW);
}


//the order of applying transformations is REVERSE in opengl LAST TRANSFORMATION HAPPENS FIRST
//this means if we start with loadIdentity then rotate by R then translate by T we get
//I*R*T*v where v are our vertices
void FaceWidget::paintGL(void)
{

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  //reload so that transformations dont stack
  glLoadIdentity();

  //gluLookAt is a viewing not a modelling transformation!!
  gluLookAt(0.0, 0.0, upVector*2*diameter, center_x, center_y, center_z, 0.0, upVector, 0.0);

  //keep the viewing trans with gluLookAt on the stack
  //here we are trying to keep light movements independent of the
  //viewing transformation of the scene objects
  glPushMatrix();
  GLfloat LightPosition[] =  { 0.0, 0.0, cameraZPosition, 0.0 };
  glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
  glPopMatrix();


  //move the objects by: we dont really need to display this
  //since the objects may get smaller after param optimization
  //we would have to consider them getting smaller and move camera
  //by this too or something .. its just very messy
  //glTranslatef(trans_x,trans_y,trans_z);

  //rotations which will be updated anytime the values of rot_x, rot_y change
  //to get the rotations around the object (which isnt at 0,0,0) we need to
  //not turn just around 1,0,0 and 0,1,0 as usual
  glRotatef(rot_z, 0.0, 0.0, 1.0);
  glRotatef(rot_y, 0.0, 1.0, 0.0);
  glRotatef(rot_x, 1.0, 0.0, 0.0);

  glTranslatef(-center_x,-center_y,-center_z);

  //does the open gl glBegin(GL_POLYGON) glEnd() stuff
  render();
}


void FaceWidget::setTexture(uchar *img_data, int img_height, int img_width)
{
    GLuint texture;

    //get a texture name
    glGenTextures(1, &texture);

    cout << "in set texture with texture " << texture << endl;

    //bind to the texture name
    glBindTexture (GL_TEXTURE_2D, texture);
    glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    /* glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); */
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    /* glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); */
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    //glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); //blends
    glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL); //covers
    //glTexImage2D (GL_TEXTURE_2D, 0, texFormat, imageWidth, imageHeight, 0, texFormat, GL_UNSIGNED_BYTE, imageData);
    glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB8, img_width, img_height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);

    delete[] img_data;

    texture_id = texture;
}
