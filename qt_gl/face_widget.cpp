#include <iostream>
#include <cmath>
#include <QGLFormat>
#include <QFileDialog>

#include "face_widget.h"

#define PI 3.14159265

using namespace std;

FaceWidget::FaceWidget(QGLWidget *parent) : QGLWidget(parent)
{
  cout << "building" << endl;
  setFormat(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer));

  expr_map.insert(std::pair<ExprType,QString>(ANGRY,"Angry"));
  expr_map.insert(std::pair<ExprType,QString>(DISGUST,"Disgust"));
  expr_map.insert(std::pair<ExprType,QString>(FEAR,"Fear"));
  expr_map.insert(std::pair<ExprType,QString>(HAPPY,"Happy"));
  expr_map.insert(std::pair<ExprType,QString>(NEUTRAL,"Neutral"));
  expr_map.insert(std::pair<ExprType,QString>(SAD,"Sad"));
  expr_map.insert(std::pair<ExprType,QString>(SURPRISE,"Suprise"));

  face_ptr = new Face();
  face_filename = "../../JaceyBinghamtonVTKFiles/M0014_HA01WH.vtk";
  face_ptr->load(face_filename.toStdString(),"../face.ppm");
  rot_x = 0;
  rot_y = 0;
}

void FaceWidget::face_file_changed(const QString str)
{    
    cout << str.toStdString() << endl;
    face_filename = str;
    face_ptr->load(face_filename.toStdString(),"../face.ppm");
    updateGL();
}

void FaceWidget::face_file_changed()
{
    cout << "ugh" << endl;
    face_filename = QFileDialog::getOpenFileName(this,
                            tr("Select file to open"),
                            "/home");

    cout << "in face_file_changed with : " << face_filename.toStdString() << endl;
    //face_filename = filename;
    updateGL();
}

void FaceWidget::mousePressEvent(QMouseEvent *event)
{
  lastPos = event->pos();
  //cout << "in press .. location: " << event->pos().x() << " " << event->pos().y() << endl;
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
  cout << "initializing" << endl;

  qglClearColor(Qt::black);

  GLfloat LightPosition[] =  { 0.0, 0.0, -2001.0, 0.0 };
  GLfloat LightSpecular[] = { 1.0, 1.0, 1.0 };
  GLfloat LightAmbient[] = { 0.0, 0.0, 0.0 };
  GLfloat LightDiffuse[] = { 1.0, 1.0, 1.0 };

  //white material specular
  GLfloat MaterialSpecular[] = { 0.0, 0.0, 1.0 };
  //128 being not that shiny
  GLfloat MaterialShininess[] = {128};

  glShadeModel (GL_SMOOTH);

  // Enable lighting
  glEnable (GL_LIGHTING);
  glEnable (GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
  glLightfv(GL_LIGHT0, GL_AMBIENT,  LightAmbient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  LightDiffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);

  // Set material parameters
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  MaterialSpecular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, MaterialShininess);

  // Enable Z-buffering
  glEnable(GL_DEPTH_TEST);

}

void FaceWidget::paintGL(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //reload so that transformations dont stack
  glLoadIdentity();
  gluLookAt(0.0, 0.0, -2000.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

  //rotations which will be updated anytime the values of rot_x, rot_y change
  glRotatef(rot_x, 1.0, 0.0, 0.0);
  glRotatef(rot_y, 0.0, 1.0, 0.0);

  //does the open gl glBegin(GL_POLYGON) glEnd() stuff
  face_ptr->display();
}

void FaceWidget::render_action()
{    
    face_ptr->interpolate();
    updateGL();
}

void FaceWidget::identity_activated(const QString str)
{
    std::cout << "identity activated " << str.toStdString() << std::endl;

}

void FaceWidget::slider_moved(int val)
{
    std::cout << "slider moved by " << (float)val/100 << std::endl;

}

void FaceWidget::paintGL2(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //reload so that transformations dont stack
  glLoadIdentity();
  gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

  //rotations which will be updated anytime the values of rot_x, rot_y change
  glRotatef(rot_x, 1.0, 0.0, 0.0);
  glRotatef(rot_y, 0.0, 1.0, 0.0);

  glBegin(GL_QUADS);

  glVertex3f(-1.0f, -1.0f, 0.0f);

  glVertex3f(1.0f, -1.0f, 0.0f);

  glVertex3f(1.0f, 1.0f, 0.0f);

  glVertex3f(-1.0f, 1.0f, 0.0f);

  glEnd();


  qglColor(Qt::blue);

  glBegin(GL_QUADS);

  glVertex3f(-1.0f, -1.0f, 1.0f);

  glVertex3f(1.0f, -1.0f, 1.0f);

  glVertex3f(1.0f, 1.0f, 1.0f);

  glVertex3f(-1.0f, 1.0f, 1.0f);

  glEnd();

  qglColor(Qt::red);

  glBegin(GL_QUADS);

  glVertex3f(-1.0f, 1.0f, -1.0f);

  glVertex3f(1.0f, 1.0f, -1.0f);

  glVertex3f(1.0f, 1.0f, 1.0f);

  glVertex3f(-1.0f, 1.0f, 1.0f);

  glEnd();

  qglColor(Qt::green);

  glBegin(GL_QUADS);

  glVertex3f(-1.0f, -1.0f, -1.0f);

  glVertex3f(1.0f, -1.0f, -1.0f);

  glVertex3f(1.0f, -1.0f, 1.0f);

  glVertex3f(-1.0f, -1.0f, 1.0f);

  glEnd();

}

void FaceWidget::resizeGL(int width, int height)
{
  cout << "in resize" << endl;

  //coordinates of 2D plane (the one we're projecting to)
  glViewport(0, 0, (GLsizei)width, (GLsizei)height);

  //switch to projection matrix
  glMatrixMode(GL_PROJECTION);
  //reset to identity matrix
  glLoadIdentity();
  //now set projection coordinates .. FOV, aspect ratio, near, far plane
  gluPerspective(60, (GLfloat)width/(GLfloat)height, 1.0, 700.0);

  //switch back to transformation matrix
  glMatrixMode(GL_MODELVIEW);
}
