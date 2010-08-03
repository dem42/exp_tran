#include <iostream>
#include <cmath>
#include <QGLFormat>
#include <QFileDialog>

#include "face_widget.h"

#define PI 3.14159265

using namespace std;

FaceWidget::FaceWidget(MyMainWindow *win, QGLWidget *parent) : QGLWidget(parent)
{
  cout << "building" << endl;
  setFormat(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer));

  expr_map.insert(std::pair<QString,ExprType>("Angry",ANGRY));
  expr_map.insert(std::pair<QString,ExprType>("Disgust",DISGUST));
  expr_map.insert(std::pair<QString,ExprType>("Fear",FEAR));
  expr_map.insert(std::pair<QString,ExprType>("Happy",HAPPY));
  expr_map.insert(std::pair<QString,ExprType>("Neutral",NEUTRAL));
  expr_map.insert(std::pair<QString,ExprType>("Sad",SAD));
  expr_map.insert(std::pair<QString,ExprType>("Surprise",SURPRISE));

  this->window = win;

  face_ptr = new Face();

  //initialize weight vectors
  int i=0;
  w_id = new long double[56];
  w_exp = new long double[7];

  for(i=0;i<56;i++)
  {
      if(i==33)w_id[i] = 0.2;
      else if(i==17)w_id[i] = 0.5;
      else if(i==50)w_id[i] = 0.3;
      else w_id[i] = 0;
  }
  w_exp[0] = 0.0;
  w_exp[1] = 0.0;
  w_exp[2] = 0.0;
  w_exp[3] = 0.2;
  w_exp[4] = 0.0;
  w_exp[5] = 0.8;
  w_exp[6] = 0.0;

  face_ptr->interpolate(w_id,w_exp);

  current_expr = 0;
  current_ident = 33;

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
  center_z =  1518-1500;
}

void FaceWidget::setTransParams(double r_x, double r_y, double r_z, double t_x, double t_y, double t_z)
{
    rot_x = r_x;
    rot_y = r_y;
    rot_z = r_z;
    cout << rot_x << " " << rot_y << " " << rot_z << endl;
    trans_x = t_x;
    trans_y = t_y;
    trans_z = t_z;
    cout << trans_x << " " << trans_y << " " << trans_z << endl;
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


void FaceWidget::paintGL3()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    //glTranslatef(0.0, 0.0, -10.0);
    gluLookAt(0.0, 0.0, -40.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    glRotatef(rot_x, 1.0, 0.0, 0.0);
    glRotatef(rot_y, 0.0, 1.0, 0.0);

    face_ptr->draw();
}



void FaceWidget::initializeGL(void)
{
  cout << "initializing" << endl;

  qglClearColor(Qt::black);

  //scary lighting
  //GLfloat LightPosition[] =  { 2.8741, -19.915, 1.0, 0.0 };
  GLfloat LightPosition[] =  { 0.0, 0.0, -1.0, 0.0 };
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
  glMatrixMode(GL_MODELVIEW);  
  //reload so that transformations dont stack
  glLoadIdentity();
  //ok the 4th, 5th and 6th parameter are what im looking at and they should
  //be the center of the sphere bounding my scene
  //2.8741 -19.915 -1518.46

  //rotations which will be updated anytime the values of rot_x, rot_y change
  //to get the rotations around the object (which isnt at 0,0,0) we need to
  //not turn just around 1,0,0 and 0,1,0 as usual
  glRotatef(rot_z, 0.0, 0.0, 1.0);
  glRotatef(rot_y, 0.0, 1.0, 0.0);
  glRotatef(rot_x, 1.0, 0.0, 0.0);

  //move the objects by:
  glTranslatef(trans_x,trans_y,trans_z);
  //gluLookAt(0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  glTranslatef(0,0,-1000);

  //rotate around norm(r) and r which is the rotation vector
  //glRotatef(145.89, 0.64663, 1.10562, 2.20011);

  //glTranslatef(center_x, center_y, center_z);
  //does the open gl glBegin(GL_POLYGON) glEnd() stuff
  face_ptr->display();
}

void FaceWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    int face_index = selectPoint(event->pos());
    cout << "FACE INDEX " << face_index << endl;
    face_ptr->setColor(face_index);
    updateGL();
}

//standard opengl GL_SELECT render mode trickery to
//get which polygon gets rendered at pos
int FaceWidget::selectPoint(const QPoint &pos)
{
    const int MaxSize = 512;
    GLuint buffer[MaxSize];
    GLint viewport[4];

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
    gluPerspective(60, (GLfloat)width()/(GLfloat)height(), 1.0, 2000.0);
    face_ptr->display();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    if (!glRenderMode(GL_RENDER))
        return -1;
    return buffer[3];
}

void FaceWidget::render_action()
{    
    face_ptr->interpolate(w_id,w_exp);
    updateGL();
}

void FaceWidget::wireFrameChecked(bool checked)
{
    face_ptr->setWireFrame(checked);
}

void FaceWidget::identity_activated(int ident)
{
    std::cout << "identity activated " << ident << std::endl;
    /*if(ident == 42){
        this->setHidden(true);
        window->adjustSize();
    }*/
    w_id[current_ident] = 0;
    w_id[ident-1] = 1;
    current_ident = ident-1;
}

void FaceWidget::expression_activated(const QString str)
{
    current_expr = expr_map[str];
    std::cout << "expression activated " << str.toStdString() << " cur expr is " << current_expr << std::endl;
    window->setSlider(w_exp[current_expr]*100);

}

void FaceWidget::slider_moved(int val)
{
    //std::cout << "slider moved by " << (float)val/100 << std::endl;
    w_exp[current_expr] = (float)val/100.0;
    face_ptr->interpolate(w_id,w_exp);
    updateGL();
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


//  width = 360;
//  height = 288;
  //coordinates of 2D plane (the one we're projecting to)
  glViewport(0, 0, (GLsizei)width, (GLsizei)height);

  //switch to projection matrix
  glMatrixMode(GL_PROJECTION);
  //reset to identity matrix
  glLoadIdentity();
  //now set projection coordinates .. FOV, aspect ratio, near, far plane
  gluPerspective(60, (GLfloat)width/(GLfloat)height, 1.0, 2000.0);

  //switch back to transformation matrix
  glMatrixMode(GL_MODELVIEW);
}
