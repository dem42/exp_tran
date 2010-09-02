#include "customizablefacewidget.h"

CustomizableFaceWidget::CustomizableFaceWidget(QGLWidget *parent) : FaceWidget(parent)
{

  viewport_width = width();
  viewport_height = height();

  customTrans = false;
}


void CustomizableFaceWidget::resizeGL(int width, int height)
{
  cout << "in resize" << endl;
  //switch to projection matrix
  glMatrixMode(GL_PROJECTION);
  glViewport(0, 0, 2*viewport_width, 2*viewport_height);

  //load the custom projection matrix (obtained from opt)
  for(int i=0;i<16;i++)
      cout << " i : " << projM[i] << endl;
  glLoadMatrixf(projM);

  //switch back to transformation matrix
  glMatrixMode(GL_MODELVIEW);
}


//the order of applying transformations is REVERSE in opengl LAST TRANSFORMATION HAPPENS FIRST
//this means if we start with loadIdentity then rotate by R then translate by T we get
//I*R*T*v where v are our vertices
void CustomizableFaceWidget::paintGL(void)
{

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  //reload so that transformations dont stack
  glLoadIdentity();

  if(customTrans == false)
  {
      //gluLookAt is a viewing not a modelling transformation!!
      //gluLookAt(0.0, 0.0, upVector*2*diameter, center_x, center_y, center_z, 0.0, upVector, 0.0);

      //gluLookAt(0.0, 0.0, cameraZPosition, 0, 0, 0, 0.0, upVector, 0.0);


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
      glTranslatef(trans_x,trans_y,trans_z);

      //rotations which will be updated anytime the values of rot_x, rot_y change
      //to get the rotations around the object (which isnt at 0,0,0) we need to
      //not turn just around 1,0,0 and 0,1,0 as usual
      glRotatef(rot_z, 0.0, 0.0, 1.0);
      glRotatef(rot_y, 0.0, 1.0, 0.0);
      glRotatef(rot_x, 1.0, 0.0, 0.0);

      //glTranslatef(-center_x,-center_y,-center_z);

  }
  else
  {
      //gluLookAt(0.0, 0.0, upVector*2*diameter, center_x, center_y, center_z, 0.0, upVector, 0.0);
      GLfloat LightPosition[] =  { 0.0, 0.0, cameraZPosition, 0.0 };
      glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
      for(int i=0;i<16;i++)
          cout << " i : " << tranM[i] << endl;
      glLoadMatrixf(tranM);
  }
  //does the open gl glBegin(GL_POLYGON) glEnd() stuff
  render();
}


void CustomizableFaceWidget::setProjectionMatrix(Matrix projM)
{    
    if(projM.getM() != 4 && projM.getN() != 4)
    {
        cerr << "proj matrix size is incorrect" << endl;
        return;
    }

    int count = 0;
    for(int i=0;i<projM.getN();i++)
        for(int j=0;j<projM.getM();j++)
        {
            this->projM[count] = (float)projM[j][i];
            count++;
        }

    viewport_height = this->projM[9];
    viewport_width = this->projM[8];

    this->projM[8] = 0.0;
    //small shif not sure why its necessary
    this->projM[9] = -0.2;
    this->projM[0] *= (1./viewport_width);
    this->projM[5] *= -(1./viewport_height);
    //coz of a stupid zero division when normalizing homogenous .. yay for opengl
    this->projM[11] = 1.0;
    this->projM[15] = 0.001;
}
void CustomizableFaceWidget::setTransformationMatrix(Matrix tranM)
{
    customTrans = true;
    if(tranM.getM() != 4 && tranM.getN() != 4)
    {
        cerr << "trans matrix size is incorrect" << endl;
        return;
    }
    int count = 0;
    for(int i=0;i<tranM.getN();i++)
        for(int j=0;j<tranM.getM();j++)
        {
            this->tranM[count] = (float)tranM[j][i];
            count++;
        }
}
