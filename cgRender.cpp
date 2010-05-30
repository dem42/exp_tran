#include <cgRender.h>
#include "Face.h"

void init() 
{
  glClearColor (0.0, 0.0, 0.0, 1.0);
  cout << "init" << endl;

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

void display(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  cout << "display" << endl;
  /*
  glutSolidCube(2);
  glFlush();
  
  glBegin(GL_POLYGON);
  
  glVertex3f(-1,-1,0);
  glVertex3f(-1,0.5,0);
  glVertex3f(1,1,0.2);
  glEnd();

  glBegin(GL_POLYGON);
  
  glVertex3f(-2,-2,0);
  glVertex3f(-2,-1,0);
  glVertex3f(-1,-1,0.2);
  glEnd();


  glFlush();
  */
  
  //glRotatef(-20,0.0,1.0,0.0);
  //glTranslatef(0,0.15,0);
  //glTranslatef(-0.02,0,0);
  
  Face f;
  f.load("../JaceyBinghamtonVTKFiles/F0004_DI02LA.vtk","face.ppm");
  //does the open gl glBegin(GL_POLYGON) glEnd() stuff
  f.display();
  
}

void reshape (int w, int h)
{
  cout << "reshape" << endl;

  glViewport (0, 0, (GLsizei) w, (GLsizei) h); 
  
  GLdouble fovy = 30;
  GLdouble aspect = -(GLfloat)w/(GLfloat)h;
  GLdouble near = 1.0;
  GLdouble far = 700.0;
  
  GLfloat eyex = 0.0;
  GLfloat eyey = 0.0;
  GLfloat eyez = -2000.0;

  GLfloat centerx = 0.0;
  GLfloat centery = 0.0;
  GLfloat centerz = 0.0;

  GLfloat upx = 0.0;
  GLfloat upy = 1.0;
  GLfloat upz = 0.0;
  

  glMatrixMode (GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(fovy, aspect, near, far);
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(eyex, eyey, eyez, centerx, centery, centerz, upx, upy, upz);
  
}

void keyboard(unsigned char key, int x, int y)
{
  switch (key) {
  case 27: // ESC
    exit(0);
    break;
  }
}

int main(int argc, char** argv)
{
  // Initialize graphics window
  glutInit(&argc, argv);
  glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize (256, 256); 
  glutInitWindowPosition (0, 0);
  glutCreateWindow (argv[0]);

  // Initialize OpenGL
  init();

  // Initialize callback functions
  glutDisplayFunc(display); 
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);

  // Start rendering 
  glutMainLoop();
}
