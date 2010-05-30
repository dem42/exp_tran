#include <GL/gl.h>
#include <GL/glut.h>

GLfloat angle = 0.0;


void renderPrimitives(void) 
{
  glBegin(GL_LINE_LOOP);
  
  glVertex3f(-1.0f, -1.0f, 0.0f);
  
  glVertex3f(1.0f, -1.0f, 0.0f);

  glVertex3f(1.0f, 1.0f, 0.0f);

  glVertex3f(-1.0f, 1.0f, 0.0f);

  glEnd();
}

void rotatingCube(void)
{
  //rotate .. angle and which axis
  glRotatef(angle, 1.0, 0.0, 0.0);
  //glRotatef(angle, 0.0, 1.0, 0.0);
  //glRotatef(angle, 0.0, 0.0, 1.0);
  
  glColor3f(1.0f, 1.0f, 0.0f);
    
  glutWireCube(2);
}


void display(void) 
{
  glClearColor(0.f, 0.f, 0.f, 1.f);
  
  //Clear the colour buffer (more buffers later on)  
  glClear (GL_COLOR_BUFFER_BIT);
  // Load the Identity Matrix to reset our drawing locations    
  glLoadIdentity();
  gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  
  //translate the camera backwards so that we draw beyond the near plane which is at 1.0f
  //camera is now at 0,0,-5 and the origin at 0,0,0
  //glTranslatef(0.0f, 0.0f, -5.0f); made obsolete by the gluLookAt
  
  //renderPrimitives();
  
  rotatingCube();
    
  glutSwapBuffers();
  angle++;  
}

void reshape(int width, int height)
{
  //coordinates of 2D plane (the one we're projecting to)
  glViewport(0, 0, (GLsizei)width, (GLsizei)height);
  
  //switch to projection matrix
  glMatrixMode(GL_PROJECTION);
  //reset to identity matrix
  glLoadIdentity();
  //now set projection coordinates .. FOV, aspect ratio, near, far plane
  gluPerspective(60, (GLfloat)width/(GLfloat)height, 1.0, 100.0);
  
  //switch back to transformation matrix
  glMatrixMode(GL_MODELVIEW);
}


int main(int argc, char** argv)
{
  //initialize GLUT
  glutInit(&argc, argv);
  
  //single or double buffering (stores a buffer)
  glutInitDisplayMode(GLUT_DOUBLE);
  
  glutInitWindowSize(500, 500);
  glutInitWindowPosition(100, 100);
  
  glutCreateWindow("Window title");

  //tell glut what to run in the main loop
  glutDisplayFunc(display);
  
  //idle func called when something changes i think
  glutIdleFunc(display);
  
  glutReshapeFunc(reshape);
    
  glutMainLoop();
  
}
