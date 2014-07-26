#include <GL/gl.h>
#include <GL/glut.h>

GLfloat angle = 0.0;

GLfloat redDiffuseMaterial[] = 
  {
    1.0, 0.0, 0.0
  };
//set the material to red
GLfloat whiteSpecularMaterial[] = 
  {
    0.0, 0.0, 1.0
  };
//set the material to white
GLfloat greenEmissiveMaterial[] = 
  {
    0.0, 1.0, 0.0
  };
//set the material to green
GLfloat whiteSpecularLight[] = 
  {
    1.0, 1.0, 1.0
  };
//set the light specular to white
GLfloat blackAmbientLight[] = 
  {
    0.0, 0.0, 0.0
  };
//set the light ambient to black
GLfloat whiteDiffuseLight[] = 
  {
    1.0, 1.0, 1.0
  };
//set the diffuse light to white
GLfloat blankMaterial[] = 
  {
    0.0, 0.0, 0.0
  };
//set the diffuse light to white
 GLfloat mShininess[] = 
   {
     128
   };
//set the shininess of the material
bool diffuse = false;
bool emissive = false;
bool specular = false;

void init (void) 
{
  glEnable (GL_DEPTH_TEST);
  glEnable (GL_LIGHTING);
  glEnable (GL_LIGHT0);
}


void light (void) 
{
  glLightfv(GL_LIGHT0, GL_SPECULAR, whiteSpecularLight);
  glLightfv(GL_LIGHT0, GL_AMBIENT, blackAmbientLight);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, whiteDiffuseLight);
}


void display (void) 
{
  glClearColor (0.0,0.0,0.0,1.0);
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  light();
  glTranslatef(0,0,-5);
  glRotatef(angle,1,1,1);
  glutSolidTeapot(2);
  glutSwapBuffers();
  angle ++;
}


void reshape (int w, int h) 
{
  
  glViewport (0, 0, (GLsizei)w, (GLsizei)h);
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  gluPerspective (60, (GLfloat)w / (GLfloat)h, 1.0, 100.0);
  glMatrixMode (GL_MODELVIEW);
}

void keyboard (unsigned char key, int x, int y) 
{
  if(key=='s')
    {
      if (specular==false)
        {
	  specular = true;
	  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, whiteSpecularMaterial);
	  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mShininess);
	}
      else if (specular==true)
        {
	  specular = false;
	  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, blankMaterial);
	  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, blankMaterial);
	}
    }
  if (key=='d')
    {
      if (diffuse==false)
        {
	  diffuse = true;
	  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, redDiffuseMaterial);
	}
      else if (diffuse==true)
	{
	  diffuse = false;
	  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, blankMaterial);
	}
    }
  if (key=='e')
    {
      if (emissive==false)
        {
	  emissive = true;
	  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, greenEmissiveMaterial);
	}
      else if (emissive==true)
	{
	  emissive = false;
	  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, blankMaterial);
	}
    }
}

int main (int argc, char **argv) 
{
  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize (500, 500);
  glutInitWindowPosition (100, 100);
  glutCreateWindow ("A basic OpenGL Window");
  init ();
  glutDisplayFunc (display);
  glutIdleFunc (display);
  glutKeyboardFunc (keyboard);
  glutReshapeFunc (reshape);
  glutMainLoop ();
  return 0;
}

