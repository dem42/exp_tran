#include <iostream>
#include <QApplication>
#include <QGLFormat>

#include "my_main_window.h"

int main(int argc, char** argv)
{

  QApplication appl(argc, argv);
  
  if(!QGLFormat::hasOpenGL())
    {
      cerr << "No support for OpenGL" << endl;
      return 1;
    }
  MyMainWindow m;
  m.show();
  
  return appl.exec();  
}
