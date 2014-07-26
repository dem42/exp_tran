#include <iostream>
#include <QApplication>
#include <QGLFormat>

#include "view/exptranwindow.h"
#include "view/exptranapplication.h"

using namespace std;

int main(int argc, char** argv)
{

  ExpTranApplication appl(argc, argv);

  if(!QGLFormat::hasOpenGL())
    {
      cerr << "No support for OpenGL" << endl;
      return 1;
    }
//  ExpTranWindow m;
//  m.show();

  return appl.exec();
}
