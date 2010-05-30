#include <QApplication>
#include <QPushButton>

int main(int argc, char** argv)
{
  QApplication appl(argc, argv);

  //this is the main window widget
  QPushButton hello("Hello World!");
  hello.resize(100,30);
    
  hello.show();
  
  return appl.exec();
}

  
