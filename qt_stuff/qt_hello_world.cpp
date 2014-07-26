#include <QApplication>
#include <QPushButton>

class MyQWidget : public QWidget 
{
public:
  MyQWidget(QWidget *parent = 0);
};

//calls constructor of the parent
MyQWidget::MyQWidget(QWidget *parent) : QWidget(parent)
{
  setFixedSize(200, 120);
  
  QPushButton *quit = new QPushButton("Quit", this);
  //set geometry is a resize and a move
  quit->setGeometry(62, 40, 75, 30);
  
  //qApp is a global variable of QApplication that points to the unique QApplication instance
  //and this is inheriting from QWidget so we dont need to :: when calling the static
  connect(quit, SIGNAL(clicked()), qApp, SLOT(quit()));
}


int main(int argc, char** argv)
{
  QApplication appl(argc, argv);
  MyQWidget myq;
  myq.show();
      
  return appl.exec();
}

  
