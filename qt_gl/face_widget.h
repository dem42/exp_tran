#ifndef FACE_WIDGET_H
#define FACE_WIDGET_H

#include <QWidget>
#include <QMouseEvent>
#include <QGLWidget>
#include "Face.h"
#include <map>
#include <QString>
#include "my_main_window.h"

class MyMainWindow;

class FaceWidget : public QGLWidget
{
    Q_OBJECT

public:
  FaceWidget(MyMainWindow *win, QGLWidget *parent = 0);
  enum ExprType { ANGRY=0, DISGUST=1, FEAR=2, HAPPY=3, NEUTRAL=4, SAD=5, SURPRISE=6 };
  void setTransParams(double r_x, double r_y, double r_z, double t_x, double t_y, double t_z);

   struct Pose {
        double rot_x;
        double rot_y;
        double rot_z;
        double trans_x;
        double trans_y;
        double trans_z;
    };

protected:
  void initializeGL(void);
  void paintGL(void);
  void paintGL2(void);
  void paintGL3(void);

  void resizeGL(int w, int h);
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void mouseDoubleClickEvent(QMouseEvent *event);

public slots:
   void face_file_changed();
   void face_file_changed(const QString str);
   void render_action();
   void expression_activated(const QString str);
   void identity_activated(int);
   void slider_moved(int);
   void wireFrameChecked(bool);

private:
  int selectPoint(const QPoint &pos);

  Face *face_ptr;
  QPoint lastPos;

  //extrinsic scene parameters
  //rotation
  float rot_x;
  float rot_y;
  float rot_z;
  //translation
  float trans_x;
  float trans_y;
  float trans_z;
  //center
  float center_x;
  float center_y;
  float center_z;

  QString face_filename;
  std::map<QString,ExprType> expr_map;
  double *w_exp;
  double *w_id;
  MyMainWindow *window;
  int current_expr;
  int current_ident;
};

#endif // FACE_WIDGET_H

