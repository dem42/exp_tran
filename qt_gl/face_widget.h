#ifndef FACE_WIDGET_H
#define FACE_WIDGET_H

#include <QWidget>
#include <QMouseEvent>
#include <QGLWidget>
#include "Face.h"
#include <map>
#include <QString>

class FaceWidget : public QGLWidget
{
    Q_OBJECT

public:
  FaceWidget(QGLWidget *parent = 0);  
  enum ExprType { ANGRY=0, DISGUST, FEAR, HAPPY, NEUTRAL, SAD, SURPRISE };

protected:
  void initializeGL(void);
  void paintGL(void);
  void paintGL2(void);
  void resizeGL(int w, int h);
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);

public slots:
   void face_file_changed();
   void face_file_changed(const QString str);
   void render_action();
   void identity_activated(const QString str);
   void slider_moved(int);

private:
  Face *face_ptr;
  QPoint lastPos;
  float rot_x;
  float rot_y;
  QString face_filename;
  std::map<ExprType,QString> expr_map;
};

#endif // FACE_WIDGET_H
