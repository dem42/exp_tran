#ifndef FACE_WIDGET_H
#define FACE_WIDGET_H

#include <QWidget>
#include <QMouseEvent>
#include <QGLWidget>
#include "Face.h"



class FaceWidget : public QGLWidget
{
    Q_OBJECT

public:
  FaceWidget(QGLWidget *parent = 0);  
  void setTransParams(double r_x, double r_y, double r_z, double t_x, double t_y, double t_z);

   struct Pose {
        double rot_x;
        double rot_y;
        double rot_z;
        double trans_x;
        double trans_y;
        double trans_z;
    };

   void render();
   void setFace(Face* face_ptr);
   void setWireFrame(bool);
   void setCameraParameters(double cameraZPosition, double upVector, double cameraDistance);

   void refreshGL();

protected:
  void initializeGL(void);
  void paintGL(void);

  void resizeGL(int w, int h);
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void mouseDoubleClickEvent(QMouseEvent *event);

public slots:
   void wireFrameChecked(bool);

private:
  int selectPoint(const QPoint &pos);

  Face *face_ptr;
  int polygonNumber;
  int face_index;
  //whether we display using (line loop)wireframe or polygon (default)
  int gl_display_style;

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
  float diameter;
  //camera
  double cameraZPosition;
  double cameraDistance;
  double upVector;

  QString face_filename;
};

#endif // FACE_WIDGET_H

