#ifndef CUSTOMIZABLEFACEWIDGET_H
#define CUSTOMIZABLEFACEWIDGET_H

#include "facewidget.h"

class CustomizableFaceWidget : public FaceWidget
{
public:
    CustomizableFaceWidget(QGLWidget *parent = 0);
    void setProjectionMatrix(Matrix projM);
    void setTransformationMatrix(Matrix tranM);
protected:    
    void paintGL(void);
    void resizeGL(int w, int h);
private:
    //booleans to tell if a trans matrix has been set
    //or if the default one should be used    
    bool customTrans;

    //4x4 column major matrices
    GLfloat projM[16];
    GLfloat tranM[16];

    //viewport parameters
    GLsizei viewport_width;
    GLsizei viewport_height;
};

#endif // CUSTOMIZABLEFACEWIDGET_H
