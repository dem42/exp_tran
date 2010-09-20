#include "vectorfieldqlabel.h"
#include "clickableqlabel.h"

#include <QPainter>
#include <QPointF>
#include <cmath>
using namespace std;
using namespace cv;

VectorFieldQLabel::VectorFieldQLabel(bool draw) : ClickableQLabel(draw)
{

}

vector<Vec2f> VectorFieldQLabel::getVectorField() const
{
    return vectorField;
}

void VectorFieldQLabel::setVectorField(vector<Vec2f>& input)
{
    vectorField.clear();
    for(vector<Vec2f>::iterator it=input.begin();it<input.end();it++)
        vectorField.push_back(*it);
}

void VectorFieldQLabel::clearVectorField()
{
    vectorField.clear();
}

void VectorFieldQLabel::paintEvent( QPaintEvent * )
{
    QPainter paint(this);
    paint.setPen(Qt::red);
    if(this->pixmap() == 0)
    {
        return;
    }
    QPixmap pmap = *(this->pixmap());
    int x,y,w,h;

    //make the points smaller if we are drawing vs larger when we are just clicking
     w = h = 4;


    int x_shift = getXShift();
    int y_shift = getYShift();

    paint.drawPixmap(0,0,pmap,x_shift,y_shift,600,650);
    vector<Point2f> curPoints = getMarked();    
    QPointF p1, p2;
    Vec2f v;
    float angle;
    const float PI = 3.1415926f;

    if(vectorField.size() == 0)
        return;

    for(unsigned int i=0;i<curPoints.size();i++)
    {
        v = vectorField[i];

        p1.setX(curPoints[i].x);
        p1.setY(curPoints[i].y);
        p2.setX( 3*v.val[0] + p1.x() );
        p2.setY( 3*v.val[1] + p1.y() );

        paint.drawLine(p1,p2);
        angle = atan2(-v.val[1], -v.val[0]);
        p1.setX( p2.x() + 9*cos(angle + PI/4) );
        p1.setY( p2.y() + 9*sin(angle + PI/4) );
        paint.drawLine(p2,p1);
        p1.setX( p2.x() + 9*cos(angle - PI/4) );
        p1.setY( p2.y() + 9*sin(angle - PI/4) );
        paint.drawLine(p2,p1);
    }
}
