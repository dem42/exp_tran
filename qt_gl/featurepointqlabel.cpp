#include "featurepointqlabel.h"

#include "clickableqlabel.h"

#include <QPainter>
#include <QPointF>
#include <cmath>
using namespace std;
using namespace cv;


//initialize static attributes
const QString FeaturePointQLabel::label_msgs[13] = {"tip of the nose","left mouth corner","right mouth corner",
                            "center of upper lip","center of bottom lip","left eye left corner",
                            "left eye right corner","right eye left corner","right eye right corner",
                            "left corner of left eyebrow","right corner of left eyebrow",
                            "left corner of right eyebrow","right corner of right eyebrow"};

const int FeaturePointQLabel::msgs_size = 13;


FeaturePointQLabel::FeaturePointQLabel() : ClickableQLabel(false)
{
}


void FeaturePointQLabel::paintEvent ( QPaintEvent * e)
{
    QPainter paint(this);
    paint.setPen(Qt::red);
    //cout << "painting" << endl;
    //QImage img("/home/martin/iowndis.png");
    //QPixmap pmap = QPixmap::fromImage(img);
    QPixmap pmap = *(this->pixmap());
    int x,y,w,h;
    //make the points smaller if we are drawing vs larger when we are just clicking
    w = h = 4;

    int x_shift = getXShift();
    int y_shift = getYShift();
    //marked from superclass is private so this covers its visibility
    vector<Point2f> marked = getMarked();

    if(this->pixmap() == 0)
    {
        cout << "there is no pixmap" << endl;
    }
    else
    {
        paint.drawPixmap(0,0,pmap,x_shift,y_shift,600,650);

        paint.drawText(QRectF(0,0,100,30),label_msgs[marked.size()%msgs_size]);
        QPointF *points = new QPointF[marked.size()];
        for(int i=0; i < marked.size(); i++)
        {
            x = marked[i].x;
            y = marked[i].y;

            paint.fillRect(x,y,w,h,Qt::red);

        }
    }
}
