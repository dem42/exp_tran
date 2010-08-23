#include "clickableqlabel.h"
#include <iostream>
#include <QPainter>

using namespace std;

ClickableQLabel::ClickableQLabel(bool draw) : drawable(draw), drawing(false)
{
    x_shift = 0;
    y_shift = 0;
}

ClickableQLabel::~ClickableQLabel()
{
    marked.clear();
}

void ClickableQLabel::setXShift(int x)
{
    x_shift = x;
}

void ClickableQLabel::setYShift(int y)
{
    y_shift = y;
}
int ClickableQLabel::getXShift() const
{
    return x_shift;
}
int ClickableQLabel::getYShift() const
{
    return y_shift;
}

vector<cv::Point2f> ClickableQLabel::getMarked() const
{
    return marked;
}

void ClickableQLabel::setMarked(std::vector<cv::Point2f> &input)
{
    marked.clear();
    for(vector<cv::Point2f>::iterator it=input.begin();it<input.end();it++)
        marked.push_back(*it);
    this->update();
}

void ClickableQLabel::clearMarked()
{
    marked.clear();
}

void ClickableQLabel::setDrawable(bool drawable)
{
    this->drawable = drawable;
}

void ClickableQLabel::mousePressEvent ( QMouseEvent * ev )
{
    cv::Point2f point(ev->x(),ev->y());
    if(drawable == true)
         drawing = true;
    marked.push_back(point);
    this->update();
}

void ClickableQLabel::mouseMoveEvent(QMouseEvent *ev)
{
    if(drawing == true)
    {
        cv::Point2f point(ev->x(),ev->y());
        marked.push_back(point);
        this->update();
    }
}
void ClickableQLabel::mouseReleaseEvent(QMouseEvent *event)
{
    drawing = false;
}

void ClickableQLabel::paintEvent ( QPaintEvent * e)
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

    if(this->pixmap() == 0)
    {
        cerr << "there is no pixmap" << endl;
    }
    else
    {
        paint.drawPixmap(0,0,pmap,x_shift,y_shift,600,650);

        QPointF *points = new QPointF[marked.size()];
        for(int i=0; i < marked.size(); i++)
        {
            x = marked[i].x;
            y = marked[i].y;

            if(drawable == false)
                paint.fillRect(x,y,w,h,Qt::red);
            else
                points[i] = QPointF(x,y);
        }
        if(drawable == true)
            paint.drawPoints(points,marked.size());
        delete points;
    }
}

