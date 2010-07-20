#include "clickableqlabel.h"
#include <iostream>
#include <QPainter>

using namespace std;

ClickableQLabel::ClickableQLabel(bool draw) : drawable(draw), drawing(false)
{
    this->setText("test text");
    this->adjustSize();
    this->setScaledContents(true);
    cout << this->hasScaledContents() << endl;
}

ClickableQLabel::~ClickableQLabel()
{
    marked.clear();
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
    if(this->pixmap() != 0) cout << "there is a pixmap" << endl;
    cout << "clicked" << ev->x() << " " << ev->y() << endl;    
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
        cout << "there is no pixmap" << endl;
    }
    else
    {
        paint.drawPixmap(0,0,pmap,20,120,600,450);
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

