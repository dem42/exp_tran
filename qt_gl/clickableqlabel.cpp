#include "clickableqlabel.h"
#include <iostream>
#include <QPainter>

using namespace std;

ClickableQLabel::ClickableQLabel()
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

void ClickableQLabel::mousePressEvent ( QMouseEvent * ev )
{
    if(this->pixmap() != 0) cout << "there is a pixmap" << endl;
    cout << "clicked" << ev->x() << " " << ev->y() << endl;
    cv::Point2f point(ev->x(),ev->y());
    marked.push_back(point);
    this->update();
}

void ClickableQLabel::paintEvent ( QPaintEvent * e)
{
    QPainter paint(this);
    cout << "painting" << endl;
    //QImage img("/home/martin/iowndis.png");
    //QPixmap pmap = QPixmap::fromImage(img);
    QPixmap pmap = *(this->pixmap());
    int x,y,w,h;
    w = 4;
    h = 4;
    if(this->pixmap() == 0)
    {
        cout << "there is no pixmap" << endl;
    }
    else
    {
        paint.drawPixmap(0,0,pmap,20,120,600,450);
        for(vector<cv::Point2f>::iterator it = marked.begin();
            it < marked.end(); it++)
        {
            x = (*it).x;
            y = (*it).y;

            paint.fillRect(x,y,w,h,Qt::red);
        }
    }
}

