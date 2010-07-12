#ifndef CLICKABLEQLABEL_H
#define CLICKABLEQLABEL_H

#include <QLabel>
#include <QMouseEvent>
#include <QPaintEvent>
#include <iostream>
#include <vector>
#include <cv.h>


class ClickableQLabel : public QLabel
{
public:    
    ClickableQLabel();
    ~ClickableQLabel();
    std::vector<cv::Point2f> getMarked() const;
protected:
    void mousePressEvent(QMouseEvent * ev);
    void paintEvent( QPaintEvent * );
private:
    std::vector<cv::Point2f> marked;
};

#endif // CLICKABLEQLABEL_H
