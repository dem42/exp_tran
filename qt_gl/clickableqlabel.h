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
    ClickableQLabel(bool draw=false);
    ~ClickableQLabel();
    std::vector<cv::Point2f> getMarked() const;
    void setMarked(std::vector<cv::Point2f>&);
    void clearMarked();
    void setDrawable(bool drawable);
protected:
    void mousePressEvent(QMouseEvent * ev);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void paintEvent( QPaintEvent * );
private:
    bool drawable;
    bool drawing;
    std::vector<cv::Point2f> marked;
};

#endif // CLICKABLEQLABEL_H
