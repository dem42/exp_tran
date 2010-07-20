#ifndef VECTORFIELDQLABEL_H
#define VECTORFIELDQLABEL_H

#include "clickableqlabel.h"

#include <cv.h>
#include <highgui.h>
#include <vector>
#include <iostream>

class VectorFieldQLabel : public ClickableQLabel
{
public:
    VectorFieldQLabel(bool draw=false);
    std::vector<cv::Vec2f> getVectorField() const;
    void setVectorField(std::vector<cv::Vec2f>&);
    void clearVectorField();
protected:
    void paintEvent( QPaintEvent * );
private:
    std::vector<cv::Vec2f> vectorField;
};

#endif // VECTORFIELDQLABEL_H
