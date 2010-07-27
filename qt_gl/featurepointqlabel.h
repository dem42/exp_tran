#ifndef FEATUREPOINTQLABEL_H
#define FEATUREPOINTQLABEL_H

#include "clickableqlabel.h"

#include <cv.h>
#include <highgui.h>
#include <vector>
#include <iostream>

class FeaturePointQLabel : public ClickableQLabel
{
public:
    FeaturePointQLabel();
protected:
    void paintEvent( QPaintEvent * );
private:
    //static attributes
    static const QString label_msgs[13];
    static const int msgs_size;
};

#endif // FEATUREPOINTQLABEL_H
