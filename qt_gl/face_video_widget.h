#ifndef FACE_VIDEO_WIDGET_H
#define FACE_VIDEO_WIDGET_H

#include <phonon>

using namespace Phonon;

class FaceVideoWidget
{
public:
    FaceVideoWidget();
    VideoWidget * get_video_widget();

private:
    VideoWidget *video_widget;
    MediaObject *m_object;

};

#endif // FACE_VIDEO_WIDGET_H
