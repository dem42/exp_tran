#ifndef TRANSFERWIDGET_H
#define TRANSFERWIDGET_H

#include <phonon>
#include <highgui.h>
#include <QWidget>
#include <QLabel>
#include "clickableqlabel.h"

class TransferWidget : public QWidget
{
    Q_OBJECT
public:
    TransferWidget(QLabel *pic = 0);
    ~TransferWidget();
    Phonon::VideoWidget* getVideoWidget() const;
    ClickableQLabel* getPicLabel() const;

public slots:
    void playSource();
    void refreshCapturing();

private:
    Phonon::MediaObject *media;
    Phonon::VideoWidget *vwidget;
    Phonon::AudioOutput *audioOutput;
    //QLabel *picLabel;
    cv::VideoCapture *capture;
    ClickableQLabel *picLabel;
};

#endif // TRANSFERWIDGET_H
