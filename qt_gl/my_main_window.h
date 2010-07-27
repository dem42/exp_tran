#ifndef MY_MAIN_WINDOW_H
#define MY_MAIN_WINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include "ui_applform.h"
#include "face_widget.h"
#include "transferwidget.h"


class FaceWidget;

class MyMainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MyMainWindow(QMainWindow *window = 0);
    ~MyMainWindow();
    void setSlider(int);

private:
    Ui::ApplForm ui;
    //custom widgets
    FaceWidget *my_widget;
    TransferWidget *trans_widget;
    //menu dialogs
    QFileDialog *f_dialog;
    QFileDialog *v_dialog;
};

#endif // MY_MAIN_WINDOW_H
