#ifndef MY_MAIN_WINDOW_H
#define MY_MAIN_WINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include "ui_applform.h"
#include "face_widget.h"

class MyMainWindow : public QMainWindow
{
public:
    MyMainWindow(QMainWindow *window = 0);
    ~MyMainWindow();


private:
    FaceWidget *my_widget;
    QFileDialog *f_dialog;
    Ui::ApplForm ui;    
};

#endif // MY_MAIN_WINDOW_H
