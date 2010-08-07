#ifndef MY_MAIN_WINDOW_H
#define MY_MAIN_WINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include "ui_applform.h"
#include "face_widget.h"
#include "transferwidget.h"

#include "facetabcontroller.h"


class MyMainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MyMainWindow(QMainWindow *window = 0);
    ~MyMainWindow();    

private:
    Ui::ApplForm ui;
    //controllers
    FaceTabController *f_controller;
    //custom widgets
    FaceWidget *face_widget1;
    FaceWidget *face_widget2;
    TransferWidget *trans_widget;
    //menu dialogs
    QFileDialog *f_dialog;
    QFileDialog *v_dialog;    
};

#endif // MY_MAIN_WINDOW_H
