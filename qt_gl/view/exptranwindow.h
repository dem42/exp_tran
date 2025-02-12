#ifndef MY_MAIN_WINDOW_H
#define MY_MAIN_WINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include "ui_applform.h"
#include "ui_settingsDialog.h"

#include "facewidget.h"
#include "customizablefacewidget.h"

#include "controller/videotabcontroller.h"
#include "controller/facetabcontroller.h"
#include "controller/transfertabcontroller.h"

#include "view/clickableqlabel.h"
#include "view/vectorfieldqlabel.h"
#include "view/featurepointqlabel.h"

#include "view/exptranabstractview.h"

class ExpTranWindow : public QMainWindow, public ExpTranAbstractView
{
    Q_OBJECT
public:
    ExpTranWindow(QMainWindow *window = 0);
    ~ExpTranWindow();
    void displayException(const std::exception &e);

    //overwritten view functions
    void setAllVideoTabButtonsDisabled(bool);
    void setAllTransferTabButtonsDisabled(bool);
    void incrementTransferProgress();
    void incrementVideoProgress();

public slots:
    void changeSettings();

private:
    Ui::ApplForm ui;
    Ui::settingsDialog sUi;
    //controllers
    FaceTabController *f_controller;
    VideoTabController *v_controller;
    TransferTabController *t_controller;
    //custom widgets
    FaceWidget *face_widget1;
    FaceWidget *face_widget2;
    CustomizableFaceWidget *face_widget3;
    //labels
    VectorFieldQLabel *flowLabelV;
    ClickableQLabel *picLabelV;
    ClickableQLabel *sourceLabel;
    ClickableQLabel *targetLabel;
    //menu dialogs
    QFileDialog *f_dialog;
    QFileDialog *v_dialog;    
    QDialog *settings;
};

#endif // MY_MAIN_WINDOW_H
