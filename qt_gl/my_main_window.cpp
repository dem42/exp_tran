#include "my_main_window.h"

#include <qimage.h>
#include <cv.h>
#include <highgui.h>

MyMainWindow::MyMainWindow(QMainWindow *window)
{    
    ui.setupUi(this);

    face_widget1 = new FaceWidget();
    face_widget2 = new FaceWidget();
    ui.verticalLayout->addWidget(face_widget1);
    f_dialog = new QFileDialog();
    v_dialog = new QFileDialog();    

    //intialize transfer widget    
    trans_widget = new TransferWidget("/home/martin/project/TrackedSmiles/S003-024.avi",face_widget2);
    //ui.verticalLayout_3->addWidget(trans_widget->getVideoWidget());

    ui.verticalLayout_3->addWidget(trans_widget->getPicLabel());    
    ui.verticalLayout_4->addWidget(trans_widget->getFlowLabel());
    face_widget2->setVisible(false);
    ui.verticalLayout_4->addWidget(face_widget2);

    ui.verticalLayoutWidget->setGeometry(10,10,650,650);
    ui.tabWidget->setTabText(0,"Face Tab");
    ui.tabWidget->setTabText(1,"Video Tab");

    //tracking determines when value change signal is emitted
    //either after the user lets go (false) or as its being dragged (true)
    ui.expSlider->setTracking(true);
    ui.expSlider->setSliderPosition(0.5*100);  


    //setup controllers
    f_controller = new FaceTabController(ui.expSlider,ui.identSlider,face_widget1);

    /*********************/
    /* connect signals and slots*/
    /*******************/
    //2 ways how to do this ... depending on which dialog we want
    //connect(ui.action_Open,SIGNAL(triggered()),my_widget,SLOT(face_file_changed()));
    connect(ui.action_Open,SIGNAL(triggered()),f_dialog,SLOT(show()));
    connect(ui.actionVideo_File,SIGNAL(triggered()),v_dialog,SLOT(show()));


    connect(ui.expComboBox,SIGNAL(activated(QString)),f_controller,SLOT(expression_activated(QString)));    
    connect(ui.expSlider,SIGNAL(valueChanged(int)),f_controller,SLOT(exp_slider_moved(int)));
    connect(ui.identSlider,SIGNAL(valueChanged(int)),f_controller,SLOT(id_slider_moved(int)));
    connect(ui.identSpinBox,SIGNAL(valueChanged(int)),f_controller,SLOT(identity_activated(int)));
    connect(ui.renderButton,SIGNAL(clicked()),f_controller,SLOT(render_action()));    

    //menu
    connect(f_dialog,SIGNAL(fileSelected(const QString)),f_controller,SLOT(face_file_changed(const QString)));
    connect(v_dialog,SIGNAL(fileSelected(const QString)),trans_widget,SLOT(video_file_changed(const QString)));

    //face widget connects    
    connect(ui.wireCheckBox,SIGNAL(toggled(bool)),face_widget1,SLOT(wireFrameChecked(bool)));

    //transfer widget connects
    connect(ui.restartButton,SIGNAL(clicked()),trans_widget,SLOT(restartCapturing()));
    connect(ui.playButton,SIGNAL(clicked()),trans_widget,SLOT(playTransfer()));
    connect(ui.pauseButton,SIGNAL(clicked()),trans_widget,SLOT(pauseTransfer()));
    connect(ui.flowButton,SIGNAL(clicked()),trans_widget,SLOT(findGoodFeaturePoints()));
    connect(ui.drawRadio,SIGNAL(clicked(bool)),trans_widget,SLOT(toggleDrawable(bool)));
    connect(ui.extractBtn,SIGNAL(clicked()),trans_widget,SLOT(startFaceTransfer()));    
    connect(ui.dropButton,SIGNAL(clicked()),trans_widget,SLOT(dropFrame()));    
    connect(ui.modelButton,SIGNAL(clicked()),trans_widget,SLOT(playBack()));
    connect(ui.cameraButton,SIGNAL(clicked()),trans_widget,SLOT(calibrate()));
}


MyMainWindow::~MyMainWindow()
{    
    delete face_widget1;
    delete f_dialog;
    delete f_controller;
}

