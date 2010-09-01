#include "exptranwindow.h"

#include <qimage.h>
#include <cv.h>
#include <highgui.h>


ExpTranWindow::ExpTranWindow(QMainWindow *window)
{    
    ui.setupUi(this);

    face_widget1 = new FaceWidget();
    face_widget2 = new FaceWidget();
    face_widget3 = new FaceWidget();
    ui.verticalLayout->addWidget(face_widget1);
    f_dialog = new QFileDialog();
    v_dialog = new QFileDialog();    

    picLabelV = new FeaturePointQLabel();
    flowLabelV = new VectorFieldQLabel();
    ui.verticalLayout_3->addWidget(picLabelV);
    ui.verticalLayout_4->addWidget(flowLabelV);
    face_widget2->setVisible(false);
    ui.verticalLayout_4->addWidget(face_widget2);


    //intialize transfer widget    
    v_controller = new VideoTabController("/home/martin/project/TrackedSmiles/S003-024.avi",
                                          picLabelV, flowLabelV, face_widget2);

    ui.verticalLayoutWidget->setGeometry(10,10,650,650);

    sourceLabel = new FeaturePointQLabel();
    targetLabel = new FeaturePointQLabel();
    ui.srcLayout->addWidget(sourceLabel);
    ui.targetLayout->addWidget(targetLabel);
    face_widget3->setVisible(false);
    ui.targetLayout->addWidget(face_widget3);

    t_controller = new TransferTabController(sourceLabel,targetLabel,ui.srcText,ui.targetTxt, face_widget3);

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
    connect(v_dialog,SIGNAL(fileSelected(const QString)),v_controller,SLOT(video_file_changed(const QString)));

    //face widget connects    
    connect(ui.wireCheckBox,SIGNAL(toggled(bool)),face_widget1,SLOT(wireFrameChecked(bool)));

    //video controller connects
    connect(ui.restartButton,SIGNAL(clicked()),v_controller,SLOT(restartCapturing()));
    connect(ui.playButton,SIGNAL(clicked()),v_controller,SLOT(playTransfer()));
    connect(ui.pauseButton,SIGNAL(clicked()),v_controller,SLOT(pauseTransfer()));
    connect(ui.flowButton,SIGNAL(clicked()),v_controller,SLOT(findGoodFeaturePoints()));
    connect(ui.drawRadio,SIGNAL(clicked(bool)),v_controller,SLOT(toggleDrawable(bool)));
    connect(ui.extractBtn,SIGNAL(clicked()),v_controller,SLOT(startFaceTransfer()));
    connect(ui.dropButton,SIGNAL(clicked()),v_controller,SLOT(dropFrame()));
    connect(ui.modelButton,SIGNAL(clicked()),v_controller,SLOT(playBack()));
    connect(ui.cameraButton,SIGNAL(clicked()),v_controller,SLOT(calibrate()));

    //transfer controller connects
    connect(ui.browseS,SIGNAL(clicked()),t_controller,SLOT(srcBrowse()));
    connect(ui.browseD,SIGNAL(clicked()),t_controller,SLOT(targetBrowse()));
    connect(ui.dropS,SIGNAL(clicked()),t_controller,SLOT(dropSrc()));
    connect(ui.dropT,SIGNAL(clicked()),t_controller,SLOT(dropTarget()));
    connect(ui.startT,SIGNAL(clicked()),t_controller,SLOT(beginTransfer()));
}


ExpTranWindow::~ExpTranWindow()
{    
    delete face_widget1;
    delete f_dialog;
    delete f_controller;
}

