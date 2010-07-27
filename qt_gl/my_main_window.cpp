#include "my_main_window.h"
#include <phonon>

#include <qimage.h>
#include <cv.h>
#include <highgui.h>

MyMainWindow::MyMainWindow(QMainWindow *window)
{    
    ui.setupUi(this);

    my_widget = new FaceWidget(this);
    ui.verticalLayout->addWidget(my_widget);
    f_dialog = new QFileDialog();
    v_dialog = new QFileDialog();

    //intialize transfer widget    
    trans_widget = new TransferWidget("/home/martin/project/test/optical_flow_input.avi");    
    //ui.verticalLayout_3->addWidget(trans_widget->getVideoWidget());

    ui.verticalLayout_3->addWidget(trans_widget->getPicLabel());    
    ui.verticalLayout_4->addWidget(trans_widget->getFlowLabel());

    ui.verticalLayoutWidget->setGeometry(10,10,650,650);
    ui.tabWidget->setTabText(0,"Face Tab");
    ui.tabWidget->setTabText(1,"Video Tab");

    //tracking determines when value change signal is emitted
    //either after the user lets go (false) or as its being dragged (true)
    ui.expSlider->setTracking(true);
    ui.expSlider->setSliderPosition(0.5*100);

    /*********************/
    /* connect signals and slots*/
    /*******************/
    //2 ways how to do this ... depending on which dialog we want
    //connect(ui.action_Open,SIGNAL(triggered()),my_widget,SLOT(face_file_changed()));
    connect(ui.action_Open,SIGNAL(triggered()),f_dialog,SLOT(show()));
    connect(ui.actionVideo_File,SIGNAL(triggered()),v_dialog,SLOT(show()));

    //menu
    connect(f_dialog,SIGNAL(fileSelected(const QString)),my_widget,SLOT(face_file_changed(const QString)));
    connect(v_dialog,SIGNAL(fileSelected(const QString)),trans_widget,SLOT(video_file_changed(const QString)));


    connect(ui.renderButton,SIGNAL(clicked()),my_widget,SLOT(render_action()));
    connect(ui.expComboBox,SIGNAL(activated(QString)),my_widget,SLOT(expression_activated(QString)));
    connect(ui.expSlider,SIGNAL(valueChanged(int)),my_widget,SLOT(slider_moved(int)));
    connect(ui.identSpinBox,SIGNAL(valueChanged(int)),my_widget,SLOT(identity_activated(int)));
    connect(ui.wireCheckBox,SIGNAL(toggled(bool)),my_widget,SLOT(wireFrameChecked(bool)));

    //transfer widget connects
    connect(ui.restartButton,SIGNAL(clicked()),trans_widget,SLOT(restartCapturing()));
    connect(ui.playButton,SIGNAL(clicked()),trans_widget,SLOT(playTransfer()));    
    connect(ui.flowButton,SIGNAL(clicked()),trans_widget,SLOT(findGoodFeaturePoints()));
    connect(ui.drawRadio,SIGNAL(clicked(bool)),trans_widget,SLOT(toggleDrawable(bool)));
    connect(ui.extractBtn,SIGNAL(clicked()),trans_widget,SLOT(startFaceTransfer()));
    connect(ui.dropButton,SIGNAL(clicked()),trans_widget,SLOT(dropFrame()));
}


MyMainWindow::~MyMainWindow()
{    
    delete my_widget;
    delete f_dialog;    
}

void MyMainWindow::setSlider(int pos)
{
    cout << "in set slider" << endl;
    ui.expSlider->setSliderPosition(pos);
}
