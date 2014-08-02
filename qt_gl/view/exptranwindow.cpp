#include "exptranwindow.h"

#include <qimage.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <QTextBrowser>

ExpTranWindow::ExpTranWindow(QMainWindow *window)
{    
    ui.setupUi(this);

    settings = new QDialog();
    sUi.setupUi(settings);
    connect(sUi.buttonBox,SIGNAL(accepted()),this,SLOT(changeSettings()));

    face_widget1 = new FaceWidget();
    face_widget2 = new FaceWidget();
    face_widget3 = new CustomizableFaceWidget();
    ui.verticalLayout->addWidget(face_widget1);
    f_dialog = new QFileDialog();
    v_dialog = new QFileDialog();    

    picLabelV = new FeaturePointQLabel();
    flowLabelV = new VectorFieldQLabel();
    ui.verticalLayout_3->addWidget(picLabelV);
    ui.verticalLayout_4->addWidget(flowLabelV);
    face_widget2->setVisible(false);
    ui.verticalLayout_4->addWidget(face_widget2);


    ui.videoTabProgressBar->setVisible(false);
    //intialize transfer widget    
    v_controller = new VideoTabController("obama.avi",
                                          picLabelV, flowLabelV, this, face_widget2);

    //ui.verticalLayoutWidget->setGeometry(10,10,650,650);

    sourceLabel = new FeaturePointQLabel();
    targetLabel = new FeaturePointQLabel();
    ui.srcLayout->addWidget(sourceLabel);
    ui.targetLayout->addWidget(targetLabel);
    face_widget3->setVisible(false);
    ui.targetLayout->addWidget(face_widget3);

    ui.transferTabProgressBar->setVisible(false);
    t_controller = new TransferTabController(sourceLabel,targetLabel,this, ui.srcText,ui.targetTxt, face_widget3);

    //tracking determines when value change signal is emitted
    //either after the user lets go (false) or as its being dragged (true)
    ui.expSlider->setTracking(true);

    //setup controllers
    f_controller = new FaceTabController(ui.expSlider,ui.identSlider,face_widget1);

    /*********************/
    /* connect signals and slots*/
    /*******************/
    //2 ways how to do this ... depending on which dialog we want
    //connect(ui.action_Open,SIGNAL(triggered()),my_widget,SLOT(face_file_changed()));
    connect(ui.action_Open,SIGNAL(triggered()),f_dialog,SLOT(show()));
    connect(ui.actionVideo_File,SIGNAL(triggered()),v_dialog,SLOT(show()));
    connect(ui.actionPreferences,SIGNAL(triggered()),settings,SLOT(show()));


    //face controller connects
    connect(ui.expComboBox,SIGNAL(activated(QString)),f_controller,SLOT(expression_activated(QString)));    
    connect(ui.expSlider,SIGNAL(valueChanged(int)),f_controller,SLOT(exp_slider_moved(int)));
    connect(ui.identSlider,SIGNAL(valueChanged(int)),f_controller,SLOT(id_slider_moved(int)));
    connect(ui.identSpinBox,SIGNAL(valueChanged(int)),f_controller,SLOT(identity_activated(int)));
    connect(ui.renderButton,SIGNAL(clicked()),f_controller,SLOT(render_action()));
    connect(ui.interExpRadio,SIGNAL(toggled(bool)),f_controller,SLOT(exp_inter_toggled(bool)));
    connect(ui.interIdRadio,SIGNAL(toggled(bool)),f_controller,SLOT(id_inter_toggled(bool)));

    //menu
    connect(f_dialog,SIGNAL(fileSelected(const QString)),f_controller,SLOT(face_file_changed(const QString)));
    connect(v_dialog,SIGNAL(fileSelected(const QString)),v_controller,SLOT(video_file_changed(const QString)));

    //face widget connects    
    connect(ui.wireRadioButton,SIGNAL(toggled(bool)),face_widget1,SLOT(wireFrameChecked(bool)));    
    connect(ui.featureRadio,SIGNAL(toggled(bool)),face_widget1,SLOT(featureToggled(bool)));
    connect(ui.mouthRadio,SIGNAL(toggled(bool)),face_widget1,SLOT(mouthToggled(bool)));

    //video controller connects
    connect(ui.restartButton,SIGNAL(clicked()),v_controller,SLOT(restartCapturing()));
    connect(ui.playButton,SIGNAL(clicked()),v_controller,SLOT(playTransfer()));
    connect(ui.pauseButton,SIGNAL(clicked()),v_controller,SLOT(pauseTransfer()));
    connect(ui.flowButton,SIGNAL(clicked()),v_controller,SLOT(findGoodFeaturePoints()));
    connect(ui.drawRadio,SIGNAL(clicked(bool)),v_controller,SLOT(toggleDrawable(bool)));
    connect(ui.extractBtn,SIGNAL(clicked()),v_controller,SLOT(extractPose()));
    connect(ui.dropButton,SIGNAL(clicked()),v_controller,SLOT(dropFrame()));
    connect(ui.modelButton,SIGNAL(clicked()),v_controller,SLOT(playBack()));
    connect(ui.cameraButton,SIGNAL(clicked()),v_controller,SLOT(calibrate()));

    connect(ui.detect,SIGNAL(clicked()),v_controller,SLOT(detect()));

    //transfer controller connects
    connect(ui.browseS,SIGNAL(clicked()),t_controller,SLOT(srcBrowse()));
    connect(ui.browseD,SIGNAL(clicked()),t_controller,SLOT(targetBrowse()));
    connect(ui.dropS,SIGNAL(clicked()),t_controller,SLOT(dropSrc()));
    connect(ui.dropT,SIGNAL(clicked()),t_controller,SLOT(dropTarget()));
    connect(ui.startT,SIGNAL(clicked()),t_controller,SLOT(beginTransfer()));
    connect(ui.restartTransferButton,SIGNAL(clicked()),t_controller,SLOT(restart()));
    connect(ui.srcFeat,SIGNAL(clicked()),t_controller,SLOT(selectGoodFeatureSrcPoints()));
    connect(ui.targetFeat,SIGNAL(clicked()),t_controller,SLOT(selectGoodFeatureTargetPoints()));

    //settings connects
    connect(sUi.optBox,SIGNAL(activated(int)),t_controller,SLOT(setOptType(int)));
    connect(sUi.regBox,SIGNAL(valueChanged(double)),t_controller,SLOT(setOptReg(double)));
    connect(sUi.nelder,SIGNAL(toggled(bool)),t_controller,SLOT(setOptNelder(bool)));
    connect(sUi.iterBox,SIGNAL(valueChanged(int)),t_controller,SLOT(setIterNum(int)));
    connect(sUi.frameBox,SIGNAL(valueChanged(int)),t_controller,SLOT(setFrameNum(int)));

    connect(sUi.optBox,SIGNAL(activated(int)),v_controller,SLOT(setOptType(int)));
    connect(sUi.regBox,SIGNAL(valueChanged(double)),v_controller,SLOT(setOptReg(double)));
    connect(sUi.nelder,SIGNAL(toggled(bool)),v_controller,SLOT(setOptNelder(bool)));
    connect(sUi.iterBox,SIGNAL(valueChanged(int)),v_controller,SLOT(setIterNum(int)));
    connect(sUi.frameBox,SIGNAL(valueChanged(int)),v_controller,SLOT(setFrameNum(int)));

    connect(sUi.constantId,SIGNAL(toggled(bool)),v_controller,SLOT(setConstID(bool)));
    connect(sUi.project2D,SIGNAL(toggled(bool)),v_controller,SLOT(setPointGen2D(bool)));
    connect(sUi.project3D,SIGNAL(toggled(bool)),v_controller,SLOT(setPointGen3D(bool)));
    connect(sUi.hybrid,SIGNAL(toggled(bool)),v_controller,SLOT(setHybrid(bool)));
    connect(sUi.none,SIGNAL(toggled(bool)),v_controller,SLOT(setNone(bool)));
    connect(sUi.projModel,SIGNAL(toggled(bool)),v_controller,SLOT(setProjModel(bool)));
    connect(sUi.firstFrame,SIGNAL(toggled(bool)),v_controller,SLOT(setWithFirstFrame(bool)));
    connect(sUi.pointsBox,SIGNAL(valueChanged(int)),v_controller,SLOT(setGenPoints(int)));

    //settings t_cont
    connect(sUi.poisson,SIGNAL(toggled(bool)),t_controller,SLOT(setPoisson(bool)));
    connect(sUi.interpolate,SIGNAL(toggled(bool)),t_controller,SLOT(setTexture(bool)));
    connect(sUi.background,SIGNAL(toggled(bool)),t_controller,SLOT(setUsingBackground(bool)));

    connect(sUi.constantId,SIGNAL(toggled(bool)),t_controller,SLOT(setConstID(bool)));
    connect(sUi.project2D,SIGNAL(toggled(bool)),t_controller,SLOT(setPointGen2D(bool)));
    connect(sUi.project3D,SIGNAL(toggled(bool)),t_controller,SLOT(setPointGen3D(bool)));

    connect(sUi.texture3d,SIGNAL(toggled(bool)),t_controller,SLOT(setTextured3D(bool)));
    connect(sUi.hybrid,SIGNAL(toggled(bool)),t_controller,SLOT(setHybrid(bool)));
    connect(sUi.projModel,SIGNAL(toggled(bool)),t_controller,SLOT(setProjModel(bool)));
    connect(sUi.none,SIGNAL(toggled(bool)),t_controller,SLOT(setNone(bool)));
    connect(sUi.firstFrame,SIGNAL(toggled(bool)),t_controller,SLOT(setWithFirstFrame(bool)));
    connect(sUi.pointsBox,SIGNAL(valueChanged(int)),t_controller,SLOT(setGenPoints(int)));
}


ExpTranWindow::~ExpTranWindow()
{    
    delete face_widget1;
    delete f_dialog;
    delete f_controller;
    delete face_widget2;
    delete face_widget3;
    delete t_controller;
    delete v_controller;
}

void ExpTranWindow::setAllVideoTabButtonsDisabled(bool disabled)
{
    ui.modelButton->setDisabled(disabled);
    ui.restartButton->setDisabled(disabled);
    ui.playButton->setDisabled(disabled);
    ui.pauseButton->setDisabled(disabled);
    ui.dropButton->setDisabled(disabled);
    ui.flowButton->setDisabled(disabled);
    ui.extractBtn->setDisabled(disabled);
    ui.cameraButton->setDisabled(disabled);
}

void ExpTranWindow::setAllTransferTabButtonsDisabled(bool disabled)
{
    ui.browseD->setDisabled(disabled);
    ui.browseS->setDisabled(disabled);
    ui.dropS->setDisabled(disabled);
    ui.dropT->setDisabled(disabled);
    ui.restartTransferButton->setDisabled(disabled);
    ui.startT->setDisabled(disabled);
    ui.srcFeat->setDisabled(disabled);
    ui.targetFeat->setDisabled(disabled);
}

void ExpTranWindow::incrementTransferProgress()
{
    static int counter = 1;
    counter++;
    ui.transferTabProgressBar->setValue(counter);
}

void ExpTranWindow::incrementVideoProgress()
{
    static int counter = 1;
    ui.videoTabProgressBar->setValue(counter);
    counter++;
}

void ExpTranWindow::displayException(const std::exception &e)
{
    QDialog d;
    QVBoxLayout *layout = new QVBoxLayout();
    QTextBrowser *browser = new QTextBrowser;
    QString str = e.what();
    browser->setText(str);
    layout->addWidget(browser);
    d.setLayout(layout);

    d.setWindowTitle("Error");
    d.exec();
}

void ExpTranWindow::changeSettings()
{
    cout << "changing settings" << endl;
}
