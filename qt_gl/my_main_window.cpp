#include "my_main_window.h"

MyMainWindow::MyMainWindow(QMainWindow *window)
{
    ui.setupUi(this);

    my_widget = new FaceWidget();
    ui.verticalLayout->addWidget(my_widget);
    f_dialog = new QFileDialog();

    //tracking determines when value change signal is emitted
    ui.expSlider->setTracking(false);
    ui.expSlider->setSliderPosition(0.5*100);

    /*********************/
    /* connect signals and slots*/
    /*******************/
    //2 ways how to do this ... depending on which dialog we want
    //connect(ui.action_Open,SIGNAL(triggered()),my_widget,SLOT(face_file_changed()));
    connect(ui.action_Open,SIGNAL(triggered()),f_dialog,SLOT(show()));
    connect(f_dialog,SIGNAL(fileSelected(const QString)),my_widget,SLOT(face_file_changed(const QString)));
    connect(ui.renderButton,SIGNAL(clicked()),my_widget,SLOT(render_action()));
    connect(ui.expComboBox,SIGNAL(activated(QString)),my_widget,SLOT(identity_activated(QString)));
    connect(ui.expSlider,SIGNAL(valueChanged(int)),my_widget,SLOT(slider_moved(int)));
}

MyMainWindow::~MyMainWindow()
{
    delete my_widget;
    delete f_dialog;
}
