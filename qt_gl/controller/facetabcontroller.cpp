#include "facetabcontroller.h"
#include "model/Vector3.h"

FaceTabController::FaceTabController(QSlider *exp_slider, QSlider *ident_slider, FaceWidget *face_widget)
{
    this->exp_slider = exp_slider;
    this->ident_slider = ident_slider;
    this->face_widget = face_widget;

    face_ptr = new Face();

    //initialize weight vectors
    w_id = new double[56];
    w_exp = new double[7];

    for(int i=0;i<56;i++)
    {
        if(i==33)w_id[i] = 0.1;
        else if(i==7)w_id[i] = 0.8;
        else if(i==20)w_id[i] = 0.1;
        else w_id[i] = 0;
    }
    w_exp[0] = 1;
    w_exp[1] = 0;
    w_exp[2] = 0;
    w_exp[3] = 0;
    w_exp[4] = 0;
    w_exp[5] = 0;
    w_exp[6] = 0;

//    w_exp[0] = 0.021423;
//    w_exp[1] = 0.00498108;
//    w_exp[2] = 0.00829062;
//    w_exp[3] = -0.00608236;
//    w_exp[4] = 0.00763084;
//    w_exp[5] = -0.0193017;
//    w_exp[6] = -0.021784;

//    w_exp[0] = 0.00883634;
//    w_exp[1] = -0.0029592;
//    w_exp[2] = -0.00662942;
//    w_exp[3] = 0.000328165;
//    w_exp[4] = -0.00611554;
//    w_exp[5] = 0.00532823;
//    w_exp[6] = 0.00375425;

    //Vector3::normalize(w_exp,7);

    for(int i=0;i<7;i++)
        cout << w_exp[i] << endl;

    face_ptr->interpolate(w_id,w_exp);

    face_widget->setFace(face_ptr);

    current_expr = 0;
    current_ident = 33;


    expr_map.insert(std::pair<QString,ExprType>("Angry",ANGRY));
    expr_map.insert(std::pair<QString,ExprType>("Disgust",DISGUST));
    expr_map.insert(std::pair<QString,ExprType>("Fear",FEAR));
    expr_map.insert(std::pair<QString,ExprType>("Happy",HAPPY));
    expr_map.insert(std::pair<QString,ExprType>("Neutral",NEUTRAL));
    expr_map.insert(std::pair<QString,ExprType>("Sad",SAD));
    expr_map.insert(std::pair<QString,ExprType>("Surprise",SURPRISE));
}

FaceTabController::~FaceTabController()
{
    delete face_ptr;
}

void FaceTabController::face_file_changed(const QString str)
{
    cout << str.toStdString() << endl;    
    face_ptr->load(str.toStdString(),"../face.ppm");
    face_widget->refreshGL();
}

void FaceTabController::exp_slider_moved(int val)
{
    w_exp[current_expr] = (float)val/100.0;
    cout << "exp slider at : " << val/100. << endl;
    face_ptr->interpolate(w_id,w_exp);
    face_widget->setFace(face_ptr);    
}

void FaceTabController::id_slider_moved(int val)
{
    w_id[current_ident] = (float)val/100.0;
    cout << "id slider at : " << val/100. << endl;
    face_ptr->interpolate(w_id,w_exp);
    face_widget->setFace(face_ptr); 
}

void FaceTabController::expression_activated(const QString str)
{
    current_expr = expr_map[str];
    std::cout << "expression activated " << str.toStdString() << " cur expr is " << current_expr << std::endl;
    exp_slider->setSliderPosition(w_exp[current_expr]*100);

}
void FaceTabController::identity_activated(int ident)
{
    std::cout << "identity activated " << ident << std::endl;    
    current_ident = ident-1;
    ident_slider->setSliderPosition(w_id[current_ident]*100);
}

void FaceTabController::render_action()
{
    face_ptr->interpolate(w_id,w_exp);
    face_widget->refreshGL();
}
