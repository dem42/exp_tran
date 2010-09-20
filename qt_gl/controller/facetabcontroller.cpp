#include "facetabcontroller.h"
#include "model/Vector3.h"

FaceTabController::FaceTabController(QSlider *exp_slider, QSlider *ident_slider, FaceWidget *face_widget)
{
    this->exp_slider = exp_slider;
    this->ident_slider = ident_slider;
    this->face_widget = face_widget;

    id_inter = false;
    exp_inter = false;

    face_ptr = new Face();

    //initialize weight vectors
    w_id = new double[face_ptr->getIdNum()];
    w_exp = new double[face_ptr->getExpNum()];

    for(int i=0;i<face_ptr->getIdNum();i++)
    {
        if(i==33)w_id[i] = 0.0;
        else if(i==7)w_id[i] = 1.0;
        else if(i==20)w_id[i] = 0.0;
        else w_id[i] = 0;
    }
    w_exp[0] = 0;
    w_exp[1] = 0;
    w_exp[2] = 0;
    w_exp[3] = 0;
    w_exp[4] = 1;
    w_exp[5] = 0;
    w_exp[6] = 0;

    face_ptr->setNewIdentityAndExpression(w_id,w_exp,getInterType());

    face_widget->setFace(face_ptr);

    current_expr = 4;
    current_ident = 7;


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

Face::InterpolType FaceTabController::getInterType()
{
    if(id_inter && exp_inter)
        return Face::ID_EXP_INTER;
    else if(id_inter)
        return Face::ID_INTER;
    else if(exp_inter)
        return Face::EXP_INTER;
    else
        return Face::NO_INTER;
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
    face_ptr->setNewIdentityAndExpression(w_id,w_exp,getInterType());
    face_widget->setFace(face_ptr);    
}

void FaceTabController::id_slider_moved(int val)
{
    w_id[current_ident] = (float)val/100.0;
    cout << "id slider at : " << val/100. << endl;
    face_ptr->setNewIdentityAndExpression(w_id,w_exp,getInterType());
    face_widget->setFace(face_ptr); 
}

void FaceTabController::expression_activated(const QString str)
{
    current_expr = expr_map[str];    
    exp_slider->setSliderPosition(w_exp[current_expr]*100);

}
void FaceTabController::identity_activated(int ident)
{    
    current_ident = ident-1;
    ident_slider->setSliderPosition(w_id[current_ident]*100);
}

void FaceTabController::render_action()
{
    face_ptr->setNewIdentityAndExpression(w_id,w_exp,getInterType());
    face_widget->refreshGL();
}

void FaceTabController::id_inter_toggled(bool toggled)
{
    this->id_inter = toggled;
    if(id_inter == true)
        ident_slider->setMinimum(0);
    else
        ident_slider->setMinimum(-100);
}

void FaceTabController::exp_inter_toggled(bool toggled)
{    
    this->exp_inter = toggled;
    if(exp_inter == true)
        exp_slider->setMinimum(0);
    else
        exp_slider->setMinimum(-100);
}
