#ifndef FACETABCONTROLLER_H
#define FACETABCONTROLLER_H

#include "face_widget.h"
#include "Face.h"
#include <QSlider>
#include <string>
#include <map>
#include <QString>

using namespace std;

class FaceTabController : public QObject
{
    Q_OBJECT
public:
    FaceTabController(QSlider *exp_slider, FaceWidget *face_widget);
    ~FaceTabController();
    enum ExprType { ANGRY=0, DISGUST=1, FEAR=2, HAPPY=3, NEUTRAL=4, SAD=5, SURPRISE=6 };
public slots:
    void identity_activated(int);
    void expression_activated(const QString str);
    void slider_moved(int);
    void face_file_changed(const QString str);
    void render_action();

private:
    QSlider *exp_slider;
    FaceWidget *face_widget;
    Face *face_ptr;

    double *w_exp;
    double *w_id;

    int current_expr;
    int current_ident;

    std::map<QString,ExprType> expr_map;
};

#endif // FACETABCONTROLLER_H
