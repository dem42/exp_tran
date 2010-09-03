#ifndef EXPTRANAPPLICATION_H
#define EXPTRANAPPLICATION_H

#include <QApplication>
#include <iostream>
#include "exptranwindow.h"

class ExpTranApplication : public QApplication
{
public:
    ExpTranApplication(int &argc, char **argv) : QApplication(argc,argv)
    {
        window = new ExpTranWindow();
    }
    ~ExpTranApplication()
    {
        delete window;
    }
    bool notify(QObject *rec, QEvent *ev)
    {
        try
        {
            return QApplication::notify(rec,ev);
        }
        catch(std::exception &e)
        {
            std::cout << " !catching exceptions! " << e.what() << std::endl;
            window->displayException(e);
            return false;
        }
        catch(...)
        {
            std::cout << " unknown exception" << std::endl;
            abort();
        }
    }
    int exec()
    {        
        window->show();
        return QApplication::exec();
    }

private:
    ExpTranWindow *window;

};

#endif // EXPTRANAPPLICATION_H
