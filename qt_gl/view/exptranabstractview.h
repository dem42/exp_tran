#ifndef EXPTRANABSTRACTVIEW_H
#define EXPTRANABSTRACTVIEW_H

class ExpTranAbstractView
{
public:
    virtual void setAllVideoTabButtonsDisabled(bool) = 0;
    virtual void setAllTransferTabButtonsDisabled(bool) = 0;
    virtual void incrementTransferProgress() = 0;
    virtual void incrementVideoProgress() = 0;
    virtual void displayException(const std::exception &e) = 0;
};

#endif // EXPTRANABSTRACTVIEW_H
