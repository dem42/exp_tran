#ifndef ERRORFUNCTION_H
#define ERRORFUNCTION_H

#include <vector>

//interface for error function

class ErrorFunction
{
public:    
    virtual double operator()(std::vector<double>&) = 0;
};

#endif // ERRORFUNCTION_H
