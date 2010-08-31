#ifndef ROSENERROR_H
#define ROSENERROR_H

#include "errorfunction.h"
#include <vector>

using namespace std;

class RosenError : public ErrorFunction
{
public:    
    double operator()(vector<double>&);
};

#endif // ROSENERROR_H
