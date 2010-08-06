#include "rosenerror.h"

double RosenError::operator()(vector<double>& x)
{    
   return (100*(x[1]-x[0]*x[0])*(x[1]-x[0]*x[0])+(1.0-x[0])*(1.0-x[0]));
}
