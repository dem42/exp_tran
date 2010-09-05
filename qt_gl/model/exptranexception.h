#ifndef EXPTRANEXCEPTION_H
#define EXPTRANEXCEPTION_H

#include <iostream>
#include <stdexcept>

using namespace std;

//exception with message
class ExpTranException : public runtime_error {
 public:
  ExpTranException ( const string &err ) : runtime_error (err ) {}
};


#endif // EXPTRANEXCEPTION_H
