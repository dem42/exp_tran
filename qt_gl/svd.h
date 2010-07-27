#ifndef SVD_H
#define SVD_H

#include "Vector3.h"
#include <string>

//make it save its state so that we dont have to recompute this
//maybe use a database?
class SVD
{
 public:
        SVD(std::string filename,int f,int e,int v);
	~SVD();
        void interpolate_expression(Point3 *face,long double *w_id,long double *w_ex,bool brute);
 protected:
        void persist();
        bool load();
 private:
        const std::string filename;
	const int n_f;
	const int n_e;
	const int n_v;	
	void compute_core_tensor(void);
	long double **core;
	long double **u2;
	long double **u3;
	long double **K;
};

#endif
