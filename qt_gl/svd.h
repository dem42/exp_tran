#include "Vector3.h"

class SVD
{
 public:
	SVD(int f,int e,int v);
	~SVD();
	void interpolate_expression(long double **face,long double *w_id,long double *w_ex);
 private:
	const int n_f;
	const int n_e;
	const int n_v;	
	void compute_core_tensor(void);
	long double **core;
	long double **u2;
	long double **u3;
	long double **K;
};

