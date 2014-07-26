/*  svd.c -- Singular value decomposition. Translated to 'C' from the
 *           original Algol code in "Handbook for Automatic Computation,
 *           vol. II, Linear Algebra", Springer-Verlag.
 *
 *  (C) 2000, C. Bond. All rights reserved.
 *
 *  This is almost an exact translation from the original, except that
 *  an iteration counter is added to prevent stalls. This corresponds
 *  to similar changes in other translations.
 *
 *  Returns an error code = 0, if no errors and 'k' if a failure to
 *  converge at the 'kth' singular value.
 * 
 */
//#include <cmalloc.h> /* for array allocation */
#include <cstdlib>
#include <cmath>    /* for 'fabs'           */
#include <cstdio>
#include <string>
#include <iostream>
#include <limits>
#include "svd.h"

#define ID 56
#define EX 7

enum Mode_space_t 
	{
		IDENTITY,
		EXPRESSION,
		VERTEX
	};

int svd(int m,int n,int withu,int withv,long double eps,long double tol,
	long double **a,long double *q,long double **u,long double **v)
{
	int i,j,k,l,l1,iter,retval;
	long double c,f,g,h,s,x,y,z;
	long double *e;

	//e = calloc(n,sizeof(long double));
	e = new long double[n];	
	if(e == NULL)fprintf(stderr,"error callocing e");

	retval = 0;

/* Copy 'a' to 'u' */    
	for (i=0;i<m;i++) {
		for (j=0;j<n;j++)
			u[i][j] = a[i][j];
	}
/* Householder's reduction to bidiagonal form. */
	g = x = 0.0;    
	for (i=0;i<n;i++) {
		e[i] = g;
		s = 0.0;
		l = i+1;
		for (j=i;j<m;j++)
			s += (u[j][i]*u[j][i]);
		if (s < tol)
			g = 0.0;
		else {
			f = u[i][i];
			g = (f < 0) ? sqrt(s) : -sqrt(s);
			h = f * g - s;
			u[i][i] = f - g;
			for (j=l;j<n;j++) {
				s = 0.0;
				for (k=i;k<m;k++)
					s += (u[k][i] * u[k][j]);
				f = s / h;
				for (k=i;k<m;k++)
					u[k][j] += (f * u[k][i]);
			} /* end j */
		} /* end s */
		q[i] = g;
		s = 0.0;
		for (j=l;j<n;j++)
			s += (u[i][j] * u[i][j]);
		if (s < tol)
			g = 0.0;
		else {
			f = u[i][i+1];
			g = (f < 0) ? sqrt(s) : -sqrt(s);
			h = f * g - s;
			u[i][i+1] = f - g;
			for (j=l;j<n;j++) 
				e[j] = u[i][j]/h;
			for (j=l;j<m;j++) {
				s = 0.0;
				for (k=l;k<n;k++) 
					s += (u[j][k] * u[i][k]);
				for (k=l;k<n;k++)
					u[j][k] += (s * e[k]);
			} /* end j */
		} /* end s */
		y = fabs(q[i]) + fabs(e[i]);                         
		if (y > x)
			x = y;
	} /* end i */

/* accumulation of right-hand transformations */
	if (withv) {
		for (i=n-1;i>=0;i--) {
			if (g != 0.0) {
				h = u[i][i+1] * g;
				for (j=l;j<n;j++)
					v[j][i] = u[i][j]/h;
				for (j=l;j<n;j++) {
					s = 0.0;
					for (k=l;k<n;k++) 
						s += (u[i][k] * v[k][j]);
					for (k=l;k<n;k++)
						v[k][j] += (s * v[k][i]);

				} /* end j */
			} /* end g */
			for (j=l;j<n;j++)
				v[i][j] = v[j][i] = 0.0;
			v[i][i] = 1.0;
			g = e[i];
			l = i;
		} /* end i */
 
	} /* end withv, parens added for clarity */

/* accumulation of left-hand transformations */
	if (withu) {
		for (i=n;i<m;i++) {
			for (j=n;j<m;j++)
				u[i][j] = 0.0;
			u[i][i] = 1.0;
		}
	}
	if (withu) {
		for (i=n-1;i>=0;i--) {
			l = i + 1;
			g = q[i];
			for (j=l;j<m;j++)  /* upper limit was 'n' */
				u[i][j] = 0.0;
			if (g != 0.0) {
				h = u[i][i] * g;
				for (j=l;j<m;j++) { /* upper limit was 'n' */
					s = 0.0;
					for (k=l;k<m;k++)
						s += (u[k][i] * u[k][j]);
					f = s / h;
					for (k=i;k<m;k++) 
						u[k][j] += (f * u[k][i]);
				} /* end j */
				for (j=i;j<m;j++) 
					u[j][i] /= g;
			} /* end g */
			else {
				for (j=i;j<m;j++)
					u[j][i] = 0.0;
			}
			u[i][i] += 1.0;
		} /* end i*/
	} /* end withu, parens added for clarity */

/* diagonalization of the bidiagonal form */
	eps *= x;
	for (k=n-1;k>=0;k--) {
		iter = 0;
test_f_splitting:
		for (l=k;l>=0;l--) {
			if (fabs(e[l]) <= eps) goto test_f_convergence;
			if (fabs(q[l-1]) <= eps) goto cancellation;
		} /* end l */

/* cancellation of e[l] if l > 0 */
cancellation:
		c = 0.0;
		s = 1.0;
		l1 = l - 1;
		for (i=l;i<=k;i++) {
			f = s * e[i];
			e[i] *= c;
			if (fabs(f) <= eps) goto test_f_convergence;
			g = q[i];
			h = q[i] = sqrt(f*f + g*g);
			c = g / h;
			s = -f / h;
			if (withu) {
				for (j=0;j<m;j++) {
					y = u[j][l1];
					z = u[j][i];
					u[j][l1] = y * c + z * s;
					u[j][i] = -y * s + z * c;
				} /* end j */
			} /* end withu, parens added for clarity */
		} /* end i */
test_f_convergence:
		z = q[k];
		if (l == k) goto convergence;

/* shift from bottom 2x2 minor */
		iter++;
		if (iter > 30) {
			retval = k;
			break;
		}
		x = q[l];
		y = q[k-1];
		g = e[k-1];
		h = e[k];
		f = ((y-z)*(y+z) + (g-h)*(g+h)) / (2*h*y);
		g = sqrt(f*f + 1.0);
		f = ((x-z)*(x+z) + h*(y/((f<0)?(f-g):(f+g))-h))/x;
/* next QR transformation */
		c = s = 1.0;
		for (i=l+1;i<=k;i++) {
			g = e[i];
			y = q[i];
			h = s * g;
			g *= c;
			e[i-1] = z = sqrt(f*f+h*h);
			c = f / z;
			s = h / z;
			f = x * c + g * s;
			g = -x * s + g * c;
			h = y * s;
			y *= c;
			if (withv) {
				for (j=0;j<n;j++) {
					x = v[j][i-1];
					z = v[j][i];
					v[j][i-1] = x * c + z * s;
					v[j][i] = -x * s + z * c;
				} /* end j */
			} /* end withv, parens added for clarity */
			q[i-1] = z = sqrt(f*f + h*h);
			c = f/z;
			s = h/z;
			f = c * g + s * y;
			x = -s * g + c * y;
			if (withu) {
				for (j=0;j<m;j++) {
					y = u[j][i-1];
					z = u[j][i];
					u[j][i-1] = y * c + z * s;
					u[j][i] = -y * s + z * c;
				} /* end j */
			} /* end withu, parens added for clarity */
		} /* end i */
		e[l] = 0.0;
		e[k] = f;
		q[k] = x;
		goto test_f_splitting;
convergence:
		if (z < 0.0) {
/* q[k] is made non-negative */
			q[k] = - z;
			if (withv) {
				for (j=0;j<n;j++)
					v[j][k] = -v[j][k];
			} /* end withv, parens added for clarity */
		} /* end z */
	} /* end k */
	
	free(e);
	return retval;
}

//odd nan bug .. IEEE standard a nan is a number f for which f!=f is true
bool my_isnan(long double f)
{
	return f!=f;
}

void read_flat(long double **a, std::string strs[ID][EX], int m, int n,
			   const int first_dim, const int second_dim, const char *dir_name, Mode_space_t flag)
{
	FILE *fid;
	int i,j,k;
	int number;
	char filename[256];
	char str_dont_care[50];
		
	for(i=0;i<first_dim;i++)
		{
			for(j=0;j<second_dim;j++)
				{
					switch(flag)
						{
						case IDENTITY : sprintf(filename,"%s/%s",dir_name,strs[i][j].c_str()); break;
						case EXPRESSION : sprintf(filename,"%s/%s",dir_name,strs[j][i].c_str()); break;
							//bit of a pointer trick here where we iterate through strs one by one row wise sometimes too much 
							//trickery just makes things confusing .. rehash this
						case VERTEX : std::cerr << "use other function for vertex" << std:: endl; exit(0);
						default: std::cerr << "unknown flag type" << std:: endl; exit(0);
						}
												
					fid = fopen(filename,"r");
					fgets(str_dont_care,50,fid);
					fgets(str_dont_care,50,fid);
					fgets(str_dont_care,50,fid);
					fgets(str_dont_care,50,fid);
					
					fscanf(fid,"%s %d",str_dont_care,&number);
					fscanf(fid,"%s",str_dont_care);
										
					for(k=0;k<n;k++)
						fscanf(fid,"%Lf",&a[i][j*n+k]);
					
					fclose(fid);
				}
		}	
}


void read_flat_vertex(long double **a, std::string strs[ID][EX], int m, int n,
			   const int first_dim, const int second_dim, const char *dir_name, Mode_space_t flag)
{
	FILE *fid;
	int i,j,k;
	int number;
	char filename[256];
	char str_dont_care[50];

	if(flag != VERTEX)
		std::cerr << "use other function " << std::endl;
		
	for(i=0;i<first_dim;i++)
		{
			for(j=0;j<second_dim;j++)
				{
					sprintf(filename,"%s/%s",dir_name,strs[i][j].c_str());
					
					fid = fopen(filename,"r");
					fgets(str_dont_care,50,fid);
					fgets(str_dont_care,50,fid);
					fgets(str_dont_care,50,fid);
					fgets(str_dont_care,50,fid);
					
					fscanf(fid,"%s %d",str_dont_care,&number);
					fscanf(fid,"%s",str_dont_care);
										
					for(k=0;k<n;k++)
						fscanf(fid,"%Lf",&a[k][i*second_dim+j]);
					
					fclose(fid);
				}
		}	
}

/*nothing better than a tasty kronecker (product) to wash down the singular value matrices*/
/* c (m*p x n*q) = a (mxn) * b(pxq)*/
void kron(long double **a, int m, int n, long double **b, int p, int q, long double **c)
{
	int i,j,k,l;
	//reordering loops will probably make this faster should it ever need to be fast
	for(i=0;i<m;i++)
		for(j=0;j<n;j++)
			for(k=0;k<p;k++)
				for(l=0;l<q;l++)
					c[i*p+k][j*q+l] = a[i][j]*b[k][l];
}

//mult c (mxr) = a (mxn) * b (nxr)
void matrix_mult(long double **a, long double **b, long double **c,int m, int n,int r)
{
	int i,j,k;
	for(i=0;i<m;i++)
		for(j=0;j<r;j++)
			c[i][j] = 0;
		
	for(i=0;i<m;i++)
		for(k=0;k<n;k++)
			for(j=0;j<r;j++)
				c[i][j] += a[i][k]*b[k][j];
	
}

//mult c (1xn) = v (1xm) * v (mxn)
void matrix_vector_mult(long double *v, long double **b, long double *c,int m, int n)
{
	int i,k;
	for(i=0;i<n;i++)
		c[i] = 0;
		
	for(k=0;k<m;k++)
		for(i=0;i<n;i++)
			c[i] += v[k]*b[k][i];	
}

void matrix_transpose(long double **a,long double **at,int m,int n)
{
	int i,j;
	
	for(i=0;i<m;i++)
		for(j=0;j<n;j++)
			at[j][i] = a[i][j];
}

void matrix_scalar_mult(long double **a, double scalar, int m, int n)
{
	int i,j;
	for(i=0;i<m;i++)
		for(j=0;j<n;j++)
			a[i][j] = scalar*a[i][j];
}


void test_matrix_mult_and_transpose_and_kron(void)
{
	int i,j;
	long double **A,**B,**C,**AT,**K;	
	/*test matrix mult*/
	A = new long double*[3];
	for(i=0;i<3;i++)
		A[i] = new long double[2];
	B = new long double*[2];
	for(i=0;i<2;i++)
		B[i] = new long double[3];
	C = new long double*[3];
	for(i=0;i<3;i++)
		C[i] = new long double[3];

	AT = new long double*[2];
	for(i=0;i<2;i++)
		AT[i] = new long double[3];
	
	K = new long double*[2*3];
	for(i=0;i<2*3;i++)
		K[i] = new long double[3*2];
	

	A[0][0] = 0;
	A[0][1] = 1;
	A[1][0] = 3;
	A[1][1] = 1;
	A[2][0] = 2;
	A[2][1] = 0;

	B[0][0] = 4;
	B[0][1] = 1;
	B[0][2] = 0;
	B[1][0] = 2;
	B[1][1] = 1;
	B[1][2] = 2;

	matrix_mult(A,B,C,3,2,3);

	for(i=0;i<3;i++)
		{			
			for(j=0;j<3;j++)
				std::cout << C[i][j] << " ";
			std::cout << std::endl;
		}
	
	std::cout << "now A transposed" << std::endl;
	
	matrix_transpose(A,AT,3,2);
	
	for(i=0;i<2;i++)
		{			
			for(j=0;j<3;j++)
				std::cout << AT[i][j] << " ";
			std::cout << std::endl;
		}

	std::cout << "now Kronecker product" << std::endl;

	kron(A,3,2,B,2,3,K);
	
	for(i=0;i<2*3;i++)
		{			
			for(j=0;j<3*2;j++)
				std::cout << K[i][j] << " ";
			std::cout << std::endl;
		}

	long double *v1,*v2;
	v1 = new long double[3];
	v2 = new long double[2];
	v1[0] = 2;
	v1[1] = 1;
	v1[2] = 10;
	matrix_vector_mult(v1,A,v2,3,2);
	
	std::cout << "now matrix vector product" << std::endl;

	for(i=0;i<2;i++)
		std::cout << v2[i] << " ";
	std::cout << "done " << std::endl;
		 
	delete[] A;
	delete[] B;
	delete[] C;
	delete[] K;
	delete[] AT;
	delete[] v1;
	delete[] v2;
}

SVD::SVD(int f,int e,int v) : n_f(f), n_e(e), n_v(v)
{
	compute_core_tensor();
}

SVD::~SVD()
{
	delete[] core;
	delete[] u2;
	delete[] u3;
	delete[] K;
}


void SVD::interpolate_expression(Point3 *face,long double *w_id,long double *w_ex)
{	
	long double *row1, *row2;
	long double **m1i,**m1e,**kr;
	int i = 0;
	
	row1 = new long double[n_f];
	row2 = new long double[n_e];

	matrix_vector_mult(w_id,u2,row1,n_f,n_f);
	matrix_vector_mult(w_ex,u3,row2,n_e,n_e);
	
	m1i = new long double*[1];
	m1i[0] = new long double[n_f];
	m1e = new long double*[1];
	m1e[0] = new long double[n_e];
	kr = new long double*[1];
	kr[0] = new long double[n_e*n_f];
	for(i=0;i<n_f;i++)
		m1i[0][i] = row1[i];
	
	for(i=0;i<n_e;i++)
		m1e[0][i] = row2[i];
	kron(m1i,1,n_f,m1e,1,n_e,kr);
	
	delete[] m1i;
	delete[] m1e;
	delete[] row1;
	delete[] row2;
	
	m1i = new long double*[n_f*n_e];
	for(i=0;i<n_f*n_e;i++)
		m1i[i] = new long double[1];
	matrix_transpose(kr,m1i,1,n_f*n_e);

	delete[] kr;
	
	m1e = new long double*[3*n_v];
	for(i=0;i<3*n_v;i++)
		m1e[i] = new long double[1];
	
	matrix_mult(core,m1i,m1e,3*n_v,n_f*n_e,1);

	delete[] m1i;
	
	for(i=0;i<3*n_v;i=i+3)
		{			
			face[i].x = (float)m1e[i][0];
			face[i].y = (float)m1e[i+1][0];
			face[i].z = (float)m1e[i+2][0];
		}
	
	delete[] m1e;	
}


void SVD::compute_core_tensor(void)
{
	long double **a,**at;
	long double **a1_flat;	
	//a2 = a*at ... a matrix times a transposed
	long double **a2;	
	long double **v,*d;
	int i,j;
	FILE *fid;
	int num;
	int m,n;
	
	std::string files[ID][EX];
	char filename[50];
		
	const char *file_list = "out_here.txt";
	const char *dir_name = "/home/martin/project/JaceyBinghamtonVTKFiles";

	//test the matrix multiply
	test_matrix_mult_and_transpose_and_kron();
			
	fid = fopen(file_list,"r");
	
	std::cout << "max(float): "
			  << std::numeric_limits<float>::max() << std::endl;
	std::cout << "max(double): "
			  << std::numeric_limits<double>::max() << std::endl;
 	std::cout << "max(long double): "
			  << std::numeric_limits<long double>::max() << std::endl;
  
	fscanf(fid,"%d",&num);
	if(num != n_f * n_e) printf("problem\n");
	
	for(i=0;i<n_f;i++)
		for(j=0;j<n_e;j++)
			{
				fscanf(fid,"%s",filename);
				files[i][j].assign(filename);
			}	

	fclose(fid);
		
	/* for(i=0;i<n_f;i++) */
	/* 	for(j=0;j<n_e;j++) */
	/* 		std::cout << files[i][j] << std::endl; */

	m = n_f;
	n = n_e * 3 * n_v;

	a = new long double*[m];
	for(i=0;i<m;i++)
		a[i] = new long double[n];
	
	at = new long double*[n];
	for(i=0;i<n;i++)
		at[i] = new long double[m];
	

	a2 = new long double*[m];	
	for(i=0;i<m;i++)
		a2[i] = new long double[m];

	/********************************/
	/*********first read indentity*/
	/********************************/
	read_flat(a,files,m,3*n_v,n_f,n_e,dir_name,IDENTITY);


	matrix_scalar_mult(a,0.01,m,n);	
	matrix_transpose(a,at,m,n);
	matrix_mult(a,at,a2,m,n,m);
	
	delete[] a;
	delete[] at;	

	u2 = new long double*[m];	
	for(i=0;i<m;i++)
		u2[i] = new long double[m];
	
	d = new long double[n];
		
	svd(m,m,1,0,0.000001,0.000001,a2,d,u2,NULL);

	delete[] a2;
	delete[] d;

	m = n_e;
	n = n_f * 3 * n_v;

	a = new long double*[m];
	for(i=0;i<m;i++)
		a[i] = new long double[n];
	
	at = new long double*[n];
	for(i=0;i<n;i++)
		at[i] = new long double[m];
	

	a2 = new long double*[m];	
	for(i=0;i<m;i++)
		a2[i] = new long double[m];
	/********************************/
	/*********then read expression*/
	/********************************/
	read_flat(a,files,m,3*n_v,n_e,n_f,dir_name,EXPRESSION);


	matrix_scalar_mult(a,0.01,m,n);	
	matrix_transpose(a,at,m,n);
	matrix_mult(a,at,a2,m,n,m);
	
	delete[] a;
	delete[] at;	

	u3 = new long double*[m];
	for(i=0;i<m;i++)
		u3[i] = new long double[m];
	
	d = new long double[n];
	
	svd(m,m,1,0,0.000001,0.000001,a2,d,u3,NULL);

	delete[] a2;
	delete[] d;

	m = 3*n_v;
	n = n_e * n_f;

	a1_flat = new long double*[m];
	for(i=0;i<m;i++)
		a1_flat[i] = new long double[n];	
	/********************************/
	/*********finally read flattened vertex tensor*/
	/********************************/	
	//finally we may need the flattened a matrix .. hmm maybe we dont need it
	//we could probably do with out the kronecker product and just flatten along mode2 and do u2^T * mode2_flat * u3
	//to get the core tensor flattened along mode 2	
	read_flat_vertex(a1_flat,files,m,3*n_v,n_f,n_e,dir_name,VERTEX);

	/********************************/
	/*********calculate flat core tensor core = a1_flat*kron(u2,u3)*/
	/********************************/	
	K = new long double*[n];
	for(i=0;i<n;i++)
		K[i] = new long double[n];

	core = new long double*[m];
	for(i=0;i<m;i++)
		core[i] = new long double[n];
	
	kron(u2,n_f,n_f,u3,n_e,n_e,K);
	//matrix_transpose(a,at,n_e*n_f,n_e*n_f);
	//delete[] at;

	matrix_mult(a1_flat,K,core,m,n,n);

	/**** test ***/
	at = new long double*[1];
	at[0] = new long double[n];

	v = new long double*[n];
	for(i=0;i<n;i++)
		v[i] = new long double[1];

	for(i=0;i<n;i++)
		at[0][i] = K[5][i];

		
	matrix_transpose(at,v,1,n);
	delete[] at;
	at = new long double*[m];
	for(i=0;i<m;i++)
		at[i] = new long double[1];

	matrix_mult(core,v,at,m,n,1);

	/* for(i=0;i<m;i++) */
	/* 	if(my_isnan(at[i][0]) == true) */
	/* 		std::cout << "at nan at " << i << std::endl; */
	
	/* for(i=0;i<m;i++) */
	/* 	std::cout << at[i][0] << " "; */
	/* std::cout << std::endl; */

}

