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

#define ID 12
#define EX 7
#define VER 5090

int svd(int m,int n,int withu,int withv,float eps,float tol,
	float **a,float *q,float **u,float **v)
{
	int i,j,k,l,l1,iter,retval;
	float c,f,g,h,s,x,y,z;
	float *e;

	//e = calloc(n,sizeof(float));
	e = new float[n];	
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

void read_flat_idenity(float **a, std::string strs[ID][EX], int m, int n, const int n_f, const int n_e, const char *dir_name)
{
	FILE *fid;
	int i,j,k;
	int number;
	char filename[256];
	char str_dont_care[50];
		
	for(i=0;i<n_f;i++)
		{
			for(j=0;j<n_e;j++)
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
						fscanf(fid,"%f",&a[i][j*n+k]);
					
					fclose(fid);
				}
		}	

}

//mult c (mxr) = a (mxn) * b (nxr)
void matrix_mult(float **a, float **b, float **c,int m, int n,int r)
{
	int i,j,k;
	
	for(i=0;i<m;i++)
		for(k=0;k<n;k++)
			for(j=0;j<r;j++)
				c[i][j] += a[i][k]*b[k][j];
}

void matrix_transpose(float **a,float **at,int m,int n)
{
	int i,j;
	
	for(i=0;i<m;i++)
		for(j=0;j<n;j++)
			at[j][i] = a[i][j];
}

void test_matrix_mult_and_transpose(void)
{
	int i,j;
	float **A,**B,**C,**AT;	
	/*test matrix mult*/
	A = new float*[3];
	for(i=0;i<3;i++)
		A[i] = new float[2];
	B = new float*[2];
	for(i=0;i<2;i++)
		B[i] = new float[3];
	C = new float*[3];
	for(i=0;i<3;i++)
		C[i] = new float[3];

	AT = new float*[2];
	for(i=0;i<2;i++)
		AT[i] = new float[3];
	
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
	
	delete[] A;
	delete[] B;
	delete[] C;
	delete[] AT;
}

int main(int argc, char **argv)
{
	float **a,**at;
	//a2 = a*at ... a matrix times a transposed
	float **a2;	
	float **u,**v,*d;
	int i,j;
	FILE *fid;
	int num;

	const int n_f = ID;	
	const int n_e = EX;
	const int n_v = VER;
	int m,n;
	
	std::string files[n_f][n_e];
	char filename[50];
		
	const char *file_list = "out_here.txt";
	const char *dir_name = "/home/martin/project/JaceyBinghamtonVTKFiles";

	//test the matrix multiply
	test_matrix_mult_and_transpose();
			
	fid = fopen(file_list,"r");
	
	fscanf(fid,"%d",&num);
	if(num != n_f * n_e) printf("problem\n");
	
	for(i=0;i<n_f;i++)
		for(j=0;j<n_e;j++)
			{
				fscanf(fid,"%s",filename);
				files[i][j].assign(filename);
			}	

	fclose(fid);
		
	for(i=0;i<n_f;i++)
		for(j=0;j<n_e;j++)
			std::cout << files[i][j] << std::endl;

	m = n_f;
	n = n_e * 3 * n_v;

	/* a = (float **)malloc(sizeof(float*)*m); */
	/* for(i=0;i<m;i++) */
	/* 	a[i] = (float *)malloc(sizeof(float)*n); */

	a = new float*[m];	
	for(i=0;i<m;i++)
		a[i] = new float[n];
	
	at = new float*[n];	
	for(i=0;i<n;i++)
		at[i] = new float[m];
	

	a2 = new float*[m];	
	for(i=0;i<m;i++)
		a2[i] = new float[m];
	
	read_flat_idenity(a,files,m,3*n_v,n_f,n_e,dir_name);
	
	matrix_transpose(a,at,m,n);
	matrix_mult(a,at,a2,m,n,m);
	
	delete[] a;
	delete[] at;	

	u = new float*[m];	
	for(i=0;i<m;i++)
		u[i] = new float[m];
	
	d = (float *)malloc(sizeof(float)*n);
	d[1] = 10;
	

	/* v = (float **)malloc(sizeof(float*)*n); */
	/* for(i=0;i<n;i++) */
	/* 	v[i] = (float *)malloc(sizeof(float)*n); */


	for(i=0;i<m;i++)
		for(j=0;j<m;j++)
			printf("a2[%d][%d] = %f\n",i,j,a2[i][j]);
	std::cout << m << " " << n << std::endl;
		
	/* a[0] = (float *)malloc(sizeof(float)*2); */
	/* a[1] = (float *)malloc(sizeof(float)*2); */
	/* a[2] = (float *)malloc(sizeof(float)*2); */
	/* a[0][0] = 2.0; */
	/* a[0][1] = 2.0; */

	/* a[1][0] = 1.0; */
	/* a[1][1] = 1.5; */

	/* a[2][0] = 3.0; */
	/* a[2][1] = 5.5; */
	
	std::cout << "this far .. seems good" << std::endl;

	svd(m,m,1,0,0.000001,0.000001,a2,d,u,NULL);

	delete[] u;
	delete[] a2;
	delete[] d;
		
	for(i=0;i<m;i++)
		for(j=0;j<m;j++)
			printf("u[%d][%d] = %lf\n",i,j,u[i][j]);

	/* for(j=0;j<m;j++) */
	/* 	printf("d[%d] = %f\n",j,d[j]); */

	/* for(i=0;i<n;i++) */
	/* 	for(j=0;j<n;j++) */
	/* 		printf("v[%d][%d] = %lf\n",i,j,v[i][j]); */

}

