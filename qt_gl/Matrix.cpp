#include "Matrix.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cmath>

Matrix::Matrix(int m,int n) : m(m), n(n)
{
    try
    {        
        mat = new long double*[m];
        for(int i=0;i<m;i++)
             mat[i] = new long double[n];

    }catch(std::bad_alloc& a)
    {
        std::cout << "ugh2 " << mat << " " << n <<  std::endl;
    }

    for(int i=0;i<m;i++)
        for(int j=0;j<n;j++)
            mat[i][j] = 0.0;

}

Matrix::~Matrix()
{
    delete[] mat;
}

long double Matrix::getElem(int i,int j) const
{    
    return mat[i][j];
}

void Matrix::setElem(int i,int j,long double elem)
{
    mat[i][j] = elem;
}

int Matrix::getM() const
{
    return m;
}

int Matrix::getN() const
{
    return n;
}

//static is only in the declaration
int Matrix::svd(int m,int n,int withu,int withv,long double eps,long double tol,
        long double **a,long double *q,long double **u,long double **v)
{
        int i,j,k,l,l1,iter,retval;
        long double c,f,g,h,s,x,y,z;
        long double *e;
        i = j = k = l = l1 = iter = retval = 0;
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

/*nothing better than a tasty kronecker (product) to wash down the singular value matrices*/
/* c (m*p x n*q) = a (mxn) * b(pxq)*/
void Matrix::kron(const Matrix &a,const Matrix &b, Matrix &c)
{
        int i,j,k,l;
        int m,n,p,q;
        m = a.getM();
        n = a.getN();
        p = b.getM();
        q = b.getN();
        //reordering loops will probably make this faster should it ever need to be fast
        for(i=0;i<m;i++)
                for(j=0;j<n;j++)
                        for(k=0;k<p;k++)
                                for(l=0;l<q;l++)
                                        c.mat[i*p+k][j*q+l] = a.mat[i][j]*b.mat[k][l];
}

//mult c (mxr) = a (mxn) * b (nxr)
void Matrix::matrix_mult(const Matrix &a,const Matrix &b, Matrix &c)
{
        int i,j,k;        
        for(i=0;i<a.getM();i++)
                for(k=0;k<b.getM();k++)
                        for(j=0;j<b.getN();j++)
                                c.mat[i][j] += a.mat[i][k]*b.mat[k][j];

}

void Matrix::transpose(Matrix &mt)
{
        int i,j;

        for(i=0;i<m;i++)
                for(j=0;j<n;j++)
                        mt.mat[j][i] = mat[i][j];
}

void Matrix::scalar_mult(double scalar)
{
        int i,j;
        for(i=0;i<m;i++)
                for(j=0;j<n;j++)
                        mat[i][j] = scalar*mat[i][j];
}

long double & Matrix::Row::operator[](int col)
{
   return parent_matrix(row,col);
}
long double & Matrix::operator()(int row, int col)
{
    return mat[row][col];
}
Matrix::Row Matrix::operator[](int row)
{
    return Matrix::Row(*this,row);
}

void Matrix::test(void)
{
        int i,j;
        //Matrix *A,*B,*C,*AT,*K;
        /*test matrix mult*/
//        A = new Matrix(3,2);
//        B = new Matrix(2,3);
//        C = new Matrix(3,3);
//        AT = new Matrix(2,3);
//        K = new Matrix(2*3,3*2);
        Matrix A(3,2),B(2,3),C(3,3),AT(2,3),K(2*3,3*2);

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

        std::cout << A.mat[1][0] << std::endl;

        Matrix::matrix_mult(A,B,C);

        for(i=0;i<3;i++)
                {
                        for(j=0;j<3;j++)
                                std::cout << C[i][j] << " ";
                        std::cout << std::endl;
                }

        std::cout << "now A transposed" << std::endl;

        A.transpose(AT);

        for(i=0;i<2;i++)
                {
                        for(j=0;j<3;j++)
                                std::cout << AT[i][j] << " ";
                        std::cout << std::endl;
                }

        std::cout << "now Kronecker product" << std::endl;

        Matrix::kron(A,B,K);

        for(i=0;i<2*3;i++)
                {
                        for(j=0;j<3*2;j++)
                                std::cout << K[i][j] << " ";
                        std::cout << std::endl;
                }
}
