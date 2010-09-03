#include "Matrix.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cmath>

Matrix::Matrix(int m,int n) : m(m), n(n)
{
    try
    {        
        mat = new double*[m];
        for(int i=0;i<m;i++)
             mat[i] = new double[n];

    }catch(std::bad_alloc& a)
    {
        std::cout << "ugh2 " << mat << " " << n <<  std::endl;
    }

    for(int i=0;i<m;i++)
        for(int j=0;j<n;j++)
            mat[i][j] = 0.0;

}
//not possible to delegate constructors (chain) in c++
Matrix::Matrix()
{
    mat = NULL;
}

Matrix::Matrix(std::vector<double> &in)
{
    m = 1;
    n = in.size();
    mat = new double*[1];
    mat[0] = new double[in.size()];
    for(unsigned int i=0;i<in.size();i++)
    {        
        mat[0][i] = in[i];
    }
}


Matrix::Matrix(Matrix &matrix)
{    
    m = matrix.getM();
    n = matrix.getN();


    mat = new double*[matrix.getM()];
    for(int i=0;i<matrix.getM(); ++i)
    {
        mat[i] = new double[matrix.getN()];
        for(int j=0;j<matrix.getN(); ++j)
        {
            mat[i][j] = matrix.mat[i][j];
        }
    }
}

Matrix::Matrix(const Matrix &matrix)
{    
    m = matrix.getM();
    n = matrix.getN();

    mat = new double*[matrix.getM()];
    for(int i=0;i<matrix.getM(); ++i)
    {
        mat[i] = new double[matrix.getN()];
        for(int j=0;j<matrix.getN(); ++j)
        {
            mat[i][j] = matrix.mat[i][j];
        }
    }
}

Matrix::Matrix(cv::Mat &matrix)
{
    m = matrix.size().height;
    n = matrix.size().width;
    mat = new double*[m];
    for(int i=0;i<m; ++i)
    {
        mat[i] = new double[n];
        for(int j=0;j<n; ++j)
        {
            mat[i][j] = matrix.at<double>(i,j);
        }
    }
}

Matrix::Matrix(const cv::Mat &matrix)
{
    m = matrix.size().height;
    n = matrix.size().width;
    mat = new double*[m];
    for(int i=0;i<m; ++i)
    {
        mat[i] = new double[n];
        for(int j=0;j<n; ++j)
        {
            mat[i][j] = matrix.at<double>(i,j);
        }
    }
}
Matrix::~Matrix()
{
    if(mat != NULL)
        delete[] mat;    
}

double Matrix::getElem(int i,int j) const
{    
    return mat[i][j];
}

void Matrix::setElem(int i,int j,double elem)
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
int Matrix::svd(int m,int n,int withu,int withv,double eps,double tol,
        double **a,double *q,double **u,double **v)
{
        int i,j,k,l,l1,iter,retval;
        double c,f,g,h,s,x,y,z;
        double *e;
        i = j = k = l = l1 = iter = retval = 0;
        //e = calloc(n,sizeof(double));
        e = new double[n];
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
Matrix Matrix::kron(const Matrix &a,const Matrix &b)
{
        Matrix c(a.getM()*b.getM(),a.getN()*b.getN());

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
        return c;
}

//mult c (mxr) = a (mxn) * b (nxr)
Matrix Matrix::matrix_mult(const Matrix &a,const Matrix &b)
{
    assert(a.getN() == b.getM());
    Matrix c(a.getM(),b.getN());
    int i,j,k;
    for(i=0;i<a.getM();i++)
        for(k=0;k<b.getM();k++)
            for(j=0;j<b.getN();j++)
                c.mat[i][j] += a.mat[i][k]*b.mat[k][j];
    return c;
 }

Matrix Matrix::eye(int n)
{
    Matrix r(n,n);
    for(int i=0;i<n;i++)
        for(int j=0;j<n;j++)
        {
            if(i==j) r.mat[i][j] = 1;
            else r.mat[i][j] = 0;
        }
    return r;
}
//A*x = b; A = U*D*V', inv(A) = V*inv(D)*U', x = V*inv(D)*U'*b
Matrix Matrix::solveLinSysSvd(const Matrix &A, const Matrix &b)
{
    int m = A.getM();
    int n = A.getN();
    int p = b.getM();
    int q = b.getN();
    std::cout << "sizes in solve lin sys " << m << " " << n << " " << p << " " << q << std::endl;
    Matrix U(m,m), UT(m,m), V(n,n), invD(m,n), result(m,q);

    assert(m == p);

    for(int i=0;i<m;i++)
        for(int j=0;j<n;j++)
            invD[i][j] = 0;

    double *d = new double[m];

    Matrix::svd(m,n,1,1,0.000001,0.000001,A.mat,d,U.mat,V.mat);

    //if there is a zero in the diagonal keep it in inv(D)
    //proof why this is okay in numerical recepies
    for(int i=0;i<m;i++)
    {
        invD[i][i] = (d[i] != 0)? 1.0/d[i] : 0;
    }

    U.transpose(UT);

    result = matrix_mult(matrix_mult(V,invD),matrix_mult(UT,b));

    delete[] d;

    return result;
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

Matrix Matrix::submatrix(int rowstart, int rowend) const
{
    //from rowstart to rowend including the row with index rowend
    int size = rowend - rowstart + 1;
    Matrix sub(size,getN());
    for(int i=0, ri = rowstart; i<size; ++i, ++ri)
        for(int j=0; j<getN(); ++j)
            sub[i][j] = mat[ri][j];
    return sub;
}

/*********************************/
/* overloaded operators */
/*********************************/

Matrix& Matrix::operator=(const Matrix& matrix)
{    
    m = matrix.getM();
    n = matrix.getN();
    //here we should also delete previous mat if not null

    mat = new double*[matrix.getM()];
    for(int i=0;i<matrix.getM(); ++i)
    {
        mat[i] = new double[matrix.getN()];
        for(int j=0;j<matrix.getN(); ++j)
        {
            mat[i][j] = matrix.mat[i][j];
        }
    }
    return *this;
}

double & Matrix::Row::operator[](int col)
{
   return parent_matrix(row,col);
}
double & Matrix::ConstRow::operator[](int col)
{
   return parent_matrix(row,col);
}
//we had a problem calling the first [] on a const Matrix
//it said passing qualifiers discards constness so we added
//a version which passes a const this (by overloading the const operator)
Matrix::ConstRow Matrix::operator[](int row) const
{
    return Matrix::ConstRow(*this,row);
}
//the above also required const overloading :
Matrix::Row Matrix::operator[](int row)
{
    return Matrix::Row(*this,row);
}

double & Matrix::operator()(int row, int col)
{
    return mat[row][col];
}
double & Matrix::operator()(int row, int col) const
{
    return mat[row][col];
}

std::ostream& operator<<(std::ostream& stream, const Matrix &matrix)
{
    for(int i=0;i<matrix.m;i++)
    {
        for(int j=0;j<matrix.n;j++)
            stream << matrix.mat[i][j] << " ";
        stream << std::endl;
    }
    return stream;
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
        Matrix M(3,3),x(3,1),b(3,1);

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

        C = Matrix::matrix_mult(A,B);

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

        K = Matrix::kron(A,B);

        std::cout << "product calced " << K.getM() << " " << K.getN()  << std::endl;

        for(i=0;i<2*3;i++)
                {
                        for(j=0;j<3*2;j++)
                                std::cout << K.mat[i][j] << " ";
                        std::cout << std::endl;
                }

        Matrix sub = K.submatrix(2,4);
        std::cout << "Submatix of K rows 2 to 4 is : \n" << sub;

        std::cout << "now test solving a linear system using SVD: " << std::endl;
        std::cout << "A*x = b; A = U*D*V', inv(A) = V*inv(D)*U', x = V*inv(D)*U'*b" << std::endl;
        //if element in diagonal of D is 0 set element in inv(D) to 0 proof in numerical recipies
        M[0][0] = 4;
        M[0][1] = 1.5;
        M[0][2] = 2;
        M[1][0] = 3;
        M[1][1] = 3;
        M[1][2] = 1;
        M[2][0] = 2;
        M[2][1] = 1;
        M[2][2] = 5;

        b[0][0] = 10.6;
        b[1][0] = 11.3;
        b[2][0] = 9;
        //x should be 1.5, 2, 0.8
        Matrix result = solveLinSysSvd(M,b);
        std::cout << " The result of the linear system is : " << std::endl;
        std::cout << result;
    }
