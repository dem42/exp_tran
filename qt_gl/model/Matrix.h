#ifndef MATRIX_H
#define MATRIX_H

#include <cv.h>
#include <vector>
#include <iostream>

//a MxN matrix

class Matrix
{
public:
    //class representing a row of the matrix
    class Row
    {
        public:
            Row(Matrix &parent, int row) : parent_matrix(parent), row(row) {}            
            double & operator[](int col);
        private:
            Matrix &parent_matrix;            
            int row;
     };
    //class representing a row of a const matrix
    class ConstRow
    {
        public:
            ConstRow(Matrix const &parent, int row) : parent_matrix(parent), row(row) {}
            inline double & operator[](int col);
        private:
            Matrix const &parent_matrix;
            int row;
     };
    /******************/
    /*constructors*/
    /*****************/
    Matrix();
    Matrix(int m,int n);
    Matrix(std::vector<double> &in);
    //since the class has a dynamically allocated variable we need
    //a copy constructor or else it will be a shallow copy
    Matrix(Matrix &matrix);    
    Matrix(const Matrix &m);
    Matrix(cv::Mat &);
    Matrix(const cv::Mat &);

    ~Matrix();


    /******************/
    /*functions*/
    /*****************/
    double getElem(int i,int j) const;
    void setElem(int i,int j,double elem);
    int getM() const;
    int getN() const;


    void transpose(Matrix &mt);
    void scalar_mult(double scalar);

    static Matrix submatrix_(Matrix &A, int rowstart, int rowend);

    void test(void);


    /********************************************/
    /*      STATIC FUNCTIONS                    */
    /********************************************/
    //function to compute the singular value decomposition
    static int svd(int m,int n,int withu,int withv,double eps,double tol,
        double **a,double *q,double **u,double **v);
    //kronecker product
    static Matrix kron_(const Matrix &a,const Matrix &b);
    static cv::Mat kronecker(const cv::Mat&a, const cv::Mat&b);
    //matrix multiplication c = a*b
    //seems to have been faster when we passed a reference back
    static Matrix matrix_mult(const Matrix &a,const Matrix &b);
    static Matrix eye(int i);
    //solve linear system with SVD .. works with singular matrices too
    //@return .. the solution x = inv(svd(A)) * b
    static Matrix solveLinSysSvd(const Matrix &A, const Matrix &b);

    //solve a linear programming problem max cTx given Ax <= b
//    static Matrix solveLinearProg(const Matrix &A, const Matrix &b,
//                                  const Matrix &m1, const Matrix &m2,
//                                  const Matrix &m3);


    /********************************************/
    /*      OPERATORS                           */
    /********************************************/
    double & operator()(int row, int col);

    Matrix::Row operator[](int row);

    Matrix::ConstRow operator[](int row) const;
    double & operator()(int row, int col) const;

    Matrix& operator=(const Matrix& m);

    //operator to cast the opencv matrix format from
    //my matrix format
    operator cv::Mat_<double>() const
    {
        cv::Mat_<double> result(m,n);

        for(int i=0;i<this->m;i++)
            for(int j=0;j<this->n;j++)
                result(i,j) = this->mat[i][j];

        return result;
    }

    friend std::ostream& operator<<(std::ostream& stream, const Matrix &m);

    //internal representation of the matrix
    //public to enhance performance of matrix loops
    //potential danger of memory being freed
    double **mat;

private:    
    int m;
    int n;
};

#endif // MATRIX_H
