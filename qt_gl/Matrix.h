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
    /*functions*/
    /*****************/
    Matrix();
    Matrix(int m,int n);
    Matrix(std::vector<double> &in);
    //since the class has a dynamically allocated variable we need
    //a copy constructor or else it will be a shallow copy
    Matrix(Matrix &matrix);
    Matrix(const Matrix &m);

    ~Matrix();

    double getElem(int i,int j) const;
    void setElem(int i,int j,double elem);
    inline int getM() const;
    inline int getN() const;

    //function to compute the singular value decomposition
    static int svd(int m,int n,int withu,int withv,double eps,double tol,
        double **a,double *q,double **u,double **v);
    //kronecker product
    static Matrix kron(const Matrix &a,const Matrix &b);
    //matrix multiplication c = a*b
    //seems to have been faster when we passed a reference back
    static Matrix matrix_mult(const Matrix &a,const Matrix &b);
    void transpose(Matrix &mt);
    void scalar_mult(double scalar);

    Matrix submatrix(int rowstart, int rowend) const;

    void test(void);

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
