#ifndef MATRIX_H
#define MATRIX_H

//a MxN matrix

class Matrix
{
public:
    class Row
    {
        public:
            Row(Matrix &parent, int row) : parent_matrix(parent), row(row) {}
            long double & operator[](int col);
        private:
            Matrix &parent_matrix;
            int row;
     };
    /******************/
    /*functions*/
    /*****************/
    Matrix(int m,int n);
    ~Matrix();

    long double getElem(int i,int j) const;
    void setElem(int i,int j,long double elem);
    int getM() const;
    int getN() const;

    static int svd(int m,int n,int withu,int withv,long double eps,long double tol,
        long double **a,long double *q,long double **u,long double **v);

    static void kron(const Matrix &a,const Matrix &b, Matrix &c);
    static void matrix_mult(const Matrix &a,const Matrix &b, Matrix &c);
    void transpose(Matrix &mt);
    void scalar_mult(double scalar);

    void test(void);

    long double & operator()(int row, int col);
    Matrix::Row operator[](int row);

    //internal representation of the matrix
    //public to enhance performance of matrix loops
    //potential danger of memory being freed
    long double **mat;

private:    
    int m;
    int n;
};

#endif // MATRIX_H
