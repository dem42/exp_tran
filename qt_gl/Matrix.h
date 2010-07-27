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
            inline long double & operator[](int col);            
        private:
            Matrix &parent_matrix;            
            int row;
     };
    class ConstRow
    {
        public:
            ConstRow(Matrix const &parent, int row) : parent_matrix(parent), row(row) {}
            inline long double & operator[](int col);
        private:
            Matrix const &parent_matrix;
            int row;
     };
    /******************/
    /*functions*/
    /*****************/
    Matrix(int m,int n);
    ~Matrix();

    long double getElem(int i,int j) const;
    void setElem(int i,int j,long double elem);
    inline int getM() const;
    inline int getN() const;

    static int svd(int m,int n,int withu,int withv,long double eps,long double tol,
        long double **a,long double *q,long double **u,long double **v);

    static void kron(const Matrix &a,const Matrix &b, Matrix &c);
    static void matrix_mult(const Matrix &a,const Matrix &b, Matrix &c);
    void transpose(Matrix &mt);
    void scalar_mult(double scalar);

    void test(void);

    inline long double & operator()(int row, int col);
    inline Matrix::Row operator[](int row);

    inline Matrix::ConstRow operator[](int row) const;
    inline long double & operator()(int row, int col) const;

    //internal representation of the matrix
    //public to enhance performance of matrix loops
    //potential danger of memory being freed
    long double **mat;

private:    
    int m;
    int n;
};

#endif // MATRIX_H
