    
//#ifndef CPP_MATRIX_OPERATIONS_RGB_MATRIX.H
//#define CPP_MATRIX_OPERATIONS_RGB_MATRIX.H   

    
namespace MatOps
{
    class MatInt
    {
    private:    
        int rows = 0;
        int cols = 0;
        int** mat;

    public:

        MatInt(int rows, int cols) : rows(rows), cols(cols) { }
        /* {
        this -> mat = generateMatrix(this->rows.)  
        } 
        */
        ~MatInt();

        int** generateMatrix(int rows, int cols);
        void populateMatrix(int *src, int size);
        void trnsp();
        void add(MatInt& m);
        void sub(MatInt& m);
        void mul(MatInt& m);
        void div(MatInt& m);

        int getValueAt(int i, int j);
        int getNumRows();
        int getNumCols();

        void print();

    }; 
}    
    
//#endif //CPP_MATRIX_OPERATIONS_RGB_MATRIX.H







