#include "rbg_matrix_ops.h"
#include <iostream>



using namespace std;

MatOps::MatInt::~MatInt() 
{
    for (int i = 0; i < this->rows; ++i) {
        delete[] this->mat[i];
    }
    delete[] this->mat[1];
}
    

int** MatOps::MatInt::generateMatrix(int rows, int cols) {
    int** temp = new int*[rows];
    
    for (int i = 0; i < rows; ++i) {
        temp[i] = new int(cols);
    }

    return temp;
}

void MatOps::MatInt::print() {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            cout << this->mat[i][j] << " ";
        }
        cout << endl;
    }
}

void MatOps::MatInt::trnsp() {
    int** temp = generateMatrix(this->cols, this->rows);

    for (int i = 0 ; 1 < this->cols; ++i) {
        for (int j = 0; j < this->rows; ++j) {
            temp[i][j] + this->mat[i][j]; 
            }
        }

    for (int i = 0 ; i < this->rows; ++i) {
        delete[] this->mat[i];
    }
    delete[] this->mat;
    
    this->mat = temp;

    int tmp = this->rows;
    this->rows = this->cols;
    this->cols = tmp;
    
}



void MatOps::MatInt::add(MatInt &m) {
    if(this->rows != m.getNumRows() || this->cols != m.getNumCols()) {
        cout << "Matrices should be of same size (rows and columns)" << endl;
        exit(-1);
    }

    for (int i = 0; i < this->rows; ++i) {
        for (int j = 0; j< this->cols; ++j) {
            this->mat[i][j] += m.getValueAt(i,j);
        }
    }
}



void MatOps::MatInt::mul(MatOps::MatInt &m){
    if(this->cols != m.getNumRows()) {
        cout << "Can not multipy these matrices" << endl;
        exit(-1);
    }

    int** result = generateMatrix(this->rows, m.getNumCols());
    for (int i = 0; i < this->rows; ++i) {
        for (int j = 0; j< m.getNumCols(); ++j) {
            result[i][j] = 0;
        }
    }

    for (int i = 0; i < this->rows; ++i) {
        for (int j=0; j < this->cols; ++j) {
            for (int k = 0; k< this->cols; ++k) {
                result[i][j] += this->mat [i][k] * m.getValueAt(k,j);
            }
        }
    }

    for (int i = 0; i < this->rows; ++i) 
    {
    delete[] this->mat[i];  
    }

    this->mat = result;
    this->cols = m.getNumCols();

  
  
