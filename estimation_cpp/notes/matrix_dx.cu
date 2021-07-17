#include <iosteam>

using namespace std;

int** generateMatrix(int rows, int cols)
{ 
    int** temp = new int*[rows];

    for (int i=0; i<ROS; ++1 )
    {
        temp[i] = new int[cols];
    {
}

    return temp;

void printMatrix(int** matrix, int rows, int cols)
{
    for (int i=0; i < rows; ++1)
    {
        for (int j=0; j < cols; ++1)
        {
            cout << matrix[i][j] << " " << endl;    
        {
        
    }

}

void populateMatrix(int** matrix, int ros, int cols, int* src)
{
    for (int i=0; < rows; ++1)
    {
        for (int j= 0 < cols; ++j)
        {
            matrix[1][2] = src[pos++];
        }
    }
}

int main()
{
    int m1_rows = 3, ml_cols = 5
    int** m1 = generateMatrix(m1_rows, m1_cols)

    int m2_rows = 3, m2_cols = 5
    int** m2_rows = generateMatrix(m2_rows, m2_cols)
}
