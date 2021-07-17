
      
      
#include "rbg_matrix_ops.h"   
#include <iostream>


//our first matrix

using namespace std;

int rbg_main()
{
  int srcA[] = {1, 2, 3, 4, 5, 6};
  MatOps::MatInt matA(2,3);
  matA.populateMatrix(srcA, 6);

  //our second matrix
  int srcB[] = {6, 5, 4, 3, 2, 1};
  MatOps::MatInt matB(3,2);
  matB.populateMatrix(srcB, 6);

  matA.print();
  cout << endl;
  matB.print();
  cout << endl;
  matA.add(matB);
  matA.print();

  return 0;
}
 
