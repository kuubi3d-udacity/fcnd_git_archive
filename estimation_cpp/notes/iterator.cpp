#include <vector>
#include <iostream>

using namespace std;

// :::: Variable definition
float dt;

int main ()
{
    vector<int> predictedState;
    vector<int> curState;
    
    for (int i =1; i <=10; i++)
    {
        predictedState.push_back(i);
        curState.push_back(i+10);
        dt = i;
        cout << i << endl;
    }

    for (int ps = 0; predictedState.size(); ps++)
    {
        cout << predictedState.size() << endl;
        predictedState[ps] = curState[ps] + dt * curState[ps+3];
        cout << ps << ". Predicted State " << predictedState[ps] << endl;
        cout << ps << endl;
    }
}   