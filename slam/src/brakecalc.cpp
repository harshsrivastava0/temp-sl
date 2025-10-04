#include <iostream>
using namespace std;

// bool withinthresh(float a, float b, float thresh){
//     if(a-b<=thresh || a-b>=-thresh){
//         return true;
//     }
//     return false;
// }

int main(){
    int c=1;
    for(double rin=1; rin<=100; rin+=1){ //in mm
        for(double rf=1;  rf<=100; rf+=1){ // in mm
            for(double l=0.001; l<=0.5; l+=0.001){ //in m
                for(int k=50; k<=5000; k+=10){ // in N/m
                    // cout << "iteration: " << c++ << endl;
                    if( 3.14159*rin*rin*(1.013 - ((0.1013*l)/(l-0.008))) + (3.14159*rf*rf*0.1013*l)/(l-0.008) - (0.008*k + 82.28) >= (2.2286)*10*10*10*10*10*10){
                        // cout << "----------------------------------------------------------------------" << endl;
                        cout << "rin: " << rin << " rf: " << rf << " l: " << l << " k: " << k << endl;
                        // cout << "----------------------------------------------------------------------" << endl;
                    }
                }
            }
        }
    }
    cout << "end";
}