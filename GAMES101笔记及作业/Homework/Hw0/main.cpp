#include<cmath>
#include<Eigen/Core>
#include<Eigen/Dense>  // °²×°Eigen ¿âºó 
#include<iostream>
using namespace std;
using namespace Eigen;

int main(){
    // TO DO: Define point P
    // TO DO: Define rotation matrix M
    // TO DO: M * P

    constexpr float PI = 3.1415926f;
    constexpr float DEG2RED = PI / 180.0f;

    std::cout << "Hw0:" << std::endl;
    float rad = 45.0f * DEG2RED; 

    Vector3f P(2.0f, 1.0f, 1.0f);
    Matrix3f M;
    M << cos(rad), -sin(rad), 1,
         sin(rad), cos(rad),  2,  
         0,        0,         1;

    Eigen::Vector3f P_prime = M * P;
    std::cout << P_prime << std::endl;
    return 0;
}
