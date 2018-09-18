#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/Dense>

#define PI 3.14159265

int main ()
{
    double x, y;
    Eigen::Vector3d gb;
    Eigen::Matrix3d Rx, Ry, R, R_alt;

    gb << 0.461105, 0.082198, -0.887432;

    x = atan2( gb(1), gb(2)) * 180/PI;
    y = atan2(-gb(0), sqrt(gb(1)*gb(1) + gb(2)*gb(2))) * 180/PI;

    Rx << 1,       0,     0,
          0,  cos(x), sin(x),
          0, -sin(x), cos(x);
    Ry << cos(y), 0, -sin(y),
               0, 1,       0,
          sin(y), 0,  cos(y);
    R_alt = Rx*Ry;

    R <<        cos(y),       0,       -sin(y),
         sin(x)*sin(y),  cos(x), sin(x)*cos(y),
         cos(x)*sin(y), -sin(x), cos(x)*cos(y);

    std::cout << "Roll: " << x << "º \n"
              << "Pitch: " << y << "º \n\n";

    std::cout << "R = \n" << R << "\n\n"
              << "R_alt = \n" << R_alt << "\n\n";

    std::cout << "R-R_alt = \n" << R-R_alt << '\n';

    return 0;
}
