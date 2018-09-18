
/*---------------------------- INCLUDES/MACROS -------------------------------*/

#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/StdVector>
#include <Eigen/Dense>

#define PI 3.14159265
#define G  9.80665


/*-------------------------------- CLASSES -----------------------------------*/

typedef struct alignment_data{
    double pitch;
    double roll;
    Eigen::Matrix3d R;
} alignment;

class imu{
    public:
        int rate;
        double gyroscope_nd;
        double gyroscope_rw;
        double accelerometer_nd;
        double accelerometer_rw;
        Eigen::Matrix4d extrinsics;
        alignment_data alignment;
        void print();
        void align(Eigen::MatrixXd &data, int samples);
};

class camera{
    public:
        int rate;
        Eigen::Vector2d resolution;
        Eigen::Vector4d distortion;
        Eigen::Vector4d intrinsics;
        Eigen::Matrix4d extrinsics;
        void print();
};


/*------------------------ FUNCTION PROTOTYPES -------------------------------*/

int loadCSV(std::string &filename, Eigen::MatrixXd &data);

template<class sensor> int loadYAML(std::string &filename, sensor &s);
template<> int loadYAML<imu>(std::string &filename, imu &s);
template<> int loadYAML<camera>(std::string &filename, camera &s);
