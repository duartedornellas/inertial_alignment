
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
    Eigen::Matrix3d R; // bRw - Rotates from the world to the body frame
} alignment;           //       Update to use quaternions

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

class pose{
    public:
        Eigen::Vector3d velocity;
        Eigen::Vector3d position;
        Eigen::Matrix3d orientation;    // Update to use quaternions
        int initialize(imu &s);
        int update(Eigen::Vector3d &a, Eigen::Vector3d &w);
        void print();
};


/*------------------------ FUNCTION PROTOTYPES -------------------------------*/

int loadCSV(std::string &filename, Eigen::MatrixXd &data);

template<class sensor> int loadYAML(std::string &filename, sensor &s);
template<> int loadYAML<imu>(std::string &filename, imu &s);
template<> int loadYAML<camera>(std::string &filename, camera &s);
