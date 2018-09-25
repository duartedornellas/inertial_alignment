
/*---------------------------- INCLUDES/MACROS -------------------------------*/

#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/StdVector>
#include <Eigen/Dense>

#define PI 3.14159265
#define G  9.80665


/*-------------------------------- CLASSES -----------------------------------*/
//--- Codependent classes declaration
class pose;

//--- Classes definition
typedef struct alignment_data{
    double pitch;
    double roll;
    Eigen::Matrix3d R;                  // wRb, Update to quaternions
} alignment;

typedef struct sensor_accelerometer{
    double nd;
    double rw;
    Eigen::Vector3d bias;
} sensor_accelerometer;

typedef struct sensor_gyroscope{
    double nd;
    double rw;
    Eigen::Vector3d bias;
} sensor_gyroscope;

class sensor_extrinsics{
    public:
        Eigen::Matrix4d T;
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        void print();
};

class ground_truth{
    public:
        //--- 'ground_truth' info
        std::string type;               // "vicon", "mle"
        sensor_extrinsics extrinsics;
        //--- 'ground_truth' data
        Eigen::MatrixXd data;
        //--- 'ground_truth' methods
        ground_truth(std::string t);
        int  get_pose(pose &pose_gt, int row);
        void print();
        void print(int index);
};

class vicon{
    public:
        Eigen::Matrix4d extrinsics;
        void print();
        Eigen::Vector3d t();
        Eigen::Matrix3d R();
};

class imu{
    public:
        //--- 'imu' info
        int rate;
        sensor_accelerometer accelerometer;
        sensor_gyroscope gyroscope;
        Eigen::Matrix4d extrinsics;
        alignment_data alignment;
        //--- 'imu' data
        Eigen::MatrixXd data;
        //--- 'imu' methods
        void print();
        void initialize(ground_truth &gt, int samples);
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
        double timestamp;
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Matrix3d orientation;    // wRb, Update to quaternions
        int initialize(ground_truth &gt, imu &s);
        int update(imu &s, int row);
        void print();
};


/*------------------------ FUNCTION PROTOTYPES -------------------------------*/

/* File I/O */
int loadCSV(std::string &filename, Eigen::MatrixXd &data);

template<class sensor> int loadYAML(std::string &filename, sensor &s);
template<> int loadYAML<vicon>(std::string &filename, vicon &s);
template<> int loadYAML<imu>(std::string &filename, imu &s);
template<> int loadYAML<camera>(std::string &filename, camera &s);

/* Misc */
void compute_error(pose &p_error, pose &p, pose & p_gt);

void align_datasets(ground_truth &gt, imu &s);
int  find_index(double t, ground_truth &gt);
void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove);
void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove);
