
#include "inertial_alignment.hpp"
#include "inertial_alignment_functions.cpp"

int main ()
{
    // Arbitrary initialization, resized in 'loadCSV'
    Eigen::MatrixXd data(3,3);

    imu my_imu;
    camera my_cam;

    std::string filename_data("./Data/data.csv");
    std::string filename_imu("./Data/imu.yaml");
    std::string filename_camera("./Data/camera.yaml");

    if(loadCSV(filename_data, data) &&
       loadYAML<imu>(filename_imu, my_imu) &&
       loadYAML<camera>(filename_camera, my_cam))
   {
        std::cout << "Success loading all files. \n\n";

        my_imu.align(data, 10);
        my_imu.print();
        my_cam.print();
    }
    else{
        std::cout << "Error loading data file. \n";
    }

    return 0;
}
