
#include "inertial_alignment.hpp"
#include "inertial_alignment_functions.cpp"

int main (int argc, char *argv[])
{
    std::string folder;
    if(argc==1){
        folder = "./Data";
    }
    else if(argc==2){
        folder = argv[1];
    }
    else{
        std::cout << "Error: wrong number of arguments. "
                  << "Use: './inertial_alignment <path_to_data_folder>' \n";
        return 0;
    }

    std::string filename_data   (folder+"/imu/data.csv");
    std::string filename_imu    (folder+"/imu/sensor.yaml");
    std::string filename_camera (folder+"/cam0/sensor.yaml");

    Eigen::MatrixXd data(3,3);    // Arbitrary initialization, resized in 'loadCSV'

    imu my_imu;
    camera my_cam;
    pose my_pose;

    if(loadCSV(filename_data, data) &&
       loadYAML<imu>(filename_imu, my_imu) &&
       loadYAML<camera>(filename_camera, my_cam))
   {
        std::cout << "Success loading all files. \n\n";

        my_imu.align(data, 10);
        my_imu.print();
        my_cam.print();

        my_pose.initialize(my_imu);
        my_pose.print();
        // inertial navigation, now!

    }

    return 0;
}
