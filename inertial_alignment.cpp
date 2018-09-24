
#include "inertial_alignment.hpp"
#include "inertial_alignment_functions.cpp"

int main (int argc, char *argv[])
{
    std::string folder;
    if(argc==1){
        folder = "../01 Datasets/The EuRoc MAV Dataset/raw/V1_01_easy/mav0";
    }
    else if(argc==2){
        folder = argv[1];
    }
    else{
        std::cout << "Error: wrong number of arguments. "
                  << "Use: './inertial_alignment <path_to_data_folder>' \n";
        return 0;
    }

    std::string filename_data_mle     (folder+"/state_groundtruth_estimate0/data.csv");
    std::string filename_data_vicon   (folder+"/vicon0/data.csv");
    std::string filename_data_imu     (folder+"/imu0/data.csv");
    std::string filename_specs_mle    (folder+"/state_groundtruth_estimate0/sensor.yaml");
    std::string filename_specs_vicon  (folder+"/vicon0/sensor.yaml");
    std::string filename_specs_imu    (folder+"/imu0/sensor.yaml");

    ground_truth gt("mle");
    imu   my_imu;
    pose  my_pose, my_pose_error;

    if(loadCSV(filename_data_mle, gt.data) &&
       loadCSV(filename_data_imu, my_imu.data) &&
       loadYAML<ground_truth>(filename_specs_mle, gt) &&
       loadYAML<imu>(filename_specs_imu, my_imu))
   {
       align_datasets(gt, my_imu);
        // gt.print();
        gt.print(0);

        my_imu.initialize(gt, 10);
        my_pose.initialize(gt, my_imu);
        my_pose.print();

        /*-------------------- Inertial navigation debug ---------------------*/
        // Eigen::Vector3d wb(0,0,0);
        // Eigen::Vector3d aw(0,0,-G);
        // Eigen::Vector3d ab;
        // ab = -my_pose.orientation.transpose()*aw;

        for(int i=0; i<1; i++){
            my_pose.update(my_imu, i);
        }
        my_pose.print();
        compute_error(my_pose_error, my_pose, gt);
        /*--------------------------------------------------------------------*/
    }
    return 0;
}
