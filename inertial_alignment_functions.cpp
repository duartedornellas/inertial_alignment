
/*---------------------------- CLASS METHODS ---------------------------------*/

void imu::print(){
    std::cout << "Rate: \n" << this->rate << '\n'
              << "Gyroscope noise density: \n" << this->gyroscope_nd << '\n'
              << "Gyroscope random walk: \n"   << this->gyroscope_rw << '\n'
              << "Accelerometer noise density: \n" << this->accelerometer_nd << '\n'
              << "Accelerometer random walk: \n"   << this->accelerometer_rw << '\n'
              << "Extrinsics: \n" << this->extrinsics << '\n'
              << "Alignment (roll): \n" << this->alignment.roll << '\n'
              << "Alignment (pitch): \n" << this->alignment.pitch << '\n'
              << "Alignment (R): \n" << this->alignment.R << "\n\n";
}

void imu::align(Eigen::MatrixXd &data, int samples){
    Eigen::Vector3d gb;
    gb << data.block(0,4,samples,1).mean(),
          data.block(0,5,samples,1).mean(),
          data.block(0,6,samples,1).mean();

    double x = atan2( gb(1), gb(2));
    double y = atan2(-gb(0), sqrt(gb(1)*gb(1) + gb(2)*gb(2)));

    this->alignment.roll = x*180/PI;
    this->alignment.pitch = y*180/PI;
    this->alignment.R <<     cos(y),       0,       -sin(y),
                         sin(x)*sin(y),  cos(x), sin(x)*cos(y),
                         cos(x)*sin(y), -sin(x), cos(x)*cos(y);
}

void camera::print(){
    std::cout << "Rate: \n" << this->rate << '\n'
              << "Resolution: \n" << this->resolution.transpose() << '\n'
              << "Distortion parameters: \n" << this->distortion.transpose() << '\n'
              << "Intrinsics: \n"   << this->intrinsics.transpose() << '\n'
              << "Extrinsics: \n" << this->extrinsics << "\n\n";
}

/*------------------------------ FUNCTIONS -----------------------------------*/

int loadCSV(std::string &filename, Eigen::MatrixXd &data){
    // Load data in std vector of std vectors
    std::vector<std::vector<double> > data_matrix;
    std::string line;
    std::ifstream filestream (filename.c_str());
    if (filestream.is_open()){
        std::cout << "Success opening file. '" << filename << "'.\n";
        while(getline(filestream, line)){
            std::stringstream lineStream(line);
            std::string cell;
            std::vector<double> data_aux;
            while(std::getline(lineStream, cell, ',')){
                data_aux.push_back(atof(cell.c_str()));
            }
            data_matrix.push_back(data_aux);
        }
        data_matrix.erase(data_matrix.begin()); // Delete first line

        // Copy data into Eigen matrix
        int lines = data_matrix.size();
        int cols  = data_matrix[0].size();
        data.resize(lines,cols);
        for(int i=0; i<lines; i++){
            for(int j=0; j<cols; j++){
                data(i,j) = data_matrix[i][j];
            }
        }
        filestream.close();
        return 1;
    }

    else{
        std::cout << "Error opening file. '" << filename << "'\n";
        return 0;
    }
}

template<> int loadYAML<imu>(std::string &filename, imu &s){
    std::string line;
    std::ifstream filestream (filename.c_str());
    if (filestream.is_open()){
        std::cout << "Success opening file '" << filename << "'.";
        while(getline(filestream, line)){
            std::stringstream lineStream(line);
            lineStream >> std::ws;
            std::string field, cell;
            std::getline(lineStream, field, ' ');
            while(std::getline(lineStream, cell, ' ')){
                if(field!="sensor_type:" &&
                   field!="data:" &&
                   field!="rate_hz:" &&
                   field!="gyroscope_noise_density:" &&
                   field!="gyroscope_random_walk:" &&
                   field!="accelerometer_noise_density:" &&
                   field!="accelerometer_random_walk:"){
                    break;
                }

                if(field=="sensor_type:"){
                    std::getline(lineStream, cell, ' ');
                    if(cell!="imu"){
                        std::cout << "Error: Incompatible yaml file/argument. "
                                  << "'sensor_type': " << cell << "; "
                                  << "'input_type': 'imu' \n";
                        filestream.close();
                        return 0;
                    }
                    break;
                }
                else if(field=="data:"){
                    field.clear();
                    int i=0, j=0;
                    cell.erase(cell.begin()); // delete '[' on 1st element
                    cell.pop_back();          // delete ',' on 1st element
                    s.extrinsics(i,j) = atof(cell.c_str());
                    j++;
                    while(std::getline(lineStream, cell, ',')){
                        s.extrinsics(i,j) = atof(cell.c_str());
                        j++;
                    }
                    j=0;
                    i++;
                    std::cout << '\n';
                    while(i<4){
                        getline(filestream, line);
                        std::stringstream lineStream(line);
                        lineStream >> std::ws;
                        std::string cell;
                        while(std::getline(lineStream, cell, ',')){
                            s.extrinsics(i,j) = atof(cell.c_str());
                            j++;
                        }
                        j=0;
                        i++;
                    }
                    break;
                }
                else if(field=="rate_hz:"){
                    s.rate = atof(cell.c_str());
                }
                else if(field=="gyroscope_noise_density:"){
                    s.gyroscope_nd = atof(cell.c_str());
                    break;
                }
                else if(field=="gyroscope_random_walk:"){
                    s.gyroscope_rw = atof(cell.c_str());
                    break;
                }
                else if(field=="accelerometer_noise_density:"){
                    s.accelerometer_nd = atof(cell.c_str());
                    break;
                }
                else if(field=="accelerometer_random_walk:"){
                    s.accelerometer_rw = atof(cell.c_str());
                    break;
                }
            }
        }
        filestream.close();
        return 1;
    }
    else{
        std::cout << "Error opening file '" << filename << "'\n";
        return 0;
    }
}

template<> int loadYAML<camera>(std::string &filename, camera &s){
    std::string line;
    std::ifstream filestream (filename.c_str());
    if (filestream.is_open()){
        std::cout << "Success opening file '" << filename << "'.";
        while(getline(filestream, line)){
            std::stringstream lineStream(line);
            lineStream >> std::ws;
            std::string field, cell;
            std::getline(lineStream, field, ' ');
            while(std::getline(lineStream, cell, ' ')){
                if(field!="sensor_type:" &&
                   field!="data:" &&
                   field!="rate_hz:" &&
                   field!="resolution:" &&
                   field!="intrinsics:" &&
                   field!="distortion_coefficients:"){
                    break;
                }

                if(field=="sensor_type:"){
                    std::getline(lineStream, cell, ' ');
                    if(cell!="camera"){
                        std::cout << "Error: Incompatible yaml file/argument. "
                                  << "'sensor_type': " << cell << "; "
                                  << "'input_type': 'camera' \n";
                        filestream.close();
                        return 0;
                    }
                    break;
                }
                else if(field=="data:"){
                    field.clear();
                    int i=0, j=0;
                    cell.erase(cell.begin()); // delete '[' on 1st element
                    cell.pop_back();          // delete ',' on 1st element
                    s.extrinsics(i,j) = atof(cell.c_str());
                    j++;
                    while(std::getline(lineStream, cell, ',')){
                        s.extrinsics(i,j) = atof(cell.c_str());
                        j++;
                    }
                    j=0;
                    i++;
                    std::cout << '\n';
                    while(i<4){
                        getline(filestream, line);
                        std::stringstream lineStream(line);
                        lineStream >> std::ws;
                        std::string cell;
                        while(std::getline(lineStream, cell, ',')){
                            s.extrinsics(i,j) = atof(cell.c_str());
                            j++;
                        }
                        j=0;
                        i++;
                    }
                    break;
                }
                else if(field=="rate_hz:"){
                    s.rate = atof(cell.c_str());
                }
                else if(field=="resolution:"){
                    field.clear();
                    cell.erase(cell.begin()); // delete '[' on 1st element
                    cell.pop_back();          // delete ',' on 1st element
                    int i=0;
                    s.resolution(i) = atof(cell.c_str());
                    i++;
                    while(std::getline(lineStream, cell, ',')){
                        s.resolution(i) = atof(cell.c_str());
                        i++;
                    }
                    break;
                }
                else if(field=="intrinsics:"){
                    field.clear();
                    cell.erase(cell.begin()); // delete '[' on 1st element
                    cell.pop_back();          // delete ',' on 1st element
                    int i=0;
                    s.intrinsics(i) = atof(cell.c_str());
                    i++;
                    while(i<4){
                        std::getline(lineStream, cell, ',');
                        s.intrinsics(i) = atof(cell.c_str());
                        i++;
                    }
                    break;
                }
                else if(field=="distortion_coefficients:"){
                    field.clear();
                    cell.erase(cell.begin()); // delete '[' on 1st element
                    cell.pop_back();          // delete ',' on 1st element
                    int i=0;
                    s.distortion(i) = atof(cell.c_str());
                    i++;
                    while(i<4){
                        std::getline(lineStream, cell, ',');
                        s.distortion(i) = atof(cell.c_str());
                        i++;
                    }
                    break;
                }
            }
        }

        filestream.close();
        return 1;
    }
    else{
        std::cout << "Error opening file '" << filename << "'\n";
        return 0;
    }
}

template<class sensor> int loadYAML(std::string &filename, sensor &s){
    std::cout << "Wrong input type. Usage: 'loadYAML<imu>(filename, imu)' or "
              << "'loadYAML<camera>(filename, camera)'. \n";
    return 0;
}
