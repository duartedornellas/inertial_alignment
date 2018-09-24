
/*---------------------------- CLASS METHODS ---------------------------------*/

/* Extrinsics */
void sensor_extrinsics::print(){
    std::cout << "GT - Extrinsics (t): \n" << this->t.transpose() << '\n'
              << "GT - Extrinsics (R): \n" << this->R << '\n';
}

/* Ground truth */
ground_truth::ground_truth(std::string t){
    if(t == "vicon" || t == "mle"){
        this->type = t;
    }
    else{
        std::cout << "Wrong initialization type. Argument should be 'vicon' or 'mle'.";
    }
}

void ground_truth::print(){
    // Print ground truth type;
    std::cout << '\n'
              << "GT - type: '" << this->type << '\'' << '\n';
    // Print extrinics
    this->extrinsics.print();
}

void ground_truth::print(int index){
        // Print GT type & extrinsics
        // this->print();

        // Print i-th line GT data
        std::cout << '\n';
        std::cout << "GT - Timestamp, i = " << index << ": \n"
                  << this->data(index,0) << '\n';
        std::cout << "GT - Position, i = " << index << ": \n"
                  << this->data(index,1) << ", "
                  << this->data(index,2)
                  << this->data(index,3) << '\n';
        std::cout << "GT - Orientation, i = " << index << ": \n"
                  << this->data(index,4) << ", "
                  << this->data(index,5)
                  << this->data(index,6)
                  << this->data(index,7) << '\n';
}

/* Vicon */
void vicon::print(){
    std::cout << '\n'
              << "Vicon extrinsics: \n" << this->extrinsics << '\n';
}

Eigen::Vector3d vicon::t(){
    return this->extrinsics.block(0,3,3,1);
}

Eigen::Matrix3d vicon::R(){
    return this->extrinsics.block(0,0,3,3);
}

/* IMU */
void imu::print(){
    std::cout << '\n'
              << "Rate: \n" << this->rate << '\n'
              << "Accelerometer Bias: \n" << this->accelerometer.bias << '\n'
              << "Accelerometer noise density: \n" << this->accelerometer.nd << '\n'
              << "Accelerometer random walk: \n"   << this->accelerometer.rw << '\n'
              << "Gyroscope Bias: \n" << this->gyroscope.bias << '\n'
              << "Gyroscope noise density: \n" << this->gyroscope.nd << '\n'
              << "Gyroscope random walk: \n"   << this->gyroscope.rw << '\n'
              << "Extrinsics: \n" << this->extrinsics << '\n'
              << "Alignment (roll): \n" << this->alignment.roll << '\n'
              << "Alignment (pitch): \n" << this->alignment.pitch << '\n'
              << "Alignment (R): \n" << this->alignment.R << "\n\n";
}

void imu::initialize(ground_truth &gt, int samples){

    Eigen::Vector3d ab, gb;
    this->gyroscope.bias << gt.data.block(0,11,samples,1).mean(),
                            gt.data.block(0,12,samples,1).mean(),
                            gt.data.block(0,13,samples,1).mean();
    this->accelerometer.bias << gt.data.block(0,14,samples,1).mean(),
                                gt.data.block(0,15,samples,1).mean(),
                                gt.data.block(0,16,samples,1).mean();

    ab << this->data.block(0,4,samples,1).mean(),
          this->data.block(0,5,samples,1).mean(),
          this->data.block(0,6,samples,1).mean();
    ab -= this->accelerometer.bias;

    double x = atan2( ab(1), ab(2));
    double y = atan2( -ab(0), sqrt(ab(1)*ab(1) + ab(2)*ab(2)));
    this->alignment.roll = x*180/PI;
    this->alignment.pitch = y*180/PI;

    // wRb - Rotates from the body to the world frame:
    this->alignment.R <<     cos(y), sin(x)*sin(y), cos(x)*sin(y),
                                  0,        cos(x),       -sin(x),
                            -sin(y), sin(x)*cos(y), cos(x)*cos(y);
}

/* Camera */
void camera::print(){
    std::cout << '\n'
              << "Rate: \n" << this->rate << '\n'
              << "Resolution: \n" << this->resolution.transpose() << '\n'
              << "Distortion parameters: \n" << this->distortion.transpose() << '\n'
              << "Intrinsics: \n"   << this->intrinsics.transpose() << '\n'
              << "Extrinsics: \n" << this->extrinsics << "\n\n";
}

/* Pose */
int pose::initialize(ground_truth &gt, imu &s){
    // Timestamp
    this->timestamp = s.data(0,0);
    // Expressed in the world ref. frame
    this->position << gt.data(0,1), gt.data(0,2), gt.data(0,3);
    this->velocity << 0, 0, 0;
    // wRb - Rotates from the body to the world frame:
    this->orientation = s.alignment.R;
    return 1;
}

int pose::update(imu &s, int index){
    // Get imu measurements
    double t = s.data(index,0);
    Eigen::Vector3d ab, wb;
    ab << s.data(index,1), s.data(index,2), s.data(index,3);
    wb << s.data(index,4), s.data(index,5), s.data(index,6);

    double rate = s.rate;
    double dt = 1/rate;

    // Timestamp update
    this->timestamp = t;

    // Orientation update: w(dR)b
    wb -= s.gyroscope.bias;
    Eigen::AngleAxisd dw (wb.norm()*dt, wb.normalized());
    Eigen::Matrix3d dR;
    dR = dw;
    this->orientation *= dR;

    // Velocity update
    ab -= s.accelerometer.bias;
    Eigen::Vector3d dv = dt * ( this->orientation*ab + Eigen::Vector3d(0,0,-G) );
    this->velocity += dv;

    // Position update
    Eigen::Vector3d dp = dt * this->velocity;
    this->position += dp;

    // Debug
    // std::cout << "\n---------------------------- DEBUG ----------------------------\n"
    //           << "rate: \t dt: \n" << s.rate << '\t' << dt << '\n'
    //           << "dR:\n" << dR << '\n'
    //           << "dw:\n" << dw.angle() << ", " << dw.axis().transpose() << '\n'
    //           << "dv:\n" << dv.transpose() << '\n'
    //           << "dp:\n" << dp.transpose() << '\n'
    //           << "---------------------------------------------------------------\n\n";

    return 0;
}

void pose::print(){
    Eigen::Quaterniond orientation_quaternion;
    orientation_quaternion = this->orientation;
    std::cout << '\n'
              << "Timestamp:\n" << this->timestamp << '\n'
              << "Position: \n" << this->position.transpose() << '\n'
              << "Velocity: \n" << this->velocity.transpose() << "\n"
              << "Orientation: \n" << orientation_quaternion.w() << ", "
                                   << orientation_quaternion.vec().transpose()
                                   << "\n\n";
}

/*------------------------------ FUNCTIONS -----------------------------------*/

/* Data I/O */
int loadCSV(std::string &filename, Eigen::MatrixXd &data){
    // Load data in std vector of std vectors
    std::vector<std::vector<double> > data_matrix;
    std::string line;
    std::ifstream filestream (filename.c_str());
    if (filestream.is_open()){
        std::cout << "Success opening file '" << filename << "'.\n";
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

template<> int loadYAML<ground_truth>(std::string &filename, ground_truth &gt){
    std::string extension = filename.substr(filename.size()-4, 4);
    if(extension=="yaml"){
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
                    field!="data:"){
                        break;
                    }

                    if(field=="sensor_type:"){
                        std::getline(lineStream, cell, ' ');
                        if(cell!="pose" && cell != "visual-inertial"){
                            std::cout << "Error: Incompatible yaml file/argument. "
                            << "'sensor_type': " << cell << "; "
                            << "'input_type': 'pose' \n";
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
                        gt.extrinsics.T(i,j) = atof(cell.c_str());
                        j++;
                        while(std::getline(lineStream, cell, ',')){
                            gt.extrinsics.T(i,j) = atof(cell.c_str());
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
                                gt.extrinsics.T(i,j) = atof(cell.c_str());
                                j++;
                            }
                            j=0;
                            i++;
                        }
                        break;
                    }
                }
            }
            filestream.close();
            gt.extrinsics.t = gt.extrinsics.T.block(0,3,3,1);
            gt.extrinsics.R = gt.extrinsics.T.block(0,0,3,3);
            return 1;
        }
        else{
            std::cout << "Error opening file '" << filename << "'\n";
            return 0;
        }
    }
    else{
        std::cout << "Wrong file extension. Should be a '.yaml' file. \n";
    }
}

template<> int loadYAML<vicon>(std::string &filename, vicon &s){
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
                   field!="data:"){
                    break;
                }

                if(field=="sensor_type:"){
                    std::getline(lineStream, cell, ' ');
                    if(cell!="pose"){
                        std::cout << "Error: Incompatible yaml file/argument. "
                                  << "'sensor_type': " << cell << "; "
                                  << "'input_type': 'pose' \n";
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
                    s.gyroscope.nd = atof(cell.c_str());
                    break;
                }
                else if(field=="gyroscope_random_walk:"){
                    s.gyroscope.rw = atof(cell.c_str());
                    break;
                }
                else if(field=="accelerometer_noise_density:"){
                    s.accelerometer.nd = atof(cell.c_str());
                    break;
                }
                else if(field=="accelerometer_random_walk:"){
                    s.accelerometer.rw = atof(cell.c_str());
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

/* Misc */
void compute_error(pose &p_error, pose &p, ground_truth &gt){
    int index = find_index(p.timestamp, gt);
    if(index != -1 && index < gt.data.rows()){ // check timestamp validity
        // fazer contas
    }
    else{
        std::cout << "Unaligned imu/ground_truth timestamps. \n";
    }
}

void align_datasets(ground_truth &gt, imu &s){
    // std::cout << std::fixed;
    // std::cout << gt.data(0,0)-s.data(0,0)  << '\n';
    if(find_index(s.data(0,0), gt) == -1){
        // Delete extra rows in 'gt.data'
        double nsamples = (gt.data(0,0)-s.data(0,0)) * pow(10,-9) * 200;
        for(int i=0; i<nsamples; i++){
            removeRow(s.data, 0);
        }
    }
    // std::cout << s.data(0,0) - gt.data(0,0)  << '\n';
}

int find_index(double t, ground_truth &gt){
    if(t < gt.data(0,0)){
      std::cout << "(t_imu < t_gt) --> "
                << (gt.data(0,0)-t) * pow(10,-9) * 200
                << " samples to go. \n";
      return -1;
    }
    for(int index=0; index<gt.data.rows(); index++){
        if(t >= gt.data(index,0) && t <= gt.data(index+1,0)){
            return index;
        }
    }
    std::cout << "(t_imu > t_gt) --> "
              << (t-gt.data(gt.data.size()-1,0)) * pow(10,-9) * 200
              << " extra samples. \n";
    return -1;
}

void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove){
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}

void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove){
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}
