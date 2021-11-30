#include "IMU.h"

void IMU::set_fields_from_string(std::string info_string){
    size_t start_i, end_i;
    
    start_i = 0;
    end_i = info_string.find(';');
    if(end_i == std::string::npos){                
        throw "Malformed info string: '" + info_string + "'";
    }
    this->imu_ip = info_string.substr(start_i, end_i -  start_i);

    start_i=end_i+1;
    end_i = info_string.find(';', start_i);
    if(end_i == std::string::npos){                
        throw "Malformed info string: '" + info_string + "'";
    }
    this->name = info_string.substr(start_i, end_i -  start_i);
    
    start_i=end_i+1;
    end_i = info_string.find(';', start_i);
    if(end_i == std::string::npos){                
        throw "Malformed info string: '" + info_string + "'";
    }
    this->host_ip = info_string.substr(start_i, end_i -  start_i);
    
    start_i=end_i+1;
    end_i = info_string.length();
    if(start_i == end_i){
        throw "Malformed info string: '" + info_string + "'";
    }
    try{                
        this->port = stoi(info_string.substr(start_i, end_i -  start_i));
    }
    catch(std::invalid_argument& e){
        throw std::string("Cannot parse port");             
    }
}

// IMU::IMU(){
//     std::cout << "This should not have been called..." << std::endl;
// }

IMU::IMU(std::string info_string){
    set_fields_from_string(info_string);
    read_remote_data = new std::atomic<bool>(true);
    read_remote_thread = new std::thread(&IMU::get_sensor_data, this);
}

void IMU::print(){
    std::cout <<"\tName: " << name << ", Host: " << host_ip << 
        ", IMU ip: " << imu_ip << ", Port: " << port << std::endl;
}

int IMU::get_port(){return port;}

std::string IMU::get_name(){return name;}

std::string IMU::get_host(){return host_ip;}

bool IMU::set_name(std::string name){
    return true;
}

bool IMU::set_port(int port){
    return true;
}

bool IMU::set_host(std::string host){
    return true;
}

void IMU::get_sensor_data(){
    int sockfd;    
    struct sockaddr_in si_me, si_other;
    // float x, y, z, w;
    int n, nh;
    unsigned int len;
    char msg_type;
    uint16_t msg_size;
    char buffer[512], *buffer_ptr;
    char server_msg[512];
    // char ipstr[40];
    // int broadcastEnable=1;
    struct timeval tv;

    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }
    
    // if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0){
    //     perror("Error");
    // }    
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
        perror("Error");
    }
    memset(&si_me, 0, sizeof(si_me));    
    memset(&si_other, 0, sizeof(si_other));    
    // memset(&si_broad_rec, 0, sizeof(si_broad_rec));    

    // Filling server information
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(port);
    si_other.sin_addr.s_addr = inet_addr(imu_ip.c_str());
    // si_other.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(port);
    // si_me.sin_addr.s_addr = INADDR_ANY;
    // servaddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    si_me.sin_addr.s_addr = inet_addr(host_ip.c_str());
    // si_other.sin_addr.s_addr = inet_addr("192.168.0.61");
    // si_me.sin_addr.s_addr = INADDR_ANY;
    
    // Bind the socket with the server address
    if ( bind(sockfd, (struct sockaddr *)&si_me, 
            sizeof(si_me)) < 0 ){
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // len = sizeof(si_other);
    // strncpy(buffer, "6", MAXLINE);
    // // strncpy(buffer, argv[1], MAXLINE);
    // sendto(sockfd, buffer, strlen(buffer), 0, 
    //     (struct sockaddr *) &si_other,
    //     len);
    // printf("%s message sent.\n", buffer);
        
    len = sizeof(si_other);
    *read_remote_data = true;
    std::cout << "Start reading remote data..." << std::endl;
    while(read_remote_data->load()){        
        std::cout << "Reading remote data... " << read_remote_data->load()  << " " << this << std::endl;
        nh = recvfrom(sockfd, buffer, sizeof(msg_type) + sizeof(msg_size), MSG_PEEK, (struct sockaddr *) &si_other, &len);
        msg_type = buffer[0];
        // if(nh == -1)
        //     break;
        memcpy(&msg_size, &buffer[1], sizeof(msg_size));
        n = recvfrom(sockfd, buffer, msg_size + nh, MSG_WAITALL, (struct sockaddr *) &si_other, &len);
        buffer_ptr = buffer + nh; // Skiping header bytes

        if (msg_type == IMU_MSG_TYPE_DATA){
            memcpy(&x, &buffer_ptr[0], sizeof(float));
            memcpy(&y, &buffer_ptr[4], sizeof(float));
            memcpy(&z, &buffer_ptr[8], sizeof(float));
            memcpy(&w, &buffer_ptr[12], sizeof(float));
            // printf("Type: %c, size %d (header %d, data %d), data: x: %.2f, y: %.2f, z: %.2f, w: %.2f\n", msg_type, msg_size, nh, n, x, y, z, w);
        }
        else if(msg_type == IMU_MSG_TYPE_INFO){
            memcpy(server_msg, buffer_ptr, msg_size);
            server_msg[msg_size] = '\0';            
            printf("%s", server_msg);        
        }
    }

    close(sockfd);
    std::cout << "Closing socket: " << read_remote_data->load() << std::endl;
    // printf("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n");  
}

void IMU::stop_data_collection(){
    std::cout << "Stoping data collection!" << std::endl;
    *read_remote_data = false;
    std::cout << read_remote_data->load() << this << std::endl;
    this->read_remote_thread->join();
}

// IMU::~IMU(){
//     stop_data_collection();
// }