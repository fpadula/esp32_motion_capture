#include "IMUManager.h"

IMUManager::IMUManager(char *interface_name){
    int broadcastEnable=1;
    struct timeval tv;
    struct ifreq ifr;

    // Creating socket file descriptor
    if ( (this->sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    // Adding broadcast permission to socket
    if (setsockopt(this->sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0){
        perror("Error");
    }

    // Adding 1s timeout to socket. We will wait at most 1s for broadcast responses
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    if (setsockopt(this->sockfd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
        perror("Error");
    }

    ifr.ifr_addr.sa_family = AF_INET;
    strncpy(ifr.ifr_name, interface_name, IFNAMSIZ-1);
    ioctl(this->sockfd, SIOCGIFADDR, &ifr);
    this->host_ip = inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);
    printf("IP associated with interface %s: %s\n", interface_name, this->host_ip);

    // Making sure sockaddr_in structures are initialized
    memset(&(this->si_me), 0, sizeof(this->si_me));
    memset(&(this->si_other), 0, sizeof(this->si_other));
    memset(&(this->si_broad_rec), 0, sizeof(this->si_broad_rec));

    // Filling server information
    (this->si_other).sin_family = AF_INET;
    (this->si_other).sin_port = htons(DISCOVERY_PORT);
    (this->si_other).sin_addr.s_addr = htonl(INADDR_BROADCAST);

    (this->si_me).sin_family = AF_INET;
    (this->si_me).sin_port = htons(DISCOVERY_PORT);
    (this->si_me).sin_addr.s_addr = inet_addr(this->host_ip);
    // Bind the socket with the host address
    if ( bind(this->sockfd, (struct sockaddr *)&(this->si_me),
            sizeof((this->si_me))) < 0 ){
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
}

void IMUManager::update_device_list(){
    unsigned int len;
    uint16_t msg_size;
    char buffer[MAXLINE], *buffer_ptr;
    char server_msg[MAXLINE];
    char ipstr[40];

    // Sending broadcast message
    len = sizeof(this->si_other);
    strncpy(buffer, "a", MAXLINE);
    // strncpy(buffer, argv[1], MAXLINE);
    sendto(this->sockfd, buffer, strlen(buffer), 0,
        (struct sockaddr *) &(this->si_other),
        len);

    len = sizeof(si_broad_rec);
    waiting_broadcst_resp = true;
    while(waiting_broadcst_resp){
        // Reading header. Note that we use flag MSG_PEEK or else recvfrom will empty the buffer
        nh = recvfrom(sockfd, buffer, sizeof(msg_type) + sizeof(msg_size), MSG_PEEK, (struct sockaddr *) &si_broad_rec, &len);
        msg_type = buffer[0];
        if(nh == -1){
            waiting_broadcst_resp = false;
            break;
        }
        memcpy(&msg_size, &buffer[1], sizeof(msg_size));
        // Now we know the message type and its size. So we read everything
        n = recvfrom(sockfd, buffer, msg_size + nh, MSG_WAITALL, (struct sockaddr *) &si_broad_rec, &len);

        buffer_ptr = buffer + nh; // Skiping header bytes

        if(msg_type == IMU_BDCST_RESP){
            memcpy(server_msg, buffer_ptr, msg_size);
            server_msg[msg_size] = '\0';

            inet_ntop(AF_INET, &si_broad_rec.sin_addr, ipstr, 40);
            // printf("Received: '%s' from %s\n", server_msg, ipstr);
            std::string info_string =  std::string(ipstr) +";"+std::string(server_msg);
            // std::cout << info_string << std::endl;
            // IMU temp = IMU(info_string);
            // std::cout << "Created object: " << &temp << std::endl;
            // imus.push_back(temp);
            imus.push_back(IMU(info_string));
        }
    }

    printf("Devices found:\n");
    for (auto i = imus.begin(); i != imus.end(); ++i)
        i->print();
}

IMU* IMUManager::bind_device(std::string device_name, int port){
    for (auto i = imus.begin(); i != imus.end(); ++i)
        if(i->get_name() == device_name)
            return &(*i);
    return NULL;
}

void IMUManager::close_connections(){
    for (auto i = imus.begin(); i != imus.end(); ++i)
        i->stop_data_collection();
}

IMUManager::~IMUManager(){
    
    
    close(this->sockfd);
}

// int main(int argc, char **argv) {
//     try{
//         IMUManager mngr((char*)"eno1");
//         // mngr.update_device_list();
//         return 0;
//     }
//     catch(std::string e){
//         std::cout << e << std::endl;
//     }
// }