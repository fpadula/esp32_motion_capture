// Client side implementation of UDP client-server model
#include <stdio.h>
#include <stdlib.h>
#include "IMU.h"
#include "IMUManager.h"
#include <chrono>
#include <thread>

using namespace std;
int main(int argc, char **argv) {

    // IMUManager mngr((char*)"eno1");
    // mngr.update_device_list();
    // return 0;
    IMU hand("192.168.0.61;Hand;192.168.0.34;60000");
    // IMU *hand = mngr.bind_device("Hand", 60000);
    // // hand->get_sensor_data();
    // while(true){
    //     cout << "x: " << hand->x << ", y: " << hand->y << ", z: " << hand->z << ", w: " << hand->w << endl;
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // }
    // mngr.close_connections();
    hand.stop_data_collection();
    return 0;

    // vector<IMU> imus;
    // int sockfd;    
    // struct sockaddr_in si_me, si_other, si_broad_rec;
    // float x, y, z, w;
    // int n, nh;
    // unsigned int len;
    // char msg_type, *host_ip;
    // uint16_t msg_size;
    // char buffer[512], *buffer_ptr;
    // char server_msg[512];
    // char ipstr[40];
    // int broadcastEnable=1;
    // struct timeval tv;
    // bool waiting_broadcst_resp;
    // struct ifreq ifr;

    // // Creating socket file descriptor
    // if ( (sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0 ) {
    //     perror("socket creation failed");
    //     exit(EXIT_FAILURE);
    // }
    
    // // Adding broadcast permission to socket
    // if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0){
    //     perror("Error");
    // }

    // // Adding 1s timeout to socket. We will wait at most 1s for broadcast responses
    // tv.tv_sec = 1;
    // tv.tv_usec = 0;
    // if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
    //     perror("Error");
    // }

    // ifr.ifr_addr.sa_family = AF_INET;
    // strncpy(ifr.ifr_name, "eno1", IFNAMSIZ-1);
    // ioctl(sockfd, SIOCGIFADDR, &ifr);
    // host_ip = inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);
    // printf("%s\n", host_ip);

    // // Making sure sockaddr_in structures are initialized
    // memset(&si_me, 0, sizeof(si_me));    
    // memset(&si_other, 0, sizeof(si_other));    
    // memset(&si_broad_rec, 0, sizeof(si_broad_rec));    

    // // Filling server information
    // si_other.sin_family = AF_INET;
    // si_other.sin_port = htons(DISCOVERY_PORT);
    // si_other.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    // si_me.sin_family = AF_INET;
    // si_me.sin_port = htons(DISCOVERY_PORT);
    // si_me.sin_addr.s_addr = inet_addr(host_ip);
    // // si_me.sin_addr.s_addr = INADDR_ANY;
    // // si_me.sin_addr.s_addr = inet_addr("192.168.0.34");
    // // Bind the socket with the host address
    // if ( bind(sockfd, (struct sockaddr *)&si_me, 
    //         sizeof(si_me)) < 0 ){
    //     perror("bind failed");
    //     exit(EXIT_FAILURE);
    // }    
    // // Sending broadcast message
    // len = sizeof(si_other);
    // strncpy(buffer, "a", MAXLINE);
    // // strncpy(buffer, argv[1], MAXLINE);
    // sendto(sockfd, buffer, strlen(buffer), 0, 
    //     (struct sockaddr *) &si_other,
    //     len);
    // // printf("%s message sent.\n", buffer);

    // len = sizeof(si_broad_rec);
    // waiting_broadcst_resp = true;
    // while(waiting_broadcst_resp){        
    //     // Reading header. Note that we use flag MSG_PEEK or else recvfrom will empty the buffer
    //     nh = recvfrom(sockfd, buffer, sizeof(msg_type) + sizeof(msg_size), MSG_PEEK, (struct sockaddr *) &si_broad_rec, &len);
    //     msg_type = buffer[0];
    //     if(nh == -1){
    //         waiting_broadcst_resp = false;
    //         break;
    //     }
    //     memcpy(&msg_size, &buffer[1], sizeof(msg_size));
    //     // Now we know the message type and its size. So we read everything
    //     n = recvfrom(sockfd, buffer, msg_size + nh, MSG_WAITALL, (struct sockaddr *) &si_broad_rec, &len);

    //     buffer_ptr = buffer + nh; // Skiping header bytes

    //     if (msg_type == IMU_MSG_TYPE_DATA){
    //         memcpy(&x, &buffer_ptr[0], sizeof(float));
    //         memcpy(&y, &buffer_ptr[4], sizeof(float));
    //         memcpy(&z, &buffer_ptr[8], sizeof(float));
    //         memcpy(&w, &buffer_ptr[12], sizeof(float));
    //         printf("Type: %c, size %d (header %d, data %d), data: x: %.2f, y: %.2f, z: %.2f, w: %.2f\n", msg_type, msg_size, nh, n, x, y, z, w);
    //     }
    //     else if(msg_type == IMU_BDCST_RESP){
    //         memcpy(server_msg, buffer_ptr, msg_size);
    //         server_msg[msg_size] = '\0';
            
    //         inet_ntop(AF_INET, &si_broad_rec.sin_addr, ipstr, 40);
    //         // printf("Received: '%s' from %s\n", server_msg, ipstr);
    //         imus.push_back(IMU(server_msg));
    //     }
    // }
    
    // for (auto i = imus.begin(); i != imus.end(); ++i)
    //     i->print();

    // // while(1){
    // //     n = recvfrom(sockfd, (uint8_t *)buffer, sizeof(float)*4, 
    // //                 MSG_WAITALL, ( struct sockaddr *) &cliaddr,
    // //                 &len);    
    // //     memcpy(&x, &buffer[0], sizeof(float));
    // //     memcpy(&y, &buffer[4], sizeof(float));
    // //     memcpy(&z, &buffer[8], sizeof(float));
    // //     memcpy(&w, &buffer[12], sizeof(float));
    // //     printf("x: %.2f, y: %.2f, z: %.2f, w: %.2f\n", x, y, z, w);
    // // }

    // close(sockfd);
    // return 0;
}
