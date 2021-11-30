// Client side implementation of UDP client-server model
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <CommConsts.h>
#include <vector>

#define PORT 60000
#define MAXLINE 1024

using namespace std;

class IMU{
    private:
        char host_ip[17], name[64], str_rep[115];
        int port;
    public:
        IMU();
        IMU(char *info_string){
            sscanf(info_string, "%63[^;];%16[^;];%d", this->name, this->host_ip, &this->port);
            // sscanf(info_string, "%s;%s;%d", this->name, this->host_ip, &this->port);
        }
        // string toString(){
        //     sprintf(str_rep, "Name: '%s', Host: %s, Port: %d", this->name, this->host_ip, this->port);
        //     return string(this->str_rep);
        // }
        void print(){
            sprintf(str_rep, "Name: '%s', Host: %s, Port: %d", this->name, this->host_ip, this->port);
            printf("%s\n", str_rep);
        }
};

// Driver code
int main(int argc, char **argv) {
    vector<IMU> imus;
    // test = new IMU("Nome;HostIP;9000");    
    // test->print();
    // delete test;
    // return 0;
    int sockfd;    
    struct sockaddr_in si_me, si_other, si_broad_rec;
    float x, y, z, w;
    int n, nh;
    unsigned int len;
    char msg_type;
    uint16_t msg_size;
    char buffer[512], *buffer_ptr;
    char server_msg[512];
    char ipstr[40];
    int broadcastEnable=1;
    struct timeval tv;

    
    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }
    
    if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0){
        perror("Error");
    }    
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
        perror("Error");
    }
    memset(&si_me, 0, sizeof(si_me));    
    memset(&si_other, 0, sizeof(si_other));    
    memset(&si_broad_rec, 0, sizeof(si_broad_rec));    

    // Filling server information
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);
    // si_other.sin_addr.s_addr = inet_addr("192.168.0.61");
    si_other.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(PORT);
    si_me.sin_addr.s_addr = INADDR_ANY;
    // servaddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    // si_me.sin_addr.s_addr = inet_addr("192.168.0.34");
    // si_other.sin_addr.s_addr = inet_addr("192.168.0.61");
    // si_me.sin_addr.s_addr = INADDR_ANY;
    
    // Bind the socket with the server address
    if ( bind(sockfd, (struct sockaddr *)&si_me, 
            sizeof(si_me)) < 0 ){
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    len = sizeof(si_other);
    // strncpy(buffer, "0", MAXLINE);
    // strncpy(buffer, "1", MAXLINE);
    strncpy(buffer, "4", MAXLINE);
    // strncpy(buffer, argv[1], MAXLINE);
    sendto(sockfd, buffer, strlen(buffer), 0, 
        (struct sockaddr *) &si_other,
        len);
    printf("%s message sent.\n", buffer);
        
    len = sizeof(si_broad_rec);
    while(1){        
        nh = recvfrom(sockfd, buffer, sizeof(msg_type) + sizeof(msg_size), MSG_PEEK, (struct sockaddr *) &si_broad_rec, &len);
        msg_type = buffer[0];
        if(nh == -1)
            break;
        memcpy(&msg_size, &buffer[1], sizeof(msg_size));
        n = recvfrom(sockfd, buffer, msg_size + nh, MSG_WAITALL, (struct sockaddr *) &si_broad_rec, &len);
        buffer_ptr = buffer + nh; // Skiping header bytes

        if (msg_type == IMU_MSG_TYPE_DATA){
            memcpy(&x, &buffer_ptr[0], sizeof(float));
            memcpy(&y, &buffer_ptr[4], sizeof(float));
            memcpy(&z, &buffer_ptr[8], sizeof(float));
            memcpy(&w, &buffer_ptr[12], sizeof(float));
            printf("Type: %c, size %d (header %d, data %d), data: x: %.2f, y: %.2f, z: %.2f, w: %.2f\n", msg_type, msg_size, nh, n, x, y, z, w);
        }
        else if(msg_type == IMU_MSG_TYPE_INFO){
            memcpy(server_msg, buffer_ptr, msg_size);
            server_msg[msg_size] = '\0';
            
            inet_ntop(AF_INET, &si_broad_rec.sin_addr, ipstr, 40);
            // printf("Received: '%s' from %s\n", server_msg, ipstr);
            imus.push_back(IMU(server_msg));
        }
    }
    for (auto i = imus.begin(); i != imus.end(); ++i)
        i->print();

    // while(1){
    //     n = recvfrom(sockfd, (uint8_t *)buffer, sizeof(float)*4, 
    //                 MSG_WAITALL, ( struct sockaddr *) &cliaddr,
    //                 &len);    
    //     memcpy(&x, &buffer[0], sizeof(float));
    //     memcpy(&y, &buffer[4], sizeof(float));
    //     memcpy(&z, &buffer[8], sizeof(float));
    //     memcpy(&w, &buffer[12], sizeof(float));
    //     printf("x: %.2f, y: %.2f, z: %.2f, w: %.2f\n", x, y, z, w);
    // }

    close(sockfd);
    return 0;
}
