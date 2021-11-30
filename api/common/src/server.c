// Server side implementation of UDP client-server model
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <CommConsts.h>
   
#define PORT 60000
// #define MAXLINE 16

// Driver code
int main() {
    int sockfd;
    uint8_t buffer[512], *buffer_ptr;
    char server_msg[512];
    char *hello = "Hello from server";
    struct sockaddr_in servaddr, cliaddr;
    float x, y, z, w;
    char msg_type;
    uint16_t msg_size;
       
    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }
       
    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));
       
    // Filling server information
    servaddr.sin_family    = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);
       
    // Bind the socket with the server address
    if ( bind(sockfd, (const struct sockaddr *)&servaddr, 
            sizeof(servaddr)) < 0 ){
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
       
    int len, n, nh;
   
    len = sizeof(cliaddr);  //len is value/resuslt

    while(1){        
        nh = recvfrom(sockfd, buffer, sizeof(msg_type) + sizeof(msg_size), MSG_PEEK, (struct sockaddr *) &cliaddr, &len);
        msg_type = buffer[0];
        memcpy(&msg_size, &buffer[1], sizeof(msg_size));
        n = recvfrom(sockfd, buffer, msg_size + nh, MSG_WAITALL, (struct sockaddr *) &cliaddr, &len);
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
            printf("%s", server_msg);
        }
    }
    // buffer[n] = '\0';
    // printf("Client : %s\n", buffer);
    // sendto(sockfd, (const char *)hello, strlen(hello), 
    //     MSG_CONFIRM, (const struct sockaddr *) &cliaddr,
    //         len);
    // printf("Hello message sent.\n"); 
       
    return 0;
}