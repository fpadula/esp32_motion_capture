#ifndef _IMUMANAGER_H_
#define _IMUMANAGER_H_

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
#include <net/if.h>
#include <sys/ioctl.h>
#include "IMU.h"

class IMUManager{
    private:
        std::vector<IMU> imus;
        int sockfd;    
        int n, nh;        
        char msg_type, *host_ip;
        
        bool waiting_broadcst_resp;
        struct sockaddr_in si_me, si_other, si_broad_rec;
    public:
        IMUManager(char *interface_name);

        void update_device_list();
        
        IMU *bind_device(std::string, int port);

        ~IMUManager();
        void close_connections();
};

#endif