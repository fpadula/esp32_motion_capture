#ifndef _IMU_H_
#define _IMU_H_

#include <string>
#include <iostream>
#include <sstream>
#include <thread>

#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <CommConsts.h>
#include <vector>
#include <atomic>

class IMU{
    private:
        std::string host_ip, imu_ip, name;
        int port;
        // volatile bool read_remote_data;
        std::atomic<bool> *read_remote_data;
        std::thread *read_remote_thread;
    public:
        float x, y, z, w;

        void get_sensor_data();
        void stop_data_collection();
        // IMU();
        IMU(std::string info_string);
        // ~IMU();
        std::string get_name();
        std::string get_host();
        int get_port();
        bool set_name(std::string name);
        bool set_port(int port);
        bool set_host(std::string host); 
        void set_fields_from_string(std::string info_string);
        void print();
        
};

#endif