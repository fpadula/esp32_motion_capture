#include <stdio.h>
#include <stdlib.h>
#include "IMU.h"
#include "IMUManager.h"

int main(int argc, char **argv) {
    // Starts manager on network interface eno1 (with its associated IP)
    IMUManager imngr((char *)"eno1");

    // Update device list (sends a broadcast to discover devices)
    imngr.update_device_list();
    // Binds device with name "hand" to this host with the specified port
    IMU *hand = imngr.bind_device("hand", 60000);
    // Binds device with name "arm" to this host with the specified port
    IMU *arm = imngr.bind_device("arm", 60001);
    // Binds device with name "forearm" to this host with the specified port
    IMU *forearm = imngr.bind_device("forearm", 60002);

    // Syncronize devices that were bound to this host. That means that their
    // internal counter and clock are both set to 0
    imngr.sync_devices();

    float q[4];
    hand->get_quaternion(q);
    arm->get_quaternion(q);
    forearm->get_quaternion(q);

    delete hand;
    delete arm;
    delete forearm;
}