#include <Arduino.h>
#include "InteractiveMenu.h"
#include "user_input.h"

// For saving data into non-volatile memory
#include <Preferences.h>

// Improve analog reading precision for battery management
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>

// Wifi
#include <WiFi.h>
#include <WiFiUdp.h>

// MPU
#include <MPU9250.h>

// Constants used to communicate with host
#include <CommConsts.h>

#define BUFFER_LEN 512

char ssid[MAX_STR_SIZE], password[MAX_STR_SIZE], host_ip[MAX_STR_SIZE];
char device_name[MAX_STR_SIZE];
int port, quat_filter, quat_iters, packet_min_interval;
char receivedChars[MAX_STR_SIZE];
bool connected_to_usb, accessing_menu;
float acc_bias[3], gyro_bias[3], mag_bias[3], mag_scale[3], mag_decli;
uint8_t buffer[BUFFER_LEN];
InteractiveMenu menu(5);
Preferences preferences;
WiFiClient client;
WiFiUDP udp, upd_discovery;
MPU9250 mpu;

volatile bool blink;

void blink_led_loop(void *parameter){
    bool led_state = HIGH;

    while(blink){
        if(led_state == HIGH)
            led_state = LOW;
        else
            led_state = HIGH;
        digitalWrite(LED_BUILTIN, led_state);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelete( NULL );
}

void enable_blink(){
    blink = true;
    xTaskCreatePinnedToCore(
        blink_led_loop,
        "blink_led_loop",
        10000,
        NULL,
        2,
        NULL,
        0
    );
}

void disable_blink(){
    blink = false;
}

void set_ip(){
    Serial.println("Type the host ip address and press enter.");

    if (read_input_text(receivedChars, MAX_STR_SIZE)){
        strncpy(host_ip, receivedChars, MAX_STR_SIZE);
        Serial.print("IP address set to '"); Serial.print(host_ip);
    }
    else
        Serial.print("Invalid ip address. Aborted.");
    Serial.println("'. Press any key to continue.");
    wait_key_press();
}

void set_ssid(){
    Serial.println("Type the network SSID and press enter.");
    if (read_input_text(receivedChars, MAX_STR_SIZE)){
        strncpy(ssid, receivedChars, MAX_STR_SIZE);
        Serial.print("SSID set to '"); Serial.print(ssid);
    }
    else
        Serial.print("Invalid network SSID. Aborted.");
    Serial.println("'. Press any key to continue.");
    wait_key_press();
}

void set_password(){
    Serial.println("Type the network password and press enter.");
    if (read_input_text(receivedChars, MAX_STR_SIZE)){
        strncpy(password, receivedChars, MAX_STR_SIZE);
        Serial.print("Password set to '"); Serial.print(password);
    }
    else
        Serial.print("Invalid network password. Aborted.");
    Serial.println("'. Press any key to continue.");
    wait_key_press();
}

void set_device_name(){
    Serial.println("Type the device name press enter.");
    if (read_input_text(receivedChars, MAX_STR_SIZE)){
        strncpy(device_name, receivedChars, MAX_STR_SIZE);
        Serial.print("Device name set to '"); Serial.print(device_name);
    }
    else
        Serial.print("Invalid device name. Aborted.");
    Serial.println("'. Press any key to continue.");
    wait_key_press();
}

void set_port(){
    Serial.println("Type the port and press enter.");
    if (read_input_int(port, receivedChars, MAX_STR_SIZE)){
        Serial.print("Port set to '"); Serial.print(port);
    }
    else
        Serial.print("Invalid port. Aborted.");
    Serial.println("'. Press any key to continue.");
    wait_key_press();
}

void set_packet_min_interval(){
    Serial.println("Type the minimum packet interval (ms) and press enter.");
    if (read_input_int(packet_min_interval, receivedChars, MAX_STR_SIZE)){
        Serial.print("Packet minimum interval set to '"); Serial.print(packet_min_interval);
    }
    else
        Serial.print("Value invalid. Aborted.");
    Serial.println("'. Press any key to continue.");
    wait_key_press();
}

void show_port(){
    Serial.print("Current port: "); Serial.println(port);
    Serial.println("Press any key to continue.");
    wait_key_press();
}

void show_wifi(){
    Serial.print("Current host ip address: "); Serial.println(host_ip);
    Serial.print("Current network SSID: "); Serial.println(ssid);
    Serial.print("Current network password: "); Serial.println(password);
    Serial.println("Press any key to continue.");
    wait_key_press();
}

void show_cal_values(){
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
    Serial.println("mag declination: ");
    Serial.print(mag_decli);
    Serial.println();
    Serial.println("filter: ");
    Serial.print(quat_filter);
    Serial.println();
    Serial.println("filter iterations: ");
    Serial.print(quat_iters);
    Serial.println();
    Serial.println("packet min interval: ");
    Serial.print(packet_min_interval);
    Serial.println();
    Serial.println("Press any key to continue.");
    wait_key_press();
}

void set_mag_dec(){
    Serial.println("Type the magnetic declination and press enter.");
    if (read_input_float(mag_decli, receivedChars, MAX_STR_SIZE)){
        mpu.setMagneticDeclination(mag_decli);
        Serial.print("Magnetic declination set to '"); Serial.print(mag_decli);
    }
    else
        Serial.print("Invalid magnetic declination. Aborted.");
    Serial.println("'. Press any key to continue.");
    wait_key_press();
}

void cali_mag_routine(){
    int i;
    enable_blink();
    mpu.calibrateMag();
    for(i = 0; i< 3; i++){
        mag_bias[i] = mpu.getMagBias(i);
        mag_scale[i] = mpu.getMagScale(i);
    }
    disable_blink();
}

void cali_mag(){
    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please wave device in a figure eight until done.");
    mpu.verbose(true);    
    Serial.println("Calibration start!");
    cali_mag_routine();
    mpu.verbose(false);
    Serial.print("Accel Gyro calibration completed");
    Serial.println(". Press any key to continue.");
    wait_key_press();
}

void cali_acc_gyro_routine(){
    int i;
    enable_blink();
    mpu.calibrateAccelGyro();
    for(i = 0; i< 3; i++){
        acc_bias[i] = mpu.getAccBias(i);
        gyro_bias[i] = mpu.getGyroBias(i);
    }
    disable_blink();
}

void cali_acc_gyro(){
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    Serial.println("Calibration start!");
    cali_acc_gyro_routine();
    mpu.verbose(false);
    Serial.print("Accel Gyro calibration completed");
    Serial.println(". Press any key to continue.");
    wait_key_press();
}

void set_quat_filter(){
    Serial.println("Type the quaternion filter and press enter.");
    Serial.println("0: NONE");
    Serial.println("1: MADGWICK");
    Serial.println("2: MAHONY");
    if (read_input_int(quat_filter, receivedChars, MAX_STR_SIZE)
    && ((quat_filter == 0) || (quat_filter == 1) || (quat_filter == 2))){
        mpu.selectFilter( (QuatFilterSel)quat_filter);
        Serial.print("Quaternion filter set to '"); Serial.print(quat_filter);
    }
    else
        Serial.print("Invalid filter number. Aborted.");
    Serial.println("'. Press any key to continue.");
    wait_key_press();
}

void set_quat_iter(){
    Serial.println("Type the quaternion filter no of iterations and press enter.");
    if (read_input_int(quat_iters, receivedChars, MAX_STR_SIZE)){
        mpu.setFilterIterations(quat_iters);
        Serial.print("Quaternion filter no of iterations set to '"); Serial.print(quat_iters);
    }
    else
        Serial.print("Invalid port. Aborted.");
    Serial.println("'. Press any key to continue.");
    wait_key_press();
}

// void set_accel_gyro_rate(){

// }

// void set_mag_rate(){

// }

void get_configs_from_memory(){
    int i;
    char param_name[32];

    strncpy(host_ip, preferences.getString("host_ip", "none").c_str(), MAX_STR_SIZE);
    strncpy(ssid, preferences.getString("ssid", "none").c_str(), MAX_STR_SIZE);
    strncpy(password, preferences.getString("password", "none").c_str(), MAX_STR_SIZE);
    strncpy(device_name, preferences.getString("device_name", "none").c_str(), MAX_STR_SIZE);
    port = preferences.getInt("port", 0);
    for(i = 0; i< 3; i++){
        sprintf(param_name, "acc_bias_%d", i);
        acc_bias[i] = preferences.getFloat(param_name, 0.);
        sprintf(param_name, "gyro_bias_%d", i);
        gyro_bias[i] = preferences.getFloat(param_name, 0.);
        sprintf(param_name, "mag_bias_%d", i);
        mag_bias[i] = preferences.getFloat(param_name, 0.);
        sprintf(param_name, "mag_scale_%d", i);
        mag_scale[i] = preferences.getFloat(param_name, 1.);
    }
    // Defaults to São Carlos, São Paulo, Brazil
    mag_decli = preferences.getFloat("mag_decli", -21.23);
    quat_filter = preferences.getInt("quat_filter", 1);
    quat_iters = preferences.getInt("quat_iters", 10);
    packet_min_interval = preferences.getInt("pckt_min_int", 25);
    // Setting loaded values to sensor
    mpu.setAccBias(acc_bias[0], acc_bias[1], acc_bias[2]);
    mpu.setGyroBias(gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    mpu.setMagBias(mag_bias[0], mag_bias[1], mag_bias[2]);
    mpu.setMagScale(mag_scale[0], mag_scale[1], mag_scale[2]);
    mpu.setMagneticDeclination(mag_decli);
    mpu.selectFilter( (QuatFilterSel)quat_filter);
    mpu.setFilterIterations(quat_iters);
}

void load_configs(){
    get_configs_from_memory();
    Serial.print("Loaded configuration from memory.");
    Serial.println(" Press any key to continue.");
    wait_key_press();
}

void set_configs_to_memory(){
    int i;
    char param_name[32];

    preferences.putString("host_ip", host_ip);
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.putString("device_name", device_name);
    preferences.putInt("port", port);

    for(i = 0; i< 3; i++){
        sprintf(param_name, "acc_bias_%d", i);
        preferences.putFloat(param_name, acc_bias[i]);
        sprintf(param_name, "gyro_bias_%d", i);
        preferences.putFloat(param_name, gyro_bias[i]);
        sprintf(param_name, "mag_bias_%d", i);
        preferences.putFloat(param_name, mag_bias[i]);
        sprintf(param_name, "mag_scale_%d", i);
        preferences.putFloat(param_name, mag_scale[i]);
    }
    preferences.putFloat("mag_decli", mag_decli);
    preferences.putInt("quat_filter", quat_filter);
    preferences.putInt("quat_iters", quat_iters);
    preferences.putInt("pckt_min_int", packet_min_interval);
}

void save_configs(){
    set_configs_to_memory();
    Serial.print("Configuration saved to non-volatile memory.");
    Serial.println(" Press any key to continue.");
    wait_key_press();
}

void exit_menu(){
    Serial.print("Exiting menu. Will now enter normal operating mode.");
    Serial.println(" Press any key to continue.");
    wait_key_press();
    accessing_menu = false;
}

void setup_menu_windows(){
    menu.config_menus[0].set_name("Main Menu");
    menu.config_menus[0].add_option("Wifi configuration", 1);
    menu.config_menus[0].add_option("Sensor data port", 2);
    menu.config_menus[0].add_option("Calibration", 3);
    menu.config_menus[0].add_option("Set device name", 0, set_device_name);
    menu.config_menus[0].add_option("Save configurations", 0, save_configs);
    menu.config_menus[0].add_option("Exit", 0, exit_menu);
    // this->config_menus[0].add_option("Exit");

    menu.config_menus[1].set_name("Wifi configuration");
    menu.config_menus[1].add_option("Set host ip address", 1, set_ip);
    menu.config_menus[1].add_option("Set SSID", 1, set_ssid);
    menu.config_menus[1].add_option("Set password", 1, set_password);
    menu.config_menus[1].add_option("Show current WiFi configuration", 1, show_wifi);
    menu.config_menus[1].add_option("Set minimum packet interval", 1, set_packet_min_interval);
    menu.config_menus[1].add_option("Back", 0);

    menu.config_menus[2].set_name("Sensor port");
    menu.config_menus[2].add_option("Set port", 2, set_port);
    menu.config_menus[2].add_option("Show current port", 2, show_port);
    menu.config_menus[2].add_option("Back", 0);

    menu.config_menus[3].set_name("Sensor configuration");
    menu.config_menus[3].add_option("Calibrate accel. and gyro.", 3, cali_acc_gyro);
    menu.config_menus[3].add_option("Calibrate magnetometer", 3, cali_mag);
    menu.config_menus[3].add_option("Set magnetic declination", 3, set_mag_dec);
    menu.config_menus[3].add_option("Set quaternion filter", 3, set_quat_filter);
    menu.config_menus[3].add_option("Set quaternion filter interations", 3, set_quat_iter);
    // menu.config_menus[3].add_option("Set accel./gyro. sample rate", 3, set_accel_gyro_rate);
    // menu.config_menus[3].add_option("Set mag. sample rate", 3, set_mag_rate);
    menu.config_menus[3].add_option("Show current sensor configuration", 3, show_cal_values);
    menu.config_menus[3].add_option("Back", 0);
}

bool has_usb_connection(unsigned long timeout){
    unsigned long elapsed, start;

    start = millis();
    elapsed = 0;
    while(!Serial.available() && (elapsed < timeout)){
        elapsed = millis() - start;
    }
    return Serial.available() != 0;
}

void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}

void print_quaternion() {
    Serial.print("X, Y, Z, W: ");
    Serial.print(mpu.getQuaternionX(), 2);
    Serial.print(", ");
    Serial.print(mpu.getQuaternionY(), 2);
    Serial.print(", ");
    Serial.print(mpu.getQuaternionZ(), 2);
    Serial.print(", ");
    Serial.println(mpu.getQuaternionW(), 2);
}

uint8_t* build_msg_header(char msg_type, uint8_t *buffer, msg_size_type msg_size){
    uint8_t *ptr;
    // msg_size_type message_size;

    // Set pointer to the beggining of the buffer (type of message)
    ptr = &buffer[0];
    // Set msg type
    ptr[0] = msg_type;
    // Increment to the second field (message size)
    ptr += sizeof(char);
    memcpy(ptr, &msg_size, sizeof(msg_size_type));    
    // Increment to the third field (message data)
    ptr += sizeof(msg_size_type);
    return ptr;
}

size_t build_msg(char msg_type, uint8_t *buffer, float *data, int data_length){
    uint8_t *ptr;
    msg_size_type msg_size;

    msg_size = data_length * sizeof(float);
    ptr = build_msg_header(msg_type, buffer, msg_size);
    for(int i = 0; i < data_length; i++){
        memcpy(&ptr[i*sizeof(float)], &data[i], sizeof(float));
    }
    return (ptr - buffer) + msg_size;
}

size_t build_msg(char msg_type, uint8_t *buffer, char *data){
    uint8_t *ptr;
    msg_size_type msg_size;

    msg_size = strlen(data);
    ptr = build_msg_header(msg_type, buffer, msg_size);
    memcpy(ptr, data, strlen(data));
    return (ptr - buffer) + msg_size;
}

void send_msg(WiFiUDP &wudp, const char* ip, uint16_t port, uint8_t *buffer, size_t buffer_size){
    wudp.beginPacket(ip, port);
    wudp.write(buffer, buffer_size);
    wudp.endPacket();
}

// void build_msg(char msg_type, uint8_t *buffer, msg_size_type buffer_size, char *data, msg_size_type_type data_size){

// }

void setup(){
    int opt;

    Serial.begin(115200);
    Wire.begin();
    preferences.begin("ESP32MCWS", false);
    mpu.setup(0x68);
    get_configs_from_memory();
    setup_menu_windows();
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.println("Press any key in the next 5s to access the configuration menu.");
    connected_to_usb = has_usb_connection(5000);
    accessing_menu = connected_to_usb;

    if(connected_to_usb){
        while(accessing_menu){
            opt = -1;
            menu.clear_screen();
            menu.show_current_menu();
            do{
                if (!read_input_int(opt, receivedChars, MAX_STR_SIZE)){
                    Serial.println("Error parsing option.");
                }
                else if (!menu.option_is_valid(opt)){
                    Serial.println("Option invalid.");
                }
            }while(!menu.option_is_valid(opt));

            menu.set_option(opt);
        }
    }
    preferences.end();

    Serial.print("Connecting to network '");Serial.print(ssid);Serial.println("'.");
    WiFi.begin(ssid, password);
    enable_blink();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    disable_blink();
    Serial.println(" Done!");
    Serial.print("WiFi connected with IP: ");
    Serial.println(WiFi.localIP());

    udp.begin(port);
    upd_discovery.begin(DISCOVERY_PORT);
}

void loop(){
    float q[4];
    char text_message[BUFFER_LEN], cmd;
    size_t msg_size;
    int packetSize;

    packetSize = upd_discovery.parsePacket();
    if(packetSize > 0){                
        memset(buffer, 0, BUFFER_LEN);
        int i = 0;
        while (upd_discovery.available() > 0){
            buffer[i] = upd_discovery.read();
            i++;
            if(i >= BUFFER_LEN){
                i = BUFFER_LEN - 1;
            }
        }         
        IPAddress remote_ip = upd_discovery.remoteIP();        
        Serial.print("Received discovery broadcast from ");
        Serial.print(remote_ip);
        Serial.print(", :");
        Serial.println((char *)buffer);
        // cmd = buffer[0];
        sprintf(text_message, "%s;%s;%d", device_name, host_ip, port);
        Serial.print("Sending sensor info: ");Serial.println(text_message);                
        msg_size = build_msg(IMU_BDCST_RESP, buffer, text_message);            
        send_msg(upd_discovery, remote_ip.toString().c_str(), DISCOVERY_PORT, buffer, msg_size);   
    }
    packetSize = udp.parsePacket();
    if(packetSize > 0){                
        memset(buffer, 0, BUFFER_LEN);
        int i = 0;
        while (udp.available() > 0){
            buffer[i] = udp.read();
            i++;
            if(i >= BUFFER_LEN){
                i = BUFFER_LEN - 1;
            }
        }                
        Serial.print("Received from host: ");
        Serial.println((char *)buffer);
        cmd = buffer[0];
        switch (cmd){
            case IMU_CAL_ACCGYRO:            
                Serial.println("Calibrate gyro!");
                cali_acc_gyro_routine();
                break;
            case IMU_CAL_MAG:            
                Serial.println("Calibrate mag!");
                cali_mag_routine();
                break;
            case IMU_CHANGEPORT:            
                Serial.println("Change port!");
                break;
            case IMU_CHANGEIP:            
                Serial.println("Change ip!");
                break;
            case IMU_SAVE_CFG:            
                Serial.println("Saving config to memory!");
                set_configs_to_memory();
                break;
            case IMU_LOAD_CFG:            
                Serial.println("Load config to memory!");
                get_configs_from_memory();
                break;
            case IMU_DISCOVER:
                sprintf(text_message, "%s;%s;%d", device_name, host_ip, port);
                Serial.print("Sending sensor info: ");Serial.println(text_message);                
                msg_size = build_msg(IMU_MSG_TYPE_INFO, buffer, text_message);            
                send_msg(udp, host_ip, port, buffer, msg_size);   
                break;
            default:
                break;
        }
    }
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + packet_min_interval) {
            prev_ms = millis();
            // q[0] = 11.1f;
            // q[1] = 22.2f;
            // q[2] = 33.3f;
            // q[3] = 44.4f;
            q[0] = mpu.getQuaternionX();
            q[1] = mpu.getQuaternionY();
            q[2] = mpu.getQuaternionZ();
            q[3] = mpu.getQuaternionW();
            
            msg_size = build_msg(IMU_MSG_TYPE_DATA, buffer, q, 4);            
            send_msg(udp, host_ip, port, buffer, msg_size);    
        }
    }
    // delay(5);
}