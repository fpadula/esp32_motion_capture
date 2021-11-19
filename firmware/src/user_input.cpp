#include "user_input.h"

void clear_input_buffer(){
    while (Serial.available()) {
        Serial.read();
    }
}

void clear_received_chars(char* buffer, int buffer_size){
    memset(buffer, 0, buffer_size);
    buffer[0] = '\0';
}

void wait_key_press(){
    clear_input_buffer();            
    while (!Serial.available()) {}            
    clear_input_buffer();            
}

bool read_input_int(int &output, char* buffer, int buffer_size){
    bool success;
    int ret_val;
    char *endptr = NULL;  

    success = read_input_text(buffer, buffer_size);
    if(success){
        errno = 0;
        ret_val = (int) strtol(buffer, &endptr, 10);
        if (errno == 0 && buffer && !*endptr){
            output = ret_val;
        }
        else{
            success = false;
        }
    }
    return success;
}

bool read_input_float(float &output, char* buffer, int buffer_size){
    bool success;
    float ret_val;
    char *endptr = NULL;  
    char *token;  
    float denom;

    success = read_input_text(buffer, buffer_size);
    if(success){
        // Breaking float into integer and fractional parts:
        token = strtok(buffer, ".");
        errno = 0;        
        ret_val = (float) strtol(token, &endptr, 10);
        if (errno == 0 && token && !*endptr){
            output = ret_val;
            // Adding fractional part, if it exists
            token = strtok(NULL, ".");
            if(token != NULL){
                errno = 0;
                ret_val = (float) strtol(token, &endptr, 10);
                if (errno == 0 && token && !*endptr){
                    denom = pow(10, strlen(token));
                    ret_val = ret_val/denom;
                    if (output > 0)
                        output += ret_val;
                    else
                        output -= ret_val;
                    success = true;
                }
                else{
                    success = false;
                }
            }
            else{
                success = true;
            }
        }
        else{
            success = false;
        }
    }
    return success;
}

bool read_input_text(char* buffer, int buffer_size){
    bool waiting_input = true;
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    int string_size;

    clear_input_buffer();
    clear_received_chars(buffer, buffer_size);            
    while (waiting_input){
        while (Serial.available()) {
            rc = Serial.read();
            // Serial.println((int) rc);
            if (rc != endMarker) {
                if(rc != '\r'){
                    buffer[ndx] = rc;
                    ndx++;
                    if (ndx >= buffer_size) {
                        ndx = buffer_size - 1;
                    }
                }
            }
            else {
                buffer[ndx] = '\0'; // terminate the string
                ndx = 0;
                waiting_input = false;                
                clear_input_buffer();
            }
        }
    }
    string_size = strlen(buffer);
    if(string_size != 0){
        return true;
    }    
    return false;
}