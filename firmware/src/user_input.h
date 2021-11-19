#ifndef _USER_INPUT_H_
#define _USER_INPUT_H_

#include <Arduino.h>
#include <errno.h>

void clear_input_buffer();

void wait_key_press();

void clear_received_chars(char* buffer, int buffer_size);

bool read_input_text(char* buffer, int buffer_size);

bool read_input_int(int &output, char* buffer, int buffer_size);

bool read_input_float(float &output, char* buffer, int buffer_size);

#endif