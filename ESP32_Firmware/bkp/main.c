#include <stdlib.h>
#include <stdio.h>

int main(){
    unsigned char buffer[4];
    int number = -4000, reconstructed_number;

    buffer[3] = (number >> 24);
    buffer[2] = (number >> 16);
    buffer[1] = (number >> 8);
    buffer[0] = number;

    reconstructed_number = (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];
    printf("Original %d, reconstructed %d", number, reconstructed_number);

    return 0;
}