#include <cstdio>
#include <cstring>
#include <wiringPi.h>
#include <wiringSerial.h>

#define GPIO0 0 // Physical 11
#define GPIO3 3 // Physical 15

#define SERIAL_DEV "/dev/ttyS0"
#define BAUD_RATE 115200

int main(void) {
    int fd;
    unsigned char test, receive_char;

    if (wiringPiSetup() == -1) {
        printf("WiringPi setup error!\n");
        return -1;
    }

    if ((fd = serialOpen(SERIAL_DEV, BAUD_RATE)) < 0) {
        printf("UART open error!\n");
        return -1;
    }

    test = 'A';
    while (true) {
        // Write to serial
        serialPutchar(fd, test);
        delay(1);

        // Read from serial
        if (serialDataAvail(fd)) {
            receive_char = serialGetchar(fd);
            printf("Received char : %d %c\n", receive_char, receive_char);
        }
    }

    return 0;
}
