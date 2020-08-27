#include <cstdio>
#include <cstring>
#include <termio.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#define GPIO0 0 // Physical 11
#define GPIO3 3 // Physical 15

#define SERIAL_DEV "/dev/ttyACM0"
#define BAUD_RATE 115200

// `getch()` implementation for Raspberry Pi
int getch(void) {
    int ch;
    struct termios buf;
    struct termios save;

    tcgetattr(0, &save);
    buf = save;
    buf.c_lflag &= ~(ICANON|ECHO);
    buf.c_cc[VMIN] = 1;
    buf.c_cc[VTIME] = 0;
    tcsetattr(0, TCSAFLUSH, &buf);
    ch = getchar();
    tcsetattr(0, TCSAFLUSH, &save);
    return ch;
}

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

    while (true) {
        test = getch(); // Get character input from console

        // Write character to serial (read by Arduino)
        if (test == 'A') {
            printf("Input A : %c\n", test);
            serialPutchar(fd, test);
        }
        else {
            printf("Input not A : %c\n", test);
            serialPutchar(fd, test);
        }

        delay(50);

        // Get output from serial (writtern by Arduino)
        while (serialDataAvail(fd)) {
            receive_char = serialGetchar(fd);
            printf("Received char : %d %c\n", receive_char, receive_char);
        }

        delay(2);
    }

    return 0;
}
