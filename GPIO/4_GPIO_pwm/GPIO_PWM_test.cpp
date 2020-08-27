#include <cstdio>
#include <cstring>

#include <softPwm.h>
#include <termio.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#define ENA 1 // Physical 12
#define IN1 4 // Physical 16
#define IN2 5 // Physical 18

#define ENB 0 // Physical 11
#define IN3 2 // Physical 13
#define IN4 3 // Physical 15

#define SERIAL_DEV "/dev/ttyACM0"
#define BAUD_RATE 115200

#define MAX_PWM_DUTY 100

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
    int pwm_a = 0;

    if (wiringPiSetup() == -1) {
        printf("WiringPi setup error!\n");
        return -1;
    }

    if ((fd = serialOpen(SERIAL_DEV, BAUD_RATE)) < 0) {
        printf("UART open error!\n");
        return -1;
    }

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    softPwmCreate(ENA, 1, MAX_PWM_DUTY);
    softPwmCreate(ENB, 1, MAX_PWM_DUTY);

    softPwmWrite(ENA, pwm_a);
    softPwmWrite(ENB, 90);

    while (true) {
        softPwmWrite(ENA, pwm_a);
        pwm_a += 1;
        if (pwm_a > 100) {
            pwm_a = 0;
        }

        delay(500);
    }

    return 0;
}
