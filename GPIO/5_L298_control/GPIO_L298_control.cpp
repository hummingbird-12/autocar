#include <csignal>
#include <cstdio>
#include <cstdlib>
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

int getch(void);
int motor_control_setup(void);
void motor_control_l(const int pwm);
void motor_control_r(const int pwm);
void sig_handler(const int sig);

int main(void) {
    unsigned char test;
    int pwm_l = 0;
    int pwm_r = 0;

    if (wiringPiSetup() == -1) {
        printf("WiringPi setup error!\n");
        return -1;
    }

    if (motor_control_setup() != 0) {
        printf("GPIO setup error!\n");
        return -1;
    }

    signal(SIGINT, sig_handler);

    while (true) {
        test = getch();
        printf("Input: %c\n", test);

        switch(test) {
            case 'w': // Increase forwards
                motor_control_l(++pwm_l);
                motor_control_r(++pwm_r);
                break;
            case 's': // Stop
                motor_control_l(0);
                motor_control_r(0);
                break;
            case 'x': // Decrease backwards
                motor_control_l(-pwm_l);
                motor_control_r(-pwm_r);
                pwm_l--;
                pwm_r--;
                break;
        }        
    }

    return 0;
}

void sig_handler(int sig) {
    printf("\n\n\nProgram and motor stop!\n\n");
    motor_control_r(0);
    motor_control_l(0);
    exit(0);
}

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

int motor_control_setup(void) {
    if (wiringPiSetup() == -1) {
        printf("WiringPi setup error!\n");
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

    softPwmWrite(ENA, 0);
    softPwmWrite(ENB, 0);

    return 0;
}

void motor_control_l(const int pwm) {
    if (pwm > 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        softPwmWrite(ENB, pwm);
    }
    else if (pwm == 0) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        softPwmWrite(ENB, 0);
    }
    else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        softPwmWrite(ENB, -pwm);
    }
}

void motor_control_r(const int pwm) {
    if (pwm > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        softPwmWrite(ENA, pwm);
    }
    else if (pwm == 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        softPwmWrite(ENA, 0);
    }
    else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        softPwmWrite(ENA, -pwm);
    }
}
