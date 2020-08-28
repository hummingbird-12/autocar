#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <pthread.h>

#include <iostream>
#include <unistd.h>

#include <softPwm.h>
#include <wiringPi.h>

/***** GPIO Motor Control *****/
#define ENA 1 // Physical 12
#define IN1 4 // Physical 16
#define IN2 5 // Physical 18

#define ENB 0 // Physical 11
#define IN3 2 // Physical 13
#define IN4 3 // Physical 15

#define MAX_PWM_DUTY 100

/***** Ultrasonic Sensor *****/
#define TRIG 21 // Physical 29
#define _ECHO 22 // Physical 31
#define OBSTACLE_DISTANCE 10

float ultrasonic_sensor_data;
bool flag_obstacle_detection;
int pwm_l, pwm_r;
bool run;

void sig_handler(int sig);
int GPIO_control_setup(void);
float ultrasonic_sensor(void);
void motor_control_l(const int pwm);
void motor_control_r(const int pwm);
void* ultrasonic_sensor_thread(void* num);
void* motor_control_thread(void* num);

int main() {
    pthread_t pthread_A, pthread_B;
    int cnt = 0;
    
    if (GPIO_control_setup() != 0) {
        printf("GPIO setup error!\n");
        return -1;
    }

    signal(SIGINT, sig_handler);

    run = true;

    printf("Create Thread A\n");
    pthread_create(&pthread_A, NULL, ultrasonic_sensor_thread, NULL);

    printf("Create Thread B\n");
    pthread_create(&pthread_B, NULL, motor_control_thread, NULL);

    // pthread_join(pthread_A, NULL);
    // pthread_join(pthread_B, NULL);

    while (true) {
        printf("Ultrasonic sensor : %6.3f [cm]\n", ultrasonic_sensor_data);
        printf("Thread test : %3d\n", cnt);
        cnt++;
        cnt %= 100;
        pwm_l = pwm_r = cnt;
        delay(500);
    }

    return 0;
}

void sig_handler(int sig) {
    run = false;
    delay(500);

    printf("\n\n\nProgram and motor stop!\n\n");
    motor_control_l(0);
    motor_control_r(0);
    exit(0);
}

int GPIO_control_setup(void) {
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

    pinMode(TRIG, OUTPUT);
    pinMode(_ECHO, INPUT);

    softPwmCreate(ENA, 1, MAX_PWM_DUTY);
    softPwmCreate(ENB, 1, MAX_PWM_DUTY);

    softPwmWrite(ENA, 0);
    softPwmWrite(ENB, 0);

    pwm_l = pwm_r = 0;

    return 0;
}

float ultrasonic_sensor(void) {
    long temp_time;
    long start_time = 0, end_time = 0;
    int duration;
    float distance;

    digitalWrite(TRIG, LOW);
    delayMicroseconds(5);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    delayMicroseconds(200); // Wait for burst signal (40kHz * 8 = 8 * 25us = 200)

    temp_time = micros();
    // Wait until ECHO pin is HIGH
    while (digitalRead(_ECHO) == LOW) {
        if (micros() - temp_time > 3000) {
            return -1;
        }
    }
    start_time = micros();

    // Wait until ECHO pin is LOW
    while (digitalRead(_ECHO) == HIGH) {
        if (micros() - start_time > 20000) {
            return -1;
        }
    }
    end_time = micros();
    duration = end_time - start_time;
    distance = duration / 58.0;

    return distance;
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

void* ultrasonic_sensor_thread(void* num) {
    while (run) {
        ultrasonic_sensor_data = ultrasonic_sensor();
        if (ultrasonic_sensor_data <= OBSTACLE_DISTANCE) {
            printf("Obstacle detected!\n\n");
            flag_obstacle_detection = true;
        }
        delay(500);
    }
    return NULL;
}

void* motor_control_thread(void* num) {
    while (run) {
        motor_control_l(pwm_l);
        motor_control_r(pwm_r);
    }
    return NULL;
}
