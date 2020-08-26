#include <cstdio>
#include <wiringPi.h>

#define GPIO0 0 // Physical 11
#define GPIO3 3 // Physical 15

int main(void) {
    if (wiringPiSetup() == -1) {
        printf("WiringPi setup error!\n");
        return -1;
    }

    pinMode(GPIO0, INPUT);
    pinMode(GPIO3, OUTPUT);

    printf("GPIO PIN3 : LOW\n");
    digitalWrite(GPIO3, LOW);
    delay(1000);

    while (true) {
        printf("GPIO0 Pin Read : %d\n", digitalRead(GPIO0));
        delay(1000);
    }

    return 0;
}
