#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <pthread.h>
#include <unistd.h>

void* function_A(void* num) {
    int i = 0;

    for (i = 0; i < 50; i++) {
        printf("Thread A i = [%3d]\n", i);        
        usleep(100);
    }

    pthread_exit(NULL);
}

void* function_B(void* num) {
    int i = 0;

    while (true) {
        printf("Thread B i = [%3d]\n", i);
        i++;
        i %= 100;
        sleep(3);
    }
}

int main() {
    pthread_t pthread_A, pthread_B;
    int cnt = 0;

    printf("Create Thread A\n");
    pthread_create(&pthread_A, NULL, function_A, NULL);

    printf("Create Thread B\n");
    pthread_create(&pthread_B, NULL, function_B, NULL);

    pthread_join(pthread_A, NULL);
    // pthread_join(pthread_B, NULL);

    while (true) {
        printf("Thread test : %3d\n", cnt);
        cnt++;
        cnt %= 100;
        sleep(1);
    }

    return 0;
}
