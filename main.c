#include <stdio.h>
#include <time.h>

int main() {

    printf("Hello, world!\n");

    clock_t begin = clock();

    float res = 0;
    int N = 10000000;
    int i;
    for (i = 0; i < N; i ++) {
        if (i % 2 == 0) {
            res += (float)i;
        } else {
            res -= (float)i;
        }
    }


    clock_t end = clock();


    printf("Result: %f\n", res);

    double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    time_spent *= 1000;



    printf("time_spent (ms): %f\n", time_spent);

    return 0;
}

