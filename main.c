#include <stdio.h>
#include<graphics.h>
#include<conio.h>

int main() {
    printf("Hello, world!\n");

    int gd = DETECT, gm;

    initgraph(&gd, &gm, "C:\\TC\\BGI");

    putpixel(25, 25, RED);

    getch();
    closegraph();

    return 0;
}

