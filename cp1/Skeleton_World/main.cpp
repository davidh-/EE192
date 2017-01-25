#include "mbed.h"
#include "rtos.h"
#include "MODSERIAL.h"

DigitalOut myled(LED_GREEN);
Serial pc(USBTX, USBRX);

int main()
{
    int i = 0;
    pc.printf("Hello World!\n");

    while (true) {
        wait(0.5f); // wait a small period of time
        pc.printf("%d \n\r", i); // print the value of variable i
        i++; // increment the variable
        myled = !myled; // toggle a led
    }
}
