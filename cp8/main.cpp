#include "mbed.h"
#include "TSL1401.h"
#include "esc.h"
#include "PID.h"

Serial bt(PTC15, PTC14);
Serial pc(USBTX, USBRX);
PwmOut servo(D5);
TSL1401 cam(D0, D1, A5);
ESC esc(D3, 5);

Ticker motor_tick;

float steps = 1000.0;
float divi = 128.0;
//20% - left = 1000
//28% - center 
//36% - right = 1800
float step = steps/divi;
float throttle = 0.0;

void updateServoFromCam() {
    int *data;
    data = cam.Capture(0, 127);
    int left = 0;
    bool left_set = false;
    int right = 0;
    for (int i = 0; i < 125; i++) {
        int v = (data[i]/65535);
        bt.printf("%i", v);
        if (v == 1 and !left_set) {
            left = i;
            left_set = true;
        }
        if (v == 1) {
            right = i;
        }
    }
    int max = (left + right)/2;
    bt.printf(" max: %i, ", max);
    float pos = 2000 - (step * max);
    float pw = pos * (0.000001);
    if (max != 0) {
        servo.pulsewidth(pw);
    }
    bt.printf("pos: %f, ", pos);
}

void runMotor() {
        char c = ' ';
        if (pc.readable()) {
            c = pc.getc();
        }
        if (bt.readable()) {
            c = bt.getc();
        }
        if (c == 'u') {
            if (throttle < 1) {
                throttle+= 0.01;
            }
            if (esc.setThrottle(throttle)) {
                printf("%f\r\n", throttle);
            }
        }
        else if (c == 'd') {
            if (throttle > 0) {
                throttle-= 0.01;
            }
            if (esc.setThrottle(throttle)) {
                printf("%f\r\n", throttle);
            }
        }
        else if (c == 'r') {
            throttle = 0;
            if (esc.setThrottle(throttle)) {
                printf("%f\r\n", throttle);                
            }
        }    
        else if (c == 'm') {
            throttle = 1;
            if (esc.setThrottle(throttle)) {
                printf("%f\r\n", throttle);                
            }
        } 
        pc.printf("throttle: %f\r\n", throttle);
        c = ' ';
}

void setup() {
    motor_tick.attach(&esc, &ESC::pulse, 0.005);
}

int main(){
    setup();
    while (true) {
        updateServoFromCam();
        runMotor();
    }
}