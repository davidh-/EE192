#include "mbed.h"
#include "TSL1401.h"

Serial pc(USBTX, USBRX);
PwmOut servo(D3);

float steps = 1000.0;
float divi = 128.0;
//20% - left = 1000
//28% - center 
//36% - right = 1800
float step = steps/divi;


int main(){
    servo.period_ms(5);
    while (true) {
        
        TSL1401 cam(D0, D1, A0);
        int *data;
        data = cam.Capture(0, 127);
        int left = 0;
        bool left_set = false;
        int right = 0;
        
        for (int i = 0; i < 125; i++) {
            int v = (data[i]/65535);
            pc.printf("%i", v);
            if (v == 0 and !left_set) {
                left = i;
                left_set = true;
            }
            if (v == 0) {
                right = i;
            }
        }
        int max = (left + right)/2;
        pc.printf("max: %i, ", max);
        float pos = 1000 + (step * max);
        float pw = pos * (0.000001);
        if (max != 0) {
            servo.pulsewidth(pw);
        }
        pc.printf("pos: %f\n", pos);
    }
}