#include "mbed.h"
#include "TSL1401.h"


#define TAOS_SI_HIGH    TAOS_SI = 1
#define TAOS_SI_LOW     TAOS_SI = 0
#define TAOS_CLK_HIGH   TAOS_CLK = 1
#define TAOS_CLK_LOW    TAOS_CLK = 0

DigitalOut TAOS_SI(D0);
DigitalOut TAOS_CLK(D1);

AnalogIn A_IN(A5);


//Interrupts
//InterruptIn hall(D2);
Ticker servo_tick;
//Ticker camera_tick;
//Timer t;

//Digital pin to read from Hall Sensor
PwmOut motor(D13);
DigitalOut brakeEN(D12);

PwmOut servo(D9);
TSL1401 cam(D0, D1, A5);
Serial bt(PTC15, PTC14);
Serial pc(USBTX, USBRX);

//DAVID
float l_end = 550;
float r_end = 1450;

float steps = r_end - l_end;
float divi = 128.0;
//20% - left = 1000
//28% - center
//36% - right = 1800
float step = steps/divi;
float throttle = 0.0;
float pw = 0.0;
float max = 0.0;
int max_i = 0;




void updateServoFromCam() {
    int PIXELS = 128;
    float t_SI = 0.00000002; //20ns
    float t_hold = 0.000000005; //5ns
    float t_w = 0.00000005; //50ns
    float t_qt = 0.00002; //20us

    TAOS_SI = 0;
    TAOS_CLK = 0;
    
    float data[PIXELS];
    int cycles = 0;
    int total_cycles = 5;

    TAOS_SI = 1;
    wait(t_SI);
    // first pixel
    TAOS_CLK = 1;
    wait(t_hold);
    TAOS_SI = 0;
    wait(t_w - t_hold);
    data[0] = A_IN.read();
    TAOS_CLK = 0;
    wait(t_w);
    for(int i = 1; i < PIXELS; i++) {
        TAOS_CLK = 1;
        wait(t_w);
        data[i] = A_IN.read();
        
        //output line data to serial:
        int v_0_1 = data[i];
            pc.printf("%i", v_0_1);
            bt.printf("%i", v_0_1);
        //calc max:
        if (data[i] > max) {
            max = data[i];
            max_i = i;
        }
        TAOS_CLK = 0;
        wait(t_w);
    }
    max = 0;
    wait(t_qt);
//        cycles += 1;
//        break;

    float pos = r_end - (step * max_i);
    pw = pos * (0.000001);
    pc.printf(" max: %i, pos: %f, l_end: %f, r_end: %f, step: %f\r\n", max_i, pos, l_end, r_end, step);
    bt.printf(" max: %, pos: %f, l_end: %f, r_end: %f, step: %f\r\n", max_i, pos, l_end, r_end, step);
}

void writePWMServo() {
    if (max != 0) {
        servo.pulsewidth(pw);
    }
}

////OPEN LOOP
int main() {

    //Set servo PWM period
    servo.period_ms(5);
    servo_tick.attach(&writePWMServo, 0.010
    );
    //camera_tick.attach(&updateServoFromCam, 0.005);

    //Set BrakeEN to HIGH
    brakeEN.write(1);

    //set pwm
    motor.period_us(50);      // 20 kHz frequency
    motor.write(0.07f);

    while(1) {
        updateServoFromCam();
    }
}

//void updateServoFromCam() {
//    int *data;
//    data = cam.Capture(0, 127);
//    int left = 0;
//    bool left_set = false;
//    int right = 0;
//    for (int i = 0; i < 125; i++) {
//        int v = (data[i]/65535);
//        printf("%i\n\r", v);
//        if (v == 1 and !left_set) {
//            left = i;
//            left_set = true;
//        }
//        if (v == 1) {
//            right = i;
//        }
//    }
//    int max = (left + right)/2;
//    //printf(" max: %i, ", max);
//    float pos = 2000 - (step * max);
//    float pw = pos * (0.000001);
//    printf("pos: %f, ", pos);
//}
//
//void writePWMServo() {
//    if (max != 0) {
//        servo.pulsewidth(pw);
//    }
//}
