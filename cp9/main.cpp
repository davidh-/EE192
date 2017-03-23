#include "mbed.h"
#include "telemetry.h"
#include "MODSERIAL.h"
// Telemetry:
MODSERIAL telemetry_serial(PTC15, PTC14); 

telemetry::MbedHal telemetry_hal(telemetry_serial);
telemetry::Telemetry telemetry_obj(telemetry_hal);

telemetry::Numeric<uint32_t> tele_time_ms(telemetry_obj, "time", "Time", "ms", 0);
telemetry::NumericArray<uint16_t, 128> tele_linescan(telemetry_obj, "linescan", "Linescan", "ADC", 0);
telemetry::Numeric<float> tele_servo_pwm(telemetry_obj, "servo", "Servo PWM", "PWM (us)", 0);
telemetry::Numeric<float> tele_motor_pwm(telemetry_obj, "motor", "Motor PWM", "%DC", 0);


// Line Camera Pins
DigitalOut TAOS_SI(D0);
DigitalOut TAOS_CLK(D1);
AnalogIn A_IN(A5);

// Interrupts
//InterruptIn hall(D2);

// Tickers
Ticker camTick;
Ticker servoTick;
Ticker telemTick;

//Digital pin to read from Hall Sensor
PwmOut motor(D13);
DigitalOut brakeEN(D12);

// Servo Pin
PwmOut servo(D9);

// Telemetry Pins
//Serial bt(PTC15, PTC14);
//Serial pc(USBTX, USBRX);

// Servo endpoints
float l_end = 1020;
//1440 center
float r_end = 1750;

//// Servo endpoints
//float l_end = 1120;
////1440 center
//float r_end = 1850;

// Servo steps
float divi = 128.0;
float steps = r_end - l_end;
float step = steps/divi;

// Servo position
float pw = 0.0;
float max = 0.0;
int max_i = 0;
float pos = 0.0;

// Motor PWM = 0.07 = 7%
float motor_pwm = 0.07f;

// Line cam vars
int PIXELS = 128;
unsigned short data[128];
float t_SI = 0.00000002; //20ns
float t_hold = 0.000000005; //5ns
float t_w = 0.00000005; //50ns
float t_qt = 0.00003375; //33.75 us



void print_serial() {
    telemetry_obj.do_io();
//    pc.printf("\n");
//    bt.printf("\n");
//    for(int i = 0; i < PIXELS; i++) {
//        int v = (data[i]/(65535*.5));
//        pc.printf("%i", v);
//        bt.printf("%i", v);
//    }
//    pc.printf("\nmax: %i, pos: %f, l_end: %f, r_end: %f\r\n", max_i, pos, l_end, r_end);
//    bt.printf("\nmax: %i, pos: %f, l_end: %f, r_end: %f\r\n", max_i, pos, l_end, r_end);

}
void updateServoFromCam() {
    TAOS_SI = 0;
    TAOS_CLK = 0;

    TAOS_SI = 1;
    wait(t_SI);
    // first pixel
    TAOS_CLK = 1;
    wait(t_hold);
    TAOS_SI = 0;
    wait(t_w - t_hold);
    data[0] = A_IN.read_u16();
    TAOS_CLK = 0;
    wait(t_w);
    for(int i = 1; i < PIXELS; i++) {
        TAOS_CLK = 1;
        wait(t_w);
        data[i] = A_IN.read_u16();
        tele_linescan[i] = data[i];
        if (data[i] > max) { //calc max
            max = data[i];
            max_i = i;
        }
        TAOS_CLK = 0;
        wait(t_w);
    }
    max = 0;
    wait(t_qt);
    pos = r_end - (step * max_i);
    pw = pos * (0.000001);
}
void updateMotorPWM() {
    tele_motor_pwm = motor_pwm*100;    
}

void writePWMServo() {
    if (max != 0) {
        tele_servo_pwm = pw*1000000;
        servo.pulsewidth(pw);
    }
}


void bluetoothsetup() {
    telemetry_serial.baud(9600);
    wait(1.1);
    telemetry_serial.printf("$$$");
    wait(1.1);
    telemetry_serial.printf("SU,38");
    wait(1.1);
    telemetry_serial.printf("R,0");
    wait(1.1);
}
//OPEN LOOP
int main() {
    telemetry_serial.baud(38400);
    
    tele_motor_pwm.set_limits(0.0, 20.0); // lower bound, upper bound
    tele_servo_pwm.set_limits(0.0, 2000.0); // lower bound, upper bound
    telemetry_obj.transmit_header();
    
    servo.period_ms(3); //Set servo PWM period = 3ms

    //Set BrakeEN to HIGH
    brakeEN.write(1);

    //set pwm
    motor.period_us(50); // 20 kHz frequency
    motor.write(motor_pwm); //12% duty cycle
    
    servoTick.attach(&writePWMServo, 0.003); //333Hz = 3ms = 0.003s

    while (1) {
        updateServoFromCam();
        updateMotorPWM();
        tele_time_ms = telemetry_hal.get_time_ms();
//        telemTick.attach(&print_serial, 0.5);
        telemetry_obj.do_io();
    }
}

