#include "mbed.h"
//TELEMETRY
#include "telemetry.h"
#include "MODSERIAL.h"

//TELEMETRY
MODSERIAL telemetry_serial(PTC15, PTC14);

telemetry::MbedHal telemetry_hal(telemetry_serial);
telemetry::Telemetry telemetry_obj(telemetry_hal);

telemetry::Numeric<float> tele_servo_pwm(telemetry_obj, "servo", "Servo PWM", "PWM (us)", 0);
telemetry::Numeric<uint32_t> tele_time_ms(telemetry_obj, "time", "Time", "ms", 0);
telemetry::Numeric<float> tele_hall_revs(telemetry_obj, "revs", "Hall revs", "count", 0);
telemetry::Numeric<float> tele_button_push(telemetry_obj, "pushed?", "Button push", "count", 0);
telemetry::Numeric<float> tele_velocity_velMA(telemetry_obj, "VELOCITY", "m/s", "count", 0);

// Line Camera Pins
//CAM1
AnalogIn A0_CAM1(PTC10);
DigitalOut SI_CAM1(PTC16);
DigitalOut CLK_CAM1(PTC17);

//CAM2
AnalogIn A0_CAM2(PTC11);
DigitalOut SI_CAM2(PTC2);
DigitalOut CLK_CAM2(PTC3);

// Interrupts Pins
//Hall Sensors
InterruptIn hallA(PTC12);
InterruptIn hallB(PTA2);

// Tickers
Ticker timestep;

// Digital pin to read from Hall Sensor
PwmOut motor(PTD1);
DigitalOut brakeEN(PTB9);

// Servo Pin
PwmOut servo(PTA1);

//Push Buttons
DigitalIn SELECT(PTC4);
DigitalIn UP(PTD0);
DigitalIn DOWN(PTD3);

volatile int revs = 0;
volatile int revsA = 0;
volatile int revsB = 0;
volatile float pw = 0.0;

//Initialize needed variables
const float refVel = 0.7; // Corresponds to m/s
const float dt = 0.010; //PI interrupt in SECONDS 50 Hz
const float Kp = 0.20; //Proportional Gain
const float Ki = 0.01; //Integral Gain
const float maxPWM = 0.30;
const float minPWM = 0.0;

//transform to m/s with gear ratio (9:1 gear ratio) (r = 32.5 mm)
const float r = 0.0325; // meters
const float gear = 9.0; // 9:1 gear down ratio
const float velTransform = (2.0*3.141592*r) / (gear*4*dt);

//Volatile variables to be used in interrupts
volatile float vel = 0.0;
volatile float velMA = 0.0;
volatile float velSum = 0.0;
volatile int resolution = 10;
volatile int count = 1;
volatile int ramp = 1;

//Volatile CONTROL-related variables to be used in interrupts
volatile float dutyInput = 0.0;
volatile float intgrlTerm = 0.0;
volatile float err = 0.0;

//VELOCITY!
//ISR for HALL sensor reading
void rpmCounterA() {
    revsA++;
}

void rpmCounterB() {
    revsB++;
}

void velocityPI() {
	//Calculate velocity (revolutions/fixed time unit)
	revs = revsA + revsB;
	vel = (float) (revs) * velTransform;

	//Code begins when switch is turned on
	if (vel > 0.0){
		//Calculate moving average
		if (count < resolution) {
			velSum = velSum + vel;
			velMA = vel;
			//printf("%i\r\n",count);
			count++;
			//printf("%f\r\n",velMA);
		} else if (count == resolution) {
			velMA = (velSum + vel)/resolution;
			//printf("%i\r\n",count);
			count++;
			//printf("%f\r\n",velMA);
		} else {
			velMA = velMA + (vel/resolution) - (velMA/resolution);
			//printf("%f\r\n",velMA);
		}

		tele_velocity_velMA = velMA;

		//Calculate ERROR
		err = refVel - velMA;

		//Accumulate error for integral term
		intgrlTerm = intgrlTerm + (Ki*err);

		//Anti-Windup check!
		if (intgrlTerm > maxPWM){
			intgrlTerm = maxPWM;
		} else if (intgrlTerm < minPWM){
			intgrlTerm = minPWM;
		}

		//Calculate INPUT value
		dutyInput = (Kp*err) + intgrlTerm;
		//dutyInput = (Kp*err);

		//Clamp REFERENCE value if needed
		if (dutyInput > maxPWM) {
			dutyInput = maxPWM;
		} else if (dutyInput < minPWM) {
			dutyInput = minPWM;
		}

		if (ramp<25){
			dutyInput = 0.07f;
			ramp++;
		}

		//ACTUATION!
		//Send INPUT value to motor
		motor.write(dutyInput);
		revsA = 0;
		revsB = 0;
	}
}

void control() {
	tele_hall_revs = revsB/1.0;
	revsB = 0;
	tele_servo_pwm = pw*1000000;

	if(DOWN) {
		tele_button_push = 1.0;
	} else {
		tele_button_push = 0.0;
	}

	velocityPI();
}

//TELEMETRY
void print_serial() {
	tele_time_ms = telemetry_hal.get_time_ms();
    telemetry_obj.do_io();
}

//Function to initialize system
void setup() {

    //TELEMETRY
    telemetry_serial.baud(38400);

    tele_servo_pwm.set_limits(0.0, 2000.0); // lower bound, upper bound
    telemetry_obj.transmit_header();

    //set pwm periods
    motor.period_us(50); // 20 kHz frequency
    servo.period_ms(10); //Set servo PWM period = 3ms

    //set regular interrupts
    hallA.rise(&rpmCounterA);
    hallA.fall(&rpmCounterA);

    hallB.rise(&rpmCounterB);
    hallB.fall(&rpmCounterB);

    //set Ticker interrupts
    timestep.attach(&control,dt);

    brakeEN.write(1);

    //set warm-up speed and command motor
    pw = 0.0015f;
    servo.pulsewidth(pw);

    //set warm-up speed and command motor
    dutyInput = 0.10f;
    motor.write(dutyInput);
}

//Main CODE
int main() {
    //run setup function
    setup();

    //run while loop
    while (1) {
    	print_serial();
    }
}
