#include "mbed.h"
//TELEMETRY
#include "telemetry.h"
#include "MODSERIAL.h"

//TELEMETRY
MODSERIAL telemetry_serial(PTC15, PTC14);

telemetry::MbedHal telemetry_hal(telemetry_serial);
telemetry::Telemetry telemetry_obj(telemetry_hal);

telemetry::Numeric<float> tele_servo_pwm(telemetry_obj, "servo", "Servo PWM", "PWM (us)", 0);
telemetry::NumericArray<uint16_t, 128> tele_linescan(telemetry_obj, "linescan", "Linescan", "ADC", 0);
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

//Initialize needed variables
volatile float refVel = 1.20; // Corresponds to m/s
const float dt = 0.010; //PI interrupt in SECONDS 50 Hz
const float Kp = 0.18; //Proportional Gain
const float Ki = 0.01; //Integral Gain
const float maxPWM = 0.20;
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

float l_end = 1110.0;
//1520 center
float r_end = 1970.0;

// Servo steps
float divi = 128.0;
float steps = r_end - l_end;
//float step = steps/divi;
float step = steps/114;

// Servo position
volatile float pw = 0.0;
volatile float max = 0.0;
volatile int max_i = 0;
volatile float pos = 0.0;
volatile int old_max_i = 0;

// Line cam vars
int PIXELS = 128;
unsigned short data[128];

const float thresh = 0.75;

volatile float exp_time = 0.001; //1ms

//VELOCITY!
//ISR for HALL sensor reading
void rpmCounterA() {
    revsA++;
}

void rpmCounterB() {
    revsB++;
}

void steeringPD() {

	max = 0;

	//Fake ANALOG READ
	SI_CAM1 = 1;
	CLK_CAM1 = 1;
	SI_CAM1 = 0;
	CLK_CAM1 = 0;
	for(int i = 0; i < PIXELS; i++) {
		CLK_CAM1 = 1;
		CLK_CAM1 = 0;
	}

	wait(exp_time);

	//Real ANALOG READ
//	TAOS_SI = 0;
//	TAOS_CLK = 0;

	SI_CAM1 = 1;
	CLK_CAM1 = 1;
	SI_CAM1 = 0;
	data[0] = A0_CAM1.read_u16();
	CLK_CAM1 = 0;

	for(int i = 1; i < PIXELS; i++) {
		CLK_CAM1 = 1;
		data[i] = A0_CAM1.read_u16();
		//TELEMETRY
		tele_linescan[i] = data[i];
//		if (data[i] > max) { //calc max
//			max = data[i];
//			max_i = i;
//		}
		if (data[i] > max) { //calc max
			if ( (i > old_max_i*thresh) || (i < old_max_i*thresh) ) {
				max = data[i];
				old_max_i = max_i;
				max_i = i;
			}
		}
		CLK_CAM1 = 0;
	}

	pos = r_end - (step * max_i);
	pw = pos * (0.000001);

	//ACTUATION!

	////Send INPUT value to servo
	if (max != 0) {
		servo.pulsewidth(pw);
		//TELEMETRY
		tele_servo_pwm = pw*1000000;
	}
}

void velocity_OPENLOOP()
{
	motor.write(0.10f);
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
		//dutyInput = (Kp*err) + intgrlTerm;
		dutyInput = (Kp*err);

		//Clamp REFERENCE value if needed
		if (dutyInput > maxPWM) {
			dutyInput = maxPWM;
		} else if (dutyInput < minPWM) {
			dutyInput = minPWM;
		}

		if (ramp<100){
			dutyInput = 0.10f;
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
//	tele_hall_revs = revsB/1.0;
//	revsB = 0;
//	tele_servo_pwm = pw*1000000;
//
//	if(UP) {
//		tele_button_push = 1.0;
//		refVel = refVel + 0.01;
//	} else if(DOWN) {
//		tele_button_push = 1.0;
//		refVel = refVel - 0.01;
//	} else {
//		tele_button_push = 0.0;
//		refVel = refVel;
//	}

	steeringPD();

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
