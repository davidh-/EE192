#include "mbed.h"
//TELEMETRY
#include "telemetry.h"
#include "MODSERIAL.h"

//TELEMETRY
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
InterruptIn hall(D10);

// Tickers
Ticker timestep;
Ticker printer;
Ticker telemTick;

// Digital pin to read from Hall Sensor
PwmOut motor(D13);
DigitalOut brakeEN(D12);

// Servo Pin
PwmOut servo(D9);

//Initialize needed variables
const float refVel = 1.8; // Corresponds to m/s
const float dt = 0.010; //PI interrupt in SECONDS 100 Hz
const float Kp = 0.20; //Proportional Gain
const float Ki = 0.01; //Integral Gain
const float maxPWM = 0.30;
const float minPWM = 0.0;
//TELEMETRY
const float dt_telem = 0.2; //telemetry clock

//transform to m/s with gear ratio (9:1 gear ratio) (r = 32.5 mm)
const float r = 0.0325; // meters
const float gear = 9.0; // 9:1 gear down ratio
const float velTransform = (2.0*3.141592*r) / (gear*2*dt);

//Volatile variables to be used in interrupts
volatile float vel = 0.0;
volatile float velMA = 0.0;
volatile float velSum = 0.0;
volatile int revs = 0;
volatile int resolution = 10;
volatile int count = 1;
volatile int ramp = 1;

//Volatile CONTROL-related variables to be used in interrupts
volatile float dutyInput = 0.0;
volatile float intgrlTerm = 0.0;
volatile float err = 0.0;

float angle2pw_us(float angle);

//Constant STEERING CONTROL-related variables
const float Kp_steer = 0.65; //in degrees of steering per pixel error
const float Kd_steer = 0.4; //in degrees/s of steering change per pixel error
const float refSteer = 58.0;

//works pretty well with .65 and .25
//ditto with .68 and .40


//Volatile STEERING CONTROL-related variables
volatile float prev_steer_err = 0.0;
volatile float err_steer = 0.0;
volatile float d_err_steer = 0.0;
volatile float steer_angle = 0.0;


int run = 0;

// Servo endpoints
//float l_end = 1020.0;
////1440 center
//float r_end = 1750.0;

const float l_end_angle = -20.0;
const float r_end_angle = 20.0;

float l_end = 1110.0;
float middle = 1520.0;
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
volatile float old_max = 0.0;

// Line cam vars
int PIXELS = 128;
unsigned short data[128];

const float thresh = 0.75;

volatile float exp_time = 0.002; //10ms

//FUNCTIONS!

//TELEMETRY
void print_serial() {
	tele_time_ms = telemetry_hal.get_time_ms();
    telemetry_obj.do_io();
}

//ISR for Velocity Printing
void printerFcn() {
    printf("%f\r\n",velMA);
}

//STEERING!
//Function for Camera Reading and servo command
void steeringPD() {

	//Fake ANALOG READ
	TAOS_SI = 1;
	TAOS_CLK = 1;
	TAOS_SI = 0;
	TAOS_CLK = 0;
	for(int i = 0; i < PIXELS; i++) {
		TAOS_CLK = 1;
		TAOS_CLK = 0;
	}

	wait(exp_time);

	//Real ANALOG READ
//	TAOS_SI = 0;
//	TAOS_CLK = 0;

	TAOS_SI = 1;
	TAOS_CLK = 1;
	TAOS_SI = 0;
	data[0] = A_IN.read_u16();
	TAOS_CLK = 0;

	for(int i = 1; i < PIXELS; i++) {
		TAOS_CLK = 1;
		data[i] = A_IN.read_u16();
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
		TAOS_CLK = 0;
	}


	//Steering PD control



	prev_steer_err = err_steer;
	err_steer = refSteer - max_i;
	d_err_steer = (err_steer - prev_steer_err) / dt;
	steer_angle = Kp_steer * err_steer + Kd_steer * d_err_steer;
	pw = angle2pw_us(steer_angle);
	pw = pw * (.000001);

	//pos = r_end - (step * max_i);
	//pw = pos * (0.000001);

	//ACTUATION!

	////Send INPUT value to servo
	if (max > 0.6 * old_max) {
		servo.pulsewidth(pw);
		//TELEMETRY
		tele_servo_pwm = pw*1000000;
	}

	max = 0;
}


float angle2pw_us(float angle) {
    float servo_pulse_width = angle * (r_end - l_end) / (r_end_angle - l_end_angle) + middle;
    if (servo_pulse_width > r_end) {
            servo_pulse_width = r_end;
    }
	else if (servo_pulse_width < l_end) {
		servo_pulse_width = l_end;
	}
	return servo_pulse_width;
}


//VELOCITY!
//ISR for HALL sensor reading
void rpmCounter() {
    revs++;
}

void velocity_OPENLOOP()
{
	motor.write(0.12f);
}

//Function for Speed Reading and motor command
void velocityPI() {
	//Calculate velocity (revolutions/fixed time unit)
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

		if (ramp<25){
			dutyInput = 0.07f;
			ramp++;
		}

		//ACTUATION!
		//Send INPUT value to motor
		motor.write(dutyInput);
		revs = 0;

		//TELEMETRY
		tele_motor_pwm = dutyInput*100;
	}
}

//ISR for timed controller using a Ticker
void control() {

	//Steering control
	steeringPD();

	//Velocity control
	velocityPI();
	//velocity_OPENLOOP();

}

//Function to initialize system
void setup() {

	//TELEMETRY
	telemetry_serial.baud(38400);

	tele_motor_pwm.set_limits(0.0, 20.0); // lower bound, upper bound
	tele_servo_pwm.set_limits(0.0, 2000.0); // lower bound, upper bound
	telemetry_obj.transmit_header();

	//set pwm periods
	motor.period_us(50); // 20 kHz frequency
	servo.period_ms(10); //Set servo PWM period = 3ms

	//set regular interrupts
	hall.rise(&rpmCounter);
	hall.fall(&rpmCounter);

	//set Ticker interrupts
	timestep.attach(&control,dt);
	//telemTick.attach(&print_serial,dt_telem);

	//set
	//printer.attach(&printerFcn,0.05);

	//set BrakeEN to HIGH to allow driving
	brakeEN.write(1);

	//set warm-up speed and command motor
	dutyInput = 0.07f;
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
