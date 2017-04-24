#include "mbed.h"
//TELEMETRY
#include "telemetry.h"
#include "MODSERIAL.h"

////TELEMETRY
//MODSERIAL telemetry_serial(PTC15, PTC14);
//
//telemetry::MbedHal telemetry_hal(telemetry_serial);
//telemetry::Telemetry telemetry_obj(telemetry_hal);
//
////telemetry::Numeric<float> tele_servo_pwm(telemetry_obj, "servo", "Servo PWM", "PWM (us)", 0);
//telemetry::NumericArray<uint16_t, 128> tele_linescan_cam1(telemetry_obj, "Camera 1", "Linescan", "ADC", 0);
//telemetry::NumericArray<uint16_t, 128> tele_linescan_cam2(telemetry_obj, "Camera 2", "Linescan", "ADC", 0);
//telemetry::Numeric<float> tele_linewidth_cam1(telemetry_obj, "Linescan camera 1", "Linewidth","pixels",0);
//telemetry::Numeric<float> tele_linewidth_cam2(telemetry_obj, "Linescan camera 2", "Linewidth", "pixels", 0);
//telemetry::Numeric<uint32_t> tele_cam1_linedetected(telemetry_obj, "cam1_linedetected", "Linewidth", "pixels", 0);
//telemetry::Numeric<uint32_t> tele_cam2_linedetected(telemetry_obj, "cam2_linedetected", "Linewidth", "pixels", 0);
//
//
////telemetry::NumericArray<uint16_t, 128> tele_linescan_derivative_cam1(telemetry_obj, "linescan", "derivative", "camera", 0);
////telemetry::NumericArray<uint16_t, 128> tele_linescan_derivative_cam2(telemetry_obj, "linescan", "derivative", "camera", 0);
//telemetry::Numeric<uint32_t> tele_time_ms(telemetry_obj, "time", "Time", "ms", 0);
////telemetry::Numeric<float> tele_cam1_max(telemetry_obj, "camera 1","max value", "pixel value", 0);
////telemetry::Numeric<float> tele_cam2_max(telemetry_obj, "camera 2","max value", "pixel value", 0);
////telemetry::Numeric<float> tele_velocity_velMA(telemetry_obj, "VELOCITY", "m/s", "count", 0);
////telemetry::Numeric<float> tele_cam1_max_i(telemetry_obj, "camera 1", "max pixel index", "index", 0);
////elemetry::NumericArray<uint16_t, 128> tele_diff_cam1(telemetry_obj, "CAMERA", "Linescan", "frame difference",0);
////telemetry::NumericArray<uint16_t, 128> tele_diff_cam2(telemetry_obj, "CAMERA", "Linescan", "frame difference",0);

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
const float refVel_max = 2.9; // Corresponds to m/s
const float refVel_min = 2.5; // Corresponds to m/s
const float refVel_decr = 0.01;
volatile float refVel = refVel_max; // Corresponds to m/s
volatile float refVel_turn = refVel_max;

const float dt = 0.010; //CONTROL LOOP TIME!
const float Kp_vel = 0.20; //Proportional Gain
const float Ki_vel = 0.01; //Integral Gain
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
volatile float err_vel = 0.0;

//Servo characteristics
float l_end = 990.0;
float middle = 1500.0;
float r_end = 2100.0;

// Servo steps
float divi = 128.0;
float steps = r_end - l_end;
//float step = steps/divi;
float step = steps/114;

// Servo position
volatile float pw = 0.0;
volatile float old_pw = 0.0;
volatile float max_cam1 = 0.0;
volatile float max_cam2 = 0.0;
volatile int max_i_cam1 = 0;
volatile int max_i_cam2 = 0;
volatile int max_i_diff = 0;
volatile float pos = 0.0;
volatile int old_max_i = 0;
volatile float old_max_cam1;
volatile float old_max_cam2;
volatile int line_detect_cam1 = 0;
volatile int line_detect_cam2 = 0;
const int cam1_lwr_bnd = 5;
const int cam1_upr_bnd = 12;
const int cam2_lwr_bnd = 4;
const int cam2_upr_bnd = 13;

// Line cam vars
int PIXELS = 128;
unsigned short data_cam1[128];
unsigned short data_cam2[128];
unsigned short prev_data_cam1[128];
unsigned short prev_data_cam2[128];
unsigned short derivative_cam1[128];
unsigned short derivative_cam2[128];
unsigned short diff_cam1[128];
unsigned short diff_cam2[128];

const float thresh = 0.8;

//CALIBRATION VARIABLES
volatile int linewidth_cam1 = 0;
volatile int linewidth_cam2 = 0;
volatile int lines_cam1 = 0;
volatile int lines_cam2 = 0;
volatile bool rising_edge_cam1 = false;
volatile bool rising_edge_cam2 = false;

//Constant STEERING CONTROL-related variables
const int refSteer = 61;

volatile float Kp_steer = 5.2; //in degrees of steering per pixel error
const float Kp_turn = 6.7;
const float Kp_strt = 5.2;

volatile float Kd_steer = 1.0; //in degrees/s of steering change per pixel error
const float Kd_turn = 1.0;
const float Kd_strt = 1.0;

volatile int err_steer = 0;
volatile int last_err_steer = 0;
volatile int derivTerm = 0;
volatile float input_steer = 0.0;
const int steering_bound = 70;

volatile float exp_time = 0.001; //1ms

void calculateLinewidth() {
	linewidth_cam1 = 0;
	linewidth_cam2 = 0;
	rising_edge_cam1 = false;
	rising_edge_cam2 = false;
	for (int i = 0; i < PIXELS; i++) {
		//tele_linescan_derivative_cam1[i] = derivative_cam1[i];
		//tele_linescan_derivative_cam2[i] = derivative_cam2[i];
		if (data_cam1[i] > thresh * max_cam1) {
			linewidth_cam1 += 1;
			rising_edge_cam1 = true;
		} else if (rising_edge_cam1) {
			rising_edge_cam1 = false;
		}
		if (data_cam2[i] > thresh * max_cam2) {
			linewidth_cam2 += 1;
			rising_edge_cam2 = true;
		} else if (rising_edge_cam2) {
			rising_edge_cam2 = false ;
		}
	}

	if (linewidth_cam1 <= cam1_upr_bnd && linewidth_cam1 >= cam1_lwr_bnd) {
		//pos = r_end - (step * max_i_cam1);
		line_detect_cam1 = 1;
	} else {
		line_detect_cam1 = 0;
	}
	if (linewidth_cam2 <= cam2_upr_bnd && linewidth_cam2 >= cam2_lwr_bnd) {
		//pos = r_end - (step * max_i_cam2);
		line_detect_cam2 = 1;
	} else {
		line_detect_cam2 = 0;
	}

//	//TELEMETRY!
//	tele_linewidth_cam1 = linewidth_cam1;
//	tele_linewidth_cam2 = linewidth_cam2;
//	tele_cam1_linedetected = line_detect_cam1;
//	tele_cam2_linedetected = line_detect_cam2;
}

//VELOCITY!
//ISR for HALL sensor reading
void rpmCounterA() {
    revsA++;
}

void rpmCounterB() {
    revsB++;
}

void read_cam_real() {
	SI_CAM1 = 1;
	SI_CAM2 = 1;
	CLK_CAM1 = 1;
	CLK_CAM2 = 1;
	SI_CAM1 = 0;
	SI_CAM2 = 0;
	data_cam1[0] = A0_CAM1.read_u16();
	CLK_CAM1 = 0;
	data_cam2[0] = A0_CAM2.read_u16();
	CLK_CAM2 = 0;

	for(int i = 1; i < PIXELS; i++) {

		CLK_CAM1 = 1;
		CLK_CAM2 = 1;
		data_cam1[i] = A0_CAM1.read_u16();
		data_cam2[i] = A0_CAM2.read_u16();

//		//TELEMETRY
//		tele_linescan_cam1[i] = data_cam1[i];
//		tele_linescan_cam2[i] = data_cam2[i];
		if (data_cam1[i] > max_cam1) { //calc max
			max_cam1 = data_cam1[i];
			max_i_cam1 = i;
		}
		if (data_cam2[i] > max_cam2) {
			max_cam2 = data_cam2[i];
			max_i_cam2 = i;
		}

		CLK_CAM1 = 0;
		CLK_CAM2 = 0;

		//		diff_cam1[i] = data_cam1[i] - prev_data_cam1[i];
		//		diff_cam2[i] = data_cam2[i] - prev_data_cam2[i];
		//		tele_diff_cam1[i] = diff_cam1[i];
		//		tele_diff_cam2[i] = diff_cam2[i];
		//		prev_data_cam1[i] = data_cam1[i];
		//		prev_data_cam2[i] = data_cam2[i];
	}
}

void read_cam_fake() {
	//Fake ANALOG READ
	SI_CAM1 = 1;
	SI_CAM2 = 1;
	CLK_CAM1 = 1;
	CLK_CAM2 = 1;
	SI_CAM1 = 0;
	SI_CAM2 = 0;
	CLK_CAM1 = 0;
	CLK_CAM2 = 0;
	for(int i = 0; i < PIXELS; i++) {
		CLK_CAM1 = 1;
		CLK_CAM2 = 1;
		CLK_CAM1 = 0;
		CLK_CAM2 = 0;
	}

	wait(exp_time);
}

void readCamera() {
	max_cam1 = 0;
	max_cam2 = 0;

	read_cam_fake();

	read_cam_real();
}

void PDcalculation(volatile int max_i) {
	//PD STEERING
	//Line is to the Right
	if (max_i > refSteer) {
		err_steer = max_i - refSteer;
		derivTerm = (err_steer - last_err_steer);

		input_steer = (Kp_steer*err_steer) + (Kd_steer*derivTerm);

		pos = middle - input_steer;

		//CLAMP to servo limits
		if (pos < l_end) {
			pos = l_end;
		} else if (pos > r_end) {
			pos = r_end;
		}

		last_err_steer = err_steer;
	}
	//Line is to the Left
	else if (max_i < refSteer) {
		err_steer = refSteer - max_i;
		derivTerm = err_steer - last_err_steer;

		input_steer = (Kp_steer*err_steer) + (Kd_steer*derivTerm);

		pos = middle + input_steer;

		//CLAMP to servo limits
		if (pos < l_end) {
			pos = l_end;
		} else if (pos > r_end) {
			pos = r_end;
		}

		last_err_steer = err_steer;
	} else {
		pw = pw;
	}
}

void steeringPD() {

//	tele_cam1_max = max_cam1;
//	tele_cam2_max = max_cam2;
	if (line_detect_cam1) {
		//pos = r_end - (step * max_i_cam1);
		PDcalculation(max_i_cam1);
		pw = pos * (0.000001);  //average of the pw
	} else if (line_detect_cam2) {
		//pos = r_end - (step * max_i_cam2);
		PDcalculation(max_i_cam2);
		pw = pos * (0.000001);  //average of the pw
	} else {
		if (pw > middle) {
			pw = r_end;
		} else {
			pw = l_end;
		}
	}

	//ACTUATION!

	////Send INPUT value to servo
	if (max_cam1!= 0) {
		servo.pulsewidth(pw);
	}
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

		//tele_velocity_velMA = velMA;

		//Calculate ERROR
		err_vel = refVel - velMA;

		//Accumulate error for integral term
		intgrlTerm = intgrlTerm + (Ki_vel*err_vel);

		//Anti-Windup check!
		if (intgrlTerm > maxPWM){
			intgrlTerm = maxPWM;
		} else if (intgrlTerm < minPWM){
			intgrlTerm = minPWM;
		}

		//Calculate INPUT value
		//dutyInput = (Kp_vel*err) + intgrlTerm;
		dutyInput = (Kp_vel*err_vel);

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

void turnControl() {

	if (line_detect_cam1 && line_detect_cam2) {
		if (max_i_cam1 > max_i_cam2) {
			max_i_diff = max_i_cam1 - max_i_cam2;
		} else if (max_i_cam1 < max_i_cam2) {
			max_i_diff = max_i_cam2 - max_i_cam1;
		}
	} else {
		max_i_diff = 0;
	}

	if (((pos > (middle + steering_bound)) || (pos < (middle - steering_bound))) || (max_i_diff > 10)) {

		//Ramp down by decrement
		refVel_turn = refVel_turn - refVel_decr;
		//Clamp to minimum speed
		if (refVel_turn < refVel_min) {
			refVel_turn = refVel_min;
		}
		refVel = refVel_turn;
		Kp_steer = Kp_turn;

	} else {
		refVel = refVel_max;
		Kp_steer = Kp_strt;
		refVel_turn = refVel_max;
	}
}

void control() {

	readCamera();

	calculateLinewidth();

	turnControl();

	steeringPD();

	velocityPI();

	refVel = refVel_max;
	Kp_steer = Kp_strt;
}

////TELEMETRY
//void print_serial() {
//	tele_time_ms = telemetry_hal.get_time_ms();
//    telemetry_obj.do_io();
//}

//Function to initialize system
void setup() {

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
//    //TELEMETRY
//    telemetry_serial.baud(38400);
//    //tele_servo_pwm.set_limits(0.0, 2000.0); // lower bound, upper bound
//    telemetry_obj.transmit_header();

    setup();

    //run while loop
	int counter = 0;
    while (1) {
    	counter++;
    	if (counter % 2 == 1) {
    		//print_serial();
    	}
    }
}
