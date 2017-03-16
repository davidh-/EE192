#include "mbed.h"

//Interrupts
InterruptIn hall(A5);
Ticker timestep;
//Timer t;

//Digital pin to read from Hall Sensor
PwmOut motor(D3);

//Initialize needed variables
const float refVel = 4.0; // Corresponds to m/s
const float dt = 0.010; //PI interrupt in SECONDS 50 Hz
const float Kp = 0.30; //Proportional Gain
//const float Ki = 0.1; //Integral Gain
const float maxPWM = 0.50;
const float minPWM = 0.00;
//transform to m/s with gear ratio (9:1 gear ratio) (r = 32.5 mm)
const float r = 0.0325; // meters
const float gear = 5.5; // 9:1 gear down ratio
const float velTransform = (2.0*3.141592*r) / (gear*2*dt);

//Volatile variables to be used in interrupts
volatile float vel = 0.0;
volatile float velSum = 0.0;
volatile float velMA = 0.0;
volatile int revs = 0;
//volatile int lastRevs = 0;
//volatile int count = 1;
volatile int resolution = 10;

volatile float dutyInput = 0.0;
//volatile float intgrlTerm = 0.0;
volatile float err = 0.0;


//ISR for HALL sensor reading
void rpmCounter() {
	revs++;
}

//ISR for timed PI controller
void PIcontrol() {
	//Calculate velocity (revolutions/fixed time unit)

	//vel = (float) (revs - lastRevs) * (velTransform);
	vel = (float) (revs) * velTransform;

	//Calculate moving average
//	if (count < resolution){
//		velSum = velSum + vel;
//		//velMA = velSum/count;
//		count++;
//		if (count = resolution){
//			velMA = velSum;
//		}
//	} else {
//		velMA = velMA + (vel/resolution) - (velMA/resolution);
//	}

	velMA = velMA + (vel/resolution) - (velMA/resolution);

	//Calculate ERROR
	err = refVel - velMA;

	//Accumulate error for integral term
	//intgrlTerm = intgrlTerm + (Ki*err);

	//Anti-Windup check!
//	if (intgrlTerm > maxPWM){
//		intgrlTerm = maxPWM;
//	} else if (intgrlTerm < minPWM){
//		intgrlTerm = minPWM;
//	}

	//Calculate INPUT value
	//dutyInput = (Kp*err) + intgrlTerm;
	dutyInput = (Kp*err);

	//Clamp REFERENCE value if needed
	if (dutyInput > maxPWM){
		dutyInput = maxPWM;
	} else if (dutyInput < minPWM){
		dutyInput = minPWM;
	}

	//Send REFERENCE value to motor
	motor.write(dutyInput);
//	lastRevs = revs;
//	revs++;
	revs = 0;
}

int main() {

	//set interrupts
	hall.rise(&rpmCounter);
	hall.fall(&rpmCounter);

	//set pwm
	motor.period_us(50);      // 20 kHz frequency

	timestep.attach(&PIcontrol,dt);
	//set pwm
	//motor.period_us(50);      // 20 kHz frequency
	//motor.write(0.20f);

	while(1) {

//		t.reset();
//		t.start();

//		velSum = velSum + velocity;
//		avgCounter++;
//
//		if (avgCounter == 10){
//			//printf("%f\r\n",(velSum/avgCounter));
//			avgCounter = 0;
//			velSum = 0.0;
//		}

		//printf("%f\r\n",velMA);

//		t.stop();

		//printf("%f\r\n",(dt-t.read()));

//		if (t.read() < dt) {
//			wait(dt-t.read());
//		}
	}
}

////Digital pin to read from Hall Sensor
//PwmOut motor(D3);
//
//int main() {
//	//set interrupts
//	//hall.rise(&rpmCounter);
//	//hall.fall(&rpmCounter);
//	//timestep.attach(&PIcontrol,dt);
//	//set pwm
//	motor.period_us(70);      // 20 kHz frequency
//	motor.write(0.30f);
//}

