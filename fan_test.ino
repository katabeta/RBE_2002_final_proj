#include <Servo.h>
#include "Arduino.h"
#include "PMotor.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <NewPing.h>
#include <LiquidCrystal.h>

#include "nav_states.h"


#define TRIGGER_PIN  23
#define ECHO_PIN     24
#define MAX_DISTANCE 400
#define TRIGGER_PIN_2 12
#define ECHO_PIN_2 13
#define FRONT_LINE 1
#define RIGHT_LINE 2
#define Z_ROT_POT 0
#define FLAME 3
#define Z_ROT 9
#define Y_ROT 10
#define MAX_Z 750
#define MIN_Z 300
#define LOW_TOL 750
#define WALL_TOL 5
#define TURN_SPEED .3
#define STRAIGHT_SPEED .2

//test
const int escPin = 4;
const int relay_en=22;
const int minPulseRate = 1000;
const int maxPulseRate = 2000;
const int throttleChangeDelay = 100;
const int LEFT_A = 5;
const int LEFT_B = 6;
const int RIGHT_A = 7;
const int RIGHT_B = 8;

PMotor* rMotor;
PMotor* lMotor;

NewPing sonar_f(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
NewPing sonar_r(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
Servo esc, yServo;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

bool rotate=true, increasing=true;

//nav variables
float set_point=0;
float offset;
//state machine
state cur_state=prep_right;
//const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(40, 41, 42, 43, 44, 45);

void setup() {

	Serial.begin(9600);
	lcd.begin(16,2);
	lcd.print("hi");
	delay(100);
	// Serial.setTimeout(500);
	pinMode(relay_en, OUTPUT);
	pinMode(Z_ROT, OUTPUT); //uncommnet for analog usage
	pinMode(FRONT_LINE, INPUT);
	pinMode(RIGHT_LINE, INPUT);
	// Attach the the servo to the correct pin and set the pulse range
	esc.attach(escPin, 1000, 2000);
	yServo.attach(Y_ROT);
	esc.write(0); //put the throttle to off
	yServo.write(90);
	Serial.println("starting");
	delay(1000); //let it register
	Serial.println("fan should be beeping");
	digitalWrite(relay_en, HIGH); //turn on the fan so it can read the 0 point
	delay(2000); //and let it chill
	pinMode(LEFT_A, OUTPUT);
	pinMode(LEFT_B, OUTPUT);
	pinMode(RIGHT_A, OUTPUT);
	pinMode(RIGHT_B, OUTPUT);
	rMotor = new PMotor(LEFT_A, LEFT_B);
	lMotor=new PMotor(RIGHT_A, RIGHT_B);
	Serial.println("Orientation Sensor Test"); Serial.println("");
	  /* Initialise the sensor */
	  while(!bno.begin())
	  {
	    /* There was a problem detecting the BNO055 ... check your connections */
	    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
	  }
	  delay(1000);
	  bno.setExtCrystalUse(true);
	for (int i = 0; i < 15; i++) {
		sensors_event_t event;
		bno.getEvent(&event);
		offset = event.orientation.x;
		delay(100);
	}
}

void pan_fan() {
	if (rotate && increasing) {
		//analogWrite(Z_ROT, 100);     //turn left uncomment for analog
		Serial.println("increasing");
	} else if (rotate && !increasing) {
		//analogWrite(Z_ROT, 200);     //turn right uncomment for analog
		Serial.println(" decreasing");
	}

	if (analogRead(Z_ROT_POT) >= MAX_Z) {
		increasing = false;
	} else if (analogRead(Z_ROT_POT) <= MIN_Z) {
		increasing = true;
	}
	if(analogRead(FLAME)<LOW_TOL){
		//cur_state=see_fire;
	}
}

void extinguish_flame() {
	analogWrite(Z_ROT, 0); //stop the z rotation
	yServo.write(160);
	bool done = false;
	while (!done) {
		yServo.write(yServo.read() - 1);
		Serial.println(analogRead(FLAME));
		if (analogRead(FLAME) < 600) {
			done = true;
		}
		delay(100);
	}
	esc.write(80);
	delay(3000); //wait to put out the fire
	rotate = false;
}

void loop() {

	// Wait for some input
//	rMotor->drive(0);
//	lMotor->drive(0);
	sensors_event_t event;
	bno.getEvent(&event);
	/* Display the floating point data */
	Serial.print("X: ");
	Serial.print(event.orientation.x, 4);
	Serial.print(" Y: ");
	Serial.print(event.orientation.y, 4);
	Serial.print(" Z: ");
	Serial.print(event.orientation.z, 4);
	Serial.print(" ");
//	//delay(50);
//	Serial.print("sonar f: ");
//	Serial.print(sonar_f.ping_in());
//	Serial.print("in ");
//	delay(20);
//	Serial.print("sonar r: ");
//	Serial.print(sonar_r.ping_in()); //ensure that the ping times are ample for these
//	Serial.print("in ");
//	Serial.print("flame: ");
//	Serial.print(analogRead(FLAME));
//	Serial.print(" f-rot: ");
//	Serial.print(analogRead(Z_ROT_POT));
//	Serial.print(" right line: ");
//	Serial.print(analogRead(RIGHT_LINE));
//	Serial.print(" front line: ");
//	Serial.print(analogRead(FRONT_LINE));
//	Serial.println("");
	yServo.write(80);

	pan_fan();

	switch(cur_state){
	case straight:{
		drive_straight();
		Serial.println("driving straight");
		if(sonar_f.ping_in()<WALL_TOL){
			//cur_state=left;
		}
		if(sonar_f.ping_in()<WALL_TOL){
			//cur_state=right;
		}
		break;
	}
	case prep_left:{
		decrease_setpoint();
		cur_state=left;
		break;
	}
	case prep_right:{
		increase_setpoint();
		cur_state=right;
		break;
	}
	case left:{
		if(turn_left()){
			cur_state=await;
		}
		break;
	}
	case right:{
		if(turn_right()){
			cur_state=await;
		}
		break;
	}
	case see_fire:{
		Serial.println("sees fire");
		//extinguish_flame();
		break;
	}
	case await:{
		Serial.println(get_relative_heading(),4);
		delay(100);
		break;
	}
	}
	delay(10);

}
void drive_straight(){
	sensors_event_t event;
	bno.getEvent(&event);
	const float gain=.01, base_speed=.2;
	float error = event.gyro.x-set_point;
	float drive_speed=error*gain;
	Serial.print("X: ");
	Serial.print(event.gyro.x, 4);
	Serial.print(" drive speed: ");
	Serial.println(drive_speed);
	if(abs(drive_speed)>.05){
		lMotor->drive(base_speed+drive_speed);
		rMotor->drive(base_speed-drive_speed);
	}
	else{
		lMotor->drive(base_speed+drive_speed);
		rMotor->drive(base_speed-drive_speed);
	}
}
//increases setpoint by 90 and wraps around to 0
float increase_setpoint(){
	if(set_point>=270){
		set_point=0;
		return set_point;
	}
	else{
		return set_point+=90;
	}
}
//decreases setpoint by 90 and wraps around to 270
float decrease_setpoint(){
	if(set_point==0){
		set_point=270;
		return set_point;
	}
	return set_point-=90;
}

float get_relative_heading(){
	  sensors_event_t event;
	  bno.getEvent(&event);
	  float abs_heading=event.orientation.x;
	if(abs_heading>=offset){
		return abs_heading-offset;
	}
	return abs_heading-offset+360.0;
}

bool turn_left(){

	float relative_heading=get_relative_heading();
	const float allowable_error=3; //may need to be 5
	Serial.print("setpoint: ");
	Serial.print(set_point);
	Serial.print(" relative_heading: ");
	Serial.print(relative_heading);
	Serial.print(" error: ");
	Serial.println(abs(relative_heading-set_point), 4);
	if(abs(relative_heading-set_point)<allowable_error||abs(relative_heading-set_point)>(360-allowable_error)){
		lMotor->drive(0);
		rMotor->drive(0);
		return true;
	}
	lMotor->drive(-TURN_SPEED);
	rMotor->drive(TURN_SPEED);
	return false;
}
bool turn_right(){
	float relative_heading = get_relative_heading();
	const float allowable_error = 3; //may need to be 5
	Serial.print("setpoint: ");
	Serial.print(set_point);
	Serial.print(" relative_heading: ");
	Serial.print(relative_heading);
	Serial.print(" error: ");
	Serial.println(abs(relative_heading - set_point), 4);
	if (abs(relative_heading-set_point) < allowable_error
			|| abs(relative_heading-set_point) > (360 - allowable_error)) {
		lMotor->drive(0);
		rMotor->drive(0);
		return true;
	}
	lMotor->drive(TURN_SPEED);
	rMotor->drive(-TURN_SPEED);
	return false;
}


