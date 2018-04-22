#include <Servo.h>
#include "Arduino.h"
#include "PMotor.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <NewPing.h>
#include <LiquidCrystal.h>

#include "nav_states.h"

/////// Sonar //////
#define TRIGGER_PIN  23
#define ECHO_PIN     24
#define MAX_DISTANCE 400
#define TRIGGER_PIN_2 12
#define ECHO_PIN_2 13
////// Line Senors //////
#define FRONT_LINE 1
#define RIGHT_LINE 2
////// Fan //////
#define Z_ROT_POT 0
#define Z_ROT 9
#define Y_ROT 10
//does a 120 degree sweep
#define Z_RANGE 120
#define MAX_Z 650
#define MIN_Z 350
////// FIRE //////
#define FLAME 3
#define LOW_TOL 750
#define FLAME_TOL 700
////// DRIVING //////
#define WALL_TOL 4
#define TURN_SPEED .2
#define STRAIGHT_SPEED .2
#define CORRECTION_SPEED .13

namespace {

const double RIGHT_CORRECT = .55;
const double LEFT_CORRECT = .45;

}

//test
const int escPin = 4;
const int relay_en = 22;
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

bool rotate = true, increasing = true;
bool found_flame=false;
//nav variables
float set_point;
float offset;
unsigned long dead_straight_time;
float des_angle;
//state machine
state cur_state;

const int rs = 40, en = 41, d4 = 37, d5 = 35, d6 = 33, d7 = 31;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {

	Serial.begin(9600);
	// set up the LCD's number of columns and rows:
	lcd.begin(16, 2);
	// Print a message to the LCD.
	lcd.print("hello, world!");
	lcd.clear();
	delay(100);
	// Serial.setTimeout(500);
	pinMode(relay_en, OUTPUT);
	pinMode(Z_ROT, OUTPUT); //uncommnet for analog usage
	pinMode(FLAME, INPUT);
	pinMode(FRONT_LINE, INPUT);
	pinMode(RIGHT_LINE, INPUT);
	// Attach the the servo to the correct pin and set the pulse range
	esc.attach(escPin, 1000, 2000);
	yServo.attach(Y_ROT);
	esc.write(0); //put the throttle to off
	yServo.write(80);
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
	lMotor = new PMotor(RIGHT_A, RIGHT_B);
	Serial.println("Orientation Sensor Test");
	Serial.println("");
	/* Initialise the sensor */
	while (!bno.begin()) {
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.println(
				"Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
	}
	delay(1000);
	bno.setExtCrystalUse(true);
	for (int i = 0; i < 15; i++) {
		sensors_event_t event;
		bno.getEvent(&event);
		offset = event.orientation.x;
		delay(100);
	}
	set_point = 0;
	cur_state = await;
}

void pan_fan() {
	int flame_val = analogRead(FLAME);
	int z_rot_val = analogRead(Z_ROT_POT);
	Serial.print("flame: ");
	Serial.print(flame_val);
	if (rotate && increasing) {
		analogWrite(Z_ROT, 105);     //turn left uncomment for analog
		Serial.println("increasing");
	} else if (rotate && !increasing) {
		analogWrite(Z_ROT, 200);     //turn right uncomment for analog
		Serial.println(" decreasing");
	}

	if (z_rot_val >= MAX_Z) {
		increasing = false;
	} else if (z_rot_val <= MIN_Z) {
		increasing = true;
	}
	if ((flame_val < FLAME_TOL)&&!found_flame) {
		found_flame=true;
		rotate=false;
		analogWrite(Z_ROT, 0);
		lMotor->drive(0);
		rMotor->drive(0);
		lcd.setCursor(0, 1);
		lcd.print("see fire");
		Serial.println("rotating to fire");
		//has the base rotate to the angle of the turret
		cur_state = init_rotate_to_fire;
	}
	else if((flame_val < FLAME_TOL)&&found_flame) {//this occurs after the flame has been found firs time
		analogWrite(Z_ROT, 0);
		rotate=false;
		cur_state=see_fire;
	}
}

void run_fan() {
	esc.write(100);
	delay(3000); //wait to put out the fire
	esc.write(0);
	rotate = false;
}
//the maximum range is 27"
void extinguish_flame() {

	yServo.write(55);
	int flame_val = analogRead(FLAME);
	bool done = false, found = false;
	if (sonar_f.ping_in() < 18) {
		lMotor->drive(0);
		rMotor->drive(0);
		analogWrite(Z_ROT, 0);
		while (!done) {
			flame_val = analogRead(FLAME);
			yServo.write(yServo.read() + 1);
			lcd.setCursor(0, 0);
			lcd.clear();
			lcd.print(flame_val);
			//Serial.println(analogRead(FLAME));
			if (flame_val < 400) {
				lcd.setCursor(0, 1);
				lcd.print("extinguishing");
				found = true;
				done = true;
			}
			if (yServo.read() > 140) { //check that limit if its too low
				done = true;
			}
			delay(100);
		}
		if (found) {
			rotate=false;
			run_fan();
			cur_state=await;
		}
	}
	else{
		//if we're too far move closer
		set_point=des_angle;
		drive_straight();
	}
}

void loop() {

	// Wait for some input
//	rMotor->drive(0);
//	lMotor->drive(0);
//	sensors_event_t event;
//	bno.getEvent(&event);
//	/* Display the floating point data */
//	Serial.print("X: ");
//	Serial.print(event.orientation.x, 4);
//	Serial.print(" Y: ");
//	Serial.print(event.orientation.y, 4);
//	Serial.print(" Z: ");
//	Serial.print(event.orientation.z, 4);
//	Serial.print(" ");
//	//delay(50);
	Serial.print("sonar f: ");
	Serial.print(sonar_f.ping_in());
	Serial.println("in ");
	delay(20);
//	Serial.print("sonar r: ");
//	Serial.println(sonar_r.ping_in()); //ensure that the ping times are ample for these
//	delay(20);
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
//	delay(100);
	pan_fan();
	lcd.setCursor(6,0);
	lcd.print(cur_state);
	switch (cur_state) {
	case straight: {
		drive_straight();
		Serial.print(" sonar right: ");
		lcd.clear();
		lcd.print("straight");
		int right_dist = sonar_r.ping_in();
		int front_dist = sonar_f.ping_in();
		char buffer[4];
		sprintf(buffer, "%d", right_dist);
		lcd.print(buffer);
		delay(10);
		if (front_dist < 9) {
			cur_state = go_left;
		}
		if (right_dist < WALL_TOL) {
			lcd.setCursor(0, 1);
			lcd.print("r close");
			lMotor->drive(TURN_SPEED);
			rMotor->drive(RIGHT_CORRECT);
		}
		if (right_dist > 7 && right_dist < 12) {
			lcd.setCursor(0, 1);
			lcd.print("l close");
			lMotor->drive(LEFT_CORRECT);
			rMotor->drive(TURN_SPEED);
		}
		if (right_dist >= 18) {
			cur_state = go_right;
		}
		break;
	}
	case go_left: {
		decrease_setpoint();
		cur_state = left;
		break;
	}
	case go_right: {
		lMotor->drive(TURN_SPEED);
		rMotor->drive(TURN_SPEED);
		delay(1500);
		increase_setpoint();
		cur_state = right;
		break;
	}
	case left: {
		if (turn_left()) {
			cur_state = straight;
		}
		break;
	}
	case right: {
		lcd.setCursor(0, 1);
		lcd.print("t-right");
		if (turn_right()) {
			dead_straight_time = millis() + 1500;
			cur_state = dead_straight;
			Serial.println("we;ve turned");
			lcd.print("done t");
		}
		break;
	}
	case dead_straight: {
		lcd.setCursor(0, 0);
		char heading[4];
		char setpoint[4];
		int rel_heading = (int) get_relative_heading();
		lcd.clear();
		lcd.setCursor(0, 0);
		sprintf(heading, "%d", rel_heading);
		lcd.print(heading);
		int int_set_point = (int) set_point;
		lcd.setCursor(0, 1);
		sprintf(setpoint, " s:%d", int_set_point);
		lcd.print(setpoint);
		if (millis() < dead_straight_time) {
			drive_straight();
		} else {
			cur_state = straight;
		}
		break;
	}
	case init_rotate_to_fire: {
		des_angle = get_relative_heading() + get_abs_turret_angle();
		cur_state = rotate_to_fire;
		break;
	}
	case rotate_to_fire: {
		int rot = get_abs_turret_angle();
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(rot);
		lcd.setCursor(0, 5);
		int heading = (int) get_relative_heading();
		lcd.print(heading);
		if (turn_to_angle(des_angle)) {
			rotate=true;
		}
		break;
	}
	case see_fire: {
		lMotor->drive(0);
		rMotor->drive(0);
		Serial.println("sees fire");
		extinguish_flame();
		break;
	}
	case init_await: {
		increase_setpoint();
		cur_state = await;
		break;
	}
	case await: {
//		drive_straight();
//		char heading[4];
//		char setpoint[4];
//		int rel_heading = (int)get_relative_heading();
//		lcd.clear();
//		lcd.setCursor(0,0);
//		sprintf(heading, "%d", rel_heading);
//		lcd.print(heading);
//		int int_set_point=(int) set_point;
//		lcd.setCursor(0,1);
//		sprintf(setpoint, " s:%d",int_set_point);
//		lcd.print(setpoint);
//		esc.write(100);
//		lcd.clear();
//		lcd.setCursor(0, 0);
//		int flame_val = analogRead(FLAME);
//		lcd.print(flame_val);
//		int sonar = sonar_f.ping_in();
//		lcd.setCursor(0, 1);
//		lcd.print(sonar);
//		delay(100);
		lMotor->drive(0);
		rMotor->drive(0);
		int rot = sonar_f.ping_in();
		lcd.clear();
		lcd.setCursor(0, 1);
		lcd.print(rot);

		break;
	}
	case init_test:{
		des_angle=get_relative_heading()+get_abs_turret_angle();
		cur_state=test;
		break;
	}
	case test: {
		int rot=get_abs_turret_angle();
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.print(rot);
		lcd.setCursor(0,5);
		int heading = (int) get_relative_heading();
		lcd.print(heading);
		if(turn_to_angle(des_angle)){
			cur_state=await;
		}
		break;
	}
	}
	delay(30);

}
//returns true when done
bool turn_to_angle(float des_angle) {
	//TODO figure out left turn and negative angles
	if(des_angle<0){
		des_angle+=360;
	}
	float relative_heading = get_relative_heading();
	int turret_angle=get_abs_turret_angle();
	float error=abs(relative_heading-des_angle);
	if(error<10){//this is the tolerance
		//this renables to rotation to find the flame again
		rotate=true;
		lMotor->drive(0);
		rMotor->drive(0);
		return true;
	}
	if(turret_angle<0){
		lMotor->drive(-TURN_SPEED);
		rMotor->drive(TURN_SPEED);
	}
	else if(turret_angle>0){
		lMotor->drive(TURN_SPEED);
		rMotor->drive(-TURN_SPEED);
	}
	else{ //this renables to rotation to find the flame again
		return true;
	}
//	if (des_angle == 0) {
//		if (relative_heading <= 180 && relative_heading != 0) {
//			lMotor->drive(-TURN_SPEED);
//			rMotor->drive(TURN_SPEED);
//			Serial.println("rotating left");
//		} else if (relative_heading > 180) {
//			lMotor->drive(TURN_SPEED);
//			rMotor->drive(-TURN_SPEED);
//			Serial.println("rotating right");
//		} else if(abs(relative_heading - des_angle)<2) {
//			lMotor->drive(0);
//			rMotor->drive(0);
//			return true;
//		}
//	} else {
//		if (relative_heading < des_angle) {
//			lMotor->drive(TURN_SPEED);
//			rMotor->drive(-TURN_SPEED);
//			Serial.println("rotating right");
//		} else if (relative_heading > des_angle) {
//			lMotor->drive(-TURN_SPEED);
//			rMotor->drive(TURN_SPEED);
//			Serial.println("rotating left");
//		} else if(abs(relative_heading - des_angle)<2){
//			lMotor->drive(0);
//			rMotor->drive(0);
//			return true;
//		}
//	}
	return false;
}
void drive_straight() {
	float relative_heading = get_relative_heading();
	const float allowable_error = 3; //may need to be 5
	Serial.print("setpoint: ");
	Serial.print(set_point);
	Serial.print(" relative_heading: ");
	Serial.print(relative_heading);
	Serial.print(" error: ");
	Serial.println(abs(relative_heading - set_point), 4);
	if (set_point == 0) {
		if (relative_heading <= 180 && relative_heading != 0) {
			lMotor->drive(TURN_SPEED);
			rMotor->drive(TURN_SPEED + CORRECTION_SPEED);
			Serial.println("rotating left");
		} else if (relative_heading > 180) {
			lMotor->drive(TURN_SPEED + CORRECTION_SPEED);
			rMotor->drive(TURN_SPEED);
			Serial.println("rotating right");
		} else {
			lMotor->drive(TURN_SPEED);
			rMotor->drive(TURN_SPEED);
		}
	} else {
		if (relative_heading < set_point) {
			lMotor->drive(TURN_SPEED + CORRECTION_SPEED);
			rMotor->drive(TURN_SPEED);
			Serial.println("rotating right");
		} else if (relative_heading > set_point) {
			lMotor->drive(TURN_SPEED);
			rMotor->drive(TURN_SPEED + CORRECTION_SPEED);
			Serial.println("rotating left");
		} else {
			lMotor->drive(TURN_SPEED);
			rMotor->drive(TURN_SPEED);
		}
	}

}
float get_abs_turret_angle(){
	const float tick_to_angle =.3; //this is half the range/ticks
	const int mid_point=485;
	int turret_pos=analogRead(Z_ROT_POT);
	int displacement = turret_pos-mid_point;
	float angle = (float)displacement*tick_to_angle;
	return angle;
}
//increases setpoint by 90 and wraps around to 0
void increase_setpoint() {
	if (set_point >= 270) {
		set_point = 0;
		return;
	} else {
		set_point += 90;
	}
}
//decreases setpoint by 90 and wraps around to 270
void decrease_setpoint() {
	if (set_point == 0) {
		set_point = 270;
		return;
	}
	set_point -= 90;
}

float get_relative_heading() {
	sensors_event_t event;
	bno.getEvent(&event);
	float abs_heading = event.orientation.x;
	if (abs_heading >= offset) {
		return abs_heading - offset;
	}
	return abs_heading - offset + 360.0;
}

bool turn_left() {

	float relative_heading = get_relative_heading();
	const float allowable_error = 5; //may need to be 5
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
	lMotor->drive(-TURN_SPEED);
	rMotor->drive(TURN_SPEED);
	return false;
}
bool turn_right() {
	float relative_heading = get_relative_heading();
	const float allowable_error = 5; //may need to be 5
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

