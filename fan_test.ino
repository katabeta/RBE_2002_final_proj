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
#define FLAME_TOL 400
#define WALL_TOL 4
#define TURN_SPEED .3
#define STRAIGHT_SPEED .2
#define CORRECTION_SPEED .13
namespace {

const double RIGHT_CORRECT = .55;
const double LEFT_CORRECT = .45;

}

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
float set_point;
float offset;
unsigned long dead_straight_time;
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
	lcd.setCursor(0,0);
	lcd.clear();
	int flame_val=analogRead(FLAME);
	lcd.print(flame_val);
	if (rotate && increasing) {
		analogWrite(Z_ROT, 100);     //turn left uncomment for analog
		Serial.println("increasing");
	} else if (rotate && !increasing) {
		analogWrite(Z_ROT, 200);     //turn right uncomment for analog
		Serial.println(" decreasing");
	}

	if (analogRead(Z_ROT_POT) >= MAX_Z) {
		increasing = false;
	} else if (analogRead(Z_ROT_POT) <= MIN_Z) {
		increasing = true;
	}
	if(flame_val<FLAME_TOL){
		lcd.setCursor(0,1);
		lcd.print("see fire");
		cur_state=see_fire;
	}
}

void extinguish_flame() {
	analogWrite(Z_ROT, 0); //stop the z rotation
	yServo.write(140);
	int flame_val=analogRead(FLAME);
	bool done = false;
	while (!done) {
		flame_val=analogRead(FLAME);
		yServo.write(yServo.read() - 1);
		lcd.setCursor(0,0);
		lcd.clear();
		lcd.print(flame_val);
		//Serial.println(analogRead(FLAME));
		if (flame_val< 100) {
			lcd.setCursor(0,1);
			lcd.print("extinguishing");
			done = true;
		}
		delay(100);
	}
	esc.write(80);
	delay(3000); //wait to put out the fire
	esc.write(0);
	rotate = false;
	cur_state=await;
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
//	delay(100);
	pan_fan();

	switch(cur_state){
	case straight:{
		drive_straight();
		Serial.print(" sonar right: ");
		lcd.clear();
		lcd.print("straight");
		char heading[4];
		int rel_heading=sonar_r.ping_in();
		sprintf(heading, "%d", rel_heading);
		lcd.print(heading);
		Serial.println(sonar_r.ping_in());
		if(sonar_f.ping_in()<9){
			cur_state=go_left;
		}
		if(sonar_r.ping_in()<WALL_TOL){
			lcd.setCursor(0,1);
			lcd.print("r close");
			lMotor->drive(TURN_SPEED);
			rMotor->drive(RIGHT_CORRECT);
		}
		if(sonar_r.ping_in()>7&&sonar_r.ping_in()<12){
			lcd.setCursor(0,1);
			lcd.print("l close");
			lMotor->drive(LEFT_CORRECT);
			rMotor->drive(TURN_SPEED);
		}
		if(sonar_r.ping_in()>=18){
			cur_state=go_right;
		}
		break;
	}
	case go_left:{
		decrease_setpoint();
		cur_state=left;
		break;
	}
	case go_right:{
		lMotor->drive(TURN_SPEED);
		rMotor->drive(TURN_SPEED);
		delay(1000);
		increase_setpoint();
		cur_state=right;
		break;
	}
	case left:{
		if(turn_left()){
			cur_state=straight;
		}
		break;
	}
	case right:{
		lcd.setCursor(0,1);
		lcd.print("t-right");
		if(turn_right()){
			dead_straight_time=millis()+3000;
			cur_state=dead_straight;
			Serial.println("we;ve turned");
			lcd.print("done t");
		}
		break;
	}
	case dead_straight:{
		lcd.setCursor(0,0);
		char heading[4];
		char setpoint[4];
		int rel_heading = (int)get_relative_heading();
		lcd.clear();
		lcd.setCursor(0,0);
		sprintf(heading, "%d", rel_heading);
		lcd.print(heading);
		int int_set_point=(int) set_point;
		lcd.setCursor(0,1);
		sprintf(setpoint, " s:%d",int_set_point);
		lcd.print(setpoint);
		if(millis()<dead_straight_time){
			drive_straight();
		}
		else{
			cur_state=straight;
		}
		break;
	}
	case see_fire:{
		Serial.println("sees fire");
		extinguish_flame();
		break;
	}
	case init_await:{
		increase_setpoint();
		cur_state=await;
		break;
	}
	case await:{
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
		//Serial.println(get_relative_heading(),4);
		//delay(100);
		break;
	}
	}
	delay(10);

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
	if(set_point==0){
		if(relative_heading<=180&&relative_heading!=0){
			lMotor->drive(TURN_SPEED);
			rMotor->drive(TURN_SPEED+CORRECTION_SPEED);
			Serial.println("rotating left");
		}
		else if(relative_heading>180){
			lMotor->drive(TURN_SPEED+CORRECTION_SPEED);
			rMotor->drive(TURN_SPEED);
			Serial.println("rotating right");
		}
		else{
			lMotor->drive(TURN_SPEED);
			rMotor->drive(TURN_SPEED);
		}
	}
	else{
		if(relative_heading<set_point){
			lMotor->drive(TURN_SPEED+CORRECTION_SPEED);
			rMotor->drive(TURN_SPEED);
			Serial.println("rotating right");
		}
		else if(relative_heading>set_point){
			lMotor->drive(TURN_SPEED);
			rMotor->drive(TURN_SPEED+CORRECTION_SPEED);
			Serial.println("rotating left");
		}
		else{
			lMotor->drive(TURN_SPEED);
			rMotor->drive(TURN_SPEED);
		}
	}

}
//increases setpoint by 90 and wraps around to 0
void increase_setpoint(){
	if(set_point>=270){
		set_point=0;
		return;
	}
	else{
		set_point+=90;
	}
}
//decreases setpoint by 90 and wraps around to 270
void decrease_setpoint(){
	if(set_point==0){
		set_point=270;
		return;
	}
	set_point-=90;
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
bool turn_right(){
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


