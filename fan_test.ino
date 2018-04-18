#include <Servo.h>
#include "Arduino.h"
#include "PMotor.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <NewPing.h>
#include <LiquidCrystal.h>
#define TRIGGER_PIN  23
#define ECHO_PIN     24
#define MAX_DISTANCE 400
#define TRIGGER_PIN_2 12
#define ECHO_PIN_2 13
#define Z_ROT_POT 0
#define RIGHT_LINE 1
#define FRONT_LINE 2
#define FLAME 3
#define Z_ROT 9
#define Y_ROT 10
#define MAX_Z 750
#define MIN_Z 300
#define LOW_TOL 750
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
Servo esc, yServo;

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

Adafruit_BNO055 bno = Adafruit_BNO055(55);

bool rotate=true, increasing=true;
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
}

void loop() {

	// Wait for some input
	rMotor->drive(0);
	lMotor->drive(0);
	sensors_event_t event;
	bno.getEvent(&event);

	/* Display the floating point data */
	Serial.print("X: ");
	Serial.print(event.orientation.x, 4);
	Serial.print("\tY: ");
	Serial.print(event.orientation.y, 4);
	Serial.print("\tZ: ");
	Serial.print(event.orientation.z, 4);
	Serial.print(" ");
	//delay(50);
	Serial.print("Ping1: ");
	Serial.print(sonar.ping_in());
	Serial.print("in ");
	//delay(20);
	Serial.print("Ping2: ");
	Serial.print(sonar2.ping_in());
	Serial.print("in ");
	Serial.print("flame: ");
	Serial.print(analogRead(FLAME));
	Serial.print(" f-rot: ");
	Serial.print(analogRead(Z_ROT_POT));
	Serial.println(" ");
	yServo.write(80);
	//zServo.writeMicroseconds(100); //this moves to robot right @ 600
	if (rotate && increasing) {
		analogWrite(Z_ROT, 110);     //turn left uncomment for analog
		Serial.println("increasing");
	} else if (rotate && !increasing) {
		analogWrite(Z_ROT, 200);     //turn right uncomment for analog
		Serial.println(" decreasing");
	}
	if (analogRead(Z_ROT_POT)>= MAX_Z) {
		increasing = false;
	} else if (analogRead(Z_ROT_POT) <= MIN_Z) {
		increasing = true;
	}
	if (analogRead(FLAME) < LOW_TOL) {
		analogWrite(Z_ROT, 0);     //turn left
		Serial.print("total angle @ ");
		Serial.println((analogRead(Z_ROT_POT)-200)/60 + event.orientation.x, 4);
		yServo.write(160);
		bool done =false;
		while(!done){
			yServo.write(yServo.read()-1);
			Serial.println(analogRead(FLAME));
			if(analogRead(FLAME)<600){
				done=true;
			}
			delay(100);
		}
		esc.write(80);
		delay(3000);//wait to put out the fire
		rotate = false;
	} else {
		Serial.println(" nothing");
		esc.write(0);
		rotate = true;
	}
	delay(10);
	//delay(100);
	//if (Serial.available() > 0) {

	// Read the new throttle value
	// esc.write(normalizeThrottle( Serial.parseInt()));

	// Print it out
//    Serial.print("Setting throttle to: ");
//    Serial.println(throttle);
//
//    // Change throttle to the new value
//    changeThrottle(throttle);


}

void changeThrottle(int throttle) {

  // Read the current throttle value
  int currentThrottle = readThrottle();

  // Are we going up or down?
  int step = 1;
  if( throttle < currentThrottle )
    step = -1;

  // Slowly move to the new throttle value
  while( currentThrottle != throttle ) {
    esc.write(currentThrottle + step);
    currentThrottle = readThrottle();
    delay(throttleChangeDelay);
  }

}

int readThrottle() {
  int throttle = esc.read();

  Serial.print("Current throttle is: ");
  Serial.println(throttle);

  return throttle;
}

// Ensure the throttle value is between 0 - 180
int normalizeThrottle(int value) {
  if( value < 0 )
    return 0;
  if( value > 180 )
    return 180;
  return value;
}
