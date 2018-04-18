/*
 * PMotor.cpp
 *
 *  Created on: Feb 19, 2018
 *      Author: kent2
 */

#include "PMotor.h"
#include "Arduino.h"

const double MAX_OUTPUT=255;
/**
 * This sets up the motor without the encoder
 */
PMotor::PMotor(int fwdPort, int revPort){
	this->fwdPort=fwdPort;
	this->revPort=revPort;
	this->speed=0;
}
/**
 * Speed must be between -1 and 1, any higher values will
 * get cut off
 */
void PMotor::drive(double speed){
	if(speed>1){speed=1;}
	if(speed<-1){speed=-1;}
	if(speed>0){
		analogWrite(fwdPort, MAX_OUTPUT*speed);
		analogWrite(revPort, 0);
	}
	else{
		analogWrite(fwdPort, 0);
		analogWrite(revPort, MAX_OUTPUT*speed*-1);
	}
	this->speed=speed;
	//Serial.println(MAX_OUTPUT*speed);
}
double PMotor::getSpeed(){
	return speed;
}



