/*
 * Encoder.cpp
 *
 *  Created on: Apr 22, 2018
 *      Author: kent2
 */
#include "Encoder.h"
#include "Arduino.h"
const float COUNT_TO_INCHES=.0107992;
/**
 * This sets up the motor without the encoder
 */
Encoder::Encoder(int inPort){
	this->inPort=inPort;
	this->count=0;
}
/**
 * Speed must be between -1 and 1, any higher values will
 * get cut off
 */
int Encoder::getCount(){
	return count;
	//Serial.println(MAX_OUTPUT*speed);
}
float Encoder::getInches(){
	noInterrupts();
	int temp_count=this->count;
	interrupts();
	return (float)temp_count*COUNT_TO_INCHES;
}

void Encoder::incrementCount(){
	this->count++;
}
void Encoder::resetCount(){
	this->count=0;
}
int Encoder::getPort(){
	return this->inPort;
}



