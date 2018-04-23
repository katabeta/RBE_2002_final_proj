/*
 * Encoder.h
 *
 *  Created on: Apr 22, 2018
 *      Author: kent2
 */

#ifndef ENCODER_H_
#define ENCODER_H_
class Encoder{
public:
	Encoder(int inPort);
	int getCount();
	int getPort();
	float getInches();
	void incrementCount();
	void resetCount();
private:
	int inPort;
	int count;
};





#endif /* ENCODER_H_ */
