/*
 * PMotor.h
 *
 *  Created on: Feb 19, 2018
 *      Author: kent2
 */

#ifndef REACTORCONTROL_PMOTOR_H_
#define REACTORCONTROL_PMOTOR_H_
class PMotor{
public:
	PMotor(int fwdPort, int revPort);
	void drive(double speed);
	double getSpeed();
private:
	int fwdPort;
	int revPort;
	double speed;
};




#endif /* REACTORCONTROL_PMOTOR_H_ */
