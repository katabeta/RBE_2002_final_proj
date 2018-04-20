/*
 * states.h
 *
 *  Created on: Apr 19, 2018
 *      Author: kent2
 */

#ifndef NAV_STATES_H_
#define NAV_STATES_H_
enum state{
	straight,
	prep_left,
	prep_right,
	left,
	right,
	see_fire,
	await
};
typedef enum state state;

void drive_straight();
float decrease_setpoint();
float increase_setpoint();
bool turn_right();
bool turn_left();
float get_relative_heading();

#endif /* NAV_STATES_H_ */
