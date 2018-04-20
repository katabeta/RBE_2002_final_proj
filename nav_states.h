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
	dead_straight,
	go_left,
	go_right,
	left,
	right,
	see_fire,
	init_await,
	await
};
typedef enum state state;

void drive_straight();
void decrease_setpoint();
void increase_setpoint();
bool turn_right();
bool turn_left();
float get_relative_heading();

#endif /* NAV_STATES_H_ */
