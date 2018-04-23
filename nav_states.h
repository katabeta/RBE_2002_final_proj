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
	init_rotate_to_fire,
	rotate_to_fire,
	reset_turret,
	move_to_fire,
	init_await,
	await,
	init_test,
	init_test_2,
	test
};
typedef enum state state;
void pan_fan();
void extinguish_flame();
void drive_straight();
void decrease_setpoint();
void increase_setpoint();
bool turn_right();
bool turn_left();
float get_relative_heading();
float get_abs_turret_angle();
bool turn_to_angle(float des_angle);
bool turret_to_zero();
void update_r_enc();
void update_l_enc();
void calc_displacement();
#endif /* NAV_STATES_H_ */
