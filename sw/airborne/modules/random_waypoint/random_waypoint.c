/*
 * random_waypoint.c
 *
 *  Created on: Jul 5, 2020
 *      Author: mark
 */


#include "autopilot.h"
#include "modules/random_waypoint/random_waypoint.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav funcitons like Inside...
#include "generated/flight_plan.h"

#define RANDOM_WAYPOINT_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[random_waypoint->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if RANDOM_WAYPOINT_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t moveWaypoint(uint8_t waypoint);
float getBoundedRandomNumber(float lower, float upper);
uint8_t tryYawMotion(void);

// Waypoint bounds (ENU)
float wp_x_min = -2.5;
float wp_x_max = 2.5;
float wp_y_min = -2.5;
float wp_y_max = 2.5;
float wp_z_min = 0.5;
float wp_z_max = 3.5;

// Waypoint margin
float wp_margin = 0.2;

// Max. heading increment
float max_yaw_cmd = 6000;
float prob_yaw_motion = 0.8; // 0-1

// Current waypoint
struct EnuCoor_f wp;


extern void random_waypoint_init(void){
	// Initialise random seed
	srand(time(NULL));
	return;
}
extern void random_waypoint_periodic(void){

  if(!autopilot_in_flight()){
	return;
  }
  // get pos. of MAV
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  // get pos. of waypoint
  wp.x = waypoint_get_x(WP_GOAL);
  wp.y = waypoint_get_y(WP_GOAL);
  wp.z = waypoint_get_alt(WP_GOAL);

  // move WP_GOAL if close to waypoint
  if (fabs(pos->x - wp.x) < wp_margin){
	  if (fabs(pos->y - wp.y) < wp_margin){
		  if (fabs(pos->z - wp.z) < wp_margin){

			  moveWaypoint(WP_GOAL);
			  // check if random yaw motion should occur
			  //tryYawMotion();

		  }
	  }
  }


  VERBOSE_PRINT("Current position: (%f, %f, %f). WP_GOAL: (%f, %f, %f).\n",
		  pos->x, pos->y, pos->z, wp.x, wp.y, wp.z);
  return;
}


uint8_t moveWaypoint(uint8_t waypoint){

	// random yaw motion
	//tryYawMotion();

	// get new coords
	wp.x = getBoundedRandomNumber(wp_x_min, wp_x_max);
	wp.y = getBoundedRandomNumber(wp_y_min, wp_y_max);
	wp.z = getBoundedRandomNumber(wp_z_min, wp_z_max);
	// set waypoint
	waypoint_set_enu(waypoint, &wp);
	VERBOSE_PRINT("Waypoint moved to (%f, %f, %f).\n", wp.x, wp.y, wp.z);

	return false;
}

uint8_t tryYawMotion(void){
	// check if heading should be moved
	float yaw_roll = getBoundedRandomNumber(0, 1);

	if (yaw_roll <= prob_yaw_motion){

		// move heading
		float nav_cmd_yaw_f = getBoundedRandomNumber(-max_yaw_cmd, max_yaw_cmd);
		nav_cmd_yaw = (int32_t)nav_cmd_yaw_f;
	}
	return false;
}

float getBoundedRandomNumber(float lower, float upper){
	float ratio = (float)rand()/RAND_MAX; // 0-1
	float number = lower + ratio * (upper - lower);

	return number;
}

