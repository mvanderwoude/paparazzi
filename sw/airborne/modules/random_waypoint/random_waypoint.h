/*
 * random_waypoint.h
 *
 *  Created on: Jul 5, 2020
 *      Author: mark
 */

#ifndef RANDOM_WAYPOINT_H_
#define RANDOM_WAYPOINT_H_

// Module functions
extern void random_waypoint_init(void);
extern void random_waypoint_periodic(void);

// Waypoint bounds (ENU)
extern float wp_x_min;
extern float wp_x_max;
extern float wp_y_min;
extern float wp_y_max;
extern float wp_z_min;
extern float wp_z_max;

extern float wp_margin;

#endif /* RANDOM_WAYPOINT_H_ */
