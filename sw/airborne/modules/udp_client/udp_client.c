/*
 * Copyright (C) kevindehecker
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/slamdunk_udp/slamdunk_udp.c"
 * @author kevindehecker
 * Communication to SLAMdunk over udp
 */

#include "udp_client.h"

#include "autopilot.h"
#include "pprzlink/pprz_transport.h"
#include "mcu_periph/udp.h"
#include "subsystems/abi.h"

#include "subsystems/datalink/telemetry.h"
#include "subsystems/datalink/downlink.h"

#include "generated/flight_plan.h"
#include "boards/bebop/actuators.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "state.h"

#include <time.h>

#include <stdio.h>

static struct server_t server = {
  .device = (&((SERVER_PORT).device)),
  .msg_available = false
};

static uint8_t mp_msg_buf[128]  __attribute__((aligned));   ///< The message buffer

clock_t starting_time = 0; // placeholder value
bool timer_running = false;

//int rpm0_prev = 0;
//int rpm1_prev = 0;
//int rpm2_prev = 0;
//int rpm3_prev = 0;

void udp_client_init(void) {
  printf("--- UDP COMMS init\n");
  pprz_transport_init(&server.transport);
  printf("--- UDP SETUP finished\n");

  char message[] = "Hello from bebop!";
  pprz_msg_send_PAYLOAD(&(server.transport.trans_tx), server.device,
    AC_ID, sizeof(message),  (unsigned char * ) &message);
}


void udp_client_periodic(void) {

  //r2k_package.rpm[0] = actuators_bebop.rpm_obs[0];
  //r2k_package.rpm[1] = actuators_bebop.rpm_obs[1];
  //r2k_package.rpm[2] = actuators_bebop.rpm_obs[2];
  //r2k_package.rpm[3] = actuators_bebop.rpm_obs[3];

  //bool motors_on = (actuators_bebop.rpm_obs[0] > 0
	//	  && actuators_bebop.rpm_obs[1] > 0
		//  && actuators_bebop.rpm_obs[2] > 0
		 // && actuators_bebop.rpm_obs[3] > 0);
  bool motors_on = autopilot_get_motors_on();

  // only send data if the motors are running
  if (motors_on){
	  if (!timer_running){
		  starting_time = clock(); // start timer
		  timer_running = true;
		  char message[5] = "START";
		  pprz_msg_send_PAYLOAD(&(server.transport.trans_tx), server.device,
			AC_ID, sizeof(message),  (unsigned char * ) &message);
	  }

	  double time_passed = (double)(clock() - starting_time)/CLOCKS_PER_SEC;

	  struct NedCoor_f *pos = stateGetPositionNed_f();
	  struct NedCoor_f *vel = stateGetSpeedNed_f();
	  struct NedCoor_f *acc = stateGetAccelNed_f();
	  struct FloatEulers *att = stateGetNedToBodyEulers_f();
	  struct FloatRates *rates = stateGetBodyRates_f();

	  // (time1, rpm4, cmd4, pos3, vel3, acc3, theta3, thetadot3) = 24
	  char messageBody[] = "%.6f,\
			  				%d,%d,%d,%d,\
			  	  	  	  	%d,%d,%d,%d,\
			  				%.3f,%.3f,%.3f\
			  				%.3f,%.3f,%.3f\
			  				%.3f,%.3f,%.3f\
			  				%.3f,%.3f,%.3f,\
			  				%.3f,%.3f,%.3f\n";
	  char message[220];
	  sprintf(message, messageBody, time_passed,
			  actuators_bebop.rpm_obs[0], actuators_bebop.rpm_obs[1],
			  actuators_bebop.rpm_obs[2], actuators_bebop.rpm_obs[3],
			  //actuators_bebop.rpm_obs[0]-rpm0_prev, actuators_bebop.rpm_obs[1]-rpm1_prev,
			  //actuators_bebop.rpm_obs[2]-rpm2_prev, actuators_bebop.rpm_obs[3]-rpm3_prev,
		      stabilization_cmd[COMMAND_THRUST], stabilization_cmd[COMMAND_ROLL],
		      stabilization_cmd[COMMAND_PITCH], stabilization_cmd[COMMAND_YAW],
			  pos->x, pos->y, pos->z,
			  vel->x, vel->y, vel->z,
			  acc->x, acc->y, acc->z,
			  att->phi, att->theta, att->psi,
			  rates->p, rates->q, rates->r);

	  pprz_msg_send_PAYLOAD(&(server.transport.trans_tx), server.device,
	    AC_ID, sizeof(message),  (unsigned char * ) &message);

	  //rpm0_prev = actuators_bebop.rpm_obs[0];
	  //rpm1_prev = actuators_bebop.rpm_obs[1];
	  //rpm2_prev = actuators_bebop.rpm_obs[2];
	  //rpm3_prev = actuators_bebop.rpm_obs[3];

  }

  // send a 'stop' message once the motors have been turned off again
  else{
	  if (timer_running){
		  char message[5] = "STOP";
		  pprz_msg_send_PAYLOAD(&(server.transport.trans_tx), server.device,
			AC_ID, sizeof(message),  (unsigned char * ) &message);

		 timer_running = false;
	  }
  }


}


/* Parse the message (not used atm) */
static inline void udp_client_parse_msg(void) {
  printf("Message received!\n");
  printf("%d %d %d %d %d %d ...\n", mp_msg_buf[0], mp_msg_buf[1], mp_msg_buf[2], mp_msg_buf[3], mp_msg_buf[4], mp_msg_buf[5]);

  /* Parse the kalamos message */
  uint8_t msg_id = mp_msg_buf[3]; // PPRZLINK v2

  switch (msg_id) {
    case PPRZ_MSG_ID_PAYLOAD:
      printf("PAYLOAD received!\n");
      printf("%s\n", &mp_msg_buf[5]);
      break;

    default:
      break;
  }
}


void udp_client_event() {
  // Check if we got some message from the Kalamos
  //printf("--- SALMDUNK event\n");
  pprz_check_and_parse(server.device, &server.transport, mp_msg_buf, &server.msg_available);

  // If we have a message we should parse it
  if (server.msg_available) {
    udp_client_parse_msg();
    server.msg_available = false;
  }
}

