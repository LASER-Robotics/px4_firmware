/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file thrust_control.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/esc_status.h>

__EXPORT int thrust_control_main(int argc, char *argv[]);

double c[] = {0.0724, 6.1490 * pow(10, -5), 0.2993, 1.2998 * pow(10,-8), 0};
double d[] = {4.2959, -1.7154 * pow(10, 5)};
double K_q[2] = {0.242, 0.0014};
double mass = 8.2 * pow(10, -3);
double radius = 11 * pow(10, -2);
double rho = 1.293;

double I_r;

double Delta = pow(10, -1);
int N = 20;
double epsilon = pow(10, -5);

double old_lambda_s_k;

double w, w_old, w_dot_hat;
clock_t begin, now;

/******************** NOTATION ********************/
// P_am_hat 	-> aerodynamic mechanical power input into the air
// C_P_am   	-> aerodynamic mechanical power coefficient computed base on the guess of lambda_s
// C_P_am_t	-> aerodynamic mechanical power coefficient measured at t
// i_hat    	-> estimated current given by esc
// w        	-> rotor RPM given by esc
// v        	-> estimated voltage given by esc
// lambda_i 	-> induced inflow ratio
// lambda_s 	-> stream inflow ratio
// lambda	-> vertical inflow ratio (lambda_i + lambda_s)
// Delta	-> small offset for stream inflow ratio
// I_r 		-> rotor moment of inertia

double compute_lambda_i(double lambda_s){
	double a = c[4];
	double b = c[4]*lambda_s + c[1];
	double _c = c[1]*(lambda_s - c[2]);
	double delta = b * b - 4 * a * _c;
	double root;
	if(delta >= 0){
		double root1 = (-b + sqrt(delta))/(2*a);
		double root2 = (-b - sqrt(delta))/(2*a);
		root = (root1 >= root2) ? root1 : root2;
	} else {
		root = -b / (2 * a);
	}
	return root;
}

double compute_C_T(double lambda_i, double lambda_s){
	double lambda = lambda_i + lambda_s;
	return c[1]*(c[2] - lambda);
}

double compute_kappa(double C_T){
	return d[0] + (d[1] * C_T);
}

double compute_C_P_am_hat(double lambda_i, double lambda_s, double C_T, double kappa){
	return c[3] + C_T * ((kappa * lambda_i) + lambda_s) * c[0];
}

// iterative algorithm to converge to the optimum lambda_s
double thrust_computation(double i_hat, double _w, double _w_dot_hat){
	double P_am_hat = (K_q[0] - ((K_q[1] * i_hat))) * i_hat * _w - (I_r * _w * _w_dot_hat);
	double C_P_am_t = P_am_hat / (i_hat * i_hat * i_hat);
	double lambda_s[N+1];
	double f[N+1];
	lambda_s[0] = old_lambda_s_k - Delta;
	int k;
	double C_T = 0;
	for(k = 0; k < N; k++){
		if(k == 1) {lambda_s[k] = old_lambda_s_k;}
		double lambda_i = compute_lambda_i(lambda_s[k]);
		C_T = compute_C_T(lambda_i, lambda_s[k]);
		double kappa = compute_kappa(C_T);
		double C_P_am_hat = compute_C_P_am_hat(lambda_i, lambda_s[k], C_T, kappa);
		f[k] = C_P_am_t - C_P_am_hat;
		if((k > 1) && (fabs(f[k] - f[k-1]) < epsilon)){break;}
		lambda_s[k+1] = lambda_s[k] - f[k]*((lambda_s[k] - lambda_s[k-1])/(f[k] - f[k-1]));
	}
	old_lambda_s_k = lambda_s[k];
	return C_T * _w * _w;
}

int thrust_control_main(int argc, char *argv[])
{
	begin = clock();
	I_r = mass * radius * radius;
	c[4] = 2 * rho * (double) M_PI_F * c[0] * c[0];
	PX4_INFO("Hello Sky!");

	/* subscribe to vehicle_acceleration topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(esc_status));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 1/200);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 100; i++) {

		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct esc_status_s esc;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(esc_status), sensor_sub_fd, &esc);

				w_old = w;
				now = clock();
				double time_elapsed = (double)(now - begin) / CLOCKS_PER_SEC;

				w = (double)esc.esc[0].esc_rpm;
				begin = clock();
				double i_hat = (double)esc.esc[0].esc_current;
				// double v_hat = (double)esc.esc[0].esc_voltage;

				w_dot_hat = (double)(w - w_old) / time_elapsed;

				double thrust = thrust_computation(i_hat, w, w_dot_hat);

				PX4_INFO("ESTIMATED THRUST IS: %8.4f, %8.4f",
						(double)thrust,
						time_elapsed);
			}
		}
	}

	PX4_INFO("exiting");

	return 0;
}
