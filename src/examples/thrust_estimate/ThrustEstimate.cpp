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
 * @file ThrustEstimate.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include "ThrustEstimate.hpp"

ThrustEstimate::ThrustEstimate() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	c[0] = 0.0724;
	c[1] = 6.1490 * pow(10, -5);
	c[2] = 0.2993;
	c[3] = 1.2998 * pow(10,-8);
	c[4] = 0;

	d[0] = 4.2959;
	d[1] = -1.7154 * pow(10, 5);

	K_q[0] = 0.242;
	K_q[1] = 0.0014;
	mass = 8.2 * pow(10, -3);
	radius = 11 * pow(10, -2);
	rho = 1.293;
	Delta = pow(10, -1);
	N = 20;
	epsilon = pow(10, -5);
	initialize_parameters();
}

ThrustEstimate::~ThrustEstimate()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool ThrustEstimate::init()
{
	// execute Run() on every sensor_accel publication
	if (!_esc_status_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void ThrustEstimate::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_esc_status_sub.updated()) {
		// clear update
		esc_status_s esc;
		_esc_status_sub.copy(&esc);

		for(int j = 0; j < 4; j++){
			old_thrust[j] = thrust[j];
			w_old[j] = w[j];
			now[j] = clock();
			double time_elapsed = (double)(now[j] - begin[j]) / CLOCKS_PER_SEC;

			w[j] = (double)esc.esc[0].esc_rpm;
			begin[j] = clock();
			i_hat[j] = (double)esc.esc[0].esc_current;
			// double v_hat = (double)esc.esc[0].esc_voltage;

			w_dot_hat[j] = (double)(w[j] - w_old[j]) / time_elapsed;

			thrust[j] = thrust_computation(i_hat[j], w[j], w_dot_hat[j], j);
		}
	}

	// Example
	//  publish some data
	vehicle_thrust_estimate_s data;
	data.timestamp = hrt_absolute_time();
	data.thrust[0] = thrust[0];
	data.thrust[1] = thrust[1];
	data.thrust[2] = thrust[2];
	data.thrust[3] = thrust[3];
	_thrust_estimate_pub.publish(data);

	perf_end(_loop_perf);
}

int ThrustEstimate::task_spawn(int argc, char *argv[])
{
	ThrustEstimate *instance = new ThrustEstimate();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ThrustEstimate::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int ThrustEstimate::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ThrustEstimate::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running thurst estimate.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("thrust_estimate", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

double ThrustEstimate::compute_lambda_i(double lambda_s){
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

double ThrustEstimate::compute_C_T(double lambda_i, double lambda_s){
	double lambda = lambda_i + lambda_s;
	return c[1]*(c[2] - lambda);
}

double ThrustEstimate::compute_kappa(double C_T){
	return d[0] + (d[1] * C_T);
}

double ThrustEstimate::compute_C_P_am_hat(double lambda_i, double lambda_s, double C_T, double kappa){
	return c[3] + C_T * ((kappa * lambda_i) + lambda_s) * c[0];
}

// iterative algorithm to converge to the optimum lambda_s
double ThrustEstimate::thrust_computation(double _i_hat, double _w, double _w_dot_hat, int index){
	double P_am_hat = (K_q[0] - ((K_q[1] * _i_hat))) * _i_hat * _w - (I_r * _w * _w_dot_hat);
	double C_P_am_t = P_am_hat / (_i_hat * _i_hat * _i_hat);
	double lambda_s[N+1];
	double f[N+1];
	if (isnan(fabs(old_lambda_s_k[index]))){
		old_lambda_s_k[index] = 0;
	}
	lambda_s[0] = old_lambda_s_k[index] - Delta;
	int k;
	double C_T = 0;
	for(k = 0; k < N; k++){
		if(k == 1) {lambda_s[k] = old_lambda_s_k[index];}
		double lambda_i = compute_lambda_i(lambda_s[k]);
		C_T = compute_C_T(lambda_i, lambda_s[k]);
		double kappa = compute_kappa(C_T);
		double C_P_am_hat = compute_C_P_am_hat(lambda_i, lambda_s[k], C_T, kappa);
		f[k] = C_P_am_t - C_P_am_hat;
		if((k > 1) && (fabs(f[k] - f[k-1]) < epsilon)){break;}
		lambda_s[k+1] = lambda_s[k] - f[k]*((lambda_s[k] - lambda_s[k-1])/(f[k] - f[k-1]));
	}
	old_lambda_s_k[index] = lambda_s[k];
	double _thrust = C_T * _w * _w;
	if ((isnan(fabs(_thrust))) || (_thrust < -100000)){
		_thrust = old_thrust[index];
	}
	return _thrust;
}

void ThrustEstimate::initialize_parameters(void){
	for(int i = 0; i < 4; i++){
		begin[i] = clock();
	}
	I_r = mass * radius * radius;
	c[4] = 2 * rho * (double) M_PI_F * c[0] * c[0];
}

extern "C" __EXPORT int thrust_estimate_main(int argc, char *argv[])
{
	return ThrustEstimate::main(argc, argv);
}
