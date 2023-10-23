/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file mc_thrust_control_params.c
 *
 * Parameters defined by the position control task for ground rovers
 *
 * This is a modification of the fixed wing params and it is designed for ground rovers.
 * It has been developed starting from the fw  module, simplified and improved with dedicated items.
 *
 * All the ackowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

/**
 * Speed proportional gain
 *
 * This is the proportional gain for the speed closed loop controller
 *
 * @unit %m/s
 * @min 0.005
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(THRUST_SPEED_P, 2.0f);

/**
 * Speed Integral gain
 *
 * This is the integral gain for the speed closed loop controller
 *
 * @unit %m/s
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(THRUST_SPEED_I, 3.0f);

/**
 * Speed proportional gain
 *
 * This is the derivative gain for the speed closed loop controller
 *
 * @unit %m/s
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(THRUST_SPEED_D, 0.001f);

/**
 * Speed integral maximum value
 *
 * This is the maxim value the integral can reach to prevent wind-up.
 *
 * @unit %m/s
 * @min 0.005
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(THRUST_SPEED_IMAX, 1.0f);
