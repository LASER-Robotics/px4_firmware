/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * @file send_cmd_to_motor.c
 * Application to get data for NN training for control allocation
 *
 * @author <davihenriqueds@gmail.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <poll.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>

__EXPORT int send_cmd_to_motor_main(int argc, char *argv[]);

int send_cmd_to_motor_main(int argc, char *argv[])
{
    struct actuator_controls_s cmd;
	memset(&cmd, 0, sizeof(cmd));
	orb_advert_t cmd_pub = orb_advertise(ORB_ID(actuator_controls_0), &cmd);

    /* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
    */
    cmd.control[0] = 0.0;
    cmd.control[1] = 0.0;
    cmd.control[2] = 0.0;
    cmd.control[3] = 1.0;

    time_t start, end;
    double elapsed;
    double duration = 3.0;
    double control_step = 0.25;

    for(int i=1; i < 5; i++){
        start = time(NULL);
        int terminate = 1;
        cmd.control[3] = 0.45;
        cmd.control[2] = i*control_step;
        PX4_INFO("Executing commands: %.6f %.6f %.6f %.6f", (double)cmd.control[0], (double)cmd.control[1], (double)cmd.control[2], (double)cmd.control[3]);
        while (terminate){
            end = time(NULL);
            elapsed = difftime(end, start);
            if(elapsed >= duration){
                terminate = 0;
            } else {
                orb_publish(ORB_ID(actuator_controls_0), cmd_pub, &cmd);
            }
        }
        PX4_INFO("Command ended");
    }
	return OK;
}