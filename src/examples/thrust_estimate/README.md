# Thrust Estimate Module

This module computes a thrust estimate of each rotor in real-time, based on measurements of current, voltage and RPM given by the ESC telemetry, and was implemented as part of the paper "A Generalized Thrust Estimation and Control Approach for Multirotors Micro Aerial Vehicles".

The thrust control is implemented in [this](https://github.com/LASER-Robotics/px4_firmware/blob/thrust_control_1.13.2/src/lib/mixer/MultirotorMixer/MultirotorMixer.cpp) file, in the *mix* function, line 321.

The configuration of both the thrust estimate and control are done by PX4 parameters.

PARAMETER  | DESCRIPTION
------------- | -------------
ROT_CTRL_TYPE     | Define the rotor control type, standard or thrust control
ROT_CTRL_THRUST_P | P gain for the PID control
ROT_CTRL_THRUST_I | I gain for the PID control
ROT_CTRL_THRUST_D | D gain for the PID control
ROT_CTRL_I_MAX    | Upper bound for the accumulated I
ROT_CTRL_PID_MAX  | Maximum output gain for the PID control
ROT_THRUST_MAX    | Maximum thrust measured in flight
ROT_PROP_SIZE     | Propeller size. Lenght from center to tip of the blade in cm
ROT_PROP_WEIGHT   | Propeller weight in grams

The PX4 firmware for this repository was build for a Pixhawk 6C using:

    make px4_fmu-v6c_default


## Reference for citing
