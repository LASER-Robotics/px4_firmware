
/**
 * Multicopter air-mode
 *
 * The air-mode enables the mixer to increase the total thrust of the multirotor
 * in order to keep attitude and rate control even at low and high throttle.
 *
 * This function should be disabled during tuning as it will help the controller
 * to diverge if the closed-loop is unstable (i.e. the vehicle is not tuned yet).
 *
 * Enabling air-mode for yaw requires the use of an arming switch.
 *
 * @value 0 Disabled
 * @value 1 Roll/Pitch
 * @value 2 Roll/Pitch/Yaw
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(MC_AIRMODE, 0);

/**
 * Motor Ordering
 *
 * Determines the motor ordering. This can be used for example in combination with
 * a 4-in-1 ESC that assumes a motor ordering which is different from PX4.
 *
 * ONLY supported for Quads.
 *
 * When changing this, make sure to test the motor response without props first.
 *
 * @value 0 PX4
 * @value 1 Betaflight / Cleanflight
 *
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(MOT_ORDERING, 0);

/**
 * Rotor control
 *
 * Determines the rotor control type.
 *
 *
 * @value 0 Simple
 * @value 1 Thrust Control
 *
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(ROT_CTRL_TYPE, 0);

/**
 * Rotor thrust control P gain.
 *
 * @decimal 5
 *
 * @group Mixer Output
 */
PARAM_DEFINE_FLOAT(ROT_CTRL_THRST_P, 1.0f);

/**
 * Rotor thrust control I gain
 *
 * @decimal 5
 *
 * @group Mixer Output
 */
PARAM_DEFINE_FLOAT(ROT_CTRL_THRST_I, 0.005f);

/**
 * Rotor thrust control D gain
 *
 * @decimal 5
 *
 * @group Mixer Output
 */
PARAM_DEFINE_FLOAT(ROT_CTRL_THRST_D, 0.005f);

/**
 * Rotor thrust control FF gain
 *
 * @decimal 5
 *
 * @group Mixer Output
 */
PARAM_DEFINE_FLOAT(ROT_CTRL_THRST_F, 0.0f);

/**
 * Rotor control PID max output
 *
 * @decimal 2
 *
 * @group Mixer Output
 */
PARAM_DEFINE_FLOAT(ROT_CTRL_PID_MAX, 1.0f);

/**
 * Rotor control I max output
 *
 * @decimal 2
 *
 * @group Mixer Output
 */
PARAM_DEFINE_FLOAT(ROT_CTRL_I_MAX, 0.1f);

/**
 * Rotor thrust max value
 *
 * Max thrust. Used as an uper bound for the thrust estimate to real mapping.
 *
 * @decimal 2
 *
 * @group Mixer Output
 */
PARAM_DEFINE_FLOAT(ROT_THRUST_MAX, 1000.0f);
