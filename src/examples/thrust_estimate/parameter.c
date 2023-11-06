/**
 * Thrust scale
 *
 * Factor that maps from estimated thrust to real thrust
 *
 * @decimal 10
 * @group Multicopter Thrust Estimate
 */
PARAM_DEFINE_FLOAT(ROT_THRUST_SCALE, 1.575164103f);

/**
 * Propeller size.
 *
 * @decimal 4
 * @group Multicopter Thrust Estimate
 */
PARAM_DEFINE_FLOAT(ROT_PROP_SIZE, 0.16f);

/**
 * Propeller weight.
 *
 * @decimal 4
 * @group Multicopter Thrust Estimate
 */
PARAM_DEFINE_FLOAT(ROT_PROP_WEIGHT, 0.014f);
