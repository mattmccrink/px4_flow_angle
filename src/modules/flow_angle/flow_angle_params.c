/**
 * Enable simulation (synthetic) mode
 *
 * When set, the driver publishes synthetic alpha/beta/airspeed instead of
 * reading hardware. Use this in SITL to validate the message, logging and
 * telemetry path with no sensors present. In milestone 2 this same flag lets
 * you regression-test the plumbing on hardware builds without air flow.
 *
 * @boolean
 * @group Flow Angle
 */
PARAM_DEFINE_INT32(FA_SIM_EN, 1);

/**
 * Publication / sampling rate
 *
 * @min 1.0
 * @max 200.0
 * @unit Hz
 * @group Flow Angle
 */
PARAM_DEFINE_FLOAT(FA_RATE, 50.0f);

/**
 * Dynamic-pressure validity threshold
 *
 * Below this q the flow angles are flagged invalid (they blow up as q -> 0).
 *
 * @min 0.0
 * @unit Pa
 * @group Flow Angle
 */
PARAM_DEFINE_FLOAT(FA_Q_MIN, 20.0f);
