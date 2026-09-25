/* ht_pid.c — Temperature PID controller with setpoint ramp, derivative
 *            filter, feed-forward, and conditional anti-windup.
 *
 * Owns: pid_setpoint_tenths, pid_running, pid_fault, pid_kp, pid_ki,
 *       pid_kd, pid_ramped_setpoint, pid_integral,
 *       pid_filtered_temperature, pid_previous_filtered_temperature,
 *       pid_output_percent, pid_p_term, pid_i_term, pid_d_term,
 *       pid_last_update.
 *
 * Called from: hardware_test.c (public PID API) and
 *              update_pid_controller() is called by HardwareTest_InputRun().
 */

#include "ht_private.h"

#include "process_interlock.h"
#include "thermistor.h"

/* ---- PID tuning and operating limits ------------------------------------ */
#define PID_CONTROL_INTERVAL_MS     250U
#define PID_SETPOINT_MIN_TENTHS     200
#define PID_SETPOINT_MAX_TENTHS    1400
#define PID_SAFETY_LIMIT_TENTHS    1500
#define PID_KP                      1.5f
#define PID_KI                      0.035f
#define PID_KD                     14.0f
#define PID_SETPOINT_RAMP_C_PER_S   0.45f   /* max ramp rate toward target */
#define PID_DERIVATIVE_FILTER_S     1.0f    /* EWM time constant for d-term */
#define PID_HEATER_RATE_PER_DUTY    0.0265f /* °C/s per duty-% from characterization */
#define PID_PREDICTION_HORIZON_S   12.0f    /* seconds ahead for predictive limit */
#define PID_MAX_HEATER_DUTY        50.0f    /* hard output ceiling (%) */
#define PID_INTEGRAL_MAX           35.0f    /* integrator windup clamp */

/* ---- State --------------------------------------------------------------- */
int16_t pid_setpoint_tenths = 700;
uint8_t pid_running;
uint8_t pid_fault;
uint32_t pid_last_update;
float   pid_kp = PID_KP;
float   pid_ki = PID_KI;
float   pid_kd = PID_KD;
float   pid_ramped_setpoint;
float   pid_integral;
float   pid_filtered_temperature;
float   pid_previous_filtered_temperature;
float   pid_output_percent;
float   pid_p_term;
float   pid_i_term;
float   pid_d_term;

/* ---- clamp_float() ------------------------------------------------------- */
float clamp_float(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

/* ---- pid_apply_output() -------------------------------------------------- */
/* Converts a 0-100 % float output to heater state and duty register. */
void pid_apply_output(float output_percent)
{
  pid_output_percent  = output_percent;
  heater_duty_percent = (uint8_t)(output_percent + 0.5f);
  heater_enabled      = (heater_duty_percent > 0U) ? 1U : 0U;
  set_heater_output();
}

/* ---- pid_stop() ---------------------------------------------------------- */
/* Stops the PID controller.  Pass fault=1 to latch a fault condition. */
void pid_stop(uint8_t fault)
{
  pid_running            = 0U;
  thermal_runaway_active = 0U;
  if (fault != 0U)
  {
    pid_fault = 1U;
    set_fan_output(100U);
  }
  pid_apply_output(0.0f);
}

/* ---- pid_start() --------------------------------------------------------- */
/* Initialises and starts the PID controller from the current temperature.
 * Returns 1 on success, 0 if preconditions are not met. */
uint8_t pid_start(void)
{
  float measurement;

  if ((ProcessInterlock_GetOwner() == PROCESS_OWNER_CONVEYOR)
      || (latest_temperature_valid == 0U)
      || (Thermistor_IsCalibrated() == 0U)
      || (latest_temperature_tenths >= PID_SAFETY_LIMIT_TENTHS))
  {
    pid_stop(1U);
    return 0U;
  }

  measurement = (float)latest_temperature_tenths / 10.0f;

  /* Clear faults and reset all thermal runaway state. */
  pid_fault              = 0U;
  thermal_runaway_fault  = 0U;
  thermal_runaway_active = 0U;
  thermal_runaway_timer  = HAL_GetTick();
  thermal_runaway_ref_tenths = latest_temperature_tenths;

  pid_running                      = 1U;
  pid_integral                     = 0.0f;
  pid_ramped_setpoint              = measurement;
  pid_filtered_temperature         = measurement;
  pid_previous_filtered_temperature = measurement;
  pid_last_update                  = HAL_GetTick();
  pid_apply_output(0.0f);
  set_fan_output(0U);
  return 1U;
}

/* ---- update_pid_controller() --------------------------------------------- */
/* Main PID tick: called every 10 ms from HardwareTest_InputRun().  Internally
 * rate-limited to PID_CONTROL_INTERVAL_MS (250 ms) so floating-point work
 * on the Cortex-M3 only runs 4 times per second.
 *
 * Algorithm: setpoint ramp → EWM derivative filter → feed-forward
 *   → PID sum → predictive output limit → conditional integration
 *   → fan braking stage. */
void update_pid_controller(void)
{
  uint32_t now;
  uint32_t elapsed_ms;
  float dt;
  float measurement;
  float requested_setpoint;
  float previous_ramped_setpoint;
  float setpoint_rate;
  float ramp_step;
  float filter_alpha;
  float temperature_rate;
  float predicted_temperature;
  float prediction_error;
  float error;
  float feedforward_output;
  float output_limit = PID_MAX_HEATER_DUTY;
  float candidate_integral;
  float unclamped_output;
  float commanded_output;
  uint8_t requested_fan_duty = 0U;

  if (pid_running == 0U) return;

  if ((latest_temperature_valid == 0U)
      || (Thermistor_IsCalibrated() == 0U)
      || (latest_temperature_tenths >= PID_SAFETY_LIMIT_TENTHS))
  {
    pid_stop(1U);
    return;
  }

  now        = HAL_GetTick();
  elapsed_ms = now - pid_last_update;
  if (elapsed_ms < PID_CONTROL_INTERVAL_MS) return;
  pid_last_update = now;

  dt = (float)elapsed_ms / 1000.0f;
  if (dt > 1.0f) dt = 1.0f;

  /* --- Setpoint ramp ----------------------------------------------------- */
  measurement              = (float)latest_temperature_tenths / 10.0f;
  requested_setpoint       = (float)pid_setpoint_tenths / 10.0f;
  previous_ramped_setpoint = pid_ramped_setpoint;
  ramp_step                = PID_SETPOINT_RAMP_C_PER_S * dt;

  if (pid_ramped_setpoint < requested_setpoint)
  {
    pid_ramped_setpoint += ramp_step;
    if (pid_ramped_setpoint > requested_setpoint)
      pid_ramped_setpoint = requested_setpoint;
  }
  else if (pid_ramped_setpoint > requested_setpoint)
  {
    pid_ramped_setpoint -= ramp_step;
    if (pid_ramped_setpoint < requested_setpoint)
      pid_ramped_setpoint = requested_setpoint;
  }
  setpoint_rate = (pid_ramped_setpoint - previous_ramped_setpoint) / dt;

  /* --- EWM derivative filter -------------------------------------------- */
  filter_alpha              = dt / (PID_DERIVATIVE_FILTER_S + dt);
  pid_filtered_temperature += filter_alpha
                              * (measurement - pid_filtered_temperature);
  temperature_rate = (pid_filtered_temperature
                      - pid_previous_filtered_temperature) / dt;
  pid_previous_filtered_temperature = pid_filtered_temperature;

  /* --- Error and predictive temperature ---------------------------------- */
  error                = pid_ramped_setpoint - measurement;
  predicted_temperature = measurement;
  if (temperature_rate > 0.0f)
    predicted_temperature += temperature_rate * PID_PREDICTION_HORIZON_S;
  prediction_error = requested_setpoint - predicted_temperature;

  pid_p_term = pid_kp * error;
  pid_d_term = -pid_kd * temperature_rate;

  /* The characterization runs give approximately 0.0265 °C/s for every
   * percent of heater duty. Feed-forward the requested ramp and let the
   * PID terms correct losses and model error. */
  feedforward_output = (setpoint_rate > 0.0f)
                       ? (setpoint_rate / PID_HEATER_RATE_PER_DUTY)
                       : 0.0f;

  /* The measured plant continues heating for roughly 17-21 s after cutoff.
   * Taper against predicted temperature instead of present error. */
  if      (prediction_error <= 0.0f)  output_limit = 0.0f;
  else if (prediction_error <  3.0f)  output_limit = 15.0f;
  else if (prediction_error <  8.0f)  output_limit = 30.0f;
  else if (prediction_error < 15.0f)  output_limit = 45.0f;

  candidate_integral = clamp_float(pid_integral + (pid_ki * error * dt),
                                   0.0f, PID_INTEGRAL_MAX);
  unclamped_output   = feedforward_output + pid_p_term
                       + candidate_integral + pid_d_term;

  /* Conditional integration prevents windup while output is saturated. */
  if (((unclamped_output > 0.0f) && (unclamped_output < output_limit))
      || ((unclamped_output >= output_limit) && (error < 0.0f))
      || ((unclamped_output <= 0.0f)         && (error > 0.0f)))
  {
    pid_integral = candidate_integral;
  }
  pid_i_term = pid_integral;

  commanded_output = clamp_float(feedforward_output + pid_p_term
                                 + pid_i_term + pid_d_term,
                                 0.0f, output_limit);

  /* --- Fan braking stage ------------------------------------------------- */
  /* Fan and heater are never commanded simultaneously. */
  if ((measurement >= (requested_setpoint + 1.0f))
      || (predicted_temperature >= (requested_setpoint + 4.0f)))
    requested_fan_duty = 100U;
  else if (predicted_temperature >= (requested_setpoint + 2.0f))
    requested_fan_duty = 70U;
  else if ((measurement >= (requested_setpoint + 0.3f))
           && (temperature_rate > 0.1f))
    requested_fan_duty = 50U;

  if ((measurement >= (requested_setpoint + 0.3f)) || (requested_fan_duty != 0U))
  {
    commanded_output = 0.0f;
    pid_integral    *= 0.98f;   /* gentle integral drain during overshoot */
  }

  /* Rate-limit upward output steps to avoid thermal shock. */
  if (commanded_output > (pid_output_percent + 10.0f))
    commanded_output = pid_output_percent + 10.0f;

  if (commanded_output > 0.0f) set_fan_output(0U);
  pid_apply_output(commanded_output);
  if (commanded_output <= 0.0f) set_fan_output(requested_fan_duty);
}
