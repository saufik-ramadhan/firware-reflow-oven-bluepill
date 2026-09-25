/* ht_heater.c — SSR heater output, fan control, direct-heater safety, and
 *               thermal runaway watchdog.
 *
 * Owns: heater_enabled, heater_duty_percent, heater_manual_duty_percent,
 *       fan_duty_percent, heater_window_active, heater_window_started_at,
 *       heater_on_time_ms, heater_safety_limit_tenths,
 *       thermal_runaway_fault, thermal_runaway_active,
 *       thermal_runaway_timer, thermal_runaway_ref_tenths.
 *
 * Called from: hardware_test.c (InputRun / Run / public API).
 *              ht_pid.c calls set_heater_output() and set_fan_output().
 */

#include "ht_private.h"

#include "gpio.h"
#include "main.h"
#include "tim.h"

/* ---- Heater configuration ----------------------------------------------- */
#define FAN_PWM_CHANNEL          TIM_CHANNEL_2
#define HEATER_WINDOW_MS         1000U
#define HEATER_DEFAULT_DUTY        50U
#define MANUAL_HEATER_LIMIT_TENTHS 1100

/* ---- Thermal runaway watchdog configuration ------------------------------ */
/* Watch window: temperature must rise at least MIN_RISE_TENTHS within this
 * period while the heater is commanding >= MIN_DUTY percent. */
#define THERMAL_RUNAWAY_WATCH_PERIOD_MS      30000UL
#define THERMAL_RUNAWAY_MIN_DUTY               20U
#define THERMAL_RUNAWAY_MIN_RISE_TENTHS         12   /* 1.2 °C */
#define THERMAL_RUNAWAY_MAX_DROP_TENTHS         30   /* 3.0 °C drop → instant fault */
#define THERMAL_RUNAWAY_TARGET_MARGIN_TENTHS    20   /* within 2 °C of setpoint */
#define THERMAL_RUNAWAY_HYSTERESIS_TENTHS       80   /* 8 °C above setpoint before re-checking */

/* ---- State --------------------------------------------------------------- */
uint8_t          heater_enabled;
uint8_t          heater_duty_percent = HEATER_DEFAULT_DUTY;
uint8_t          heater_manual_duty_percent = HEATER_DEFAULT_DUTY;
uint8_t          fan_duty_percent;
uint8_t          heater_window_active;
uint32_t         heater_window_started_at;
uint32_t         heater_on_time_ms;
volatile int16_t heater_safety_limit_tenths;
uint8_t          thermal_runaway_fault;
uint8_t          thermal_runaway_active;
uint32_t         thermal_runaway_timer;
int16_t          thermal_runaway_ref_tenths;

/* ---- ht_heater_init() ---------------------------------------------------- */
void ht_heater_init(void)
{
  heater_safety_limit_tenths = 0;
  fan_duty_percent            = 0U;
  heater_window_active        = 0U;
  heater_window_started_at    = HAL_GetTick();
  heater_on_time_ms           = 0U;
  thermal_runaway_fault       = 0U;
  thermal_runaway_active      = 0U;

  /* PA6 drives the SSR in a software time-proportioning window. TIM3 is
   * dedicated to the 25 kHz four-wire fan control signal on PA7. */
  HAL_GPIO_WritePin(PWM_HEATER_GPIO_Port, PWM_HEATER_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim3, FAN_PWM_CHANNEL, 0U);
  if (HAL_TIM_PWM_Start(&htim3, FAN_PWM_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ---- set_heater_output() ------------------------------------------------- */
/* Writes the current heater state (enabled + duty) to the SSR GPIO immediately.
 * Must only be called from the inputTask context. */
void set_heater_output(void)
{
  GPIO_PinState pin = (heater_enabled != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(PWM_HEATER_GPIO_Port, PWM_HEATER_Pin, pin);
}

/* ---- set_fan_output() ---------------------------------------------------- */
/* Sets the fan PWM compare register.  0 = off, 100 = full speed. */
void set_fan_output(uint8_t duty_percent)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);
  fan_duty_percent = duty_percent;
  __HAL_TIM_SET_COMPARE(&htim3, FAN_PWM_CHANNEL,
                         (period * duty_percent) / 100U);
}

/* ---- update_heater_output() ---------------------------------------------- */
/* Software 1-second time-proportioning window. Called every 10 ms from
 * inputTask; drives the SSR on/off to achieve the commanded duty cycle. */
void update_heater_output(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t window_elapsed;

  if (heater_enabled == 0U)
  {
    HAL_GPIO_WritePin(PWM_HEATER_GPIO_Port, PWM_HEATER_Pin, GPIO_PIN_RESET);
    heater_window_active = 0U;
    return;
  }

  if (heater_window_active == 0U)
  {
    heater_window_active    = 1U;
    heater_window_started_at = now;
    heater_on_time_ms       = (HEATER_WINDOW_MS * heater_duty_percent) / 100U;
    HAL_GPIO_WritePin(PWM_HEATER_GPIO_Port, PWM_HEATER_Pin, GPIO_PIN_SET);
    return;
  }

  window_elapsed = now - heater_window_started_at;

  if (window_elapsed >= HEATER_WINDOW_MS)
  {
    /* Start a new window. */
    heater_window_started_at = now;
    heater_on_time_ms        = (HEATER_WINDOW_MS * heater_duty_percent) / 100U;
    HAL_GPIO_WritePin(PWM_HEATER_GPIO_Port, PWM_HEATER_Pin, GPIO_PIN_SET);
  }
  else if (window_elapsed >= heater_on_time_ms)
  {
    HAL_GPIO_WritePin(PWM_HEATER_GPIO_Port, PWM_HEATER_Pin, GPIO_PIN_RESET);
  }
}

/* ---- enforce_direct_heater_safety() -------------------------------------- */
/* Hard safety check called from both the 10 ms inputTask and the 50 ms
 * defaultTask.  Cuts the heater if the temperature sensor is invalid, the
 * safety limit is exceeded, or a thermal runaway fault has been latched. */
void enforce_direct_heater_safety(void)
{
  if (thermal_runaway_fault != 0U)
  {
    heater_enabled = 0U;
    set_heater_output();
    return;
  }

  if (heater_enabled == 0U) return;
  if (heater_safety_limit_tenths <= 0) return;

  if ((latest_temperature_valid == 0U)
      || (latest_temperature_tenths >= heater_safety_limit_tenths))
  {
    heater_enabled            = 0U;
    heater_safety_limit_tenths = 0;
    set_heater_output();
  }
}

/* ---- trigger_thermal_runaway_fault() ------------------------------------- */
/* Emergency shutoff: latch the fault flag, cut heater power, spin fan to
 * maximum, and stop the PID controller if it was running. */
void trigger_thermal_runaway_fault(void)
{
  thermal_runaway_fault       = 1U;
  thermal_runaway_active      = 0U;
  heater_enabled              = 0U;
  heater_duty_percent         = 0U;
  heater_safety_limit_tenths  = 0;
  set_heater_output();
  if (pid_running != 0U)
  {
    pid_stop(1U);
  }
  else
  {
    set_fan_output(100U);
  }
}

/* ---- check_thermal_runaway() --------------------------------------------- */
/* Watchdog called every 10 ms from HardwareTest_InputRun().
 *
 * Logic summary:
 *  - Only active when heater duty >= THERMAL_RUNAWAY_MIN_DUTY.
 *  - In PID mode: temperature must rise MIN_RISE_TENTHS within
 *    WATCH_PERIOD_MS while the error is still above TARGET_MARGIN. An
 *    unexpected drop of MAX_DROP_TENTHS triggers an immediate fault.
 *  - In direct-heater mode: same rise-or-fault logic; timer resets
 *    automatically when temperature is near the safety cutoff (expected
 *    plateau) so normal steady-state does not false-trigger.
 */
void check_thermal_runaway(void)
{
  uint32_t now;
  uint8_t  heating;
  int16_t  current_temp;

  /* A latched fault keeps the heater off on every tick. */
  if (thermal_runaway_fault != 0U)
  {
    heater_enabled = 0U;
    set_heater_output();
    return;
  }

  /* Need a valid temperature reading to make a decision. */
  if (latest_temperature_valid == 0U) return;

  heating = ((heater_enabled != 0U)
             && (heater_duty_percent >= THERMAL_RUNAWAY_MIN_DUTY)) ? 1U : 0U;

  if (heating == 0U)
  {
    thermal_runaway_active = 0U;
    return;
  }

  now          = HAL_GetTick();
  current_temp = latest_temperature_tenths;

  /* First tick at significant duty: latch baseline. */
  if (thermal_runaway_active == 0U)
  {
    thermal_runaway_active    = 1U;
    thermal_runaway_timer     = now;
    thermal_runaway_ref_tenths = current_temp;
    return;
  }

  if (pid_running != 0U)
  {
    /* PID mode: check that temperature is rising toward setpoint. */
    int16_t error_tenths = pid_setpoint_tenths - current_temp;

    if (error_tenths >= THERMAL_RUNAWAY_TARGET_MARGIN_TENTHS)
    {
      /* We are still noticeably below target — heater should be heating. */
      if (current_temp < (thermal_runaway_ref_tenths - THERMAL_RUNAWAY_MAX_DROP_TENTHS))
      {
        /* Significant drop while heater is on: broken SSR / detached NTC. */
        trigger_thermal_runaway_fault();
        return;
      }

      if ((current_temp - thermal_runaway_ref_tenths) >= THERMAL_RUNAWAY_MIN_RISE_TENTHS)
      {
        /* Good rise detected — reset the watchdog window. */
        thermal_runaway_ref_tenths = current_temp;
        thermal_runaway_timer      = now;
      }
      else if ((now - thermal_runaway_timer) >= THERMAL_RUNAWAY_WATCH_PERIOD_MS)
      {
        /* No rise within the allowed window. */
        trigger_thermal_runaway_fault();
        return;
      }
    }
    else
    {
      /* Near or above setpoint. Allow transient overshoot but watch for
       * persistent large error with heater still on hard. */
      if ((error_tenths >= THERMAL_RUNAWAY_HYSTERESIS_TENTHS)
          && (heater_duty_percent >= 30U))
      {
        if ((now - thermal_runaway_timer) >= THERMAL_RUNAWAY_WATCH_PERIOD_MS)
        {
          trigger_thermal_runaway_fault();
          return;
        }
      }
      else
      {
        /* Temperature is where it should be — reset baseline. */
        thermal_runaway_ref_tenths = current_temp;
        thermal_runaway_timer      = now;
      }
    }
  }
  else
  {
    /* Direct-heater mode: expected plateau near the safety cutoff temperature.
     * Reset the timer near the cutoff so a stable plateau does not fault. */
    if ((heater_safety_limit_tenths > 0)
        && (current_temp >= (heater_safety_limit_tenths - 20)))
    {
      thermal_runaway_ref_tenths = current_temp;
      thermal_runaway_timer      = now;
    }
    else
    {
      if (current_temp < (thermal_runaway_ref_tenths - THERMAL_RUNAWAY_MAX_DROP_TENTHS))
      {
        trigger_thermal_runaway_fault();
        return;
      }

      if ((current_temp - thermal_runaway_ref_tenths) >= THERMAL_RUNAWAY_MIN_RISE_TENTHS)
      {
        thermal_runaway_ref_tenths = current_temp;
        thermal_runaway_timer      = now;
      }
      else if ((now - thermal_runaway_timer) >= THERMAL_RUNAWAY_WATCH_PERIOD_MS)
      {
        trigger_thermal_runaway_fault();
        return;
      }
    }
  }
}
