/* hardware_test.c — Hardware test and interactive UI coordinator.
 *
 * This module acts as the thin top-level coordinator for:
 *   - ht_sensor.c: ADC sampling and temperature calculation
 *   - ht_heater.c: SSR time-proportioning, fan drive, and thermal runaway watchdog
 *   - ht_pid.c:    Closed-loop PID controller
 *   - ht_ui.c:     OLED user interface, buttons, and state machine
 */

#include "hardware_test.h"
#include "ht_private.h"

#include "adc.h"
#include "i2c.h"
#include "process_interlock.h"
#include "thermistor.h"

/* Coordinator-owned state */
volatile uint8_t hardware_initialized;

void HardwareTest_Init(void)
{
  hardware_initialized = 0U;

  ht_heater_init();

  (void)HAL_ADCEx_Calibration_Start(&hadc1);
  Thermistor_Init();
  ht_sensor_init();
  latest_adc = read_adc_average();
  latest_temperature_valid = Thermistor_Calculate(
      latest_adc, &latest_temperature_tenths, &latest_resistance_ohm);

  ht_ui_init();

  hardware_initialized = 1U;
}

void HardwareTest_InputRun(void)
{
  if (hardware_initialized == 0U)
  {
    return;
  }

  update_buttons();
  if ((ui_page == UI_CAL_EDIT)
      && (calibration_reference_tracks_measurement != 0U)
      && (latest_temperature_valid != 0U))
  {
    calibration_reference_tenths = latest_temperature_tenths;
  }
  update_controls();
  update_button_auto_repeat();
  update_pid_controller();
  check_thermal_runaway();
  enforce_direct_heater_safety();
  update_heater_output();
  ui_poll_process();
}

void HardwareTest_Run(void)
{
  uint16_t adc;

  if (hardware_initialized == 0U)
  {
    return;
  }

  if ((HAL_GetTick() - last_display_update) < DISPLAY_UPDATE_MS)
  {
    return;
  }
  last_display_update = HAL_GetTick();

  adc = read_adc_average();
  latest_adc = adc;
  latest_temperature_valid = Thermistor_Calculate(
      latest_adc, &latest_temperature_tenths, &latest_resistance_ohm);
  enforce_direct_heater_safety();
  if ((ui_page == UI_CAL_HEATER)
      && ((latest_temperature_valid == 0U)
          || (latest_temperature_tenths >= CALIBRATION_MAX_TEMP_TENTHS)))
  {
    heater_enabled = 0U;
    set_heater_output();
  }
  if (oled_ready != 0U)
  {
    SSD1306_Clear(&oled);
    ui_draw();

    if (SSD1306_Update(&oled) != HAL_OK)
    {
      oled_ready = 0U;
    }
  }
  else if (HAL_I2C_IsDeviceReady(&hi2c1, (OLED_ADDRESS_7BIT << 1U),
                                 1U, 20U) == HAL_OK)
  {
    oled_ready = (SSD1306_Init(&oled, &hi2c1, OLED_ADDRESS_7BIT) == HAL_OK)
                 ? 1U : 0U;
  }
}

void HardwareTest_GetStatus(HardwareTestStatus *status)
{
  if (status == NULL)
  {
    return;
  }

  status->screen = (HardwareTestScreen)current_screen;
  status->adc = latest_adc;
  status->temperature_tenths = latest_temperature_tenths;
  status->resistance_ohm = latest_resistance_ohm;
  status->pid_setpoint_tenths = pid_setpoint_tenths;
  status->temperature_valid = latest_temperature_valid;
  status->thermistor_calibrated = Thermistor_IsCalibrated();
  status->heater_enabled = heater_enabled;
  status->heater_duty_percent = heater_duty_percent;
  status->fan_duty_percent = fan_duty_percent;
  status->pid_running = pid_running;
  status->pid_fault = pid_fault;
  status->thermal_runaway_fault = thermal_runaway_fault;
}

uint8_t HardwareTest_PIDStartAt(int16_t setpoint_tenths)
{
  HardwareTest_PIDSetSetpoint(setpoint_tenths);
  return pid_start();
}

void HardwareTest_PIDSetSetpoint(int16_t setpoint_tenths)
{
  if (setpoint_tenths < PID_SETPOINT_MIN_TENTHS)
  {
    setpoint_tenths = PID_SETPOINT_MIN_TENTHS;
  }
  else if (setpoint_tenths > PID_SETPOINT_MAX_TENTHS)
  {
    setpoint_tenths = PID_SETPOINT_MAX_TENTHS;
  }
  pid_setpoint_tenths = setpoint_tenths;
}

void HardwareTest_PIDStop(void)
{
  pid_stop(0U);
}

void HardwareTest_FanSetDuty(uint8_t duty_percent)
{
  if ((duty_percent > 0U) && (heater_enabled != 0U))
  {
    heater_enabled = 0U;
    pid_output_percent = 0.0f;
    set_heater_output();
  }
  set_fan_output(duty_percent);
}

uint8_t HardwareTest_HeaterStartAtDuty(uint8_t duty,
                                       int16_t safety_limit_tenths)
{
  pid_stop(0U);
  if ((ProcessInterlock_GetOwner() == PROCESS_OWNER_CONVEYOR)
      || (duty == 0U) || (duty > 100U)
      || (safety_limit_tenths <= 0)
      || (latest_temperature_valid == 0U)
      || (Thermistor_IsCalibrated() == 0U)
      || (latest_temperature_tenths >= safety_limit_tenths))
  {
    HardwareTest_HeaterStop();
    return 0U;
  }

  thermal_runaway_fault = 0U;
  thermal_runaway_active = 0U;
  thermal_runaway_timer = HAL_GetTick();
  thermal_runaway_ref_tenths = latest_temperature_tenths;
  heater_duty_percent = duty;
  heater_safety_limit_tenths = safety_limit_tenths;
  heater_enabled = 1U;
  set_heater_output();
  return 1U;
}

void HardwareTest_HeaterStop(void)
{
  heater_enabled = 0U;
  heater_safety_limit_tenths = 0;
  thermal_runaway_active = 0U;
  set_heater_output();
}
