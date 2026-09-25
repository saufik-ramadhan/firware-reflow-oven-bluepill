/* ht_ui.c — OLED user-interface: enumerations, button state machine, screen
 *           drawing, menu navigation, and process-event polling.
 *
 * Owns: oled, oled_ready, last_display_update, current_screen, ui_page,
 *       all ui_* state variables, ButtonState array, motor/servo test state.
 *
 * Called from: hardware_test.c (HardwareTest_InputRun / HardwareTest_Run).
 */

#include "ht_private.h"

#include "conveyor_app.h"
#include "conveyor_config.h"
#include "conveyor_sequencer.h"
#include "gpio.h"
#include "hardware_test.h"
#include "heater_characterization.h"
#include "i2c.h"
#include "inspection.h"
#include "process_interlock.h"
#include "reflow.h"
#include "ssd1306.h"
#include "task_stack_report.h"
#include "text_format.h"
#include "thermistor.h"

#include <string.h>

/* ---- UI configuration --------------------------------------------------- */
#define OLED_ADDRESS_7BIT        0x3cU
#define BUTTON_DEBOUNCE_SAMPLES  2U
#define DISPLAY_UPDATE_MS        50U
#define REFERENCE_REPEAT_DELAY_MS 400U
#define REFERENCE_REPEAT_RATE_MS   50U
#define CALIBRATION_MAX_TEMP_TENTHS 1500
#define ADC_FULL_SCALE           4095U
#define ADC_REFERENCE_MV         3300U
#define HEATER_DUTY_STEP           25U
#define MOTOR_TEST_DEFAULT_DUTY    40U
#define MOTOR_TEST_MIN_DUTY        30U
#define MANUAL_HEATER_LIMIT_TENTHS 1100

/* Servo pulse widths come from the conveyor configuration. */
static const uint16_t servo_test_pulses_us[3] = {
  CONVEYOR_SERVO_LEFT_US,
  CONVEYOR_SERVO_CENTER_US,
  CONVEYOR_SERVO_RIGHT_US
};

/* ---- Button debounce state ----------------------------------------------- */
typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
  uint8_t       raw_pressed;
  uint8_t       stable_pressed;
  uint8_t       stable_samples;
  uint16_t      press_count;
  uint32_t      pressed_at;
  uint32_t      last_repeat_at;
} ButtonState;

/* Forward declarations of UI event handlers (used before their definition). */
static void ui_button(uint8_t button);
static void ui_adjust(int8_t direction);
static void ui_line(uint8_t row, const char *value);

/* ---- State owned by this module ----------------------------------------- */
SSD1306_HandleTypeDef oled;
uint8_t  oled_ready;
uint32_t last_display_update;
Screen   current_screen;
UiPage   ui_page;

static UiPage    ui_return_page;
static UiPage    ui_stopped_from;
static UiPage    ui_fault_from;
static UiMessage ui_message;
static uint8_t   ui_finished_e2e;
static uint8_t   ui_home_selection;
static uint8_t   ui_e2e_profile;
static uint8_t   ui_profile_selection;
static uint8_t   ui_cal_selection;
static uint8_t   ui_pid_selection;
static uint8_t   ui_diag_selection;
static uint8_t   ui_stack_selection;
static uint8_t   ui_edit_selection;
static int32_t   ui_edit_value;
static uint8_t   ui_detail;
static uint8_t   ui_run_seen;
static uint8_t   ui_diag_start_pending;
static uint8_t   ui_d_press_active;
static uint8_t   ui_d_long_handled;
static uint32_t  ui_d_pressed_at;
static uint32_t  ui_message_at;
static uint32_t  ui_selection_at;

static uint8_t motor_test_duty_percent = MOTOR_TEST_DEFAULT_DUTY;
static uint8_t servo_test_selection    = 1U;

static ButtonState buttons[4] = {
  {BTN_A_GPIO_Port, BTN_A_Pin, 0U, 0U, 0U, 0U, 0U, 0U},
  {BTN_B_GPIO_Port, BTN_B_Pin, 0U, 0U, 0U, 0U, 0U, 0U},
  {BTN_C_GPIO_Port, BTN_C_Pin, 0U, 0U, 0U, 0U, 0U, 0U},
  {BTN_D_GPIO_Port, BTN_D_Pin, 0U, 0U, 0U, 0U, 0U, 0U}
};

static uint16_t handled_press_count[4];

/* =========================================================================
 * Button debouncing
 * ======================================================================= */

void update_buttons(void)
{
  for (uint8_t i = 0U; i < 4U; ++i)
  {
    uint8_t raw = (HAL_GPIO_ReadPin(buttons[i].port, buttons[i].pin)
                   == GPIO_PIN_RESET) ? 1U : 0U;
    if (raw == buttons[i].raw_pressed)
    {
      if (buttons[i].stable_samples < BUTTON_DEBOUNCE_SAMPLES)
      {
        ++buttons[i].stable_samples;
        if (buttons[i].stable_samples == BUTTON_DEBOUNCE_SAMPLES)
        {
          buttons[i].stable_pressed = raw;
          if (raw != 0U)
          {
            ++buttons[i].press_count;
            buttons[i].pressed_at    = HAL_GetTick();
            buttons[i].last_repeat_at = HAL_GetTick();
          }
        }
      }
    }
    else
    {
      buttons[i].raw_pressed    = raw;
      buttons[i].stable_samples = 0U;
    }
  }
}

void update_button_auto_repeat(void)
{
  /* Buttons B (index 1) and C (index 2) auto-repeat in menus that scroll
   * or adjust values. */
  for (uint8_t i = 1U; i <= 2U; ++i)
  {
    if (buttons[i].stable_pressed == 0U) continue;
    uint32_t held = HAL_GetTick() - buttons[i].pressed_at;
    uint32_t since_repeat = HAL_GetTick() - buttons[i].last_repeat_at;
    if ((held >= REFERENCE_REPEAT_DELAY_MS)
        && (since_repeat >= REFERENCE_REPEAT_RATE_MS))
    {
      buttons[i].last_repeat_at = HAL_GetTick();
      ui_button(i);
    }
  }
}

/* =========================================================================
 * Core UI helpers
 * ======================================================================= */

static void ui_open(UiPage page)
{
  ui_page = page;
  if ((page == UI_E2E_RUN) || (page == UI_E2E_PREP)
      || (page == UI_E2E_CONFIRM))
    current_screen = SCREEN_CONVEYOR;
  else if ((page == UI_PROFILE_RUN) || (page == UI_PROFILE_EDIT)
           || (page == UI_PROFILE_CONFIRM))
    current_screen = SCREEN_REFLOW;
  else if ((page == UI_CAL_MENU) || (page == UI_CAL_EDIT)
           || (page == UI_CAL_HEATER) || (page == UI_CAL_SAVE))
    current_screen = SCREEN_THERMISTOR_CALIBRATION;
  else if ((page == UI_PID_RUN) || (page == UI_PID_EDIT))
    current_screen = SCREEN_PID;
  else if ((page != UI_DIAG_RUN) && (page != UI_MESSAGE))
    current_screen = SCREEN_HOME;
}

static void ui_show_message(UiMessage message, UiPage return_page)
{
  if (message == UI_MSG_STOPPED) ui_stopped_from = ui_page;
  if (message == UI_MSG_FAULT)   ui_fault_from   = ui_page;
  ui_message     = message;
  ui_return_page = return_page;
  ui_message_at  = HAL_GetTick();
  ui_open(UI_MESSAGE);
}

static uint8_t ui_can_heat(void)
{
  int16_t limit = (ui_page == UI_E2E_CONFIRM)
                  ? ((ui_e2e_profile != 0U) ? 1250 : 1100)
                  : ((ui_page == UI_PROFILE_CONFIRM) ? 1250
                     : ((ui_page == UI_DIAG_RUN)
                        ? ((current_screen == SCREEN_CHARACTERIZATION) ? 500 : 1100)
                        : 1500));
  if (latest_temperature_valid == 0U)
  {
    ui_show_message(UI_MSG_SENSOR, ui_page);
    return 0U;
  }
  if (Thermistor_IsCalibrated() == 0U)
  {
    ui_show_message(UI_MSG_CALIBRATION, ui_page);
    return 0U;
  }
  if (latest_temperature_tenths >= limit)
  {
    ui_show_message(UI_MSG_HOT, ui_page);
    return 0U;
  }
  if (ProcessInterlock_GetOwner() != PROCESS_OWNER_NONE)
  {
    ui_show_message(UI_MSG_BUSY, ui_page);
    return 0U;
  }
  return 1U;
}

static void ui_stop(void)
{
  if (ui_page == UI_E2E_RUN)
  {
    ConveyorApp_RequestAbort();
  }
  else if (ui_page == UI_PROFILE_RUN)
  {
    Reflow_RequestStop();
    HardwareTest_PIDStop();
  }
  else if (ui_page == UI_PID_RUN)
  {
    HardwareTest_PIDStop();
  }
  else if (ui_page == UI_DIAG_RUN)
  {
    if (current_screen == SCREEN_CHARACTERIZATION)
    {
      if ((HeaterCharacterization_IsRunning() != 0U)
          || (ui_diag_start_pending != 0U))
        HeaterCharacterization_RequestStop();
      ui_diag_start_pending = 0U;
    }
    else if (current_screen == SCREEN_MOTOR_TEST)
      ConveyorApp_ManualMotorStop();
    else if (current_screen == SCREEN_SERVO_TEST)
    {
      ConveyorApp_ManualServoStop();
      servo_test_selection = 1U;
    }
    else if (current_screen == SCREEN_TASK_STACK)
    {
      /* Read-only page; there is no actuator to stop. */
    }
    else
      HardwareTest_HeaterStop();
  }
  else if (ui_page == UI_CAL_HEATER)
    HardwareTest_HeaterStop();
}

static uint8_t ui_start_calibration_heater(void)
{
  if ((latest_temperature_valid == 0U)
      || (latest_temperature_tenths >= CALIBRATION_MAX_TEMP_TENTHS)
      || (ProcessInterlock_GetOwner() != PROCESS_OWNER_NONE)
      || (pid_running != 0U) || (Reflow_IsRunning() != 0U)
      || (HeaterCharacterization_IsRunning() != 0U)) return 0U;
  heater_duty_percent        = heater_manual_duty_percent;
  heater_safety_limit_tenths = CALIBRATION_MAX_TEMP_TENTHS;
  heater_enabled             = 1U;
  set_heater_output();
  return 1U;
}

static uint8_t ui_is_active(void)
{
  ConveyorAppStatus conveyor;
  if (ui_page == UI_E2E_RUN)
  {
    ConveyorApp_GetStatus(&conveyor);
    return (conveyor.state != CONVEYOR_SEQ_IDLE) ? 1U : 0U;
  }
  if (ui_page == UI_PROFILE_RUN) return Reflow_IsRunning();
  if (ui_page == UI_PID_RUN)     return pid_running;
  if (ui_page == UI_CAL_HEATER)  return heater_enabled;
  if (ui_page == UI_DIAG_RUN)
  {
    if (current_screen == SCREEN_CHARACTERIZATION)
      return HeaterCharacterization_IsRunning();
    if (current_screen == SCREEN_MOTOR_TEST)
    {
      ConveyorApp_GetStatus(&conveyor);
      return conveyor.manual_test_active;
    }
    if (current_screen == SCREEN_SERVO_TEST)
    {
      ConveyorApp_GetStatus(&conveyor);
      return conveyor.manual_servo_test_active;
    }
    if (current_screen == SCREEN_TASK_STACK) return 0U;
    return heater_enabled;
  }
  return 0U;
}

static uint8_t ui_any_active(void)
{
  ConveyorAppStatus conveyor;
  ConveyorApp_GetStatus(&conveyor);
  return ((conveyor.state != CONVEYOR_SEQ_IDLE)
          || (conveyor.manual_test_active != 0U)
          || (conveyor.manual_servo_test_active != 0U)
          || (Reflow_IsRunning() != 0U)
          || (HeaterCharacterization_IsRunning() != 0U)
          || (pid_running != 0U) || (heater_enabled != 0U)) ? 1U : 0U;
}

static void ui_stop_all(void)
{
  ConveyorAppStatus conveyor;
  ConveyorApp_GetStatus(&conveyor);
  if ((ui_page == UI_E2E_RUN)
      || ((conveyor.state != CONVEYOR_SEQ_IDLE)
      && (conveyor.state != CONVEYOR_SEQ_ESTOP)
      && (conveyor.state != CONVEYOR_SEQ_MOTOR_FAULT)
      && (conveyor.state != CONVEYOR_SEQ_PCB_TIMEOUT)
      && (conveyor.state != CONVEYOR_SEQ_HEATER_FAULT)))
    ConveyorApp_RequestAbort();
  ConveyorApp_ManualMotorStop();
  ConveyorApp_ManualServoStop();
  if ((Reflow_IsRunning() != 0U) || (ui_page == UI_PROFILE_RUN))
    Reflow_RequestStop();
  if ((HeaterCharacterization_IsRunning() != 0U)
      || (ui_diag_start_pending != 0U))
    HeaterCharacterization_RequestStop();
  ui_diag_start_pending = 0U;
  if (pid_running != 0U) HardwareTest_PIDStop();
  HardwareTest_HeaterStop();
}

static void enter_calibration_screen(void)
{
  heater_enabled  = 0U;
  set_heater_output();
  current_screen                          = SCREEN_THERMISTOR_CALIBRATION;
  calibration_point_index                 = 0U;
  calibration_captured_mask               = 0U;
  calibration_reference_tracks_measurement = 1U;
  calibration_reference_tenths            = latest_temperature_tenths;
}

/* =========================================================================
 * ui_back() — D-button short-press navigation
 * ======================================================================= */

static void ui_back(void)
{
  if (ui_page == UI_MESSAGE)
  {
    if ((ui_message == UI_MSG_FAULT)
        && (ui_return_page == UI_E2E_RUN))
    {
      ConveyorApp_AcknowledgeFault();
      ui_open(UI_HOME);
    }
    else if ((ui_message == UI_MSG_STOPPED)
             && (ui_return_page == UI_HOME))
    {
      ConveyorAppStatus conveyor;
      ConveyorApp_GetStatus(&conveyor);
      if (((ui_stopped_from == UI_E2E_RUN)
           && (conveyor.state == CONVEYOR_SEQ_IDLE) && (ui_run_seen == 0U))
          || ((conveyor.state != CONVEYOR_SEQ_IDLE)
           && (conveyor.state != CONVEYOR_SEQ_ESTOP)
           && (conveyor.state != CONVEYOR_SEQ_MOTOR_FAULT)
           && (conveyor.state != CONVEYOR_SEQ_PCB_TIMEOUT)
           && (conveyor.state != CONVEYOR_SEQ_HEATER_FAULT))
          || (Reflow_IsRunning() != 0U)
          || (HeaterCharacterization_IsRunning() != 0U)) return;
      if ((conveyor.state == CONVEYOR_SEQ_ESTOP)
          || (conveyor.state == CONVEYOR_SEQ_MOTOR_FAULT)
          || (conveyor.state == CONVEYOR_SEQ_PCB_TIMEOUT)
          || (conveyor.state == CONVEYOR_SEQ_HEATER_FAULT))
        ConveyorApp_AcknowledgeFault();
      if (ui_stopped_from == UI_CAL_HEATER) Thermistor_Init();
      ui_open(UI_HOME);
    }
    else ui_open(ui_return_page);
  }
  else if ((ui_page == UI_E2E_PREP) || (ui_page == UI_PROFILE_MENU)
           || (ui_page == UI_CAL_MENU) || (ui_page == UI_PID_MENU)
           || (ui_page == UI_DIAG_MENU) || (ui_page == UI_STATUS))
  {
    if (ui_page == UI_CAL_MENU)
    {
      HardwareTest_HeaterStop();
      Thermistor_Init();
    }
    ui_open(UI_HOME);
  }
  else if (ui_page == UI_E2E_CONFIRM) ui_open(UI_E2E_PREP);
  else if (ui_page == UI_PROFILE_EDIT || ui_page == UI_PROFILE_CONFIRM)
    ui_open(UI_PROFILE_MENU);
  else if (ui_page == UI_CAL_EDIT || ui_page == UI_CAL_SAVE)
    ui_open(UI_CAL_MENU);
  else if (ui_page == UI_CAL_HEATER)
  {
    HardwareTest_HeaterStop();
    ui_open(UI_CAL_MENU);
  }
  else if (ui_page == UI_PID_EDIT) ui_open(UI_PID_MENU);
  else if (ui_page == UI_DIAG_RUN)
  {
    ui_stop();
    ui_open(UI_DIAG_MENU);
  }
  else if ((ui_page == UI_E2E_RUN) || (ui_page == UI_PROFILE_RUN)
           || (ui_page == UI_PID_RUN))
  {
    if ((ui_is_active() == 0U) && (ui_run_seen != 0U)) ui_open(UI_HOME);
    else ui_detail ^= 1U;
  }
}

/* =========================================================================
 * ui_adjust() — B/C knob value adjustment
 * ======================================================================= */

static void ui_adjust(int8_t direction)
{
  int32_t next;
  if (ui_page == UI_CAL_EDIT)
  {
    adjust_calibration_reference(direction);
    return;
  }
  if (ui_page == UI_CAL_HEATER)
  {
    if (heater_enabled == 0U)
    {
      next = (int32_t)heater_manual_duty_percent + direction * 25;
      if (next < 25) next = 25;
      if (next > 100) next = 100;
      heater_manual_duty_percent = (uint8_t)next;
    }
    return;
  }
  if ((ui_page == UI_DIAG_RUN) && (current_screen == SCREEN_HEATER))
  {
    if (heater_enabled == 0U)
    {
      next = (int32_t)heater_manual_duty_percent + direction * 25;
      if (next < 25) next = 25;
      if (next > 100) next = 100;
      heater_manual_duty_percent = (uint8_t)next;
    }
    return;
  }
  if (ui_page == UI_PROFILE_EDIT)
  {
    ReflowStatus status;
    Reflow_GetStatus(&status);
    int32_t lo = (ui_edit_selection == 0U) ? 400
                 : ((ui_edit_selection == 1U)
                    ? status.preheat_tenths + 50
                    : status.soaking_tenths + 50);
    int32_t hi = (ui_edit_selection == 0U)
                 ? status.soaking_tenths - 50
                 : ((ui_edit_selection == 1U)
                    ? status.reflow_tenths - 50 : 1200);
    next = ui_edit_value + direction * 10;
    if (next < lo || next > hi)
    {
      ui_show_message(UI_MSG_LIMIT, UI_PROFILE_EDIT);
      return;
    }
    ui_edit_value = next;
  }
  else if (ui_page == UI_PID_EDIT)
  {
    int32_t step    = (ui_edit_selection == 0U) ? 5 : 1;
    int32_t maximum = (ui_edit_selection == 0U) ? 1400
                      : ((ui_edit_selection == 1U) ? 1000
                         : ((ui_edit_selection == 2U) ? 500 : 5000));
    next = ui_edit_value + direction * step;
    if (next < ((ui_edit_selection == 0U) ? 200 : 0) || next > maximum)
    {
      ui_show_message(UI_MSG_LIMIT, UI_PID_EDIT);
      return;
    }
    ui_edit_value = next;
  }
}

/* =========================================================================
 * ui_button() — A-button action dispatch
 * ======================================================================= */

static void ui_button(uint8_t button)
{
  ConveyorAppStatus conveyor;
  ReflowStatus reflow;

  if (button == 3U) return; /* D handled by update_controls() directly */

  /* B (1) and C (2): scrolling and value adjustment. */
  if ((button == 1U) || (button == 2U))
  {
    int8_t delta = (button == 1U) ? -1 : 1;
    if ((ui_page == UI_HOME) || (ui_page == UI_PROFILE_MENU)
        || (ui_page == UI_CAL_MENU) || (ui_page == UI_PID_MENU)
        || (ui_page == UI_DIAG_MENU)) ui_selection_at = HAL_GetTick();

    if (ui_page == UI_HOME)
      ui_home_selection = (uint8_t)((ui_home_selection + 8 + delta) % 8);
    else if (ui_page == UI_PROFILE_MENU)
      ui_profile_selection = (uint8_t)((ui_profile_selection + 4 + delta) % 4);
    else if (ui_page == UI_CAL_MENU)
      ui_cal_selection = (uint8_t)((ui_cal_selection + 4 + delta) % 4);
    else if (ui_page == UI_PID_MENU)
      ui_pid_selection = (uint8_t)((ui_pid_selection + 5 + delta) % 5);
    else if (ui_page == UI_DIAG_MENU)
      ui_diag_selection = (uint8_t)((ui_diag_selection + 5 + delta) % 5);
    else if ((ui_page == UI_E2E_PREP) && (ui_e2e_profile != 2U))
      ConveyorApp_AdjustSpeed((button == 1U) ? 1 : -1);
    else if ((ui_page == UI_PROFILE_EDIT) || (ui_page == UI_PID_EDIT)
             || (ui_page == UI_CAL_EDIT) || (ui_page == UI_CAL_HEATER))
      ui_adjust((button == 1U) ? 1 : -1);
    else if (ui_page == UI_E2E_RUN)
    {
      ConveyorApp_GetStatus(&conveyor);
      if (conveyor.state == CONVEYOR_SEQ_INSPECTION)
        Inspection_SubmitManualResult((button == 1U) ? 1U : 0U);
    }
    else if (ui_page == UI_DIAG_RUN)
    {
      if (current_screen == SCREEN_TASK_STACK)
        ui_stack_selection = (uint8_t)((ui_stack_selection
                                        + TASK_STACK_REPORT_COUNT + delta)
                                       % TASK_STACK_REPORT_COUNT);
      else if (current_screen == SCREEN_CHARACTERIZATION && ui_is_active() == 0U)
        HeaterCharacterization_AdjustDuty((button == 1U) ? 1 : -1);
      else if (current_screen == SCREEN_MOTOR_TEST)
      {
        int16_t duty = (int16_t)motor_test_duty_percent
                       + ((button == 1U) ? 10 : -10);
        if (duty >= MOTOR_TEST_MIN_DUTY && duty <= 100)
        {
          motor_test_duty_percent = (uint8_t)duty;
          ConveyorApp_ManualMotorSetDuty(motor_test_duty_percent);
        }
      }
      else if (current_screen == SCREEN_SERVO_TEST)
      {
        int8_t next = (int8_t)servo_test_selection
                      + ((button == 1U) ? -1 : 1);
        if ((next >= 0) && (next <= 2))
        {
          servo_test_selection = (uint8_t)next;
          ConveyorApp_ManualServoSetPulse(
              servo_test_pulses_us[servo_test_selection]);
        }
      }
      else if (current_screen == SCREEN_HEATER && ui_is_active() == 0U)
        ui_adjust((button == 1U) ? 1 : -1);
    }
    return;
  }

  /* A (0): confirm / start / stop. */
  if (ui_page == UI_HOME)
  {
    if (ui_home_selection <= 2U)
    {
      ui_e2e_profile = ui_home_selection;
      ui_open(UI_E2E_PREP);
    }
    else if (ui_home_selection == 3U) ui_open(UI_PROFILE_MENU);
    else if (ui_home_selection == 4U)
    {
      if ((ProcessInterlock_GetOwner() != PROCESS_OWNER_NONE)
          || (pid_running != 0U) || (Reflow_IsRunning() != 0U))
      {
        ui_show_message(UI_MSG_BUSY, UI_HOME);
        return;
      }
      enter_calibration_screen();
      ui_open(UI_CAL_MENU);
    }
    else if (ui_home_selection == 5U) ui_open(UI_PID_MENU);
    else if (ui_home_selection == 6U) ui_open(UI_DIAG_MENU);
    else ui_open(UI_STATUS);
  }
  else if (ui_page == UI_E2E_PREP) ui_open(UI_E2E_CONFIRM);
  else if (ui_page == UI_E2E_CONFIRM)
  {
    if (ui_e2e_profile == 2U)
    {
      ui_show_message(UI_MSG_SN63_LOCKED, UI_E2E_CONFIRM);
      return;
    }
    if (ui_can_heat() == 0U) return;
    uint8_t started = (ui_e2e_profile != 0U)
        ? ConveyorApp_RequestStartProfile() : ConveyorApp_RequestStart();
    if (started != 0U)
    {
      ui_run_seen = 0U;
      ui_open(UI_E2E_RUN);
    }
    else ui_show_message(UI_MSG_BUSY, UI_E2E_CONFIRM);
  }
  else if (ui_page == UI_PROFILE_MENU)
  {
    if (ui_profile_selection == 3U) ui_open(UI_PROFILE_CONFIRM);
    else
    {
      Reflow_GetStatus(&reflow);
      ui_edit_selection = ui_profile_selection;
      ui_edit_value = (ui_edit_selection == 0U) ? reflow.preheat_tenths
                      : ((ui_edit_selection == 1U) ? reflow.soaking_tenths
                         : reflow.reflow_tenths);
      ui_open(UI_PROFILE_EDIT);
    }
  }
  else if (ui_page == UI_PROFILE_EDIT)
  {
    if (Reflow_IsRunning() != 0U)
    {
      ui_show_message(UI_MSG_BUSY, UI_PROFILE_MENU);
      return;
    }
    Reflow_GetStatus(&reflow);
    while ((uint8_t)reflow.selected != ui_edit_selection)
    {
      Reflow_SelectNextProfile();
      Reflow_GetStatus(&reflow);
    }
    Reflow_AdjustSelectedTemperature((int16_t)(ui_edit_value
         - ((ui_edit_selection == 0U) ? reflow.preheat_tenths
            : ((ui_edit_selection == 1U) ? reflow.soaking_tenths
               : reflow.reflow_tenths))));
    ui_open(UI_PROFILE_MENU);
  }
  else if (ui_page == UI_PROFILE_CONFIRM)
  {
    if (ui_can_heat() == 0U) return;
    if (latest_temperature_tenths >= 1250)
    {
      ui_show_message(UI_MSG_HOT, UI_PROFILE_CONFIRM);
      return;
    }
    Reflow_RequestStart();
    ui_run_seen = 0U;
    ui_open(UI_PROFILE_RUN);
  }
  else if (ui_page == UI_CAL_MENU)
  {
    if (ui_cal_selection < 2U)
    {
      calibration_point_index                 = ui_cal_selection;
      calibration_reference_tracks_measurement = 1U;
      calibration_reference_tenths            = latest_temperature_tenths;
      ui_open(UI_CAL_EDIT);
    }
    else if (ui_cal_selection == 2U) ui_open(UI_CAL_HEATER);
    else ui_open(UI_CAL_SAVE);
  }
  else if (ui_page == UI_CAL_EDIT)
  {
    if ((latest_temperature_valid != 0U)
        && Thermistor_SetCalibrationPoint(calibration_point_index,
             latest_adc, calibration_reference_tenths) != 0U)
    {
      calibration_captured_mask |= (uint8_t)(1U << calibration_point_index);
      ui_show_message(UI_MSG_CAPTURED, UI_CAL_MENU);
    }
    else ui_show_message(UI_MSG_SENSOR, UI_CAL_EDIT);
  }
  else if (ui_page == UI_CAL_HEATER)
  {
    if (heater_enabled != 0U) HardwareTest_HeaterStop();
    else if (ui_start_calibration_heater() == 0U)
      ui_show_message(UI_MSG_SENSOR, UI_CAL_HEATER);
  }
  else if (ui_page == UI_CAL_SAVE)
  {
    HardwareTest_HeaterStop();
    if ((calibration_captured_mask == 0x03U)
        && (Thermistor_CalibrationReady() != 0U))
    {
      uint8_t saved = (Thermistor_SaveCalibration() == HAL_OK) ? 1U : 0U;
      Thermistor_Init();
      ui_show_message(saved ? UI_MSG_SAVED : UI_MSG_SAVE_FAILED, UI_CAL_MENU);
    }
    else ui_show_message(UI_MSG_CALIBRATION, UI_CAL_MENU);
  }
  else if (ui_page == UI_PID_MENU)
  {
    if (ui_pid_selection == 4U)
    {
      if (ui_can_heat() == 0U) return;
      if (pid_start() != 0U)
      {
        ui_run_seen = 1U;
        ui_open(UI_PID_RUN);
      }
      else ui_show_message(UI_MSG_HOT, UI_PID_MENU);
    }
    else
    {
      ui_edit_selection = ui_pid_selection;
      ui_edit_value = (ui_edit_selection == 0U) ? pid_setpoint_tenths
                      : ((ui_edit_selection == 1U) ? (int32_t)(pid_kp * 100.0f + 0.5f)
                         : ((ui_edit_selection == 2U) ? (int32_t)(pid_ki * 1000.0f + 0.5f)
                            : (int32_t)(pid_kd * 100.0f + 0.5f)));
      ui_open(UI_PID_EDIT);
    }
  }
  else if (ui_page == UI_PID_EDIT)
  {
    if ((pid_running != 0U) || (Reflow_IsRunning() != 0U))
    {
      ui_show_message(UI_MSG_BUSY, UI_PID_MENU);
      return;
    }
    if (ui_edit_selection == 0U) pid_setpoint_tenths = (int16_t)ui_edit_value;
    else if (ui_edit_selection == 1U) pid_kp = (float)ui_edit_value / 100.0f;
    else if (ui_edit_selection == 2U) pid_ki = (float)ui_edit_value / 1000.0f;
    else pid_kd = (float)ui_edit_value / 100.0f;
    ui_open(UI_PID_MENU);
  }
  else if (ui_page == UI_DIAG_MENU)
  {
    if      (ui_diag_selection == 0U) current_screen = SCREEN_CHARACTERIZATION;
    else if (ui_diag_selection == 1U) current_screen = SCREEN_HEATER;
    else if (ui_diag_selection == 2U) current_screen = SCREEN_MOTOR_TEST;
    else if (ui_diag_selection == 3U)
    {
      current_screen       = SCREEN_SERVO_TEST;
      servo_test_selection = 1U;
    }
    else
    {
      current_screen      = SCREEN_TASK_STACK;
      ui_stack_selection  = 0U;
    }
    ui_open(UI_DIAG_RUN);
  }
  else if (ui_page == UI_DIAG_RUN)
  {
    if (current_screen == SCREEN_TASK_STACK) return;
    if (ui_is_active() != 0U) ui_stop();
    else if (current_screen == SCREEN_CHARACTERIZATION)
    {
      if (ui_diag_start_pending != 0U) ui_stop();
      else if (ui_can_heat() != 0U)
      {
        HeaterCharacterization_RequestStart();
        ui_diag_start_pending = 1U;
      }
    }
    else if (current_screen == SCREEN_HEATER)
    {
      if ((ui_can_heat() != 0U)
          && (HardwareTest_HeaterStartAtDuty(
                heater_manual_duty_percent, MANUAL_HEATER_LIMIT_TENTHS) == 0U))
        ui_show_message(UI_MSG_HOT, UI_DIAG_RUN);
    }
    else if (current_screen == SCREEN_SERVO_TEST)
    {
      if (ConveyorApp_ManualServoStart(
              servo_test_pulses_us[servo_test_selection]) == 0U)
        ui_show_message(UI_MSG_BUSY, UI_DIAG_RUN);
    }
    else if (ConveyorApp_ManualMotorStart(motor_test_duty_percent) == 0U)
      ui_show_message(UI_MSG_BUSY, UI_DIAG_RUN);
  }
  else if (ui_page == UI_MESSAGE) ui_back();
  else if ((ui_page == UI_E2E_RUN) || (ui_page == UI_PROFILE_RUN)
           || (ui_page == UI_PID_RUN))
  {
    if ((ui_is_active() != 0U) || (ui_run_seen == 0U))
    {
      ui_stop();
      ui_show_message(UI_MSG_STOPPED, UI_HOME);
    }
    else if (ui_page == UI_E2E_RUN)
    {
      ConveyorApp_GetStatus(&conveyor);
      if ((conveyor.state == CONVEYOR_SEQ_ESTOP)
          || (conveyor.state == CONVEYOR_SEQ_MOTOR_FAULT)
          || (conveyor.state == CONVEYOR_SEQ_PCB_TIMEOUT)
          || (conveyor.state == CONVEYOR_SEQ_HEATER_FAULT))
        ConveyorApp_AcknowledgeFault();
      ui_open(UI_HOME);
    }
    else ui_open(UI_HOME);
  }
}

/* =========================================================================
 * update_controls() — D-button long-press and short-press routing
 * ======================================================================= */

void update_controls(void)
{
  uint8_t d_pressed = buttons[3].stable_pressed;

  /* A, B, C: dispatch on new press. */
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    if (handled_press_count[i] != buttons[i].press_count)
    {
      handled_press_count[i] = buttons[i].press_count;
      ui_button(i);
    }
  }
  handled_press_count[3] = buttons[3].press_count;

  /* D: long-press (>= 1500 ms) = emergency stop; short-press = back. */
  if (d_pressed != 0U)
  {
    if (ui_d_press_active == 0U)
    {
      ui_d_press_active  = 1U;
      ui_d_long_handled  = 0U;
      ui_d_pressed_at    = HAL_GetTick();
    }
    else if ((ui_d_long_handled == 0U)
             && ((HAL_GetTick() - ui_d_pressed_at) >= 1500U))
    {
      ui_d_long_handled = 1U;
      if ((ui_any_active() != 0U)
          || (ui_page == UI_E2E_RUN) || (ui_page == UI_PROFILE_RUN)
          || ((ui_page == UI_DIAG_RUN)
              && (current_screen != SCREEN_TASK_STACK)))
      {
        ui_stop_all();
        ui_show_message(UI_MSG_STOPPED, UI_HOME);
      }
      else if (ui_page == UI_MESSAGE)
        ui_back();
      else
      {
        HardwareTest_HeaterStop();
        if ((ui_page == UI_CAL_MENU) || (ui_page == UI_CAL_EDIT)
            || (ui_page == UI_CAL_HEATER) || (ui_page == UI_CAL_SAVE))
          Thermistor_Init();
        ui_open(UI_HOME);
      }
    }
  }
  else if (ui_d_press_active != 0U)
  {
    if (ui_d_long_handled == 0U) ui_back();
    ui_d_press_active = 0U;
    ui_d_long_handled = 0U;
  }
}

/* =========================================================================
 * Display helpers
 * ======================================================================= */

static void ui_line(uint8_t row, const char *value)
{
  memset(&oled.buffer[(uint16_t)row * SSD1306_WIDTH], 0, SSD1306_WIDTH);
  SSD1306_DrawString(&oled, 0U, row, value);
}

static void ui_list(const char *title, const char *const *items,
                    uint8_t count, uint8_t selected, uint8_t first_row,
                    uint8_t visible)
{
  char line[22];
  uint8_t offset = (selected >= visible) ? (selected - visible + 1U) : 0U;
  ui_line(0U, title);
  for (uint8_t row = 0U; row < visible; ++row)
  {
    uint8_t index = offset + row;
    if (index >= count) break;
    (void)TextFormat(line, sizeof(line), "%c %.18s",
                     (index == selected) ? '>' : ' ', items[index]);
    ui_line(first_row + row, line);
    if ((index == selected)
        && ((HAL_GetTick() - ui_selection_at) < 160U))
      SSD1306_DrawLine(&oled, 7U, (first_row + row) * 8U + 7U,
                      115U, (first_row + row) * 8U + 7U);
  }
}

static void format_temperature(char *text, size_t size, int16_t tenths)
{
  int16_t magnitude = (tenths < 0) ? (int16_t)-tenths : tenths;
  (void)TextFormat(text, size, "%s%d.%d", (tenths < 0) ? "-" : "",
                 magnitude / 10, magnitude % 10);
}

static void draw_button_line(uint8_t row,
                             char first_name,
                             const ButtonState *first,
                             char second_name,
                             const ButtonState *second)
{
  char line[22];
  (void)TextFormat(line, sizeof(line), "%c:%c%03u %c:%c%03u",
                 first_name, first->stable_pressed ? 'P' : '-',
                 first->press_count,
                 second_name, second->stable_pressed ? 'P' : '-',
                 second->press_count);
  SSD1306_DrawString(&oled, 0U, row, line);
}

/* =========================================================================
 * Screen drawing functions
 * ======================================================================= */

static void draw_heater_screen(uint32_t millivolts)
{
  char line[22];

  SSD1306_DrawString(&oled, 0U, 0U, "SSR HEATER TEST");
  (void)TextFormat(line, sizeof(line), "SSR:%s DUTY:%3u%%",
                 heater_enabled ? "RUN" : "OFF", heater_duty_percent);
  SSD1306_DrawString(&oled, 0U, 1U, line);
  draw_button_line(2U, 'A', &buttons[0], 'B', &buttons[1]);
  draw_button_line(3U, 'C', &buttons[2], 'D', &buttons[3]);

  (void)TextFormat(line, sizeof(line), "ADC:%4u %4lumV", latest_adc,
                 (unsigned long)millivolts);
  SSD1306_DrawString(&oled, 0U, 5U, line);

  if (latest_temperature_valid != 0U)
  {
    char temperature[10];
    format_temperature(temperature, sizeof(temperature),
                       latest_temperature_tenths);
    (void)TextFormat(line, sizeof(line), "NTC:%s C %s", temperature,
                   Thermistor_IsCalibrated() ? "CAL" : "DEF");
  }
  else
  {
    (void)TextFormat(line, sizeof(line), "NTC: OPEN/SHORT");
  }
  SSD1306_DrawString(&oled, 0U, 6U, line);
  SSD1306_DrawString(&oled, 0U, 7U, "A:ON B:+ C:- D:CAL");
}

static void draw_pid_screen(void)
{
  char line[22];
  char process_value[10] = "--.-";
  char setpoint[10];
  char ramped_setpoint[10];
  int16_t ramped_tenths = (int16_t)((pid_ramped_setpoint >= 0.0f)
                            ? (pid_ramped_setpoint * 10.0f + 0.5f)
                            : (pid_ramped_setpoint * 10.0f - 0.5f));

  if (latest_temperature_valid != 0U)
    format_temperature(process_value, sizeof(process_value),
                       latest_temperature_tenths);
  format_temperature(setpoint, sizeof(setpoint), pid_setpoint_tenths);
  format_temperature(ramped_setpoint, sizeof(ramped_setpoint), ramped_tenths);

  SSD1306_DrawString(&oled, 0U, 0U, "TEMPERATURE PID");
  (void)TextFormat(line, sizeof(line), "PEL:%.6s SP:%.6s",
                 process_value, setpoint);
  SSD1306_DrawString(&oled, 0U, 1U, line);
  (void)TextFormat(line, sizeof(line), "R:%.6s H:%3u%%", ramped_setpoint,
                 (unsigned int)(pid_output_percent + 0.5f));
  SSD1306_DrawString(&oled, 0U, 2U, line);
  (void)TextFormat(line, sizeof(line), "%s FAN:%3u%%",
                 pid_fault ? "FAULT" : (pid_running ? "RUN" : "READY"),
                 fan_duty_percent);
  SSD1306_DrawString(&oled, 0U, 3U, line);
  (void)TextFormat(line, sizeof(line), "P:%d I:%d D:%d",
                 (int)pid_p_term, (int)pid_i_term, (int)pid_d_term);
  SSD1306_DrawString(&oled, 0U, 4U, line);
  (void)TextFormat(line, sizeof(line), "CAL:%s SAFE<150C",
                 Thermistor_IsCalibrated() ? "OK" : "NO");
  SSD1306_DrawString(&oled, 0U, 5U, line);
  SSD1306_DrawString(&oled, 0U, 6U, "A:RUN B:+.5 C:-.5");
  SSD1306_DrawString(&oled, 0U, 7U, "D:OFF/REFLOW");
}

static const char *reflow_state_name(ReflowState state)
{
  switch (state)
  {
    case REFLOW_STATE_PREHEAT:    return "PREHEAT";
    case REFLOW_STATE_SOAKING:    return "SOAKING";
    case REFLOW_STATE_REFLOW:     return "REFLOW";
    case REFLOW_STATE_TIMED_TEST: return "TEST-HEAT";
    case REFLOW_STATE_COOLING:    return "COOLING";
    case REFLOW_STATE_IDLE:
    default:                      return "IDLE";
  }
}

static uint8_t reflow_graph_y(int16_t temperature_tenths)
{
  int32_t clamped = temperature_tenths;
  if (clamped < 200)       clamped = 200;
  else if (clamped > 1200) clamped = 1200;
  return (uint8_t)(47 - (((clamped - 200) * 31) / 1000));
}

static void draw_reflow_screen(void)
{
  ReflowStatus status;
  char line[22];
  char temperature[10] = "--.-";
  char selected_name;
  int16_t target;
  uint8_t preheat_degrees, soaking_degrees, reflow_degrees;
  uint8_t previous_valid = 0U, previous_x = 0U, previous_y = 0U;

  Reflow_GetStatus(&status);
  if (latest_temperature_valid != 0U)
    format_temperature(temperature, sizeof(temperature), latest_temperature_tenths);

  (void)TextFormat(line, sizeof(line), "%s %sC %lus",
                 reflow_state_name(status.state), temperature,
                 (unsigned long)status.elapsed_seconds);
  SSD1306_DrawString(&oled, 0U, 0U, line);

  selected_name  = (status.selected == REFLOW_PROFILE_PREHEAT) ? 'P'
                   : ((status.selected == REFLOW_PROFILE_SOAKING) ? 'S' : 'R');
  preheat_degrees = (uint8_t)(status.preheat_tenths / 10);
  soaking_degrees = (uint8_t)(status.soaking_tenths / 10);
  reflow_degrees  = (uint8_t)(status.reflow_tenths / 10);
  (void)TextFormat(line, sizeof(line), "P%03u S%03u R%03u >%c",
                 preheat_degrees, soaking_degrees, reflow_degrees, selected_name);
  SSD1306_DrawString(&oled, 0U, 1U, line);

  if      (status.state == REFLOW_STATE_PREHEAT) target = status.preheat_tenths;
  else if (status.state == REFLOW_STATE_SOAKING) target = status.soaking_tenths;
  else if (status.state == REFLOW_STATE_REFLOW)  target = status.reflow_tenths;
  else if (status.selected == REFLOW_PROFILE_PREHEAT) target = status.preheat_tenths;
  else if (status.selected == REFLOW_PROFILE_SOAKING) target = status.soaking_tenths;
  else                                                target = status.reflow_tenths;

  if (status.state != REFLOW_STATE_COOLING)
  {
    uint8_t target_y = reflow_graph_y(target);
    for (uint8_t x = 0U; x < SSD1306_WIDTH; x += 4U)
      SSD1306_DrawPixel(&oled, x, target_y, 1U);
  }

  for (uint8_t i = 0U; i < status.graph_count; ++i)
  {
    uint8_t sample = status.graph_temperature_degrees[i];
    if (sample == 0xffU) { previous_valid = 0U; continue; }
    uint8_t y = reflow_graph_y((int16_t)sample * 10);
    if (previous_valid != 0U)
      SSD1306_DrawLine(&oled, previous_x, previous_y, i, y);
    else
      SSD1306_DrawPixel(&oled, i, y, 1U);
    previous_valid = 1U;
    previous_x = i;
    previous_y = y;
  }

  (void)TextFormat(line, sizeof(line), "%sH:%3u F:%3u T:%3d",
                 status.fault ? "!" : " ", heater_duty_percent,
                 fan_duty_percent, target / 10);
  SSD1306_DrawString(&oled, 0U, 6U, line);
  SSD1306_DrawString(&oled, 0U, 7U,
                     (status.state == REFLOW_STATE_IDLE)
                     ? "A:RUN B:+ C:- D:SEL"
                     : "A:STOP D-HOLD:NEXT");
}

static const char *characterization_fault_name(HeaterCharacterizationFault fault)
{
  switch (fault)
  {
    case HEATER_CHARACTERIZATION_FAULT_SENSOR:          return "SENSOR";
    case HEATER_CHARACTERIZATION_FAULT_CALIBRATION:     return "CAL";
    case HEATER_CHARACTERIZATION_FAULT_START_HOT:       return "HOT";
    case HEATER_CHARACTERIZATION_FAULT_HEATING_TIMEOUT: return "HEAT-TIME";
    case HEATER_CHARACTERIZATION_FAULT_COOLING_TIMEOUT: return "COOL-TIME";
    case HEATER_CHARACTERIZATION_FAULT_ABORTED:         return "ABORT";
    case HEATER_CHARACTERIZATION_FAULT_NONE:
    default:                                            return "NONE";
  }
}

static void draw_characterization_screen(void)
{
  HeaterCharacterizationStatus status;
  char line[22];
  char temperature[8] = "--.-";
  char peak[8], overshoot[8];
  int32_t rate_magnitude, average_magnitude;

  HeaterCharacterization_GetStatus(&status);
  if (latest_temperature_valid != 0U)
    format_temperature(temperature, sizeof(temperature), latest_temperature_tenths);
  format_temperature(peak, sizeof(peak), status.peak_temperature_tenths);
  format_temperature(overshoot, sizeof(overshoot), status.overshoot_tenths);
  rate_magnitude    = (status.rate_milli_c_per_s < 0)
                      ? -status.rate_milli_c_per_s : status.rate_milli_c_per_s;
  average_magnitude = (status.average_rate_milli_c_per_s < 0)
                      ? -status.average_rate_milli_c_per_s
                      : status.average_rate_milli_c_per_s;

  (void)TextFormat(line, sizeof(line), "%s %sC",
                   HeaterCharacterization_StateName(status.state), temperature);
  SSD1306_DrawString(&oled, 0U, 0U, line);
  (void)TextFormat(line, sizeof(line), "DUTY:%3u%% CUT:110C", status.duty_percent);
  SSD1306_DrawString(&oled, 0U, 1U, line);
  (void)TextFormat(line, sizeof(line), "RATE:%c%lu.%03luC/s",
                   (status.rate_milli_c_per_s < 0) ? '-' : '+',
                   (unsigned long)(rate_magnitude / 1000L),
                   (unsigned long)(rate_magnitude % 1000L));
  SSD1306_DrawString(&oled, 0U, 2U, line);
  (void)TextFormat(line, sizeof(line), "AVG :%c%lu.%03luC/s",
                   (status.average_rate_milli_c_per_s < 0) ? '-' : '+',
                   (unsigned long)(average_magnitude / 1000L),
                   (unsigned long)(average_magnitude % 1000L));
  SSD1306_DrawString(&oled, 0U, 3U, line);
  (void)TextFormat(line, sizeof(line), "PEAK:%sC OV:%sC", peak, overshoot);
  SSD1306_DrawString(&oled, 0U, 4U, line);
  (void)TextFormat(line, sizeof(line), "TIME:%lus F:%s",
                   (unsigned long)(status.elapsed_ms / 1000UL),
                   characterization_fault_name(status.fault));
  SSD1306_DrawString(&oled, 0U, 5U, line);
  SSD1306_DrawString(&oled, 0U, 6U, "D-HOLD:NEXT");
  SSD1306_DrawString(&oled, 0U, 7U,
                     HeaterCharacterization_IsRunning()
                     ? "A:STOP AUTO LOGGING"
                     : "A:RUN B:+25 C:-25");
}

static const char *process_owner_name(ProcessOwner owner)
{
  if (owner == PROCESS_OWNER_CONVEYOR) return "BELT";
  if (owner == PROCESS_OWNER_HEATER)   return "HEAT";
  return "FREE";
}

static void draw_motor_test_screen(void)
{
  ConveyorAppStatus conveyor;
  char line[22];

  ConveyorApp_GetStatus(&conveyor);
  SSD1306_DrawString(&oled, 0U, 0U, "DC MOTOR TEST");
  (void)TextFormat(line, sizeof(line), "STATE: %s",
                   conveyor.manual_test_active ? "RUNNING" : "STOPPED");
  SSD1306_DrawString(&oled, 0U, 2U, line);
  (void)TextFormat(line, sizeof(line), "PWM  : %3u%%", motor_test_duty_percent);
  SSD1306_DrawString(&oled, 0U, 3U, line);
  (void)TextFormat(line, sizeof(line), "PULSE: %lu",
                   (unsigned long)conveyor.current_pulses);
  SSD1306_DrawString(&oled, 0U, 4U, line);
  SSD1306_DrawString(&oled, 0U, 6U, "A:START / STOP");
  SSD1306_DrawString(&oled, 0U, 7U, "B:+10 C:-10 D:NEXT");
}

static void draw_servo_test_screen(void)
{
  ConveyorAppStatus conveyor;
  char line[22];
  static const char *const positions[3] = {"KIRI", "TENGAH", "KANAN"};

  ConveyorApp_GetStatus(&conveyor);
  ui_line(0U, "UJI SERVO PA2");
  ui_line(2U, conveyor.manual_servo_test_active ? "STATUS: AKTIF"
                                                : "STATUS: SIAP");
  (void)TextFormat(line, sizeof(line), "POSISI: %s", positions[servo_test_selection]);
  ui_line(3U, line);
  (void)TextFormat(line, sizeof(line), "PULSA: %u us",
                   servo_test_pulses_us[servo_test_selection]);
  ui_line(4U, line);
  ui_line(6U, conveyor.manual_servo_test_active ? "A:STOP B/C:POSISI"
                                                : "A:MULAI B/C:POSISI");
  ui_line(7U, "D:TENGAH & KEMBALI");
}

static void draw_task_stack_screen(void)
{
  char line[22];
  uint8_t first = (ui_stack_selection >= 4U) ? (ui_stack_selection - 3U) : 0U;

  ui_line(0U, "TASK STACK");
  ui_line(1U, "CONSUMPTION (BYTE)");
  for (uint8_t row = 0U; row < 4U; ++row)
  {
    uint8_t index = first + row;
    TaskStackReport report;

    if ((index >= TASK_STACK_REPORT_COUNT)
        || (TaskStackReport_Get(index, &report) == 0U))
      continue;
    (void)TextFormat(line, sizeof(line), "%c%-7s %4u/%4u",
                     (index == ui_stack_selection) ? '>' : ' ',
                     report.name, report.peak_used_bytes,
                     report.allocated_bytes);
    ui_line(2U + row, line);
  }
  ui_line(6U, "PEAK / ALOKASI");
  ui_line(7U, "B/C:GESER D:KEMBALI");
}

/* =========================================================================
 * ui_message_text() — fault / status message strings (Bahasa Indonesia)
 * ======================================================================= */

static const char *ui_message_text(void)
{
  switch (ui_message)
  {
    case UI_MSG_SENSOR:      return "SENSOR NTC GAGAL";
    case UI_MSG_CALIBRATION: return "NTC BELUM DIKALIBRASI";
    case UI_MSG_BUSY:        return "PROSES SEDANG BERJALAN";
    case UI_MSG_HOT:         return "SUHU TERLALU TINGGI";
    case UI_MSG_LIMIT:       return "BATAS TERCAPAI";
    case UI_MSG_CAPTURED:    return "TITIK DIAMBIL";
    case UI_MSG_SAVED:       return "KALIBRASI DISIMPAN";
    case UI_MSG_SAVE_FAILED: return "SIMPAN GAGAL";
    case UI_MSG_STOPPED:     return "PROSES DIHENTIKAN";
    case UI_MSG_SN63_LOCKED: return "SN63 TERKUNCI";
    case UI_MSG_FINISHED:
    {
      ConveyorAppStatus conveyor;
      if (ui_finished_e2e != 0U)
      {
        ConveyorApp_GetStatus(&conveyor);
        if (conveyor.state == CONVEYOR_SEQ_ESTOP)   return "E-STOP - BERHENTI";
        if (conveyor.state == CONVEYOR_SEQ_PCB_TIMEOUT)
                                                    return "PCB TIMEOUT - FAIL";
      }
      return "PROFIL SELESAI";
    }
    case UI_MSG_FAULT:
    {
      ConveyorAppStatus conveyor;
      if (ui_fault_from == UI_E2E_RUN)
      {
        ConveyorApp_GetStatus(&conveyor);
        if (conveyor.state == CONVEYOR_SEQ_MOTOR_FAULT) return "MOTOR/ENCODER GAGAL";
        if (conveyor.state == CONVEYOR_SEQ_PCB_TIMEOUT)  return "PCB/IR TIMEOUT";
        if (conveyor.state == CONVEYOR_SEQ_HEATER_FAULT) return "PEMANAS GAGAL";
      }
      if (ui_fault_from == UI_PROFILE_RUN)
      {
        ReflowStatus status;
        Reflow_GetStatus(&status);
        if (status.fault == 2U) return "INTERLOCK SIBUK";
        if (latest_temperature_valid == 0U) return "SENSOR NTC GAGAL";
        if (latest_temperature_tenths >= 1250) return "SUHU TERLALU TINGGI";
        return "PROFIL/PID GAGAL";
      }
      if (ui_fault_from == UI_PID_RUN)
      {
        if (thermal_runaway_fault != 0U) return "RUNAWAY TERMAL";
        return "PID/SENSOR GAGAL";
      }
      if ((ui_fault_from == UI_DIAG_RUN) && (thermal_runaway_fault != 0U))
        return "RUNAWAY TERMAL";
      return "PROSES GAGAL";
    }
    default: return "STATUS";
  }
}

/* =========================================================================
 * ui_draw() — Full screen render for every 50 ms display tick
 * ======================================================================= */

void ui_draw(void)
{
  static const char *const home_items[] = {
    "E2E UJI 5 DETIK", "E2E PROFIL 120C POC", "E2E SN63 TERKUNCI",
    "PROFIL SUHU",
    "KALIBRASI NTC", "KENDALI PID", "DIAGNOSTIK", "STATUS ALAT"
  };
  static const char *const cal_items[] = {
    "TITIK 1", "TITIK 2", "PEMANAS BANTU", "SIMPAN KALIBRASI"
  };
  static const char *const diag_items[] = {
    "KARAKTER HEATER", "UJI HEATER MANUAL", "UJI MOTOR DC", "UJI SERVO",
    "TASK STACK CONSUM."
  };
  char line[22];
  char temperature[10] = "--.-";
  ReflowStatus reflow;
  ConveyorAppStatus conveyor;
  InspectionStatus inspection;

  if (latest_temperature_valid != 0U)
    format_temperature(temperature, sizeof(temperature), latest_temperature_tenths);

  if (ui_page == UI_HOME)
  {
    ui_list("REFLOW CTRL   READY", home_items, 8U, ui_home_selection, 2U, 5U);
    (void)TextFormat(line, sizeof(line), "PEL %sC CAL:%s", temperature,
                     Thermistor_IsCalibrated() ? "OK" : "NO");
    ui_line(1U, line);
    ui_line(7U, "B:^ C:v A:BUKA");
  }
  else if (((ui_page == UI_E2E_PREP) || (ui_page == UI_E2E_CONFIRM))
           && (ui_e2e_profile == 2U))
  {
    ui_line(0U, (ui_page == UI_E2E_PREP) ? "E2E SN63 1/2" : "E2E SN63 2/2");
    ui_line(1U, "SN63/PB37 CAIR 183C");
    ui_line(2U, "NTC DI PELAT");
    ui_line(3U, "SUHU PCB:TAK DIUKUR");
    ui_line(4U, "BATAS TERMAL:BELUM UJI");
    ui_line(5U, "START TERKUNCI");
    ui_line(6U, (ui_page == UI_E2E_PREP) ? "A:LANJUT" : "A:ALASAN");
    ui_line(7U, "D:KEMBALI");
  }
  else if (ui_page == UI_E2E_PREP || ui_page == UI_E2E_CONFIRM)
  {
    ConveyorApp_GetStatus(&conveyor);
    ui_line(0U, (ui_e2e_profile == 1U)
            ? ((ui_page == UI_E2E_PREP) ? "E2E POC 1/2" : "E2E POC 2/2")
            : ((ui_page == UI_E2E_PREP) ? "E2E UJI 5S 1/2" : "E2E UJI 5S 2/2"));
    ui_line(1U, "BELT>HEAT>INSP>SORT");
    if (ui_e2e_profile == 1U)
    {
      Reflow_GetStatus(&reflow);
      (void)TextFormat(line, sizeof(line), "P%d S%d R%d C",
                       reflow.preheat_tenths / 10,
                       reflow.soaking_tenths / 10,
                       reflow.reflow_tenths / 10);
      ui_line(2U, line);
    }
    else ui_line(2U, "HEAT 25% / 5 DETIK");
    (void)TextFormat(line, sizeof(line), "NTC:%s CAL:%s",
                     latest_temperature_valid ? "OK" : "ERR",
                     Thermistor_IsCalibrated() ? "OK" : "NO");
    ui_line(3U, line);
    Inspection_GetStatus(&inspection);
    (void)TextFormat(line, sizeof(line), "VISION:%s IR:%s",
                     inspection.vision_online ? "OK" : "--",
                     conveyor.ir_detected ? "YES" : "NO");
    ui_line(4U, line);
    (void)TextFormat(line, sizeof(line), "SPEED:%u%%", conveyor.speed_percent);
    ui_line(5U, line);
    ui_line(6U, (ui_page == UI_E2E_PREP) ? "B:+ C:- A:LANJUT"
            : ((ui_e2e_profile == 1U) ? "90/75/75S COOL<=50C"
               : "TIMEOUT: FAIL"));
    ui_line(7U, (ui_page == UI_E2E_PREP) ? "D:KEMBALI"
            : "A:MULAI D:KEMBALI");
  }
  else if (ui_page == UI_E2E_RUN)
  {
    ConveyorApp_GetStatus(&conveyor);
    Inspection_GetStatus(&inspection);
    (void)TextFormat(line, sizeof(line), "%s %s",
                     (ui_e2e_profile != 0U) ? "E2E POC" : "E2E UJI 5S",
                     ConveyorSequencer_StateName(conveyor.state));
    ui_line(0U, line);
    ui_line(1U, "BELT>HEAT>INSP>SORT");
    uint8_t stage = ((conveyor.state == CONVEYOR_SEQ_MOVING_TO_HEATER)
                    || (conveyor.state == CONVEYOR_SEQ_START_REQUESTED)) ? 0U
                    : ((conveyor.state == CONVEYOR_SEQ_HEATING_WAIT) ? 1U
                       : ((conveyor.state == CONVEYOR_SEQ_INSPECTION) ? 2U : 3U));
    char active_first  = ((HAL_GetTick() / 250U) % 2U == 0U) ? '.' : ':';
    char active_second = (active_first == '.') ? ':' : '.';
    (void)TextFormat(line, sizeof(line), "[%c%c] [%c%c] [%c%c] [%c%c]",
                     (stage > 0U) ? '#' : active_first,
                     (stage > 0U) ? '#' : active_second,
                     (stage > 1U) ? '#' : ((stage == 1U) ? active_first : ' '),
                     (stage > 1U) ? '#' : ((stage == 1U) ? active_second : ' '),
                     (stage > 2U) ? '#' : ((stage == 2U) ? active_first : ' '),
                     (stage > 2U) ? '#' : ((stage == 2U) ? active_second : ' '),
                     (stage == 3U) ? active_first : ' ',
                     (stage == 3U) ? active_second : ' ');
    ui_line(2U, line);
    if (ui_detail != 0U)
      (void)TextFormat(line, sizeof(line), "M:%u%% POS:%lu",
                       conveyor.motor_percent,
                       (unsigned long)conveyor.current_pulses);
    else
      (void)TextFormat(line, sizeof(line), "PEL %sC H:%u%%", temperature,
                       heater_duty_percent);
    ui_line(3U, line);
    if ((ui_e2e_profile != 0U) && (conveyor.state == CONVEYOR_SEQ_HEATING_WAIT))
    {
      int16_t target_c;
      Reflow_GetStatus(&reflow);
      if (reflow.state == REFLOW_STATE_IDLE)
      {
        ui_line(4U, "TAHAP:MENUNGGU");
        ui_line(5U, "PROFIL DIMULAI...");
      }
      else
      {
        target_c = (reflow.state == REFLOW_STATE_PREHEAT) ? (reflow.preheat_tenths / 10)
                   : ((reflow.state == REFLOW_STATE_SOAKING) ? (reflow.soaking_tenths / 10)
                      : ((reflow.state == REFLOW_STATE_REFLOW)
                         ? (reflow.reflow_tenths / 10) : 50));
        (void)TextFormat(line, sizeof(line), "TAHAP:%s", reflow_state_name(reflow.state));
        ui_line(4U, line);
        (void)TextFormat(line, sizeof(line), "T:%dC %lus F:%u%%",
                         target_c, (unsigned long)reflow.elapsed_seconds,
                         fan_duty_percent);
        ui_line(5U, line);
      }
    }
    else if (conveyor.state == CONVEYOR_SEQ_HEATING_WAIT)
    {
      (void)TextFormat(line, sizeof(line), "HEAT %lus/5S",
                       (unsigned long)(conveyor.state_elapsed_ms / 1000UL));
      ui_line(4U, line);
      ui_line(5U, "DUTY:25%  BATAS:110C");
    }
    else
    {
      (void)TextFormat(line, sizeof(line), "PCB:%lu  IR:%s",
                       (unsigned long)inspection.board_id,
                       conveyor.ir_detected ? "YES" : "NO");
      ui_line(4U, line);
      (void)TextFormat(line, sizeof(line), "VIS:%s P:%u F:%u",
                       inspection.vision_online ? "OK" : "--",
                       inspection.pass_count, inspection.fail_count);
      ui_line(5U, line);
    }
    ui_line(6U, (conveyor.state == CONVEYOR_SEQ_INSPECTION)
             ? "B:PASS C:FAIL A:STOP" : "A:STOP D:DETAIL");
    ui_line(7U, "D!:HENTIKAN");
  }
  else if (ui_page == UI_PROFILE_MENU)
  {
    const char *items[4];
    char p[20], s[20], r[20];
    Reflow_GetStatus(&reflow);
    (void)TextFormat(p, sizeof(p), "PREHEAT    %3dC", reflow.preheat_tenths / 10);
    (void)TextFormat(s, sizeof(s), "SOAKING    %3dC", reflow.soaking_tenths / 10);
    (void)TextFormat(r, sizeof(r), "REFLOW     %3dC", reflow.reflow_tenths / 10);
    items[0] = p; items[1] = s; items[2] = r; items[3] = "TINJAU & MULAI";
    ui_list("PROFIL SUHU IDLE", items, 4U, ui_profile_selection, 1U, 4U);
    ui_line(5U, "BATAS P<S<R, GAP 5C");
    ui_line(6U, "90S / 75S / 75S");
    ui_line(7U, "B:^ C:v A:BUKA D:<");
  }
  else if (ui_page == UI_PROFILE_EDIT)
  {
    const char *name = (ui_edit_selection == 0U) ? "PREHEAT"
                       : ((ui_edit_selection == 1U) ? "SOAKING" : "REFLOW");
    (void)TextFormat(line, sizeof(line), "ATUR %s", name);
    ui_line(0U, line);
    (void)TextFormat(line, sizeof(line), "TARGET [ %ld C ]", (long)(ui_edit_value / 10));
    ui_line(2U, line);
    ui_line(4U, "LANGKAH 1C / IDLE");
    ui_line(5U, "URUTAN P < S < R");
    ui_line(6U, "B:+ C:- A:SIMPAN");
    ui_line(7U, "D:BATAL");
  }
  else if (ui_page == UI_PROFILE_CONFIRM)
  {
    Reflow_GetStatus(&reflow);
    ui_line(0U, "TINJAU PROFIL");
    (void)TextFormat(line, sizeof(line), "P:%dC 90S", reflow.preheat_tenths / 10);
    ui_line(1U, line);
    (void)TextFormat(line, sizeof(line), "S:%dC 75S", reflow.soaking_tenths / 10);
    ui_line(2U, line);
    (void)TextFormat(line, sizeof(line), "R:%dC 75S", reflow.reflow_tenths / 10);
    ui_line(3U, line);
    ui_line(4U, "COOLING <=50C");
    (void)TextFormat(line, sizeof(line), "PELAT:%sC", temperature);
    ui_line(5U, line);
    ui_line(6U, "HEATER AWAL: OFF");
    ui_line(7U, "A:MULAI D:KEMBALI");
  }
  else if (ui_page == UI_PROFILE_RUN)
  {
    if (ui_detail == 0U)
    {
      draw_reflow_screen();
      ui_line(7U, "A:STOP D:DETAIL");
    }
    else
    {
      Reflow_GetStatus(&reflow);
      ui_line(0U, "PROFIL SUHU DETAIL");
      (void)TextFormat(line, sizeof(line), "TAHAP %s", reflow_state_name(reflow.state));
      ui_line(1U, line);
      (void)TextFormat(line, sizeof(line), "PELAT %sC", temperature);
      ui_line(2U, line);
      (void)TextFormat(line, sizeof(line), "P:%d S:%d R:%d C",
                       reflow.preheat_tenths / 10,
                       reflow.soaking_tenths / 10,
                       reflow.reflow_tenths / 10);
      ui_line(3U, line);
      (void)TextFormat(line, sizeof(line), "WAKTU %lus",
                       (unsigned long)reflow.elapsed_seconds);
      ui_line(4U, line);
      (void)TextFormat(line, sizeof(line), "HEAT:%u%% FAN:%u%%",
                       heater_duty_percent, fan_duty_percent);
      ui_line(5U, line);
      ui_line(6U, "A:STOP D:GRAFIK");
      ui_line(7U, "D!:HENTIKAN");
    }
  }
  else if (ui_page == UI_CAL_MENU)
  {
    char p1[20], p2[20];
    const char *items[4];
    (void)TextFormat(p1, sizeof(p1), "TITIK 1 %s",
                     (calibration_captured_mask & 1U) ? "OK" : "BELUM");
    (void)TextFormat(p2, sizeof(p2), "TITIK 2 %s",
                     (calibration_captured_mask & 2U) ? "OK" : "BELUM");
    items[0] = p1; items[1] = p2;
    items[2] = cal_items[2]; items[3] = cal_items[3];
    ui_list("KALIBRASI NTC", items, 4U, ui_cal_selection, 1U, 4U);
    ui_line(5U, "P1/P2 HARUS VALID");
    ui_line(7U, "B:^ C:v A:BUKA D:<");
  }
  else if (ui_page == UI_CAL_EDIT)
  {
    char reference[10];
    format_temperature(reference, sizeof(reference), calibration_reference_tenths);
    (void)TextFormat(line, sizeof(line), "KAL NTC TITIK %u", calibration_point_index + 1U);
    ui_line(0U, line);
    (void)TextFormat(line, sizeof(line), "TERUKUR %sC", temperature);
    ui_line(1U, line);
    (void)TextFormat(line, sizeof(line), "REF [ %sC ]", reference);
    ui_line(2U, line);
    (void)TextFormat(line, sizeof(line), "ADC %u", latest_adc);
    ui_line(3U, line);
    ui_line(5U, "RUJUK TERMOMETER");
    ui_line(6U, "B:+.1 C:-.1");
    ui_line(7U, "A:AMBIL D:KEMBALI");
  }
  else if (ui_page == UI_CAL_HEATER)
  {
    ui_line(0U, "PEMANAS KALIBRASI");
    (void)TextFormat(line, sizeof(line), "PELAT %sC", temperature);
    ui_line(1U, line);
    (void)TextFormat(line, sizeof(line), "HEATER %s", heater_enabled ? "ON" : "OFF");
    ui_line(2U, line);
    (void)TextFormat(line, sizeof(line), "DUTY %u%%", heater_manual_duty_percent);
    ui_line(3U, line);
    ui_line(4U, "BATAS 150C");
    ui_line(6U, heater_enabled ? "A:OFF" : "A:ON B:+ C:-");
    ui_line(7U, "D:MATI & KEMBALI");
  }
  else if (ui_page == UI_CAL_SAVE)
  {
    ui_line(0U, "SIMPAN KALIBRASI");
    ui_line(2U, (calibration_captured_mask == 3U)
            ? "P1 & P2 TERAMBIL" : "P1/P2 BELUM LENGKAP");
    ui_line(4U, "HEATER AKAN MATI");
    ui_line(6U, "A:SIMPAN");
    ui_line(7U, "D:KEMBALI");
  }
  else if (ui_page == UI_PID_MENU)
  {
    const char *items[5];
    char sp[20], kp[20], ki[20], kd[20];
    (void)TextFormat(sp, sizeof(sp), "SETPOINT %d.%dC",
                     pid_setpoint_tenths / 10, pid_setpoint_tenths % 10);
    (void)TextFormat(kp, sizeof(kp), "KP       %d.%02d",
                     (int)pid_kp, (int)(pid_kp * 100.0f) % 100);
    (void)TextFormat(ki, sizeof(ki), "KI       0.%03d", (int)(pid_ki * 1000.0f));
    (void)TextFormat(kd, sizeof(kd), "KD       %d.%02d",
                     (int)pid_kd, (int)(pid_kd * 100.0f) % 100);
    items[0] = sp; items[1] = kp; items[2] = ki; items[3] = kd;
    items[4] = "MULAI PID";
    ui_list("KENDALI PID IDLE", items, 5U, ui_pid_selection, 1U, 5U);
    ui_line(6U, "NILAI HANYA SESI INI");
    ui_line(7U, "B:^ C:v A:BUKA D:<");
  }
  else if (ui_page == UI_PID_EDIT)
  {
    ui_line(0U, "ATUR PID");
    if (ui_edit_selection == 0U)
      (void)TextFormat(line, sizeof(line), "SETPOINT [%ld.%ldC]",
                       (long)(ui_edit_value / 10), (long)(ui_edit_value % 10));
    else if (ui_edit_selection == 2U)
      (void)TextFormat(line, sizeof(line), "KI [0.%03ld]", (long)ui_edit_value);
    else
      (void)TextFormat(line, sizeof(line), "%s [%ld.%02ld]",
                       (ui_edit_selection == 1U) ? "KP" : "KD",
                       (long)(ui_edit_value / 100), (long)(ui_edit_value % 100));
    ui_line(2U, line);
    ui_line(4U, "PID HARUS IDLE");
    ui_line(5U, "NILAI HANYA SESI INI");
    ui_line(6U, "B:+ C:- A:SIMPAN");
    ui_line(7U, "D:BATAL");
  }
  else if (ui_page == UI_PID_RUN)
  {
    draw_pid_screen();
    if (ui_detail != 0U)
    {
      ui_line(0U, "PID DETAIL");
      (void)TextFormat(line, sizeof(line), "KP:%d.%02d KI:%d.%03d",
                       (int)pid_kp, (int)(pid_kp * 100.0f) % 100,
                       (int)pid_ki, (int)(pid_ki * 1000.0f) % 1000);
      ui_line(4U, line);
      (void)TextFormat(line, sizeof(line), "KD:%d.%02d",
                       (int)pid_kd, (int)(pid_kd * 100.0f) % 100);
      ui_line(5U, line);
    }
    ui_line(6U, "A:STOP D:DETAIL");
    ui_line(7U, "D!:HENTIKAN");
  }
  else if (ui_page == UI_DIAG_MENU)
  {
    ui_list("DIAGNOSTIK", diag_items, 5U, ui_diag_selection, 1U, 4U);
    ui_line(5U, "UJI & MONITOR TASK");
    ui_line(6U, "PEAK SEJAK BOOT");
    ui_line(7U, "B:^ C:v A:BUKA D:<");
  }
  else if (ui_page == UI_DIAG_RUN)
  {
    if (current_screen == SCREEN_CHARACTERIZATION) draw_characterization_screen();
    else if (current_screen == SCREEN_MOTOR_TEST)  draw_motor_test_screen();
    else if (current_screen == SCREEN_SERVO_TEST)  draw_servo_test_screen();
    else if (current_screen == SCREEN_TASK_STACK)  draw_task_stack_screen();
    else draw_heater_screen(((uint32_t)latest_adc * ADC_REFERENCE_MV) / ADC_FULL_SCALE);
    if ((current_screen != SCREEN_SERVO_TEST)
        && (current_screen != SCREEN_TASK_STACK))
    {
      ui_line(6U, ui_is_active() ? "A:STOP" : "A:MULAI B:+ C:-");
      ui_line(7U, "D:MATI & KEMBALI");
    }
  }
  else if (ui_page == UI_STATUS)
  {
    ConveyorApp_GetStatus(&conveyor);
    Inspection_GetStatus(&inspection);
    ui_line(0U, "STATUS ALAT");
    (void)TextFormat(line, sizeof(line), "PEL %sC CAL:%s", temperature,
                     Thermistor_IsCalibrated() ? "OK" : "NO");
    ui_line(1U, line);
    (void)TextFormat(line, sizeof(line), "ADC %u IR:%s", latest_adc,
                     conveyor.ir_detected ? "YES" : "NO");
    ui_line(2U, line);
    ui_line(3U, inspection.vision_online ? "VISION:OK" : "VISION:OFFLINE");
    (void)TextFormat(line, sizeof(line), "HEATER:%s FAN:%u%%",
                     heater_enabled ? "ON" : "OFF", fan_duty_percent);
    ui_line(4U, line);
    (void)TextFormat(line, sizeof(line), "MOTOR:%u%% LOCK:%s",
                     conveyor.motor_percent, process_owner_name(conveyor.process_owner));
    ui_line(5U, line);
    ui_line(7U, "D:BERANDA");
  }
  else if (ui_page == UI_MESSAGE)
  {
    ui_line(0U, "! STATUS ALAT");
    ui_line(2U, ui_message_text());
    if (ui_message == UI_MSG_SN63_LOCKED)
      ui_line(3U, "PCB TANPA TERMOKOPEL");
    if ((ui_message == UI_MSG_CAPTURED) || (ui_message == UI_MSG_SAVED)
        || (ui_message == UI_MSG_FINISHED))
    {
      uint32_t width = (HAL_GetTick() - ui_message_at) / 4U;
      if (width > 116U) width = 116U;
      SSD1306_DrawLine(&oled, 0U, 30U, (uint8_t)width, 30U);
    }
    (void)TextFormat(line, sizeof(line), "PELAT:%sC", temperature);
    ui_line(4U, line);
    ui_line(6U, "A:AKUI");
    ui_line(7U, "D:KEMBALI");
  }
}

/* =========================================================================
 * ui_poll_process() — Process event monitor (called from InputRun)
 * ======================================================================= */

void ui_poll_process(void)
{
  ConveyorAppStatus conveyor;
  ReflowStatus reflow;

  /* Auto-dismiss the CAPTURED calibration point message after 450 ms. */
  if ((ui_page == UI_MESSAGE) && (ui_message == UI_MSG_CAPTURED)
      && ((HAL_GetTick() - ui_message_at) >= 450U))
  {
    ui_open(UI_CAL_MENU);
    return;
  }

  if (ui_page == UI_E2E_RUN)
  {
    ConveyorApp_GetStatus(&conveyor);
    if (conveyor.state != CONVEYOR_SEQ_IDLE) ui_run_seen = 1U;
    if ((conveyor.state == CONVEYOR_SEQ_MOTOR_FAULT)
        || (conveyor.state == CONVEYOR_SEQ_PCB_TIMEOUT)
        || (conveyor.state == CONVEYOR_SEQ_HEATER_FAULT))
      ui_show_message(UI_MSG_FAULT, UI_E2E_RUN);
    else if (conveyor.state == CONVEYOR_SEQ_ESTOP)
      ui_show_message(UI_MSG_STOPPED, UI_HOME);
    else if ((ui_run_seen != 0U) && (conveyor.state == CONVEYOR_SEQ_IDLE))
    {
      ui_finished_e2e = 1U;
      ui_show_message(UI_MSG_FINISHED, UI_HOME);
    }
  }
  else if (ui_page == UI_PROFILE_RUN)
  {
    Reflow_GetStatus(&reflow);
    if (reflow.state != REFLOW_STATE_IDLE) ui_run_seen = 1U;
    if ((ui_run_seen != 0U) && (reflow.state == REFLOW_STATE_IDLE))
    {
      ui_finished_e2e = 0U;
      ui_show_message(reflow.fault ? UI_MSG_FAULT : UI_MSG_FINISHED, UI_HOME);
    }
    else if ((ui_run_seen == 0U) && (reflow.fault != 0U))
      ui_show_message(UI_MSG_FAULT, UI_PROFILE_MENU);
  }
  else if ((ui_page == UI_PID_RUN)
           && ((pid_fault != 0U) || (thermal_runaway_fault != 0U)))
    ui_show_message(UI_MSG_FAULT, UI_PID_MENU);
  else if ((ui_page == UI_DIAG_RUN)
           && (current_screen == SCREEN_HEATER)
           && (thermal_runaway_fault != 0U))
    ui_show_message(UI_MSG_FAULT, UI_DIAG_RUN);
  else if ((ui_page == UI_DIAG_RUN)
           && (current_screen == SCREEN_CHARACTERIZATION))
  {
    HeaterCharacterizationStatus status;
    HeaterCharacterization_GetStatus(&status);
    if (status.state != HEATER_CHARACTERIZATION_IDLE)
      ui_diag_start_pending = 0U;
  }
}

/* =========================================================================
 * ht_ui_init() — Called from HardwareTest_Init()
 * ======================================================================= */

void ht_ui_init(void)
{
  current_screen       = SCREEN_HOME;
  ui_page              = UI_HOME;
  oled_ready = (SSD1306_Init(&oled, &hi2c1, OLED_ADDRESS_7BIT) == HAL_OK)
               ? 1U : 0U;
  last_display_update  = HAL_GetTick() - DISPLAY_UPDATE_MS;
}
