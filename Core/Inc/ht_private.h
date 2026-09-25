/* ht_private.h — Internal header shared by hardware_test.c and its sub-modules.
 *
 * IMPORTANT: Do NOT include this file from public headers or from any .c file
 * outside Core/Src/. All declarations here are implementation details.
 *
 * Each state variable is *defined* in one .c file (noted in the comment) and
 * declared extern here so every other module can read or write it directly.
 * This keeps the split clean without getter/setter overhead on Cortex-M3.
 */

#ifndef HT_PRIVATE_H
#define HT_PRIVATE_H

#include <stdint.h>
#include <stddef.h>
#include "ssd1306.h"

/* -------------------------------------------------------------------------
 * Shared constants
 * ---------------------------------------------------------------------- */
#ifndef OLED_ADDRESS_7BIT
#define OLED_ADDRESS_7BIT           0x3cU
#endif
#ifndef DISPLAY_UPDATE_MS
#define DISPLAY_UPDATE_MS           50U
#endif
#ifndef CALIBRATION_MAX_TEMP_TENTHS
#define CALIBRATION_MAX_TEMP_TENTHS 1500
#endif
#ifndef PID_SETPOINT_MIN_TENTHS
#define PID_SETPOINT_MIN_TENTHS     200
#endif
#ifndef PID_SETPOINT_MAX_TENTHS
#define PID_SETPOINT_MAX_TENTHS    1400
#endif

/* -------------------------------------------------------------------------
 * Enumerations (defined here, used by both ht_ui.c and ht_heater/pid/sensor)
 * ---------------------------------------------------------------------- */

/* Current active OLED screen layout. Mirrors HardwareTestScreen in the public
 * header but is kept separate to avoid exposing internal screen IDs publicly. */
typedef enum
{
  SCREEN_HEATER = 0,
  SCREEN_THERMISTOR_CALIBRATION,
  SCREEN_PID,
  SCREEN_REFLOW,
  SCREEN_CHARACTERIZATION,
  SCREEN_MOTOR_TEST,
  SCREEN_CONVEYOR,
  SCREEN_HOME,
  SCREEN_SERVO_TEST,
  SCREEN_TASK_STACK
} Screen;

typedef enum
{
  UI_HOME,
  UI_E2E_PREP,
  UI_E2E_CONFIRM,
  UI_E2E_RUN,
  UI_PROFILE_MENU,
  UI_PROFILE_EDIT,
  UI_PROFILE_CONFIRM,
  UI_PROFILE_RUN,
  UI_CAL_MENU,
  UI_CAL_EDIT,
  UI_CAL_HEATER,
  UI_CAL_SAVE,
  UI_PID_MENU,
  UI_PID_EDIT,
  UI_PID_RUN,
  UI_DIAG_MENU,
  UI_DIAG_RUN,
  UI_STATUS,
  UI_MESSAGE
} UiPage;

typedef enum
{
  UI_MSG_NONE,
  UI_MSG_SENSOR,
  UI_MSG_CALIBRATION,
  UI_MSG_BUSY,
  UI_MSG_HOT,
  UI_MSG_LIMIT,
  UI_MSG_CAPTURED,
  UI_MSG_SAVED,
  UI_MSG_SAVE_FAILED,
  UI_MSG_STOPPED,
  UI_MSG_FINISHED,
  UI_MSG_SN63_LOCKED,
  UI_MSG_FAULT
} UiMessage;

/* -------------------------------------------------------------------------
 * State owned by hardware_test.c (coordinator)
 * ---------------------------------------------------------------------- */
extern volatile uint8_t hardware_initialized; /* defined in hardware_test.c */

/* -------------------------------------------------------------------------
 * State owned by ht_sensor.c
 * ---------------------------------------------------------------------- */
extern uint16_t latest_adc;
extern int16_t  latest_temperature_tenths;
extern uint32_t latest_resistance_ohm;
extern uint8_t  latest_temperature_valid;
extern uint8_t  calibration_point_index;
extern uint8_t  calibration_captured_mask;
extern uint8_t  calibration_reference_tracks_measurement;
extern int16_t  calibration_reference_tenths;

/* -------------------------------------------------------------------------
 * State owned by ht_heater.c
 * ---------------------------------------------------------------------- */
extern uint8_t          heater_enabled;
extern uint8_t          heater_duty_percent;
extern uint8_t          heater_manual_duty_percent;
extern uint8_t          fan_duty_percent;
extern uint8_t          heater_window_active;
extern uint32_t         heater_window_started_at;
extern uint32_t         heater_on_time_ms;
extern volatile int16_t heater_safety_limit_tenths;
extern uint8_t          thermal_runaway_fault;
extern uint8_t          thermal_runaway_active;
extern uint32_t         thermal_runaway_timer;
extern int16_t          thermal_runaway_ref_tenths;

/* -------------------------------------------------------------------------
 * State owned by ht_pid.c
 * ---------------------------------------------------------------------- */
extern int16_t pid_setpoint_tenths;
extern uint8_t pid_running;
extern uint8_t pid_fault;
extern float   pid_kp;
extern float   pid_ki;
extern float   pid_kd;
extern float   pid_ramped_setpoint;
extern float   pid_output_percent;
extern float   pid_p_term;
extern float   pid_i_term;
extern float   pid_d_term;

/* -------------------------------------------------------------------------
 * State owned by ht_ui.c
 * ---------------------------------------------------------------------- */
extern SSD1306_HandleTypeDef oled;
extern uint8_t  oled_ready;
extern uint32_t last_display_update;
extern Screen   current_screen;
extern UiPage   ui_page;

/* -------------------------------------------------------------------------
 * Internal functions — ht_sensor.c
 * ---------------------------------------------------------------------- */
void     ht_sensor_init(void);
uint16_t read_adc_average(void);
void     adjust_calibration_reference(int16_t change_tenths);

/* -------------------------------------------------------------------------
 * Internal functions — ht_heater.c
 * ---------------------------------------------------------------------- */
void set_heater_output(void);
void set_fan_output(uint8_t duty_percent);
void update_heater_output(void);
void enforce_direct_heater_safety(void);
void check_thermal_runaway(void);
void trigger_thermal_runaway_fault(void);
void ht_heater_init(void);

/* -------------------------------------------------------------------------
 * Internal functions — ht_pid.c
 * ---------------------------------------------------------------------- */
float   clamp_float(float value, float minimum, float maximum);
void    pid_apply_output(float output_percent);
void    pid_stop(uint8_t fault);
uint8_t pid_start(void);
void    update_pid_controller(void);

/* -------------------------------------------------------------------------
 * Internal functions — ht_ui.c
 * ---------------------------------------------------------------------- */
void update_buttons(void);
void update_button_auto_repeat(void);
void update_controls(void);
void ui_draw(void);
void ui_poll_process(void);
void ht_ui_init(void);

#endif /* HT_PRIVATE_H */
