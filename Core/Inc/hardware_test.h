#ifndef HARDWARE_TEST_H
#define HARDWARE_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  HARDWARE_TEST_SCREEN_HEATER = 0,
  HARDWARE_TEST_SCREEN_CALIBRATION,
  HARDWARE_TEST_SCREEN_PID,
  HARDWARE_TEST_SCREEN_REFLOW,
  HARDWARE_TEST_SCREEN_CHARACTERIZATION,
  HARDWARE_TEST_SCREEN_MOTOR_TEST,
  HARDWARE_TEST_SCREEN_CONVEYOR,
  HARDWARE_TEST_SCREEN_HOME,
  HARDWARE_TEST_SCREEN_SERVO_TEST,
  HARDWARE_TEST_SCREEN_TASK_STACK
} HardwareTestScreen;

typedef struct
{
  HardwareTestScreen screen;
  uint16_t adc;
  int16_t temperature_tenths;
  uint32_t resistance_ohm;
  int16_t pid_setpoint_tenths;
  uint8_t temperature_valid;
  uint8_t thermistor_calibrated;
  uint8_t heater_enabled;
  uint8_t heater_duty_percent;
  uint8_t fan_duty_percent;
  uint8_t pid_running;
  uint8_t pid_fault;
  uint8_t thermal_runaway_fault;
} HardwareTestStatus;

void HardwareTest_Init(void);
void HardwareTest_InputRun(void);
void HardwareTest_Run(void);
void HardwareTest_GetStatus(HardwareTestStatus *status);
uint8_t HardwareTest_PIDStartAt(int16_t setpoint_tenths);
void HardwareTest_PIDSetSetpoint(int16_t setpoint_tenths);
void HardwareTest_PIDStop(void);
void HardwareTest_FanSetDuty(uint8_t duty_percent);
uint8_t HardwareTest_HeaterStartAtDuty(uint8_t duty_percent,
                                       int16_t safety_limit_tenths);
void HardwareTest_HeaterStop(void);

#ifdef __cplusplus
}
#endif

#endif /* HARDWARE_TEST_H */
