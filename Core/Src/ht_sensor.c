/* ht_sensor.c — NTC thermistor ADC reading and calibration reference helpers.
 *
 * Owns: latest_adc, latest_temperature_tenths, latest_resistance_ohm,
 *       latest_temperature_valid, calibration_point_index,
 *       calibration_captured_mask, calibration_reference_tracks_measurement,
 *       calibration_reference_tenths.
 *
 * Called from: hardware_test.c (HardwareTest_Run / HardwareTest_Init).
 */

#include "ht_private.h"

#include "adc.h"
#include "thermistor.h"

#include <stdint.h>

/* ---- ADC DMA configuration ---------------------------------------------- */
#define ADC_FULL_SCALE        4095U
#define ADC_DMA_BUFFER_SIZE     32U

/* Circular DMA buffer automatically filled by DMA1_Channel1 on TIM3 TRGO triggers */
static volatile uint16_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE];
static uint8_t adc_dma_started = 0U;

/* ---- State --------------------------------------------------------------- */
uint16_t latest_adc;
int16_t  latest_temperature_tenths;
uint32_t latest_resistance_ohm;
uint8_t  latest_temperature_valid;

uint8_t  calibration_point_index;
uint8_t  calibration_captured_mask;
uint8_t  calibration_reference_tracks_measurement;
int16_t  calibration_reference_tenths;

/* ---- ht_sensor_init() ---------------------------------------------------- */
/* Starts continuous circular DMA sampling triggered by TIM3 TRGO. */
void ht_sensor_init(void)
{
  if (adc_dma_started == 0U)
  {
    (void)HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_DMA_BUFFER_SIZE);
    adc_dma_started = 1U;
  }
}

/* ---- read_adc_average() -------------------------------------------------- */
/* Computes the average of the circular DMA buffer with the highest and lowest
 * samples discarded for outlier rejection. Lock-free and non-blocking. */
uint16_t read_adc_average(void)
{
  if (adc_dma_started == 0U)
  {
    ht_sensor_init();
  }

  uint32_t total = 0U;
  uint16_t minimum = ADC_FULL_SCALE;
  uint16_t maximum = 0U;

  for (uint8_t i = 0U; i < ADC_DMA_BUFFER_SIZE; ++i)
  {
    uint16_t sample = adc_dma_buffer[i];
    total += sample;
    if (sample < minimum) minimum = sample;
    if (sample > maximum) maximum = sample;
  }

  /* Discard single highest and lowest samples (outlier rejection) */
  total -= minimum;
  total -= maximum;

  return (uint16_t)((total + ((ADC_DMA_BUFFER_SIZE - 2U) / 2U)) / (ADC_DMA_BUFFER_SIZE - 2U));
}

/* ---- adjust_calibration_reference() -------------------------------------- */
/* Moves the calibration reference temperature by change_tenths (can be
 * positive or negative) in 0.1 °C steps; called by the UI auto-repeat. */
void adjust_calibration_reference(int16_t change_tenths)
{
  calibration_reference_tracks_measurement = 0U;
  calibration_reference_tenths =
      (int16_t)(calibration_reference_tenths + change_tenths);
}
