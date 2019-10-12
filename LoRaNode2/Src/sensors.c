/**
 ******************************************************************************
 * @file    research/board.c
 * @author  Radley Scott
 * @date    15012019
 * @brief	Board hardware and peripheral initialisation for use on the sensor
 * 			board.
 *
 * External functions
 *				void BRD_init(void)
 *				void BRD_led_on(void)
 *				void BRD_led_off(void)
 *				void BRD_led_toggle(void)
 *				uint8_t BRD_button_pushed(void)
 *				void BRD_button_unpush(void)
 *				void BRD_debuguart_putc(unsigned char c)
 *				void ADC_init(void)
 *				void ADC_Deinit(void)
 *				uint8_t ADC_get(void)
 *				void ADC_fill_buffer(uint16_t pos)
 *				uint8_t ADC_get_buffer(uint16_t pos)
 *				void TIM2_Init(void)
 *				void TIM2_Deinit(void)
 *				void HAL_Delayus(uint32_t us)
 *				void BRD_delay(int counter)
 *				extern void debug_putc(char c)
 *				extern void debug_flush()
 *				extern unsigned char debug_getc()
 *				extern void debug_rxflush()
 *				extern void debug_printf (const char *fmt, ...)
 ******************************************************************************
 */

/* ----------------------------------------------------------------------
** Includes
** ------------------------------------------------------------------- */
#include "sensors.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include <stdarg.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sx1272.h"

/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;

extern I2C_HandleTypeDef hi2c1;
/* ----------------------------------------------------------------------
** Global Variables
** ------------------------------------------------------------------- */
uint8_t adcBuffer[SAMPLES];
uint8_t ave_sig[SAMPLES];
uint8_t raw_signals[SAMPLES * SIGNALS];
float32_t fftInput[FFT_SAMPLES];
uint8_t vibrationOutput[VIBE_SIZE];
float32_t dspOut[FFT_SIZE];
float32_t filtInput[FFT_SIZE];
uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = FFT_SIZE/BLOCK_SIZE;
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
static uint8_t adcValue = 0;
uint8_t ts[2], data[PAYLOAD_LENGTH];
uint8_t sigALERT = 0, tempALERT;
extern uint8_t tempOutput[3];
//FIR filter coefficients calculated in MATLAB using fir1(20, 2kHz/10kHz/2)
const float32_t firCoeffs32[NUM_TAPS] = {
		0.0, -0.00212227114882539, -0.00632535399151418, -0.0116118103776210, -0.0123546567489824,
		0.0, 0.0317744975585673, 0.0814359075642177, 0.137493781701943, 0.182125490388735,
		0.199168830106960, 0.182125490388735, 0.137493781701943, 0.0814359075642177, 0.0317744975585673,
		0.0, -0.0123546567489824, -0.0116118103776210, -0.00632535399151418, -0.00212227114882539,
		0.0
};

/* ----------------------------------------------------------------------
** Function Prototypes
** ------------------------------------------------------------------- */

/**
 * @brief  Sets signal array for averaging signals
 * @param  None
 * @retval None
 */
void make_sig_array(void) {

	for (uint16_t i = 0; i < SAMPLES; i++) {
		ave_sig[i] = 0;
	}
}

/**
 * @brief  Return ADC value from ADC buffer at specified position
 * @param  uint16_t pos < SAMPLES
 * @retval None
 */
uint8_t ADC_get_buffer(uint16_t pos) {
	return adcBuffer[pos];
}


/**
  * @brief  Filters captured data - LPF: 2kHz
  * @param  None
  * @retval None
  */
void vibe_filter(void) {

	arm_fir_instance_f32 Filt;

	/* Offset DC component and scale up values from ADC data */
	for (int i = 0; i < SAMPLES; i++) {
		filtInput[i] = ((float32_t)ADC_get_buffer(i) - 127);
	}

	/* Call FIR init function to initialize the instance structure. */
	arm_fir_init_f32(&Filt, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);

	/* Call the FIR process function for every blockSize samples */
	for(uint32_t i=0; i < numBlocks; i++) {
		arm_fir_f32(&Filt, &filtInput[0] + (i * blockSize), &dspOut[0] + (i * blockSize), blockSize);
	}
}


/**
  * @brief  Performs Fast Fourier Transform of vibration sensor data
  * @param  None
  * @retval None
  */
void vibe_fft(void) {

	arm_cfft_radix4_instance_f32 Signal;	/* ARM CFFT module */

	/* Change to complex form of ADC data */
	for (uint16_t i = 0; i < FFT_SAMPLES; i += 2) {
		/* Real part */
		fftInput[(uint16_t)i] = dspOut[((uint16_t)i)/2];
		/* Imaginary part */
		fftInput[(uint16_t)(i + 1)] = 0;
	}

	/* Initialize the CFFT/CIFFT module */
	arm_cfft_radix4_init_f32(&Signal, FFT_SIZE, 0, 1);

	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_radix4_f32(&Signal, fftInput);

	/* Process the data through the Complex Magnitude Module for calculating the magnitude at each bin */
	arm_cmplx_mag_f32(fftInput, dspOut, FFT_SIZE);

	/* Set up package of 105 samples*/
	for (int i = 0; i < 105; i++) {
		vibrationOutput[i] = 0;//(uint8_t)(dspOut[i]/256);
		vibrationOutput[40] = 126;
		if (vibrationOutput[i] >= 127) {
			sigALERT = 1;
		}
	}
	/* Remove DC - 50Hz frequency content */
	for (int i = 0; i < 1; i++) {
		vibrationOutput[i] = 0;
	}
}

/**
 * @brief  Return current converted ADC value
 * @param  None
 * @retval uint8_t adcValue
 */
uint8_t ADC_get(void) {
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        adcValue = HAL_ADC_GetValue(&hadc1);
    }

    /* Start new conversion */
    HAL_ADC_Start(&hadc1);

    return adcValue;
}


/**
 * @brief  Place current converted ADC value into
 * ADC buffer at specified position
 * @param  uint16_t pos < SAMPLES
 * @retval None
 */
void ADC_fill_buffer(uint16_t col, uint8_t row) {

	*(raw_signals + row*SAMPLES + col) = ADC_get();
}

/**
 * @brief  Function to remove noise by signal averaging
 * @param  None
 * @retval None
 */
void signal_averaging(void) {

	for (uint16_t j = 0; j < SAMPLES; j++) {

		for (uint8_t i = 0; i < SIGNALS; i++) {

			ave_sig [j] = ave_sig[j] + *(raw_signals + i*SAMPLES + j);
		}

		ave_sig[j] = ave_sig[j] / 10;
   }

	for (int i = 0; i < 1024; i++) {
		adcBuffer[i] = ave_sig[i];
	}

}


void log_temp(void) {
    uint8_t ts[2];

    HAL_I2C_Mem_Read(&hi2c1, 0x51 << 1, 0x7, 1, ts, 2, 100);
    tempOutput[0] = convert_to_temp(ts, 0);

    HAL_I2C_Mem_Read(&hi2c1, 0x52 << 1, 0x7, 1, ts, 2, 100);
    tempOutput[1] = convert_to_temp(ts, 1);

    HAL_I2C_Mem_Read(&hi2c1, 0x53 << 1, 0x7, 1, ts, 2, 100);
    tempOutput[2] = convert_to_temp(ts, 2);
}

uint8_t convert_to_temp(uint8_t *tsVal, uint8_t sensor) {
    float temp = 0.0;

    temp = (tsVal[1] << 8) | tsVal[0];
    temp = temp * 0.02 - 273.15;

    temp = 41;
    if (temp > 40) {
    	tempALERT = tempALERT | (1 << sensor);
    }

    return (uint8_t) temp;
}

/**
* @brief Sets up payload to be sent
* @param None
* @retval None
*/
void set_payload(void) {

    /*Pack Temperature data*/
    for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
        data[i] = tempOutput[i];
    }

    /*Pack vibration data*/
    for(int i = 0; i < VIBE_SIZE; i++) {
        data[i + NUM_TEMP_SENSORS] = vibrationOutput[i];
    }

    data[PAYLOAD_LENGTH - 2] = tempALERT;
    data[PAYLOAD_LENGTH - 1] = sigALERT;

}
