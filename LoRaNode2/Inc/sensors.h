/* board.h
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdio.h>

#define DEBOUNCE_THRESHOLD		400
#define FFT_SAMPLES				2048
#define FFT_SIZE				1024
#define BLOCK_SIZE            	32
#define NUM_TAPS              	21
#define VIBE_SIZE				105
#define NUM_TEMP_SENSORS		3

enum {
	SAMPLES = 1024, // First sample ignored so add 1
	SIGNALS = 10
};


/**External functions*/
void make_sig_array(void);
uint8_t ADC_get(void);
void ADC_fill_buffer(uint16_t col, uint8_t row);
uint8_t ADC_get_buffer(uint16_t pos);
void signal_averaging(void);
void vibe_fft(void);
void vibe_filter(void);
void log_temp(void);
void set_payload(void);
uint8_t convert_to_temp(uint8_t *tsVal, uint8_t sensor);

#endif /* BOARD_BOARD_H_ */

