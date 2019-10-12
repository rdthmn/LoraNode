/**
 * Driver for SX1272 LoRa nodes using STM34F446RE. This is a 2018-19 summer research project under supervision of Dr. Chandima
 * Ekanyake and Dr. Hui Ma, School of ITEE, The University of Queensland, Brisbane, QLD, Australia. The project aims to
 * create a LPWAN network for stand-alone, pole-mounted transformers within Queensland teritory.
 *
 * Created on: 23 November, 2018
 * Author: Trung Q. Cao - 44592394
 */


#ifndef SX1272_H_
#define SX1272_H_

#include "stm32f4xx_hal.h"

/* Registers Map */
#define REG_LR_FIFO									0x00
#define REG_LR_OP_MODE								0x01
#define REG_LR_FREQ_MSB								0x06
#define REG_LR_FREQ_MIB								0x07
#define REG_LR_FREQ_LSB								0x08
#define REG_LR_PA_CONFIG							0x09

#define REG_LR_PA_RAMP								0x0A
#define REG_LR_OCP									0x0B
#define REG_LR_LNA									0x0C
#define REG_LR_FIFOADDRPTR                          0x0D
#define REG_LR_FIFOTXBASEADDR                       0x0E
#define REG_LR_FIFORXBASEADDR                       0x0F
#define REG_LR_FIFORXCURRENTADDR 				  	0x10
#define REG_LR_IRQFLAGSMASK                         0x11
#define REG_LR_IRQFLAGS                             0x12
#define REG_LR_RXNBBYTES                            0x13
#define REG_LR_RXHEADERCNTVALUE_MSB                 0x14
#define REG_LR_RXHEADERCNTVALUE_LSB                 0x15
#define REG_LR_RXPACKETCNTVALUE_MSB                 0x16
#define REG_LR_RXPACKETCNTVALUE_LSB                 0x17
#define REG_LR_MODEMSTAT                            0x18
#define REG_LR_PKTSNRVALUE                          0x19
#define REG_LR_PKTRSSIVALUE                         0x1A
#define REG_LR_RSSIVALUE                            0x1B
#define REG_LR_HOPCHANNEL                           0x1C
#define REG_LR_MODEMCONFIG1                         0x1D
#define REG_LR_MODEMCONFIG2                         0x1E
#define REG_LR_SYMBTIMEOUTLSB                       0x1F
#define REG_LR_PREAMBLEMSB                          0x20
#define REG_LR_PREAMBLELSB                          0x21
#define REG_LR_PAYLOADLENGTH                        0x22 // and RX length for implicit
#define REG_LR_RX_MAX_PAYLOADLENGTH                 0x23 // length limit for explicit mode
#define REG_LR_HOPPERIOD                            0x24
#define REG_LR_RXBYTEADDR 							0x25
#define REG_LR_FREQ_ERR_MSB                         0x28  // est_freq_error
#define REG_LR_FREQ_ERR_MIB                         0x29  // est_freq_error
#define REG_LR_FREQ_ERR_LSB                         0x2A  // est_freq_error
#define REG_LR_RSSI_WIDEBAND                        0x2C
#define REG_LR_DECTECT_OPTIMIZE                     0x31  // if_freq_auto, ...
#define REG_LR_INVERT_IQ                            0x33  // invert IQ
#define REG_LR_DETECTION_THRESHOLD                  0x37
#define REG_LR_SYNC_WORD                            0x39  // default 0x12 (value of 0x21 will isolate network)
#define REG_LR_GAIN_DRIFT                           0x3A
#define REG_LR_DRIFT_INVERT                         0x3B

#define REG_IO_MAPPING_1                            0x40
#define REG_IO_MAPPING_2                            0x41
#define REG_AGC_REF                                 0x43
#define REG_AGC_THRES_1                             0x44
#define REG_AGC_THRES_2                             0x45
#define REG_AGC_THRES_3                             0x46
#define REG_PLL_HOP                                 0x4B
#define REG_TCXO                                    0x58
#define REG_PA_DAC                                  0x5A
#define REG_PLL                                     0x5C
#define REG_PLL_LOW_PN                              0x5E
#define REG_BIT_RATE_FRAC                           0x70


#define MAX_PACKET_LENGTH                           255
#define PAYLOAD_LENGTH                              108
#define HEADER_LENGTH                               2
#define CRC_LENGTH                                  2

#define MAP_DIO0_LORA_RXDONE   0x00  // 00------
#define MAP_DIO0_LORA_TXDONE   0x40  // 01------
#define MAP_DIO1_LORA_RXTOUT   0x00  // --00----
#define MAP_DIO1_LORA_NOP      0x30  // --11----
#define MAP_DIO2_LORA_NOP      0xC0  // ----11--

/* Bit options */
/* ---------- RegOpMode ---------- */
#define LRANGE_MODE 	7
#define REG_SHARE 		6
#define MODE 			0

typedef enum {
	LORA = 1,
	FSK = 0,
} lrange_mode_t;

typedef enum {
	LORA_ACCESS = 0,
	FSK_ACCESS = 1,
} reg_share_t;

typedef enum {
	SLEEP = 0,
	STDBY = 1,
	FSTX = 2,
	TX = 3,
	FSRX = 4,
	RX_CONT = 5,
	RX_SINGLE = 6,
	CAD = 7,
} op_mode_t;

/* ---------- RegPaConfig ---------- */
#define PA_SELECT 		7
#define POWER_OUT 		0

typedef enum {
	RFIO = 0,
	PA_BOOST = 1,
} pa_select_t;

/* ---------- RegPaRamp ---------- */
#define PLL_OPT			4
#define PA_RAMP			0

typedef enum {
	LP_RTX = 1,
	PL_RX_LN_TX = 0,
} ppl_off_t;

typedef enum {
	R_3400U = 0,
	R_2M = 1,
	R_1M = 2,
	R_500U = 3,
	R_250U = 4,
	R_125U = 5,
	R_100U = 6,
	R_62U = 7,
	R_50U = 8,
	R_40U = 9,
	R_31U = 10,
	R_25U = 11,
	R_20U = 12,
	R_15U = 13,
	R_12U = 14,
	R_10U = 15,
} pa_ramp_t;

/* ---------- RegOcp ---------- */
#define OCP 			5
#define OCP_TRIM 		0

typedef enum {
	OCP_DISABLED = 0,
	OCP_ENABLED = 1,
} ocp_t;

/* ---------- RegLna ---------- */
#define LNA_GAIN 			5
#define LAN_BOOST 			0

typedef enum {
	GAIN_OFF = 0,
	GAIN_MAX = 1,
	GAIN_2 = 2,
	GAIN_3 = 3,
	GAIN_4 = 4,
	GAIN_5 = 5,
	GAIN_MIN = 6,
} lna_gain_t;

typedef enum {
	BOOST_OFF = 0,
	BOOST_50 = 1,
	BOOST_100 = 2,
	BOOST_150 = 3,
} lna_boost_t;

/* ---------- RegIrqFlagsMask ---------- */
#define RXTIMEOUT_MASK 				7
#define RXDONE_MASK 				6
#define CRC_ERR_MASK				5
#define VALID_HDR_MASK				4
#define TX_DONE_MASK				3
#define CAD_DONE_MASK				2
#define FHSS_CHANGE_CH_MASK			1
#define CAD_DETECTED_MASK			0

/* ---------- RegIrqFlags ---------- */
#define RXTIMEOUT 					0x80
#define RXDONE 						0x40
#define CRC_ERR						0x20
#define VALID_HDR					0x10
#define TX_DONE						0x8
#define CAD_DONE					0x4
#define FHSS_CHANGE_CH				0x2
#define CAD_DETECTED				0x1

/* ---------- RegModemConfig1 ---------- */
#define BAND_WIDTH					6
#define CODING_RATE					3
#define HEADER_MODE					2
#define RX_CRC						1
#define LOW_DATA_RATE_OPT			0

typedef enum {
	BW_125K = 0,
	BW_250K = 1,
	BW_500K = 2,
} bandwidth_t;

typedef enum {
	CR_4_5 = 1,
	CR_4_6 = 2,
	CR_4_7 = 3,
	CR_4_8 = 4,
} coding_rate_t;

typedef enum {
	EXPLICIT_HEADER = 0,
	IMPLICIT_HEADER = 1,
} header_mode_t;

typedef enum {
	CRC_DISABLED = 0,
	CRC_ENABLED = 1,
} rx_crc_t;

typedef enum {
	OPT_DISABLED = 0,
	OPT_ENABLED = 1,
} low_data_rate_opt_t;

/* ---------- RegModemConfig 2 --------- */
#define SPREADING_FACTOR 			4
#define TX_MODE						3
#define AGC_MODE					2
#define SYMB_TO_MSB					0

typedef enum {
	SF_6 = 6,
	SF_7 = 7,
	SF_8 = 8,
	SF_9 = 9,
	SF_10 = 10,
	SF_11 = 11,
	SF_12 = 12,
} spreading_factor_t;

typedef enum {
	TX_NORMAL = 0,
	TX_CONTINOUS = 1,
} tx_mode_t;

typedef enum {
	EXTERNAL = 0,
	INTERNAL = 1,
} agc_mode_t;

/* ---------- Channels ---------- */
typedef enum {
	CH_8652 = 0xD84CCC, // channel 10, central freq = 865.20MHz
	CH_8655 = 0xD86000, // channel 11, central freq = 865.50MHz
	CH_8658 = 0xD87333, // channel 12, central freq = 865.80MHz
	CH_8661 = 0xD88666, // channel 13, central freq = 866.10MHz
	CH_8664 = 0xD89999, // channel 14, central freq = 866.40MHz
	CH_8667 = 0xD8ACCC, // channel 15, central freq = 866.70MHz
	CH_8670 = 0xD8C000, // channel 16, central freq = 867.00MHz
	CH_8675 = 0xD8E000, // channel 17, central freq = 867.50MHz
	CH_8680 = 0xD90000, // channel 16, central freq = 868.00MHz
	CH_8685 = 0xD92000, // channel 18, central freq = 868.50MHz
	CH_9031 = 0xE1D99A, // channel 00, central freq = 903.40MHz
	CH_9052 = 0xE24F5C, // channel 01, central freq = 905.24MHz
	CH_9074 = 0xE2D999, // channel 02, central freq = 907.40MHz
	CH_9095 = 0xE363D7, // channel 03, central freq = 909.56MHz
	CH_9117 = 0xE3EE14, // channel 04, central freq = 911.72MHz
	CH_9139 = 0xE47851, // channel 05, central freq = 913.88MHz
	CH_9160 = 0xE50000, // channel 06, central freq = 916.04MHz
	CH_9182 = 0xE58CCC, // channel 07, central freq = 918.20MHz
	CH_9203 = 0xE6170A, // channel 08, central freq = 920.36MHz
	CH_9225 = 0xE6A147, // channel 09, central freq = 922.52MHz
	CH_9247 = 0xE72B85, // channel 10, central freq = 924.68MHz
	CH_9266 = 0xE7B5C2, // channel 11, central freq = 926.84MHz
	CH_9268 = 0xE4C000,
} channel_t;

/* SPI Options */
#define WRITE 0x80
#define READ 0x0
#define SPI_TIMEOUT 100

/* CRC Types */
#define CRC_TYPE_IBM 					1
#define CRC_TYPE_CCITT 					0

#define POLYNOMIAL_IBM 					0x8005
#define POLYNOMIAL_CCITT				0x1021

#define CRC_IBM_SEED 					0xFFFF
#define CRC_CCITT_SEED 					0x1D0F

/* Device Configuration */
typedef struct {
	uint8_t Fifo;
	uint8_t OpMode;
	uint32_t CarrierFreq;
	uint8_t PowerConfig;
	uint8_t PowerRamp;
	uint8_t OverCurrProc;
	uint8_t LowNoiseAmp;
	uint8_t FifoAddrPtr;
	uint8_t FifoTxBaseAddr;
	uint8_t FifoRxBaseAddr;
	uint8_t FifoRxCurrentAddr;
	uint8_t IrqMask;
	uint16_t ModemConfig;
	uint16_t SymbTimeOut;
	uint16_t Preamble;
	uint8_t PayLoadLength;
	uint8_t MaxPayLoad;
	uint8_t HopPeriod;
	uint8_t DectectOpt;
	uint8_t InvertIQ;
	uint8_t DectectThres;
	uint8_t SyncWord;
} Config_Group;


/* SPI Driver */
void spi_select();
void spi_deselect();

/* LoRa Driver */
void sx1272_lora_init();
void sx1272_set_op_mode(uint8_t op);
void sx1272_set_freq(uint32_t freq);
void sx1272_set_pa_config(uint8_t pa);
void sx1272_set_pa_ramp(uint8_t ramp);
void sx1272_set_ocp(uint8_t ocp);
void sx1272_set_lna(uint8_t lna);
void sx1272_set_fifo_addr_ptr(uint8_t addr);
void sx1272_set_tx_base(uint8_t addr);
void sx1272_set_rx_base(uint8_t addr);
void sx1272_set_irq_mask(uint8_t mask);
void sx1272_set_modem_config(uint8_t config1, uint8_t config2);
void sx1272_set_symb_timeout(uint16_t timeout);
void sx1272_set_preamble(uint16_t preamble);
void sx1272_set_payload_length(uint8_t length);
void sx1272_set_max_payload(uint8_t max);
void sx1272_set_hop_period(uint8_t hop);
void sx1272_set_detect_opt(uint8_t detect);
void sx1272_set_invert_iq(uint8_t inv);
void sx1272_set_detect_thres(uint8_t thres);
void sx1272_set_sync_word(uint8_t sync);
void sx1272_set_dio_mapping(uint8_t map);

void sx1272_clear_irq_flags();
void sx1272_write_fifo(uint8_t value);
void sx1272_clear_fifo();
void sx1272_sleep();
void sx1272_stdby();

uint8_t sx1272_get_rx_current_ptr();
uint8_t sx1272_get_irq_flags();
uint8_t sx1272_get_op_mode();
uint8_t sx1272_get_received_payload_length();
uint8_t sx1272_get_hop_channel();
uint8_t sx1272_send(uint8_t dest_addr, uint8_t *data, uint8_t size, uint8_t ret, uint32_t timeout);
uint8_t sx1272_receive(uint8_t *rx_buffer, uint8_t size, uint32_t timeout);
uint16_t sx1272_get_modem_config();
uint32_t sx1272_get_freq();

uint16_t compute_crc(uint16_t crc, uint8_t data, uint16_t pol);
uint16_t radio_packet_crc_compute(uint8_t *buffer, uint8_t size, uint8_t crc_type);
float calculate_time_on_air();

#endif /* SX1272_H_ */
