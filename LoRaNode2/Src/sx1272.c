/**
 * Driver for SX1272 LoRa nodes using STM34F446RE micro-controllers. This is a 2018-19 summer research project under the
 * supervision of Dr. Chandima Ekanyake and Dr. Hui Ma, School of ITEE, The University of Queensland, Brisbane, QLD, Australia.
 * The project aims to create a LPWAN network for stand-alone, pole-mounted transformers within Queensland teritory.
 *
 * Created on: 23 November, 2018
 * Author: Trung Q. Cao - 44592394
 */

#include "sx1272.h"
#include <math.h>
#include "main.h"

SPI_HandleTypeDef hspi1;

/**
 * Explicitly selects the device to start SPI communication protocol.
 */
void spi_select() {
	/* SPI Motorola standard, active low, CPOL = 0, CPHA = 0 */
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
}

/**
 * Explicitly deselects the device to stop SPI communication protocol.
 */
void spi_deselect() {
	/* SPI Motorola standard, active low, CPOL = 0, CPHA = 0 */
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
}

void spi_push(uint8_t *txData, uint8_t size) {
	spi_select();
	HAL_SPI_Transmit(&hspi1, txData, size, SPI_TIMEOUT);
	spi_deselect();
}

void spi_read(uint8_t reg, uint8_t *rxData, uint8_t size) {
	uint8_t cmd = reg | READ;

	spi_select();
	HAL_SPI_Transmit(&hspi1, &cmd, 1, SPI_TIMEOUT);
	HAL_SPI_Receive(&hspi1, rxData, size, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_lora_init() {

	/* Reset pulse for programming mode */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

	/* Wait until the device is ready to be programmed */
	HAL_Delay(10);

	/* Set current supply option */
	sx1272_set_ocp(0x2B);

	/* Set operating mode. Registers are only to be modified in either LORA_SLEEP or LORA_STDBY mode and
	 * FSK_SLEEP mode must be entered to initialize mode change */
	sx1272_set_op_mode(0);
	sx1272_sleep();
	/* Set LoRa standby mode in order to configure control registers */
	sx1272_set_op_mode((LORA << LRANGE_MODE) | (LORA_ACCESS << REG_SHARE) | (STDBY << MODE));

    /* Set modem configurations. To achieve high immunity to EMI caused by near by transformer, maximum coding rate,
     * spreading factor and minimum bandwidth are selected */
	sx1272_set_modem_config(
		(BW_125K << BAND_WIDTH) | (CR_4_8 << CODING_RATE) | (EXPLICIT_HEADER << HEADER_MODE) | (CRC_ENABLED << RX_CRC),
		(SF_7 << SPREADING_FACTOR) | (INTERNAL << AGC_MODE));

	/* Set max pay load length to 255 bytes */
	sx1272_set_max_payload(MAX_PACKET_LENGTH);
	/* Set pay load length to 112 bytes */
	sx1272_set_payload_length(PAYLOAD_LENGTH + HEADER_LENGTH + CRC_LENGTH);

	/* Set base frequency */
	sx1272_set_freq(CH_8675);

	/* Set power output to +20dBm */
	sx1272_set_pa_config(0x02 | (RFIO << PA_SELECT));

	/* Clear flags */
	sx1272_clear_irq_flags();

	/* Reset FIFO pointer */
	sx1272_set_fifo_addr_ptr(0);
}

/**
 * Sets the operation mode of the device. This register can only be written in SLEEP/STDBY mode.
 *
 * @param op The operation mode to be set
 */
void sx1272_set_op_mode(uint8_t op) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_OP_MODE | WRITE;
	cmd[1] = op;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

/**
 * Sets the frequency channel of the device. This register can only be written in SLEEP/STDBY mode. The frequency is a 24-bit value,
 * being stored in 3 registers respectively.
 *
 * @param freq The frequency to be set
 */
void sx1272_set_freq(uint32_t freq) {
	uint8_t cmd[2];

	/* Set most significant bits */
	cmd[0] = REG_LR_FREQ_MSB | WRITE;
	cmd[1] = (uint8_t) (freq >> 16);

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();

	/* Set mid bits */
	cmd[0] = REG_LR_FREQ_MIB | WRITE;
	cmd[1] = (uint8_t) (freq >> 8);

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();

	/* Set least significant bits */
	cmd[0] = REG_LR_FREQ_LSB | WRITE;
	cmd[1] = (uint8_t) (freq);

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_set_pa_config(uint8_t pa) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_PA_CONFIG | WRITE;
	cmd[1] = pa;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_set_pa_ramp(uint8_t ramp) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_PA_RAMP | WRITE;
	cmd[1] = ramp;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_set_ocp(uint8_t ocp) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_OCP | WRITE;
	cmd[1] = ocp;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_set_lna(uint8_t lna) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_LNA | WRITE;
	cmd[1] = lna;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_set_fifo_addr_ptr(uint8_t addr) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_FIFOADDRPTR | WRITE;
	cmd[1] = addr;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_set_tx_base(uint8_t addr) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_FIFOTXBASEADDR | WRITE;
	cmd[1] = addr;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_set_rx_base(uint8_t addr) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_FIFORXBASEADDR | WRITE;
	cmd[1] = addr;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_set_irq_mask(uint8_t mask) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_IRQFLAGSMASK | WRITE;
	cmd[1] = mask;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_set_modem_config(uint8_t config1, uint8_t config2) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_MODEMCONFIG1 | WRITE;
	cmd[1] = config1;
	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();

	cmd[0] = REG_LR_MODEMCONFIG2 | WRITE;
	cmd[1] = config2;
	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_set_symb_timeout(uint16_t timeout) {
	uint8_t config2 = 0;
	/* Reserves the modem config */
	uint8_t read = REG_LR_MODEMCONFIG2 | WRITE;
	HAL_SPI_TransmitReceive(&hspi1, &read, &config2, 2, SPI_TIMEOUT);
	/* Add high bits of symbol timeout */
	config2 |= (uint8_t) ((timeout & 0x300) >> 8);

	uint8_t cmd[2];
	/* Write high bits of symbol timeout */
	cmd[0] = REG_LR_MODEMCONFIG2 | WRITE;
	cmd[1] = config2;
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);

	/* Write low bits of symbol timeout */
	cmd[0] = REG_LR_SYMBTIMEOUTLSB | WRITE;
	cmd[1] = (uint8_t) (timeout);
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
}

void sx1272_set_preamble(uint16_t preamble) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_PREAMBLEMSB | WRITE;
	cmd[1] = (uint8_t) (preamble >> 8);
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);

	cmd[0] = REG_LR_PREAMBLELSB | WRITE;
	cmd[1] = (uint8_t) (preamble);
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
}

void sx1272_set_payload_length(uint8_t length) {
    uint8_t cmd[2];

    cmd[0] = REG_LR_PAYLOADLENGTH  | WRITE;
    cmd[1] = length;

    spi_select();
    HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
    spi_deselect();
}

void sx1272_set_max_payload(uint8_t max) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_RX_MAX_PAYLOADLENGTH | WRITE;
	cmd[1] = max;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_set_hop_period(uint8_t hop) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_HOPPERIOD | WRITE;
	cmd[1] = hop;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_set_detect_opt(uint8_t detect) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_DECTECT_OPTIMIZE | WRITE;
	cmd[1] = detect;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_set_invert_iq(uint8_t inv) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_INVERT_IQ | WRITE;
	cmd[1] = inv;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_set_detect_thres(uint8_t thres) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_DETECTION_THRESHOLD | WRITE;
	cmd[1] = thres;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_set_sync_word(uint8_t sync) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_SYNC_WORD | WRITE;
	cmd[1] = sync;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_set_dio_mapping(uint8_t map) {
    uint8_t cmd[2];

    cmd[0] = REG_IO_MAPPING_1 | WRITE;
    cmd[1] = map;

    spi_select();
    HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
    spi_deselect();
}

void sx1272_clear_irq_flags() {
	uint8_t cmd[2];

	cmd[0] = REG_LR_IRQFLAGS | WRITE;
	/* Writes 1s to clear flags */
	cmd[1] = 0xFF;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_write_fifo(uint8_t value) {
	uint8_t cmd[2];

	cmd[0] = REG_LR_FIFO | WRITE;
	cmd[1] = value;

	spi_select();
	HAL_SPI_Transmit(&hspi1, cmd, 2, SPI_TIMEOUT);
	spi_deselect();
}

void sx1272_clear_fifo() {
    sx1272_sleep();
}

void sx1272_sleep() {
	sx1272_set_op_mode((LORA << LRANGE_MODE) | (LORA_ACCESS << REG_SHARE) | (SLEEP << MODE));
}

void sx1272_stdby() {
    sx1272_set_op_mode((LORA << LRANGE_MODE) | (LORA_ACCESS << REG_SHARE) | (STDBY << MODE));
}

uint8_t sx1272_send(uint8_t dest_addr, uint8_t *data, uint8_t size, uint8_t ret, uint32_t timeout) {
    uint8_t status = 1, op = 0;
    uint32_t time;

    /* Save the current mode */
    op = sx1272_get_op_mode();
    sx1272_stdby();
    sx1272_set_sync_word(0x34); //LoRa MAC preamble
    sx1272_set_dio_mapping(MAP_DIO0_LORA_TXDONE | MAP_DIO1_LORA_NOP | MAP_DIO2_LORA_NOP);
    /* GPIO-based interrupt has been set, mask out this bit */
    sx1272_set_irq_mask(~(TX_DONE));

    /* Clears flags */
    sx1272_clear_irq_flags();

    /* Sets TX pointer to bottom of FIFO page */
    sx1272_set_tx_base(0x00);
    sx1272_set_fifo_addr_ptr(0x00);

    /* Writes packet length */
    sx1272_write_fifo(dest_addr);
    /* Writes node ID */
    sx1272_write_fifo(size);
    /* Writes pay load */
    for (int i = 0; i < PAYLOAD_LENGTH; i++) {
        sx1272_write_fifo(data[i]);
    }
    /* 2 bytes CRC for pay load */
    uint16_t crc = radio_packet_crc_compute(data, size, CRC_TYPE_IBM);
    sx1272_write_fifo((uint8_t) (crc >> 8));
    sx1272_write_fifo((uint8_t) crc);

    /* Initializes TX mode to send the packet in FIFO */
    sx1272_set_op_mode((LORA << LRANGE_MODE) | (LORA_ACCESS << REG_SHARE) | (TX << MODE));
    status = sx1272_get_op_mode();
    uint8_t flags = sx1272_get_irq_flags();

    time = HAL_GetTick();
    /* Polls IRQ register for TxDone flag */
    while (!((flags & TX_DONE) >> TX_DONE_MASK)) {
        flags = sx1272_get_irq_flags();
        if (HAL_GetTick() - time > timeout) {
            return 3;
        }
    }

    /* Packet has been successfully sent */
    status = 0;

    /* Clears FIFO */
    sx1272_sleep();
    /* Restores previous mode */
    sx1272_set_op_mode(op);
    /* Clears flags */
    sx1272_clear_irq_flags();

    return status;
}

uint8_t sx1272_receive(uint8_t *rx_buffer, uint8_t size, uint32_t timeout) {
    uint8_t status = 1, flags = 0, op = 0, prev, new;

    /* Prevents overflow FIFO read */
    if (size > MAX_PACKET_LENGTH) {
        return status;
    }

    /* Saves the current mode */
    op = sx1272_get_op_mode();

    /* Configures receiver mode */
    sx1272_set_detect_opt(0x3);
    sx1272_set_pa_ramp(0x09);
    sx1272_set_lna(0x23);
    sx1272_set_sync_word(0x34); //LoRa MAC preamble

    /* Enables IO interrupts */
    sx1272_set_dio_mapping(MAP_DIO0_LORA_RXDONE | MAP_DIO1_LORA_RXTOUT | MAP_DIO2_LORA_NOP);
    sx1272_clear_irq_flags();

    /* Sets RX base current to the bottom of FIFO page */
    sx1272_set_rx_base(0x00);
    /* Saves current FIFO RX pointer */
    prev = sx1272_get_rx_current_ptr();
    /* Points FIFO pointer to the last packet received */
    sx1272_set_fifo_addr_ptr(prev);

    /* Registers the packet length */
    sx1272_set_payload_length(size + CRC_LENGTH);

    /* Initializes RxContinuous mode to receive packets */
    sx1272_set_op_mode((LORA << LRANGE_MODE) | (LORA_ACCESS << REG_SHARE) | (RX_CONT << MODE));

    flags = sx1272_get_irq_flags();
    /* Polls for ValidHeader flag */
    while (!((flags & VALID_HDR) >> VALID_HDR_MASK)) {
        flags = sx1272_get_irq_flags();
        HAL_Delay(10); //Might use threads and semaphore when running FreeRTOS
    }

    /* Polls for RxDone flag */
    while (!((flags & RXDONE) >> RXDONE_MASK)) {
        flags = sx1272_get_irq_flags();
        HAL_Delay(10); //Might use threads and semaphore when running FreeRTOS
    }

    /* Checks for valid header CRC */
    if ((flags & CRC_ERR) >> CRC_ERR_MASK) {
        status = 4;
        /* Restores initial mode */
        sx1272_set_op_mode(op);
        sx1272_clear_irq_flags();
        return status;
    }

    /* Checks for how many bytes have been received */
    new = sx1272_get_received_payload_length();

    if ((new - prev) != size + CRC_LENGTH) {
        /* Error has occurred */
        status = 3;
    } else {
        uint8_t packet[size + CRC_LENGTH];
        /* Reads and copies received data in FIFO to buffer */
        spi_select();
        uint8_t cmd = REG_LR_FIFO | READ;
        HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
        HAL_SPI_Receive(&hspi1, packet, size + CRC_LENGTH, 100);
        spi_deselect();

        uint16_t crc = (((uint16_t) packet[size]) << 8) |
                        ((uint16_t) packet[size + 1]);
        if (crc != radio_packet_crc_compute(packet + HEADER_LENGTH, PAYLOAD_LENGTH, CRC_TYPE_IBM)) {
            /* Pay load CRC error */
            status = 5;
            return packet[size];
        } else if (size != packet[0]) {
            /* Pay load length mismatched */
            status = 2;
        } else {
            /* Packet is successfully received */
            status = 0;

            for (int i = 0; i < size; i++) {
                rx_buffer[i] = packet[i];
            }

            /* Restores initial mode */
            sx1272_set_op_mode(op);
            sx1272_clear_irq_flags();
        }
    }

    return status;
}

uint8_t sx1272_get_rx_current_ptr() {
    uint8_t cmd = REG_LR_FIFORXCURRENTADDR | READ;
    uint8_t addr = 0;

    spi_select();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, SPI_TIMEOUT);
    HAL_SPI_Receive(&hspi1, &addr, 1, SPI_TIMEOUT);
    spi_deselect();

    return addr;
}

uint8_t sx1272_get_irq_flags() {
	uint8_t flags = 0;
	uint8_t cmd =  REG_LR_IRQFLAGS | READ;

	spi_select();
	HAL_SPI_Transmit(&hspi1, &cmd, 1, SPI_TIMEOUT);
	HAL_SPI_Receive(&hspi1, &flags, 1, SPI_TIMEOUT);
	spi_deselect();

	return flags;
}

uint8_t sx1272_get_op_mode() {
	uint8_t cmd = REG_LR_OP_MODE | READ;
	uint8_t op = 0;

	spi_select();
	HAL_SPI_Transmit(&hspi1, &cmd, 1, SPI_TIMEOUT);
	HAL_SPI_Receive(&hspi1, &op, 1, SPI_TIMEOUT);
	spi_deselect();

	return op;
}

uint8_t sx1272_get_received_payload_length() {
    uint8_t cmd = REG_LR_RXNBBYTES | READ;
    uint8_t bytes = -1;

    spi_select();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, SPI_TIMEOUT);
    HAL_SPI_Receive(&hspi1, &bytes, 1, SPI_TIMEOUT);
    spi_deselect();

    return bytes;
}

uint8_t sx1272_get_hop_channel() {
    uint8_t cmd = REG_LR_HOPCHANNEL | READ;
    uint8_t hop = -1;

    spi_select();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, SPI_TIMEOUT);
    HAL_SPI_Receive(&hspi1, &hop, 1, SPI_TIMEOUT);
    spi_deselect();

    return hop;
}

uint16_t sx1272_get_modem_config() {
	uint8_t config1 = 0, config2 = 0;
	uint16_t config = 0;

	uint8_t cmd = REG_LR_MODEMCONFIG1 | READ;
	spi_select();
	HAL_SPI_Transmit(&hspi1, &cmd, 1, SPI_TIMEOUT);
	HAL_SPI_Receive(&hspi1, &config1, 1, SPI_TIMEOUT);
	spi_deselect();

	cmd = REG_LR_MODEMCONFIG2 | READ;
	spi_select();
	HAL_SPI_Transmit(&hspi1, &cmd, 1, SPI_TIMEOUT);
	HAL_SPI_Receive(&hspi1, &config2, 1, SPI_TIMEOUT);
	spi_deselect();

	/* Mask out MSB of symbol timeout */
	config = ((((uint16_t) config1) << 8) | ((uint16_t) config2));

	return config;
}

uint32_t sx1272_get_freq() {
	uint32_t freq = 0;
	uint8_t freq1 = 0, freq2 = 0, freq3 = 0;

	uint8_t cmd = REG_LR_FREQ_MSB | READ;
	spi_select();
	HAL_SPI_Transmit(&hspi1, &cmd, 1, SPI_TIMEOUT);
	HAL_SPI_Receive(&hspi1, &freq1, 1, SPI_TIMEOUT);
	spi_deselect();

	cmd = REG_LR_FREQ_MIB | READ;
	spi_select();
	HAL_SPI_Transmit(&hspi1, &cmd, 1, SPI_TIMEOUT);
	HAL_SPI_Receive(&hspi1, &freq2, 1, SPI_TIMEOUT);
	spi_deselect();

	cmd = REG_LR_FREQ_LSB | READ;
	spi_select();
	HAL_SPI_Transmit(&hspi1, &cmd, 1, SPI_TIMEOUT);
	HAL_SPI_Receive(&hspi1, &freq3, 1, SPI_TIMEOUT);
	spi_deselect();

	freq = (((uint32_t) freq1) << 16) | (((uint32_t) freq2) << 8) | ((uint32_t) freq3);
	return freq;
}

/**
 * CRC algorithm implementation. See SX1272/73 Semtech datasheet page 123 for further details.
 *
 * @param crc Previous CRC value
 * @param data New data to be added to the CRC
 * @param pol CRC polynomial selection
 *
 * @retval Newly computed CRC
 */
uint16_t compute_crc(uint16_t crc, uint8_t data, uint16_t pol) {
	for (int i = 0; i < 8; i++) {
		if ((((crc & 0x8000) >> 8) ^ (data & 0x80)) != 0) {
			crc <<= 1;
			crc ^= pol;
		} else {
			crc <<= 1;
		}

		data <<= 1;
	}

	return crc;
}

/**
 * CRC algorithm implementation. See SX1272/73 Semtech datasheet page 123 for further details.
 *
 * @param buffer Array containing the data
 * @param size Data buffer length
 * @param crc_type Select the CRC polynomial and seed
 *
 * @retval Buffer computed CRC
 */
uint16_t radio_packet_crc_compute(uint8_t *buffer, uint8_t size, uint8_t crc_type) {
	uint16_t crc, pol;
	pol = (crc_type == CRC_TYPE_IBM) ? POLYNOMIAL_IBM : POLYNOMIAL_CCITT;
	crc = (crc_type == CRC_TYPE_IBM) ? CRC_IBM_SEED : CRC_CCITT_SEED;

	for (int i = 0; i < size; i++) {
		crc = compute_crc(crc, buffer[i], pol);
	}

	if (crc_type == CRC_TYPE_IBM) {
		return crc;
	} else {
		return (uint16_t) (~crc);
	}
}

float calculate_time_on_air() {
    float toa = 0.0;
    float tsym = 125000 / pow(2.0, 7);

    toa = 8 + 4.25 + 4 + (4 + 4);

    toa *= tsym;

    return toa;
}

