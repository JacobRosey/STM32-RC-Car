/*
 * nrf.c
 *
 *  Created on: Sep 13, 2026
 *      Author: jrose
 */
#include "main.h"
#include "nrf.h"
#include <string.h>
#define READ_CMD 0x00
#define WRITE_CMD 0x20
#define CONFIG 0x00
#define CONFIG_RESET 0x08
#define TX_MODE 0x00
#define RX_MODE 0x01
#define ENABLE_CRC (1U << 3)
#define POWER_ON (1U << 1)
#define WRITE_TX 0xA0
#define WRITE_TX_NOACK 0xB0
#define READ_RX {0x61, 0x00, 0x00, 0x00, 0x00}
#define FIFO_STATUS 0x17
#define FLUSH_TX 0xE1
#define AUTO_ACK_REGISTER 0x01
#define DISABLE_AUTO_ACK 0x00
#define FEATURE 0x1D
#define ACTIVATE 0x50
#define ACTIVATE_DATA 0x73
#define DYNPD           0x1C
#define EN_DPL          (1U << 2) //
#define EN_ACK_PAY      (1U << 1) // enable acknowledgement payload
#define DPL_P0          (1U << 0)
#define STATUS 0x07
#define TX_DS (1u << 5) // if bit set: data was sent by transmitter (and ack received)
#define RX_DR (1u << 6) // if bit set: data is in rx fifo
#define MAX_RETRIES (1u << 4) // if bit set: max retries reached (can configure max retries; ARC in 0x04 register)
#define TX_FIFO_FULL (1u << 5) // if bit set: transmitter fifo full
#define TX_REGISTER 0x10
#define RX_REGISTER_PIPE0 0x0A
// Writes TEST to pipe 0's queue (last 3 bits of 0xA8 == 000)
// will be replaced with telemetry later
#define W_ACK_PAYLOAD {0xA8, 0x54, 0x45, 0x53, 0x54}
#define TX_ADDRESS_BYTES 5
#define MAX_SPI_CMD_LEN 8
#define RX_PW_P0 0x11
#define TRANSMIT_PAYLOAD_WIDTH 2 // 2 joysticks == 2 values to transmit
#define ACK_PAYLOAD_WIDTH 4		// sending T, E, S, T in ack payload for now, telemetry later
static const uint8_t SET_TX_ADDRESS[5] = {
    0xE5, 0xD4, 0xC3, 0xB2, 0xA1
};
static SPI_HandleTypeDef *spi;
static TIM_HandleTypeDef *timer;
// TODO: Refactor this purpose-built implementation into a reusable nRF24L01
// driver by removing RC-car-specific configuration and assumptions.
/**
 * @brief Reads a single-byte register from the nRF24L01.
 *
 * @param addr Address of the register to read.
 * @param[out] output Pointer to the variable where the register value will be stored.
 * @return NRF_Status Status indicating whether the operation succeeded.
 */
static NRF_Status Read_Register(uint8_t addr, uint8_t *output) {
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	tx_data[0] = READ_CMD | (addr & 0x1F);
	tx_data[1] = 0x00; // send dummy byte to receive register value (SPI full duplex)
	// CSN is active low
	HAL_GPIO_WritePin(SPI_CSN_GPIO_Port, SPI_CSN_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status =
	  HAL_SPI_TransmitReceive(spi, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(SPI_CSN_GPIO_Port, SPI_CSN_Pin, GPIO_PIN_SET);
	if (status != HAL_OK) return NRF_SPI_FAILED_READ;
	// Use output parameter to give caller access to register value
    *output = rx_data[1];
    return NRF_OK;
}
/**
 * @brief Reads an nRF24L01 register containing multiple bytes.
 *
 * @param addr Address of the register to read.
 * @param bytes Number of bytes to be read.
 * @param[out] output Pointer to a buffer of at least @p bytes bytes where the register data will be stored.
 * @return NRF_Status Status indicating whether the operation succeeded.
 */
static NRF_Status Read_Multi_Byte_Register(uint8_t addr, uint8_t bytes, uint8_t output[]){
	uint8_t tx_data[bytes+1];
	uint8_t rx_data[bytes+1];
	tx_data[0] = READ_CMD | (addr & 0x1F);
	// send dummy bytes to receive caller-defined # of bytes from register
	for (int i=1; i <= bytes; i++){
		tx_data[i] = 0x00;
	}
	// CSN is active low
	HAL_GPIO_WritePin(SPI_CSN_GPIO_Port, SPI_CSN_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status =
		  HAL_SPI_TransmitReceive(spi, tx_data, rx_data, bytes+1, 100);
	HAL_GPIO_WritePin(SPI_CSN_GPIO_Port, SPI_CSN_Pin, GPIO_PIN_SET);
	if (status != HAL_OK){
		return NRF_SPI_FAILED_READ;
	}
	// Strip status byte
	memcpy(output, rx_data + 1, bytes);
	return NRF_OK;
}
/**
 * @brief Writes to a single-byte nRF24L01 register.
 *
 * @param addr Address of the register to write to.
 * @param value Value to be written to the register.
 * @return NRF_Status Status indicating whether the operation succeeded.
 */
static NRF_Status Write_Register(uint8_t addr, uint8_t value){
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	tx_data[0] = WRITE_CMD | (addr & 0x1F);
	tx_data[1] = value;
	// CSN is active low
	HAL_GPIO_WritePin(SPI_CSN_GPIO_Port, SPI_CSN_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status =
		  HAL_SPI_TransmitReceive(spi, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(SPI_CSN_GPIO_Port, SPI_CSN_Pin, GPIO_PIN_SET);
	if (status != HAL_OK){
		return NRF_SPI_FAILED_WRITE;
	}
	return NRF_OK;
}
/**
 * @brief Writes to an nRF24L01 register containing multiple bytes.
 *
 * @param addr Address of the register to write to.
 * @param bytes Number of bytes to be written.
 * @param values Pointer to the buffer holding bytes to be written.
 * @return NRF_Status Status indicating whether the operation succeeded.
 */
static NRF_Status Write_Multi_Byte_Register(uint8_t addr, uint8_t bytes, const uint8_t values[]){
	uint8_t tx_data[bytes+1];
	uint8_t rx_data[bytes+1];
	tx_data[0] = WRITE_CMD | (addr & 0x1F);
	// write all provided bytes to register
	for (int i=1; i <= bytes; i++){
		tx_data[i] = values[i-1];
	}
	// CSN is active low
	HAL_GPIO_WritePin(SPI_CSN_GPIO_Port, SPI_CSN_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status =
		  HAL_SPI_TransmitReceive(spi, tx_data, rx_data, bytes+1, 100);
	HAL_GPIO_WritePin(SPI_CSN_GPIO_Port, SPI_CSN_Pin, GPIO_PIN_SET);
	if (status != HAL_OK){
		return NRF_SPI_FAILED_WRITE;
	}
	return NRF_OK;
}
// not really necessary to have these as separate functions for my use case
// but probably useful in a properly generalized driver
static NRF_Status Set_Tx_Address(){
	NRF_Status status;
	status = Write_Multi_Byte_Register(TX_REGISTER, TX_ADDRESS_BYTES, SET_TX_ADDRESS);
	// might want to check read result here to ensure write succeeded?
	//uint8_t result[TX_ADDRESS_BYTES + 1];
	//status = Read_Multi_Byte_Register(TX_REGISTER, TX_ADDRESS_BYTES, result);
	return status;
}
// not really necessary to have these as separate functions for my use case
// but probably useful in a properly generalized driver
// to make it more generally useful, i'd probably do something like:
/**
 * @brief Configures the receive address for an nRF24L01 data pipe.
 *
 * @param nrf     Pointer to the nRF device handle.
 * @param pipe    Receive data pipe to configure.
 * @param address Pointer to the receive address.
 * @return NRF_Status Status indicating whether the operation succeeded.
 */
static NRF_Status Set_Rx_Address(){
	NRF_Status status;
	status = Write_Multi_Byte_Register(RX_REGISTER_PIPE0, TX_ADDRESS_BYTES, SET_TX_ADDRESS);
	//might want to check read result here to ensure write succeeded?
	//uint8_t result[TX_ADDRESS_BYTES + 1];
	//status = Read_Multi_Byte_Register(RX_REGISTER_PIPE0, TX_ADDRESS_BYTES, result);
	return status;
}
/**
 * @brief Sends predefined commands (in datasheet) to nRF24L01 over SPI
 *
 * @param cmd[] Pointer to the buffer holding the command byte and (possibly) subsequent data bytes
 * @param length Number of bytes to be sent (command + data).
 * @param[out] output Pointer to the buffer where the received bytes will be stored.
 * @return NRF_Status Status indicating whether the operation succeeded.
 */
static NRF_Status Send_SPI_Command(uint8_t cmd[], uint8_t length, uint8_t *output){
	if ( length == 0 || length > MAX_SPI_CMD_LEN) return NRF_SPI_INVALID_COMMAND;
	uint8_t rx_data[MAX_SPI_CMD_LEN];
	HAL_GPIO_WritePin(SPI_CSN_GPIO_Port, SPI_CSN_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status =
		  HAL_SPI_TransmitReceive(spi, cmd, rx_data, length, 100);
	HAL_GPIO_WritePin(SPI_CSN_GPIO_Port, SPI_CSN_Pin, GPIO_PIN_SET);
	if(status != HAL_OK){
		return NRF_SPI_FAILED_COMMAND;
	}
	// copy received data to output parameter if caller cares
	if (output != NULL) {
		memcpy(output, rx_data, length);
	}
	return NRF_OK;
}
/**
 * @brief Writes byte payload to TX FIFO
 *
 * @param values[] Bytes to be written to TX FIFO.
 * @return NRF_Status Status indicating whether the operation succeeded.
 */
static NRF_Status WritePayload(int8_t values[]){
	uint8_t tx_data[TRANSMIT_PAYLOAD_WIDTH + 1] = {[0] = WRITE_TX};
	for (int i = 1; i < TRANSMIT_PAYLOAD_WIDTH + 1; i++) {
	    tx_data[i] = values[i - 1];
	}
	NRF_Status status = Send_SPI_Command(tx_data, TRANSMIT_PAYLOAD_WIDTH + 1, NULL);
	if (status != NRF_OK) return status;
	return NRF_OK;
}
/**
 * @brief Blocking sleep for caller-defined duration
 * @param duration Duration to sleep (in microseconds)
 */
static void Microsecond_Sleep(uint16_t duration){
		HAL_TIM_Base_Start(timer);
		uint16_t initial = timer->Instance->CNT;
		// deal with integer promotion by casting expression result to uint16
		// otherwise uint16_t - uint16_t is promoted to signed 32 bit - signed 32 bit,
		// which could result in negative numbers being evaluated against duration
		while((uint16_t)(timer->Instance->CNT - initial) <= duration);
		HAL_TIM_Base_Stop(timer);
}

/**
 * @brief Configures the nRF24l01 as a receiver
 *
 * @param hspi1 Pointer to the SPI handle
 * @param htim3 Pointer to the TIM handle
 * @return NRF_Status Status indicating whether the operation succeeded.
 */
NRF_Status NRF_RX_Init(SPI_HandleTypeDef *hspi1, TIM_HandleTypeDef *htim3){
	spi = hspi1;
	timer = htim3;
	NRF_Status status;
	uint8_t register_value;
	uint8_t activate_cmd[2] = {ACTIVATE, ACTIVATE_DATA};
	status = Send_SPI_Command(activate_cmd, 2, NULL);
	if (status != NRF_OK) return status;
	// Enable dynamic payloads + ACK payloads
	status = Write_Register(FEATURE, EN_DPL | EN_ACK_PAY);
	if (status != NRF_OK) return status;
	status = Read_Register(FEATURE, &register_value);
	if(register_value != (EN_DPL | EN_ACK_PAY)) return NRF_SPI_UNEXPECTED_READ;
	status = Write_Register(DYNPD, DPL_P0);
	if (status != NRF_OK) return status;
	// power on nRF in RX mode with CRC enabled
	status = Write_Register(CONFIG, RX_MODE | ENABLE_CRC | POWER_ON);
	if (status != NRF_OK) return status;
	status = Read_Register(CONFIG, &register_value);
	if (status != NRF_OK) return status;
	if (register_value != (RX_MODE | ENABLE_CRC | POWER_ON))
		return NRF_SPI_UNEXPECTED_READ;
//	status = Read_Register(0x05, &register_value);
//
//	status = Read_Register(0x06, &register_value);
//  if(status != NRF_OK) return status;
	// Must wait 130 us after power on before receiving transmissions
	Microsecond_Sleep(145);
	status = Set_Rx_Address();
	if (status!= NRF_OK) return status;
	// define payload width for RX pipe 0
	status = Write_Register(RX_PW_P0, ACK_PAYLOAD_WIDTH);
	// Pulse CE High to begin listening for transmissions
	HAL_GPIO_WritePin(SPI_CE_GPIO_Port, SPI_CE_Pin, GPIO_PIN_SET);
	return status;
}
/**
 * @brief Configures the nRF24l01 as a transmitter
 *
 * @param hspi1 Pointer to the SPI handle
 * @param htim3 Pointer to the TIM handle
 * @return NRF_Status Status indicating whether the operation succeeded.
 */
NRF_Status NRF_TX_Init(SPI_HandleTypeDef *hspi1, TIM_HandleTypeDef *htim3){
	spi = hspi1;
	timer = htim3;
	NRF_Status status;
	uint8_t register_value;
	uint8_t activate_cmd[2] = {ACTIVATE, ACTIVATE_DATA};
	status = Send_SPI_Command(activate_cmd, 2, NULL);
	if (status != NRF_OK) return status;
	// Enable dynamic payloads + ACK payloads
	status = Write_Register(FEATURE, EN_DPL | EN_ACK_PAY);
	if (status != NRF_OK) return status;
	status = Read_Register(FEATURE, &register_value);
	if(register_value != (EN_DPL | EN_ACK_PAY)) return NRF_SPI_UNEXPECTED_READ;
	status = Write_Register(DYNPD, DPL_P0);
	if (status != NRF_OK) return status;
	// Power on nRF in TX mode with CRC enabled
	status = Write_Register(CONFIG, TX_MODE | ENABLE_CRC | POWER_ON);
	if (status != NRF_OK) return status;
	status = Read_Register(CONFIG, &register_value);
	if (status != NRF_OK)
		return status;
	if (register_value != (TX_MODE | ENABLE_CRC | POWER_ON))
	  return NRF_SPI_UNEXPECTED_READ;
//	status = Read_Register(0x05, &register_value);
//
//	status = Read_Register(0x06, &register_value);
	// Must wait 150 us after power on before sending transmissions
	Microsecond_Sleep(175);
	status = Set_Tx_Address();
	if(status!= NRF_OK) return status;
	status = Set_Rx_Address();
	return status;
}
/**
 * @brief Resets the NRF24L01 to default configuration
 *
 * @param hspi1 Pointer to the SPI handle
 * @return NRF_Status Status indicating whether the operation succeeded.
 */
NRF_Status NRF_Reset(SPI_HandleTypeDef *hspi1){
	spi = hspi1;
	NRF_Status status;
	uint8_t registerValue;
	status = Write_Register(CONFIG, CONFIG_RESET);
	if (status != NRF_OK) return status;
	status = Read_Register(CONFIG, &registerValue);
	if (registerValue != CONFIG_RESET)
	  return NRF_SPI_UNEXPECTED_READ;
	return NRF_OK;
}

/**
 * @brief Fetches the newest packet from RX FIFO
 *
 * @param[out] rx_data pointer to buffer in which to place newest packet
 * @return NRF_Status Status indicating whether the operation succeeded.
 */
static NRF_Status Get_Newest_Packet(int8_t rx_data[]){
	// Only sending 1 command byte, but need to send 5 clock pulses
	// to receive status byte + payload_length data bytes
	uint8_t raw[TRANSMIT_PAYLOAD_WIDTH + 1];
	uint8_t read[] = READ_RX;
	NRF_Status status;

	uint8_t rx_status = FIFO_STATUS;
	uint8_t byte;
	status = Read_Register(rx_status, &byte);
	if (status != NRF_OK) return status;

	if(byte & 0x01) return NRF_RX_FIFO_EMPTY;

	// Pulse CE low to stop listening for transmissions while draining FIFO
	HAL_GPIO_WritePin(SPI_CE_GPIO_Port, SPI_CE_Pin, GPIO_PIN_RESET);

	// Read until the FIFO is empty. Last packet drained from fifo is the newest
	do {
		status = Send_SPI_Command(read, TRANSMIT_PAYLOAD_WIDTH + 1, raw);
		if (status != NRF_OK) return status;
		memcpy(rx_data, raw + 1, TRANSMIT_PAYLOAD_WIDTH); // strip status byte
		status = Read_Register(FIFO_STATUS, &rx_status);
		if (status != NRF_OK) return status;
	} while (!(rx_status & 0x01)); // continue while data in rx fifo

	// Resume listening for transmissions
	HAL_GPIO_WritePin(SPI_CE_GPIO_Port, SPI_CE_Pin, GPIO_PIN_SET);
	return NRF_OK;
}

/**
 * @brief Transmits data using the nRF24L01
 *
 * @param values[] Pointer to the buffer containing data to be transmitted.
 * @param[out] rx_data Pointer to the buffer in which the ACK payload will be placed.
 * @return NRF_Status Status indicating whether the operation succeeded.
 */
NRF_Status NRF_Transmit(int8_t values[], int8_t rx_data[]){
	NRF_Status nrf_status;
	uint8_t fifo_status = 0x00;
	uint8_t radio_status = 0x00;
	uint8_t flush_tx = FLUSH_TX;
	nrf_status = Read_Register(FIFO_STATUS, &fifo_status);
	if (nrf_status != NRF_OK) return nrf_status;
	// If TX_Queue is full, new writes won't occur. need to flush
	if (fifo_status & (TX_FIFO_FULL)){
		nrf_status = Send_SPI_Command(&flush_tx, 1, NULL);
		if(nrf_status != NRF_OK) return nrf_status;
	}
	nrf_status = WritePayload(values);
	if(nrf_status != NRF_OK) return nrf_status;
	// now there is data in the tx queue, so set chip enable (CE) high for
	// 10 microseconds (at least) to transmit the data.
	HAL_GPIO_WritePin(SPI_CE_GPIO_Port, SPI_CE_Pin, GPIO_PIN_SET);
	Microsecond_Sleep(15);
	HAL_GPIO_WritePin(SPI_CE_GPIO_Port, SPI_CE_Pin, GPIO_PIN_RESET);
	// Poll TX status and max retries up to 10 times, waiting 10 us between attempts
	// TODO: Use the nRF IRQ pin instead of polling.
	// IRQ handler can inspect STATUS, handle the event, and clear its interrupt flag.
	// I.E. I'd check which bit is set, write to clear, then continue according to the result
	for (int i=0; i<10; i++){
		nrf_status = Read_Register(STATUS, &radio_status);
		if(nrf_status != NRF_OK) return nrf_status;
		if(radio_status & (TX_DS)){
			uint8_t read[] = READ_RX;
			uint8_t raw[5];
			// Read from rx fifo to get telemetry from ack payload
			nrf_status = Send_SPI_Command(read, ACK_PAYLOAD_WIDTH + 1, raw);
			if (nrf_status!= NRF_OK) return nrf_status;
			memcpy(rx_data, raw + 1, ACK_PAYLOAD_WIDTH);
			Write_Register(STATUS, TX_DS); // Write 1 to clear TX_DS (data sent) bit
			return NRF_TRANSMIT_SUCCESS;
		}
		if(radio_status & (MAX_RETRIES)){
			// Write to clear
			nrf_status = Write_Register(STATUS, MAX_RETRIES);
			return NRF_MAX_RETRIES;
		}
		Microsecond_Sleep(130);
	}
	nrf_status = Send_SPI_Command(&flush_tx, 1, NULL); // just discard the failed transmission(s), already stale
	if (nrf_status != NRF_OK) return nrf_status;
	return NRF_TRANSMIT_TIMEOUT;
}
/**
 * @brief Reads received data from the nRF24L01.
 *
 * @param[out] rx_data Pointer to the buffer in which the received packet will be placed.
 * @return NRF_Status Status indicating whether the operation succeeded.
 */
NRF_Status NRF_Receive(int8_t rx_data[]){
	NRF_Status status;
	uint8_t cmd[] = W_ACK_PAYLOAD; // {cmd, dummy, dummy, dummy, dummy} ; replace dummy with telemetry
	status = Send_SPI_Command(cmd, ACK_PAYLOAD_WIDTH + 1, NULL);
	if (status != NRF_OK) return status;

	status = Get_Newest_Packet(rx_data);

	return status;
}
