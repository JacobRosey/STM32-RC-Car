/*
 * nrf.h
 *
 *  Created on: Sep 13, 2026
 *      Author: jrose
 */

#ifndef INC_NRF_H_
#define INC_NRF_H_

typedef enum {
    NRF_OK,
	NRF_TRANSMIT_SUCCESS,
	NRF_TRANSMIT_TIMEOUT,
	NRF_RECEIVE_SUCCESS,
	NRF_RECEIVE_TIMEOUT,
	NRF_MAX_RETRIES,
	NRF_TX_FIFO_FULL,
	NRF_TX_FIFO_EMPTY,
	NRF_RX_FIFO_FULL,
	NRF_RX_FIFO_EMPTY,
	NRF_SPI_INVALID_COMMAND,
	NRF_SPI_FAILED_COMMAND,
    NRF_SPI_FAILED_WRITE,
	NRF_SPI_FAILED_READ,
	NRF_SPI_UNEXPECTED_READ,
    NRF_FAILED_INIT
} NRF_Status;

NRF_Status NRF_TX_Init(SPI_HandleTypeDef*, TIM_HandleTypeDef*);
NRF_Status NRF_RX_Init(SPI_HandleTypeDef*, TIM_HandleTypeDef*);
NRF_Status NRF_Reset(SPI_HandleTypeDef*);
NRF_Status NRF_Transmit(int8_t[], int8_t[]); // Would have Packet class as arg here
NRF_Status NRF_Receive(int8_t[]);
// Probably a function that polls for packet reception

#endif /* INC_NRF_H_ */
