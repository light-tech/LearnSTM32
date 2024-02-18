  
/*
 *  nrf24l01_plus.h
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
 * 
 */


#ifndef __NRF24L01P_H__
#define __NRF24L01P_H__


#include "main.h"    // header from stm32cubemx code generate
#include <stdbool.h>

typedef struct {
	GPIO_TypeDef* port;
	uint16_t pin;
} PinInfo;

// Structure to config the module so that we can use multiple modules in the library functions
typedef struct {
	SPI_HandleTypeDef *hspi;
	PinInfo csn;
	PinInfo ce;
	uint16_t payloadLength;
} nRF24L01P;

/* nRF24L01+ typedefs */
typedef uint8_t count;
typedef uint8_t widths;
typedef uint8_t length;
typedef uint16_t delay;
typedef uint16_t channel;

typedef enum
{
    _250kbps = 2,
    _1Mbps   = 0,
    _2Mbps   = 1
} air_data_rate;

typedef enum
{
    _0dBm  = 3,
    _6dBm  = 2,
    _12dBm = 1,
    _18dBm = 0
} output_power;


/* Main Functions */
void nrf24l01p_rx_init(nRF24L01P *nrf, channel MHz, air_data_rate bps);
void nrf24l01p_tx_init(nRF24L01P *nrf, channel MHz, air_data_rate bps);

void nrf24l01p_rx_receive(nRF24L01P *nrf, uint8_t* rx_payload);
void nrf24l01p_tx_transmit(nRF24L01P *nrf, uint8_t* tx_payload);

// Check tx_ds or max_rt
void nrf24l01p_tx_irq(nRF24L01P *nrf);


/* Sub Functions */
void nrf24l01p_reset(nRF24L01P *nrf);

void nrf24l01p_prx_mode(nRF24L01P *nrf);
void nrf24l01p_ptx_mode(nRF24L01P *nrf);

void nrf24l01p_power_up(nRF24L01P *nrf);
void nrf24l01p_power_down(nRF24L01P *nrf);

uint8_t nrf24l01p_get_status(nRF24L01P *nrf);
uint8_t nrf24l01p_get_fifo_status(nRF24L01P *nrf);

// Static payload lengths
void nrf24l01p_rx_set_payload_widths(nRF24L01P *nrf, widths bytes);

uint8_t nrf24l01p_read_rx_fifo(nRF24L01P *nrf, uint8_t* rx_payload);
uint8_t nrf24l01p_write_tx_fifo(nRF24L01P *nrf, uint8_t* tx_payload);

void nrf24l01p_flush_rx_fifo(nRF24L01P *nrf);
void nrf24l01p_flush_tx_fifo(nRF24L01P *nrf);

// Clear IRQ pin. Change LOW to HIGH
void nrf24l01p_clear_rx_dr(nRF24L01P *nrf);
void nrf24l01p_clear_tx_ds(nRF24L01P *nrf);
void nrf24l01p_clear_max_rt(nRF24L01P *nrf);

void nrf24l01p_set_rf_channel(nRF24L01P *nrf, channel MHz);
void nrf24l01p_set_rf_tx_output_power(nRF24L01P *nrf, output_power dBm);
void nrf24l01p_set_rf_air_data_rate(nRF24L01P *nrf, air_data_rate bps);

void nrf24l01p_set_crc_length(nRF24L01P *nrf, length bytes);
void nrf24l01p_set_address_widths(nRF24L01P *nrf, widths bytes);
void nrf24l01p_auto_retransmit_count(nRF24L01P *nrf, count cnt);
void nrf24l01p_auto_retransmit_delay(nRF24L01P *nrf, delay us);


/* nRF24L01+ Commands */
#define NRF24L01P_CMD_R_REGISTER                  0b00000000
#define NRF24L01P_CMD_W_REGISTER                  0b00100000
#define NRF24L01P_CMD_R_RX_PAYLOAD                0b01100001
#define NRF24L01P_CMD_W_TX_PAYLOAD                0b10100000
#define NRF24L01P_CMD_FLUSH_TX                    0b11100001
#define NRF24L01P_CMD_FLUSH_RX                    0b11100010
#define NRF24L01P_CMD_REUSE_TX_PL                 0b11100011
#define NRF24L01P_CMD_R_RX_PL_WID                 0b01100000
#define NRF24L01P_CMD_W_ACK_PAYLOAD               0b10101000
#define NRF24L01P_CMD_W_TX_PAYLOAD_NOACK          0b10110000
#define NRF24L01P_CMD_NOP                         0b11111111    

/* nRF24L01+ Registers */
#define NRF24L01P_REG_CONFIG            0x00
#define NRF24L01P_REG_EN_AA             0x01
#define NRF24L01P_REG_EN_RXADDR         0x02
#define NRF24L01P_REG_SETUP_AW          0x03
#define NRF24L01P_REG_SETUP_RETR        0x04
#define NRF24L01P_REG_RF_CH             0x05
#define NRF24L01P_REG_RF_SETUP          0x06
#define NRF24L01P_REG_STATUS            0x07
#define NRF24L01P_REG_OBSERVE_TX        0x08    // Read-Only
#define NRF24L01P_REG_RPD               0x09    // Read-Only
#define NRF24L01P_REG_RX_ADDR_P0        0x0A
#define NRF24L01P_REG_RX_ADDR_P1        0x0B
#define NRF24L01P_REG_RX_ADDR_P2        0x0C
#define NRF24L01P_REG_RX_ADDR_P3        0x0D
#define NRF24L01P_REG_RX_ADDR_P4        0x0E
#define NRF24L01P_REG_RX_ADDR_P5        0x0F
#define NRF24L01P_REG_TX_ADDR           0x10
#define NRF24L01P_REG_RX_PW_P0          0x11
#define NRF24L01P_REG_RX_PW_P1          0x12
#define NRF24L01P_REG_RX_PW_P2          0x13
#define NRF24L01P_REG_RX_PW_P3          0x14
#define NRF24L01P_REG_RX_PW_P4          0x15
#define NRF24L01P_REG_RX_PW_P5          0x16
#define NRF24L01P_REG_FIFO_STATUS       0x17
#define NRF24L01P_REG_DYNPD             0x1C
#define NRF24L01P_REG_FEATURE           0x1D


#endif /* __NRF24L01P_H__ */
