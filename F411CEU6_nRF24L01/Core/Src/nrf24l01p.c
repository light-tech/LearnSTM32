/*
 *  nrf24l01_plus.c
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
 * 
 */


#include "nrf24l01p.h"

static void cs_high(nRF24L01P *nrf)
{
    HAL_GPIO_WritePin(nrf->csn.port, nrf->csn.pin, GPIO_PIN_SET);
}

static void cs_low(nRF24L01P *nrf)
{
    HAL_GPIO_WritePin(nrf->csn.port, nrf->csn.pin, GPIO_PIN_RESET);
}

static void ce_high(nRF24L01P *nrf)
{
    HAL_GPIO_WritePin(nrf->ce.port, nrf->ce.pin, GPIO_PIN_SET);
}

static void ce_low(nRF24L01P *nrf)
{
    HAL_GPIO_WritePin(nrf->ce.port, nrf->ce.pin, GPIO_PIN_RESET);
}

static uint8_t read_register(nRF24L01P *nrf, uint8_t reg)
{
    uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;
    uint8_t status;
    uint8_t read_val;

    cs_low(nrf);
    HAL_SPI_TransmitReceive(nrf->hspi, &command, &status, 1, 2000);
    HAL_SPI_Receive(nrf->hspi, &read_val, 1, 2000);
    cs_high(nrf);

    return read_val;
}

static uint8_t write_register(nRF24L01P *nrf, uint8_t reg, uint8_t value)
{
    uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;
    uint8_t status;
    uint8_t write_val = value;

    cs_low(nrf);
    HAL_SPI_TransmitReceive(nrf->hspi, &command, &status, 1, 2000);
    HAL_SPI_Transmit(nrf->hspi, &write_val, 1, 2000);
    cs_high(nrf);

    return write_val;
}


/* nRF24L01+ Main Functions */
void nrf24l01p_rx_init(nRF24L01P *nrf, channel MHz, air_data_rate bps)
{
    nrf24l01p_reset(nrf);

    nrf24l01p_prx_mode(nrf);
    nrf24l01p_power_up(nrf);

    nrf24l01p_rx_set_payload_widths(nrf, nrf->payloadLength);

    nrf24l01p_set_rf_channel(nrf, MHz);
    nrf24l01p_set_rf_air_data_rate(nrf, bps);
    nrf24l01p_set_rf_tx_output_power(nrf, _0dBm);

    nrf24l01p_set_crc_length(nrf, 1);
    nrf24l01p_set_address_widths(nrf, 5);

    nrf24l01p_auto_retransmit_count(nrf, 3);
    nrf24l01p_auto_retransmit_delay(nrf, 250);
    
    ce_high(nrf);
}

void nrf24l01p_tx_init(nRF24L01P *nrf, channel MHz, air_data_rate bps)
{
    nrf24l01p_reset(nrf);

    nrf24l01p_ptx_mode(nrf);
    nrf24l01p_power_up(nrf);

    nrf24l01p_set_rf_channel(nrf, MHz);
    nrf24l01p_set_rf_air_data_rate(nrf, bps);
    nrf24l01p_set_rf_tx_output_power(nrf, _0dBm);

    nrf24l01p_set_crc_length(nrf, 1);
    nrf24l01p_set_address_widths(nrf, 5);

    nrf24l01p_auto_retransmit_count(nrf, 3);
    nrf24l01p_auto_retransmit_delay(nrf, 250);

    ce_high(nrf);
}

void nrf24l01p_rx_receive(nRF24L01P *nrf, uint8_t* rx_payload)
{
    nrf24l01p_read_rx_fifo(nrf, rx_payload);
    nrf24l01p_clear_rx_dr(nrf);

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void nrf24l01p_tx_transmit(nRF24L01P *nrf, uint8_t* tx_payload)
{
    nrf24l01p_write_tx_fifo(nrf, tx_payload);
}

void nrf24l01p_tx_irq(nRF24L01P *nrf)
{
    uint8_t tx_ds = nrf24l01p_get_status(nrf);
    tx_ds &= 0x20;

    if(tx_ds)
    {   
        // TX_DS
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        nrf24l01p_clear_tx_ds(nrf);
    }

    else
    {
        // MAX_RT
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
        nrf24l01p_clear_max_rt(nrf);
    }
}

/* nRF24L01+ Sub Functions */
void nrf24l01p_reset(nRF24L01P *nrf)
{
    // Reset pins
    cs_high(nrf);
    ce_low(nrf);

    // Reset registers
    write_register(nrf, NRF24L01P_REG_CONFIG, 0x08);
    write_register(nrf, NRF24L01P_REG_EN_AA, 0x3F);
    write_register(nrf, NRF24L01P_REG_EN_RXADDR, 0x03);
    write_register(nrf, NRF24L01P_REG_SETUP_AW, 0x03);
    write_register(nrf, NRF24L01P_REG_SETUP_RETR, 0x03);
    write_register(nrf, NRF24L01P_REG_RF_CH, 0x02);
    write_register(nrf, NRF24L01P_REG_RF_SETUP, 0x07);
    write_register(nrf, NRF24L01P_REG_STATUS, 0x7E);
    write_register(nrf, NRF24L01P_REG_RX_PW_P0, 0x00);
    write_register(nrf, NRF24L01P_REG_RX_PW_P0, 0x00);
    write_register(nrf, NRF24L01P_REG_RX_PW_P1, 0x00);
    write_register(nrf, NRF24L01P_REG_RX_PW_P2, 0x00);
    write_register(nrf, NRF24L01P_REG_RX_PW_P3, 0x00);
    write_register(nrf, NRF24L01P_REG_RX_PW_P4, 0x00);
    write_register(nrf, NRF24L01P_REG_RX_PW_P5, 0x00);
    write_register(nrf, NRF24L01P_REG_FIFO_STATUS, 0x11);
    write_register(nrf, NRF24L01P_REG_DYNPD, 0x00);
    write_register(nrf, NRF24L01P_REG_FEATURE, 0x00);

    // Reset FIFO
    nrf24l01p_flush_rx_fifo(nrf);
    nrf24l01p_flush_tx_fifo(nrf);
}

void nrf24l01p_prx_mode(nRF24L01P *nrf)
{
    uint8_t new_config = read_register(nrf, NRF24L01P_REG_CONFIG);
    new_config |= 1 << 0;

    write_register(nrf, NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_ptx_mode(nRF24L01P *nrf)
{
    uint8_t new_config = read_register(nrf, NRF24L01P_REG_CONFIG);
    new_config &= 0xFE;

    write_register(nrf, NRF24L01P_REG_CONFIG, new_config);
}

uint8_t nrf24l01p_read_rx_fifo(nRF24L01P *nrf, uint8_t* rx_payload)
{
    uint8_t command = NRF24L01P_CMD_R_RX_PAYLOAD;
    uint8_t status;

    cs_low(nrf);
    HAL_SPI_TransmitReceive(nrf->hspi, &command, &status, 1, 2000);
    HAL_SPI_Receive(nrf->hspi, rx_payload, nrf->payloadLength, 2000);
    cs_high(nrf);

    return status;
}

uint8_t nrf24l01p_write_tx_fifo(nRF24L01P *nrf, uint8_t* tx_payload)
{
    uint8_t command = NRF24L01P_CMD_W_TX_PAYLOAD;
    uint8_t status;

    cs_low(nrf);
    HAL_SPI_TransmitReceive(nrf->hspi, &command, &status, 1, 2000);
    HAL_SPI_Transmit(nrf->hspi, tx_payload, nrf->payloadLength, 2000);
    cs_high(nrf);

    return status;
}

void nrf24l01p_flush_rx_fifo(nRF24L01P *nrf)
{
    uint8_t command = NRF24L01P_CMD_FLUSH_RX;
    uint8_t status;

    cs_low(nrf);
    HAL_SPI_TransmitReceive(nrf->hspi, &command, &status, 1, 2000);
    cs_high(nrf);
}

void nrf24l01p_flush_tx_fifo(nRF24L01P *nrf)
{
    uint8_t command = NRF24L01P_CMD_FLUSH_TX;
    uint8_t status;

    cs_low(nrf);
    HAL_SPI_TransmitReceive(nrf->hspi, &command, &status, 1, 2000);
    cs_high(nrf);
}

uint8_t nrf24l01p_get_status(nRF24L01P *nrf)
{
    uint8_t command = NRF24L01P_CMD_NOP;
    uint8_t status;

    cs_low(nrf);
    HAL_SPI_TransmitReceive(nrf->hspi, &command, &status, 1, 2000);
    cs_high(nrf);

    return status;
}

uint8_t nrf24l01p_get_fifo_status(nRF24L01P *nrf)
{
    return read_register(nrf, NRF24L01P_REG_FIFO_STATUS);
}

void nrf24l01p_rx_set_payload_widths(nRF24L01P *nrf, widths bytes)
{
    write_register(nrf, NRF24L01P_REG_RX_PW_P0, bytes);
}

void nrf24l01p_clear_rx_dr(nRF24L01P *nrf)
{
    uint8_t new_status = nrf24l01p_get_status(nrf);
    new_status |= 0x40;

    write_register(nrf, NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_clear_tx_ds(nRF24L01P *nrf)
{
    uint8_t new_status = nrf24l01p_get_status(nrf);
    new_status |= 0x20;

    write_register(nrf, NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_clear_max_rt(nRF24L01P *nrf)
{
    uint8_t new_status = nrf24l01p_get_status(nrf);
    new_status |= 0x10;

    write_register(nrf, NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_power_up(nRF24L01P *nrf)
{
    uint8_t new_config = read_register(nrf, NRF24L01P_REG_CONFIG);
    new_config |= 1 << 1;

    write_register(nrf, NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_power_down(nRF24L01P *nrf)
{
    uint8_t new_config = read_register(nrf, NRF24L01P_REG_CONFIG);
    new_config &= 0xFD;

    write_register(nrf, NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_crc_length(nRF24L01P *nrf, length bytes)
{
    uint8_t new_config = read_register(nrf, NRF24L01P_REG_CONFIG);
    
    switch(bytes)
    {
        // CRCO bit in CONFIG resiger set 0
        case 1:
            new_config &= 0xFB;
            break;
        // CRCO bit in CONFIG resiger set 1
        case 2:
            new_config |= 1 << 2;
            break;
    }

    write_register(nrf, NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_address_widths(nRF24L01P *nrf, widths bytes)
{
    write_register(nrf, NRF24L01P_REG_SETUP_AW, bytes - 2);
}

void nrf24l01p_auto_retransmit_count(nRF24L01P *nrf, count cnt)
{
    uint8_t new_setup_retr = read_register(nrf, NRF24L01P_REG_SETUP_RETR);
    
    // Reset ARC register 0
    new_setup_retr |= 0xF0;
    new_setup_retr |= cnt;
    write_register(nrf, NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_auto_retransmit_delay(nRF24L01P *nrf, delay us)
{
    uint8_t new_setup_retr = read_register(nrf, NRF24L01P_REG_SETUP_RETR);

    // Reset ARD register 0
    new_setup_retr |= 0x0F;
    new_setup_retr |= ((us / 250) - 1) << 4;
    write_register(nrf, NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_set_rf_channel(nRF24L01P *nrf, channel MHz)
{
	uint16_t new_rf_ch = MHz - 2400;
    write_register(nrf, NRF24L01P_REG_RF_CH, new_rf_ch);
}

void nrf24l01p_set_rf_tx_output_power(nRF24L01P *nrf, output_power dBm)
{
    uint8_t new_rf_setup = read_register(nrf, NRF24L01P_REG_RF_SETUP) & 0xF9;
    new_rf_setup |= (dBm << 1);

    write_register(nrf, NRF24L01P_REG_RF_SETUP, new_rf_setup);
}

void nrf24l01p_set_rf_air_data_rate(nRF24L01P *nrf, air_data_rate bps)
{
    // Set value to 0
    uint8_t new_rf_setup = read_register(nrf, NRF24L01P_REG_RF_SETUP) & 0xD7;
    
    switch(bps)
    {
        case _1Mbps: 
            break;
        case _2Mbps: 
            new_rf_setup |= 1 << 3;
            break;
        case _250kbps:
            new_rf_setup |= 1 << 5;
            break;
    }
    write_register(nrf, NRF24L01P_REG_RF_SETUP, new_rf_setup);
}
