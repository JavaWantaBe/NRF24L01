/****************************************************************************
* Title                 :   NRF24L01 Hardware Access Layer
* Filename              :   nrf24l01_hal.h
* Author                :   RBL
* Origin Date           :   06/01/2016
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials   Description
*  06/01/16         .1            RBL      Interface Created.
*
*****************************************************************************/
/** 
 * @file nrf24l01_hal.h
 * @brief Hardware Access Layer for the 24l01 radio.
 *
 * @date 8 Jan 2016
 * @author Richard Lowe
 * @copyright GNU Public License
 *
 * @version .1 - Initial testing and verification
 *
 * @note Test configuration:
 *   MCU:             STM32F107VC
 *   Dev.Board:       EasyMx Pro v7
 *   Oscillator:      72 Mhz internal
 *   Ext. Modules:    NRF C click
 *   SW:              ARM 4.7
 *
 */
#ifndef NRF_HAL_H_
#define NRF_HAL_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include "main.h"
#include "stm32f0xx.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
// CS Chip select
#define NRF24L01_CSS_PIN				NRF_CSS_Pin
#define NRF24L01_CSS_PORT				NRF_CSS_GPIO_Port
// CE Chip enable
#define NRF24L01_CE_PIN					NRF_CE_Pin
#define NRF24L01_CE_PORT				NRF_CE_GPIO_Port

#define NRF_SPI_TIMEOUT 100
#define NRF_MAX_PACKET_SIZE 32

/******************************************************************************
* Configuration Constants
*******************************************************************************/
/* Registers */
#define NRF_CONFIG 0x00
#define NRF_EN_AA 0x01
#define NRF_EN_RXADDR 0x02
#define NRF_SETUP_AW 0x03
#define NRF_SETUP_RETR 0x04
#define NRF_RF_CH 0x05
#define NRF_RF_SETUP 0x06
#define NRF_STATUS 0x07
#define NRF_OBSERVE_TX 0x08
#define NRF_CD 0x09
#define NRF_RX_ADDR_P0 0x0A
#define NRF_RX_ADDR_P1 0x0B
#define NRF_RX_ADDR_P2 0x0C
#define NRF_RX_ADDR_P3 0x0D
#define NRF_RX_ADDR_P4 0x0E
#define NRF_RX_ADDR_P5 0x0F
#define NRF_TX_ADDR 0x10
#define NRF_RX_PW_P0 0x11
#define NRF_RX_PW_P1 0x12
#define NRF_RX_PW_P2 0x13
#define NRF_RX_PW_P3 0x14
#define NRF_RX_PW_P4 0x15
#define NRF_RX_PW_P5 0x16
#define NRF_FIFO_STATUS 0x17
#define NRF_DYNPD 0x1C
#define NRF_FEATURE 0x1D

/* Commands */
#define NRF_CMD_R_REGISTER 0x00
#define NRF_CMD_W_REGISTER 0x20
#define NRF_CMD_R_RX_PAYLOAD 0x61
#define NRF_CMD_W_TX_PAYLOAD 0xA0
#define NRF_CMD_FLUSH_TX 0xE1
#define NRF_CMD_FLUSH_RX 0xE2
#define NRF_CMD_REUSE_TX_PL 0xE3
#define NRF_CMD_R_RX_PL_WID 0x60
#define NRF_CMD_W_ACK_PAYLOAD 0xA8
#define NRF_CMD_W_TX_PAYLOAD_NOACK 0xB0
#define NRF_CMD_NOP 0xFF

/******************************************************************************
* Macros
*******************************************************************************/
// Chip Select
#define NRF24L01_CSS_LOW() HAL_GPIO_WritePin(NRF24L01_CSS_PORT, NRF24L01_CSS_PIN, GPIO_PIN_RESET)
#define NRF24L01_CSS_HIGH() HAL_GPIO_WritePin(NRF24L01_CSS_PORT, NRF24L01_CSS_PIN, GPIO_PIN_SET)
// Chip Enable
#define NRF24L01_CE_LOW() HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_RESET)
#define NRF24L01_CE_HIGH() HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_SET)

/******************************************************************************
* Typedefs
*******************************************************************************/
/**
 * @enum Result of OP
 */
typedef enum
{
    NRF_OK,         //!< NRF_OK
    NRF_ERROR       //!< NRF_ERROR
} NRF_RESULT_t;

/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

NRF_RESULT_t NRF_SetupGPIO( void );
NRF_RESULT_t NRF_SendCommand( uint8_t cmd, uint8_t* tx, uint8_t* rx, uint8_t len );
/* CMD */
NRF_RESULT_t NRF_ReadRegister( uint8_t reg, uint8_t* data );
NRF_RESULT_t NRF_WriteRegister( uint8_t reg, uint8_t* data );
NRF_RESULT_t NRF_ReadRXPayload( uint8_t* data, uint8_t len );
NRF_RESULT_t NRF_WriteTXPayload( uint8_t* data, uint8_t len );
NRF_RESULT_t NRF_FlushTX( void );
NRF_RESULT_t NRF_FlushRX( void );

#ifdef __cplusplus
} // extern "C"
#endif

#endif /*File_H_*/

/*** End of File **************************************************************/
