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
#include "nrf24l01_defs.h"
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

/******************************************************************************
* Configuration Constants
*******************************************************************************/


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


/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the HAL layer
 *
 * @param PTX or PRX mode
 *
 * @return int result of init
 * @retval -1 failed
 * @retval 0 successful
 */
int nrf_hal_init( void );

/**
 * @brief Basis function write_reg.
 *
 * Use this function to write a new value to
 * a radio register.
 *
 * @param reg Register to write
 * @param value New value to write
 * @return Status register
*/
uint8_t nrf_write_reg( uint8_t address, uint8_t *value, uint8_t count );

/**
 * @brief Basis function read_reg.
 *
 * Use this function to read the contents
 * of one radios register.
 *
 * @param reg Register to read
 * @return Register contents
*/
uint8_t nrf_read_reg( uint8_t address, uint8_t *value, uint8_t count );

/**
 *
 * @param command
 * @return
 */
uint8_t nrf_command( uint8_t command );


uint8_t nrf_write_payload( uint8_t pipe, uint8_t *tx_pload, uint8_t length, bool ack);

/**
 * @brief nrf_hal_delay
 *
 * @param ms
 */
void nrf_hal_delay( uint16_t ms );


#ifdef __cplusplus
} // extern "C"
#endif

#endif /*File_H_*/

/*** End of File **************************************************************/
