/*******************************************************************************
* Title                 :   HAL Implementation
* Filename              :   nrf24l01_hal.c
* Author                :   RBL
* Origin Date           :   12/12/2015
* Notes                 :   None
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  12/12/15           .1         RBL      Module Created.
*
*******************************************************************************/
/**
 * @file nrf_hal.c
 * @brief Implementation adapted from Nordic
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include "nrf24l01_hal.h"
#include "nrf24l01_defs.h"
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#if defined( __GNUC__ )
#if defined( STM32F0 )
#include"stm32f0xx.h"
#elif defined( STM32F1 )
#include "stm32f1xx.h"
#elif defined( STM32F2 )
#include "stm32f2xx.h"
#elif defined( STM32F3 )
#include "stm32f3xx.h"
#elif defined( STM32F4 )
#include "stm32f4xx.h"
#elif defined( STM32F7 )
#include "stm32f7xx.h"
#endif
#endif

extern SPI_HandleTypeDef hspi1;
/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static __IO bool u_timeout;
static enum
{
	NRF_TX_MODE,
	NRF_RX_MODE
} nrf_mode;

/******************************************************************************
* Function Prototypes
*******************************************************************************/
static inline void CE_PULSE( void );
static void nrf_hal_us_delay( uint16_t us);


/******************************************************************************
* Function Definitions
*******************************************************************************/
static void nrf_hal_us_delay( uint16_t us)
{

	while(!u_timeout);
}


/**
 * @brief Pulses the CE to nRF24L01 for at least 10 us
 */
static inline void CE_PULSE()
{
    NRF24L01_CE_HIGH();
  	nrf_hal_us_delay(10);
    NRF24L01_CE_LOW();
}


int nrf_hal_init( )
{
    /* nRF24L01 goes into standby-I mode if CE is low. Otherwise next payload
     * in TX FIFO is transmitted. If TX FIFO is empty and CE is still high,
     * nRF24L01 enters standby-II mode.
     *
     * Active RX/TX mode is started by setting CE high.
     */
    NRF24L01_CSS_HIGH();
    nrf_mode = NRF_RX_MODE;
}


uint8_t nrf_write_reg( uint8_t address, uint8_t *value, uint8_t count )
{
	assert( value != NULL);
	assert( count <= 5 );
	uint8_t status[6] = {0};
	uint8_t data[6] = {0};

	data[0] = NRF_WRITE_REG | ( address & 0x1F );
	memcpy( (void*)data, (void*)value, count );

	if( address == DYNPD )
	{
		uint8_t feature;
		uint8_t enabled;
		nrf_read_reg( FEATURE, &feature, 1 );
		nrf_read_reg( EN_AA, &enabled, 1 );

		if( !((feature & 0x04) && (enabled & value[0])))
		{
			return status[0];
		}
	}

	NRF24L01_CSS_LOW();

	if( HAL_SPI_TransmitReceive( &hspi1, &data[0], &status[0], count + 1, 100 ) != HAL_OK )
	{
		Error_Handler();
	}

	NRF24L01_CSS_HIGH();

	return status[0];
}

uint8_t nrf_read_reg( uint8_t address, uint8_t *value, uint8_t count )
{
	assert( value != NULL);
	assert( count <= 5 );
	uint8_t status = 0;

	NRF24L01_CSS_LOW();

	if( HAL_SPI_TransmitReceive( &hspi1, &address, &status, 1, 100 ) != HAL_OK )
	{
		Error_Handler();
	}
	else
	{
		if( HAL_SPI_Receive( &hspi1, value, count, 100 ) != HAL_OK )
		{
			Error_Handler();
		}
	}

	NRF24L01_CSS_HIGH();
	return status;
}

uint8_t nrf_command( uint8_t command )
{
	uint8_t status;

	NRF24L01_CSS_LOW();

	if( HAL_SPI_TransmitReceive( &hspi1, &command, &status, 1, 100 ) != HAL_OK )
	{
		Error_Handler();
	}

	NRF24L01_CSS_HIGH();
	return status;
}


uint8_t nrf_write_payload( uint8_t pipe, uint8_t *tx_pload, uint8_t length, bool ack)
{
	static uint8_t buffer[33];

	buffer[0] = ( ack ) ? (WR_ACK_PLOAD | (pipe & 0x07)) : WR_TX_PLOAD;
#ifdef NRF_DMA

#else

#endif
}


void nrf_delay( uint16_t ms )
{
   	HAL_Delay( ms );
}


/*************** END OF FUNCTIONS ***************************************************************************/
