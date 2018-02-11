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
#include "nrf24l01_driver.h"
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

/******************************************************************************
* Function Prototypes
*******************************************************************************/
static inline void CE_PULSE( void );
static void nrf_hal_us_delay( uint16_t us);

/******************************************************************************
* Function Definitions
*******************************************************************************/
NRF_RESULT_t NRF_SetupGPIO()
{

   GPIO_InitTypeDef GPIO_InitStructure;

//   // CE pin
//   GPIO_InitStructure.Pin = dev->NRF_CE_GPIO_PIN;
//   GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
//   GPIO_InitStructure.Pull = GPIO_NOPULL;
//
//   HAL_GPIO_Init( dev->NRF_CE_GPIOx, &GPIO_InitStructure );
//   // end CE pin
//
//   // IRQ pin
//   GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
//   GPIO_InitStructure.Pull = GPIO_PULLUP;
//   GPIO_InitStructure.Pin = dev->NRF_IRQ_GPIO_PIN;
//   HAL_GPIO_Init( dev->NRF_IRQ_GPIOx, &GPIO_InitStructure );
//
//   // CSN pin
//   GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStructure.Pull = GPIO_NOPULL;
//   GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
//   GPIO_InitStructure.Pin = dev->NRF_CSN_GPIO_PIN;
//
//   HAL_GPIO_Init( dev->NRF_CSN_GPIOx, &GPIO_InitStructure );
//   /* Enable and set EXTI Line Interrupt to the given priority */
//   HAL_NVIC_SetPriority( dev->NRF_IRQn, dev->NRF_IRQ_preempt_priority, dev->NRF_IRQ_sub_priority );
//   HAL_NVIC_EnableIRQ( dev->NRF_IRQn );
//   // end IRQ pin
//
//   NRF_CS_RESETPIN( dev );
//   NRF_CE_RESETPIN( dev );

   return NRF_OK;
}


/** Implementation of private functions **/
NRF_RESULT_t NRF_SendCommand( uint8_t cmd, uint8_t* tx, uint8_t* rx, uint8_t len )
{
   uint8_t myTX[len + 1];
   uint8_t myRX[len + 1];

   myTX[0] = cmd;

   int i = 0;
   for( i = 0; i < len; i++ )
   {
       myTX[1 + i] = tx[i];
       myRX[i] = 0;
   }

   NRF24L01_CSS_LOW();

   if( HAL_SPI_TransmitReceive( &hspi1, myTX, myRX, 1 + len, NRF_SPI_TIMEOUT ) != HAL_OK )
   {
       return NRF_ERROR;
   }

   for( i = 0; i < len; i++ )
   {
       rx[i] = myRX[1 + i];
   }

   NRF24L01_CSS_HIGH();

   return NRF_OK;
}


NRF_RESULT_t NRF_ReadRegister( uint8_t reg, uint8_t* data )
{
   uint8_t tx = 0;
   return NRF_SendCommand( NRF_CMD_R_REGISTER | reg, &tx, data, 1 );
}

NRF_RESULT_t NRF_WriteRegister( uint8_t reg, uint8_t* data )
{
   uint8_t rx = 0;
   return NRF_SendCommand( NRF_CMD_W_REGISTER | reg, data, &rx, 1 );
}

NRF_RESULT_t NRF_ReadRXPayload( uint8_t* data, uint8_t len )
{
    uint8_t tx[32];
    return NRF_SendCommand( NRF_CMD_R_RX_PAYLOAD, tx, data, len);
}

NRF_RESULT_t NRF_WriteTXPayload( uint8_t* data, uint8_t len )
{
    uint8_t rx[32];
    return NRF_SendCommand( NRF_CMD_W_TX_PAYLOAD, data, rx, len);
}

NRF_RESULT_t NRF_FlushTX()
{
   uint8_t rx = 0;
   uint8_t tx = 0;
   return NRF_SendCommand( NRF_CMD_FLUSH_TX, &tx, &rx, 0 );
}

NRF_RESULT_t NRF_FlushRX()
{
   uint8_t rx = 0;
   uint8_t tx = 0;
   return NRF_SendCommand(NRF_CMD_FLUSH_RX, &tx, &rx, 0 );
}
/*************** END OF FUNCTIONS ***************************************************************************/
