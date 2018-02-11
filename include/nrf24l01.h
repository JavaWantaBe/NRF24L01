#ifndef NRF24L01_H_
#define NRF24L01_H_


#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx_hal.h"
#include "nrf24l01_driver.h"


typedef enum
{
    NRF_SHOCKBURST = 0,
    NRF_ENHANCED,
    NRF_ENHANCED_SHOCKBURST
} NRF_RADIO_MODE_t;


typedef enum
{
    NRF_DATA_RATE_1MBPS = 0,
    NRF_DATA_RATE_2MBPS = 1,
    NRF_DATA_RATE_250KBPS = 2
} NRF_DATA_RATE_t;

typedef enum
{
    NRF_TX_PWR_M18dBm = 0,
    NRF_TX_PWR_M12dBm = 1,
    NRF_TX_PWR_M6dBm = 2,
    NRF_TX_PWR_0dBm = 3
} NRF_TX_PWR_t;

typedef enum
{
    NRF_ADDR_WIDTH_3 = 1,
    NRF_ADDR_WIDTH_4 = 2,
    NRF_ADDR_WIDTH_5 = 3
} NRF_ADDR_WIDTH_t;

typedef enum
{
    NRF_CRC_WIDTH_1B = 0,
    NRF_CRC_WIDTH_2B = 1
} NRF_CRC_WIDTH_t;

typedef enum
{
    NRF_STATE_RX = 1,
    NRF_STATE_TX = 0
} NRF_TXRX_STATE_t;

typedef enum
{
   NRF_WAIT_250us = 0,
   NRF_WAIT_500us,
   NRF_WAIT_750us,
   NRF_WAIT_1000us,
   NRF_WAIT_1250us,
   NRF_WAIT_1500us,
   NRF_WAIT_1750us,
   NRF_WAIT_2000us,
   NRF_WAIT_2250us,
   NRF_WAIT_2500us,
   NRF_WAIT_2750us,
   NRF_WAIT_3000us,
   NRF_WAIT_3250us,
   NRF_WAIT_3500us,
   NRF_WAIT_3750us,
   NRF_WAIT_4000us,
} NRF_RETRANS_DELAY_t;

typedef struct
{
    NRF_RADIO_MODE_t mode;
    NRF_DATA_RATE_t date_rate;
    NRF_TX_PWR_t tx_power;
    NRF_CRC_WIDTH_t crc_width;
    NRF_ADDR_WIDTH_t addr_width;
    uint8_t payload_len : 6;        // PayloadLength   : maximum is 32 Bytes
    uint8_t retransmit_count :4;    // RetransmitCount : can be 0~15 times
    NRF_RETRANS_DELAY_t retransmit_delay;    // RetransmitDelay : 0[250uS]~0x0F(4000us), LSB:250us

    __IO uint8_t BUSY_FLAG;
    __IO NRF_TXRX_STATE_t state;

    /* Usr interface, Rx/Tx Buffer */
    uint8_t* rx_buffer;
    uint8_t* tx_buffer;

    /* nrf24L01 transmit/receive frequency define */
    /* channel can be 0~127 */
    uint8_t rf_channel :7;

    /* nrf24L01 Tx/Rx address
     Pipe [0:1] has maximum 5 Bytes address
     Pipe [2:5] has 3 Bytes address */
    uint8_t* rx_address;
    uint8_t* tx_address;

} nrf24l01_dev_t;

/* Initialization routine */
NRF_RESULT_t NRF_Init( nrf24l01_dev_t* dev );

/* EXTI Interrupt Handler */
void NRF_IRQ_Handler( nrf24l01_dev_t* dev );

/* Blocking Data Sending / Receiving FXs */
NRF_RESULT_t NRF_SendPacket( nrf24l01_dev_t* dev, uint8_t* data );
NRF_RESULT_t NRF_ReceivePacket( nrf24l01_dev_t* dev, uint8_t* data );

/* Non-Blocking Data Sending / Receiving FXs */
NRF_RESULT_t NRF_PushPacket( nrf24l01_dev_t* dev, uint8_t* data );
NRF_RESULT_t NRF_PullPacket( nrf24l01_dev_t* dev, uint8_t* data );

/* RF_SETUP */
NRF_RESULT_t NRF_SetDataRate( nrf24l01_dev_t* dev, NRF_DATA_RATE_t rate );
NRF_RESULT_t NRF_SetTXPower( nrf24l01_dev_t* dev, NRF_TX_PWR_t pwr );
NRF_RESULT_t NRF_SetCCW( nrf24l01_dev_t* dev, bool activate );

/* STATUS */
NRF_RESULT_t NRF_ClearInterrupts( nrf24l01_dev_t* dev );

/* RF_CH */
NRF_RESULT_t NRF_SetRFChannel( nrf24l01_dev_t* dev, uint8_t ch );

/* SETUP_RETR */
NRF_RESULT_t NRF_SetRetransmittionCount( nrf24l01_dev_t* dev, uint8_t count );
NRF_RESULT_t NRF_SetRetransmittionDelay( nrf24l01_dev_t* dev, NRF_RETRANS_DELAY_t delay );

/* SETUP_AW */
NRF_RESULT_t NRF_SetAddressWidth( nrf24l01_dev_t* dev, NRF_ADDR_WIDTH_t width );

/* EN_RXADDR */
NRF_RESULT_t NRF_EnableRXPipe( nrf24l01_dev_t* dev, uint8_t pipe );

/* EN_AA */
NRF_RESULT_t NRF_EnableAutoAcknowledgement( nrf24l01_dev_t* dev, uint8_t pipe );

/* CONFIG */
NRF_RESULT_t NRF_EnableCRC( nrf24l01_dev_t* dev, bool activate );
NRF_RESULT_t NRF_SetCRCWidth( nrf24l01_dev_t* dev, NRF_CRC_WIDTH_t width );
NRF_RESULT_t NRF_PowerUp( nrf24l01_dev_t* dev, bool powerUp );
NRF_RESULT_t NRF_RXTXControl( nrf24l01_dev_t* dev, NRF_TXRX_STATE_t rx );
NRF_RESULT_t NRF_EnableRXDataReadyIRQ( nrf24l01_dev_t* dev, bool activate );
NRF_RESULT_t NRF_EnableTXDataSentIRQ( nrf24l01_dev_t* dev, bool activate );
NRF_RESULT_t NRF_EnableMaxRetransmitIRQ( nrf24l01_dev_t* dev, bool activate );

/* RX_ADDR_P0 */
NRF_RESULT_t NRF_SetRXAddress_P0( nrf24l01_dev_t* dev, uint8_t* address );  // 5bytes of address

/* TX_ADDR */
NRF_RESULT_t NRF_SetTXAddress( nrf24l01_dev_t* dev, uint8_t* address );  // 5bytes of address

/* RX_PW_P0 */
NRF_RESULT_t NRF_SetRXPayloadWidth_P0( nrf24l01_dev_t* dev, uint8_t width );


/************************************
 ***** Test Functions ***************
 ***********************************/
// PLL Clock Functions
void NRF_SetPLLMode( nrf24l01_dev_t* dev, uint8_t pll_mode );
uint8_t NRF_GetPLLMode( void );
// LNA Gain Functions
void NRF_SetLNAGain( nrf24l01_dev_t* dev, uint8_t lna_gain );
// Get LNA gain
uint8_t NRF_GetLNAGain( void );


#endif /* NRF24L01_H_ */
