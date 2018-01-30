/*******************************************************************************
* Title                 :   NRF24L01 Hardware Layer
* Filename              :   nrf_l01_hw.c
* Author                :   RBL
* Origin Date           :   21/12/2015
* Notes                 :   None
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  29/12/15         .1              RBL     Module Created.
*  29/01/16         .2              RBL     Fixed multi-reg write
*
*******************************************************************************/
/**
 * @file nrf_l01_hw.c
 * @brief Hardware layer for accessing NRF24L01+ radio
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include "nrf24l01_hw.h"
#include "nrf24l01_hal.h"
#include <stdint.h>
#include <stdbool.h>

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
static nrf_radio_mode_t cur_mode;

/******************************************************************************
* Function Prototypes
*******************************************************************************/

/**
 * @brief Write ack payload
 *
 * Writes the payload that will be transmitted with the ack on the given pipe.
 *
 * @param pipe Pipe that transmits the payload
 * @param tx_pload Pointer to the payload data
 * @param length Size of the data to transmit
 */
static uint8_t nrf_write_ack_pload( uint8_t pipe, uint8_t *tx_pload,  uint8_t length );

/**
 * @brief Reads the payload width of the received ack payload
 *
 * @return Payload width of the received ack payload
 */
static inline uint8_t nrf_read_rx_pl_w( void );

/**
 * @brief nrf_get_fifo_status
 *
 * @return
 */
static inline uint8_t nrf_get_fifo_status( void );

/******************************************************************************
* Function Definitions
*******************************************************************************/
/** @name  << BASIC FUNCTIONS >> */

static inline uint8_t nrf_write_ack_pload( uint8_t pipe, uint8_t *tx_pload, uint8_t length )
{
    uint8_t status;
	status = nrf_write_reg( WR_ACK_PLOAD | pipe, tx_pload, length );
	return status;
}

static inline uint8_t nrf_read_rx_pl_w()
{
	uint8_t pl_w;
	nrf_read_reg( RD_RX_PLOAD_W, &pl_w, 1 );
    return pl_w;
}

static inline uint8_t nrf_get_fifo_status()
{
    uint8_t status = 0;
	nrf_read_reg( FIFO_STATUS, &status, 1 );
	return status;
}


/****************************************************************************
 * Public Functions
 ***************************************************************************/
/****************************************
 ********** Setup Functions *************
 ***************************************/
int nrf_hw_init( nrf_radio_mode_t mode, nrf_operation_mode_t operational_mode )
{
	cur_mode = mode;
	nrf_hal_init();
}

// Interrupt functions
void nrf_set_irq_mode( nrf_irq_source_t int_source, bool irq_state )
{
	uint8_t state;

	nrf_read_reg( CONFIG, &state, 1 );
	state = ( irq_state ) ? ( state & ~NRF_SET_BIT( int_source )) : ( state | NRF_SET_BIT( int_source ) );
	nrf_write_reg( CONFIG, &state, 1 );
}

void nrf_clear_irq_flag( nrf_irq_source_t int_source )
{
    uint8_t status = NRF_SET_BIT( int_source );
	nrf_write_reg( STATUS, &status, 1 );
}

void nrf_enable_ack_pl( bool enabled )
{
    uint8_t feature;

    nrf_read_reg( FEATURE , &feature, 1);
	feature = ( enabled ) ? (feature |= 0x02) : (feature & ~ 0x02);
    nrf_write_reg( FEATURE, &feature, 1 );
}

void nrf_enable_dynamic_pl( bool enabled )
{
	uint8_t feature;

	nrf_read_reg( FEATURE , &feature, 1);
	feature = ( enabled ) ? (feature |= 0x04) : (feature & ~ 0x04);
	nrf_write_reg( FEATURE, &feature, 1 );
}


void nrf_setup_dyn_pl( uint8_t setup )
{
    nrf_write_reg( DYNPD, setup & ~0xC0 );
}

void nrf_enable_dynamic_ack( bool enaled )
{
	uint8_t feature;

	nrf_read_reg( FEATURE , &feature, 1);
	feature = ( enabled ) ? (feature |= 0x01) : (feature & ~ 0x01);
	nrf_write_reg( FEATURE, &feature, 1 );
}


// CRC Functions
void nrf_set_crc_mode( nrf_crc_mode_t crc_mode )
{
    nrf_write_reg( CONFIG, ( nrf_hal_read_reg( CONFIG ) &
    			~( ( NRF_BIT_3 | NRF_BIT_2 ) | ( uint8_t )( crc_mode << 2 ) ) ) );
}

uint8_t nrf_get_crc_mode( void )
{
    return ( nrf_hal_read_reg( CONFIG ) & ( NRF_BIT_3 | NRF_BIT_2 ) ) >> CRCO;
}


// Pipe Functions
void nrf_open_pipe( nrf_address_t pipe_num, bool auto_ack )
{
	uint8_t pipes;
	uint8_t ack;

	nrf_read_reg( EN_RXADDR, &pipes, 1 );
	pipes = pipes | (NRF_SET_BIT( pipe_num ) );
	nrf_write_reg( EN_RXADDR, &pipes, 1 );

	nrf_read_reg( EN_AA, &ack, 1 );
	ack = ( auto_ack ) ? (ack | (NRF_SET_BIT( pipe_num ) ) ) : (ack & ~(NRF_SET_BIT( pipe_num ) ) );
	nrf_write_reg( EN_AA, &ack, 1 );
}

void nrf_close_pipe( nrf_address_t pipe_num )
{
    uint8_t pipes;

    nrf_read_reg( EN_RXADDR, &pipes, 1 );
    pipes = pipes & ~(NRF_SET_BIT( pipe_num ) );
	nrf_write_reg( EN_RXADDR, &pipes, 1 );

	if( cur_mode == NRF_ENHANCED || cur_mode == NRF_ENHANCED_BIDIR )
	{
		uint8_t ack;
		nrf_read_reg( EN_AA, &ack, 1 );
		ack = ack & ~(NRF_SET_BIT( pipe_num ) );
		nrf_write_reg( EN_AA, &ack, 1 );
	}
}


uint8_t nrf_get_pipe_status( nrf_address_t pipe_num )
{
    uint8_t en_rx, en_aa;

    en_rx = nrf_hal_read_reg( EN_RXADDR ) & ( 1 << ( uint8_t )pipe_num );
    en_aa = nrf_hal_read_reg( EN_AA ) & ( 1 << ( uint8_t )pipe_num );

    en_rx >>= ( uint8_t )pipe_num;
    en_aa >>= ( uint8_t )pipe_num;

    return ( en_aa << 1 ) + en_rx;
}


void nrf_set_address( nrf_address_t address, uint8_t *addr )
{
    switch( address )
    {
        case NRF_TX:
        case NRF_PIPE0:
        case NRF_PIPE1:
            nrf_write_multibyte_reg( ( uint8_t )address, addr, 0 );
            break;
        case NRF_PIPE2:
        case NRF_PIPE3:
        case NRF_PIPE4:
        case NRF_PIPE5:
            nrf_write_reg( RX_ADDR_P0 + ( uint8_t ) address, *addr );
            break;
        default:
            break;
    }
}


uint8_t nrf_get_address( nrf_address_t address, uint8_t *addr )
{
    switch( address )
    {
        case NRF_PIPE0:
        case NRF_PIPE1:
        case NRF_TX:
            return nrf_read_multibyte_reg( address, addr );
        default:
            *addr = nrf_hal_read_reg( RX_ADDR_P0 + address );
            return nrf_get_address_width();
    }
}

void nrf_set_address_width( nrf_address_width_t address_width )
{
    nrf_write_reg( SETUP_AW, ( uint8_t )( address_width - 2 ) );
}

uint8_t nrf_get_address_width( void )
{
    return ( nrf_hal_read_reg( SETUP_AW ) + 2 );
}




// Auto re-transmit and number of transmissions
void nrf_set_auto_retr( uint8_t retr, uint16_t delay )
{
    retr = NRF_MIN( retr, 15 );
    nrf_write_reg( SETUP_RETR, ( ( ( delay / 250 ) - 1 ) << 4 ) | retr );
}

uint8_t nrf_get_auto_retr_status( void )
{
    return nrf_hal_read_reg( OBSERVE_TX );
}

uint8_t nrf_get_transmit_attempts( void )
{
    return nrf_hal_read_reg( OBSERVE_TX ) &
    		( NRF_BIT_3 | NRF_BIT_2 | NRF_BIT_1 | NRF_BIT_0 );
}

uint8_t nrf_get_packet_lost_ctr( void )
{
    return ( nrf_hal_read_reg( OBSERVE_TX ) &
    		( NRF_BIT_7 | NRF_BIT_6 | NRF_BIT_5 | NRF_BIT_4 ) ) >> 4;
}

void nrf_set_rx_pload_width( uint8_t pipe_num, uint8_t pload_width )
{
    nrf_write_reg( RX_PW_P0 + pipe_num, pload_width );
}

uint8_t nrf_get_rx_pload_width( uint8_t pipe_num )
{
    return nrf_hal_read_reg( RX_PW_P0 + pipe_num );
}

/******************************************
 ******** Operation Functions *************
 *****************************************/
// Operation Modes and Radio Settings
void nrf_set_operation_mode( nrf_operation_mode_t op_mode )
{
    if( op_mode == NRF_PRX )
        nrf_write_reg( CONFIG,
                       ( nrf_hal_read_reg( CONFIG ) | ( 1 << PRIM_RX ) ) );
    else
        nrf_write_reg( CONFIG,
                       ( nrf_hal_read_reg( CONFIG ) & ~( 1 << PRIM_RX ) ) );
}

uint8_t nrf_get_operation_mode()
{
    return ( nrf_hal_read_reg( CONFIG ) & ( 1 << PRIM_RX ) );
}

void nrf_start_active_rx()
{
   if( nrf_get_operation_mode() & ( 1 << PRIM_RX ) )
       nrf_hal_listen();
}

void nrf_enter_standby_mode()
{
    nrf_hal_ignore();
}

// Power Mode Functions
void nrf_set_power_mode( nrf_pwr_mode_t pwr_mode )
{
    if( pwr_mode == NRF_PWR_UP )
    {
        nrf_write_reg( CONFIG, ( nrf_hal_read_reg( CONFIG ) | ( 1 << PWR_UP ) ) );
    } else {
        nrf_write_reg( CONFIG,
                       ( nrf_hal_read_reg( CONFIG ) & ~( 1 << PWR_UP ) ) );
    }
}

uint8_t nrf_get_power_mode()
{
    return ( nrf_hal_read_reg( CONFIG ) & ( 1 << PWR_UP ) ) >> PWR_UP;
}


// Channel and Channel Power Functions
void nrf_set_rf_channel( uint8_t channel )
{
    nrf_write_reg( RF_CH, channel );
}

uint8_t nrf_get_rf_channel( void )
{
    return nrf_hal_read_reg( RF_CH );
}

void nrf_set_output_power( nrf_output_power_t power )
{
    nrf_write_reg( RF_SETUP,
                   ( nrf_hal_read_reg( RF_SETUP ) & ~( ( 1 << RF_PWR1 ) |
                           ( 1 << RF_PWR0 ) ) ) | ( ( uint8_t )( power ) << 1 ) );
}

uint8_t nrf_get_output_power()
{
    return ( nrf_hal_read_reg( RF_SETUP ) & ( ( 1 << RF_PWR1 ) |
             ( 1 << RF_PWR0 ) ) ) >> RF_PWR0;
}


// DataRate Functions
void nrf_set_datarate( nrf_datarate_t datarate )
{
    if( datarate == NRF_1MBPS )
        nrf_write_reg( RF_SETUP,
                       ( nrf_hal_read_reg( RF_SETUP ) & ~( 1 << RF_DR ) ) );
    else
        nrf_write_reg( RF_SETUP,
                       ( nrf_hal_read_reg( RF_SETUP ) | ( 1 << RF_DR ) ) );
}

uint8_t nrf_get_datarate( void )
{
    return ( nrf_hal_read_reg( RF_SETUP ) & ( 1 << RF_DR ) ) >> RF_DR;
}

//void nrf_delay( uint16_t ms )
//{
//    nrf_hal_delay( ms );
//}


/******************************************
 ******** Status Functions *************
 *****************************************/
// FIFO Functions
uint8_t nrf_get_tx_fifo_status( void )
{
    return ( ( nrf_get_fifo_status() & ( ( 1 << TX_FIFO_FULL ) |
                                         ( 1 << TX_EMPTY ) ) ) >> 4 );
}

bool nrf_tx_fifo_empty( void )
{
    return ( bool )( ( nrf_get_fifo_status() >> TX_EMPTY ) & 1 );
}

bool nrf_tx_fifo_full( void )
{
    return ( bool )( ( nrf_get_fifo_status() >> TX_FIFO_FULL ) & 1 );
}

uint8_t nrf_get_rx_fifo_status( void )
{
    return ( nrf_get_fifo_status() & ( ( 1 << RX_FULL ) |
                                       ( 1 << RX_EMPTY ) ) );
}

bool nrf_rx_fifo_empty( void )
{
    if( nrf_get_rx_data_source() == 7 )
        return true;
    else
        return false;
}

bool nrf_rx_fifo_full( void )
{
    return ( bool )( ( nrf_get_fifo_status() >> RX_EMPTY ) & 1 );
}

void nrf_flush_rx( void )
{
    nrf_write_reg( FLUSH_RX, 0 );
}

void nrf_flush_tx( void )
{
    nrf_write_reg( FLUSH_TX, 0 );
}

bool nrf_get_carrier_detect( void )
{
    return nrf_hal_read_reg( CD ) & 1;
}


/*************************************
 *********** Data Operations *********
 ************************************/
nrf_address_t nrf_get_rx_data_source( void )
{
    return ( ( nrf_nop() & ( NRF_BIT_3 | NRF_BIT_2 | NRF_BIT_1 ) ) >> 1 );
}

// Transmit and Recieve Functions
// Fixed: returns length==0 and pipe==7 means FIFO empty
uint16_t nrf_read_rx_pload( uint8_t *rx_pload )
{
    return nrf_read_multibyte_reg( ( uint8_t )( NRF_RX_PLOAD ), rx_pload );
}

void nrf_write_tx_pload( uint8_t *tx_pload, uint8_t length )
{
    nrf_write_multibyte_reg( ( uint8_t )( NRF_TX_PLOAD ), tx_pload, length );
}

// Reuse Functions
void nrf_reuse_tx( void )
{
    nrf_write_reg( REUSE_TX_PL, 0 );
}

bool nrf_get_reuse_tx_status( void )
{
    return ( bool )( ( nrf_get_fifo_status() & ( 1 << TX_REUSE ) ) >>
                     TX_REUSE );
}

/************************************
 ***** Test Functions ***************
 ***********************************/
// PLL Clock Functions
void nrf_set_pll_mode( nrf_pll_mode_t pll_mode )
{
    if( pll_mode == NRF_PLL_LOCK )
        nrf_write_reg( RF_SETUP,
                       ( nrf_hal_read_reg( RF_SETUP ) | ( 1 << PLL_LOCK ) ) );
    else
        nrf_write_reg( RF_SETUP,
                       ( nrf_hal_read_reg( RF_SETUP ) & ~( 1 << PLL_LOCK ) ) );
}

nrf_pll_mode_t nrf_get_pll_mode( void )
{
    return ( nrf_pll_mode_t )( ( nrf_hal_read_reg( RF_SETUP ) &
                                 ( 1 << PLL_LOCK ) ) >> PLL_LOCK );
}

// LNA Gain Functions
void nrf_set_lna_gain( nrf_lna_mode_t lna_gain )
{
    if( lna_gain == NRF_LNA_HCURR )
        nrf_write_reg( RF_SETUP,
                       ( nrf_hal_read_reg( RF_SETUP ) | ( 1 << LNA_HCURR ) ) );
    else
        nrf_write_reg( RF_SETUP,
                       ( nrf_hal_read_reg( RF_SETUP ) & ~( 1 << LNA_HCURR ) ) );
}

// Get LNA gain
nrf_lna_mode_t nrf_get_lna_gain( void )
{
    return ( nrf_lna_mode_t ) ( ( nrf_hal_read_reg( RF_SETUP ) &
                                  ( 1 << LNA_HCURR ) ) >> LNA_HCURR );
}


/*************** END OF FUNCTIONS ***************************************************************************/
