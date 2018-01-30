/*******************************************************************************
* Title                 :   Nordic nRF24L01+ API
* Filename              :   nrf.c
* Author                :   RBL
* Origin Date           :   11/01/2016
* Notes                 :   None
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  11/01/16           .1         RBL        Module Created.
*
*******************************************************************************/
/**
 * @file nrf.c
 * @brief This module contains the
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "nrf24l01.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define NRF_ERROR  -1
#define NRF_SUCCESS 0

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static volatile bool success_tx;
static volatile uint32_t timeout;
static uint8_t tx_address[5];
static bool ack_enabled;

/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/
nrf_status_t nrf24l01_init( nrf_radio_t radio )
{
	bool ack = ( radio.mode == NRF_SHOCK_BURST ) ? false : true;

	nrf_hw_init(radio.mode, radio.primary);
	nrf_close_pipe( NRF_ALL );
    nrf_clear_irq_flag( NRF_MAX_RT );     /**< Max retries interrupt */
    nrf_clear_irq_flag( NRF_TX_DS );      /**< TX data sent interrupt */
    nrf_clear_irq_flag( NRF_RX_DR );
    nrf_flush_rx();
    nrf_flush_tx(); /* Flush FIFOs */
    
    /* Set device's addresses */
	nrf_set_address( NRF_PIPE0, address->p0 );
	/* Open pipe0, without/autoack */
	nrf_open_pipe( NRF_PIPE0, false );

	if( address->p1[0] > 0 )
	{
		nrf_set_address( NRF_PIPE1, address->p1 );
		nrf_open_pipe( NRF_PIPE1, ack );
	}

	if( address->p2[0] > 0 )
	{
		nrf_set_address( NRF_PIPE2, address->p2 );
		nrf_open_pipe( NRF_PIPE2, ack );
	}

	if( address->p3[0] > 0 )
	{
		nrf_set_address( NRF_PIPE3, address->p3 );
		nrf_open_pipe( NRF_PIPE3, ack );
	}

	if( address->p4[0] > 0 )
	{
		nrf_set_address( NRF_PIPE4, address->p4 );
		nrf_open_pipe( NRF_PIPE4, ack );
	}

	if( address->p5[0] > 0 )
	{
		nrf_set_address( NRF_PIPE5, address->p5 );
		nrf_open_pipe( NRF_PIPE5, ack );
	}

	if( radio.mode == NRF_ENHANCED || radio.mode == NRF_ENHANCED_BIDIR )
		nrf_set_auto_retr( NRF_RETRANSMITS, NRF_RETRANS_DELAY );
	else
		nrf_set_auto_retr( 0, NRF_RETRANS_DELAY );
	if( radio.mode == NRF_ENHANCED_BIDIR )
	{
		nrf_enable_ack_pl();        // Enable ack payload
		nrf_enable_dynamic_pl();    // Enables dynamic payload
		nrf_setup_dyn_pl( 0x3f );   // Sets up dynamic payload on all data pipes.
	}

    nrf_set_power_mode( NRF_PWR_UP );
    nrf_delay( NRF_POWER_UP_DELAY );   // Wait for the radio to
	if( radio.primary == NRF_PRX )
		nrf_start_active_rx();
	else
		nrf_enter_standby_mode();

}





uint8_t nrf_send_data( uint8_t *address, uint8_t *data_out, uint8_t count )
{
    uint32_t time_out;
    
    count = MIN( count, NRF_PAYLOAD_LENGTH );
    success_tx = false;

    if( address != NULL && memcmp( address, tx_address, 5 ) )
        nrf_set_address( NRF_TX, address );
    else
        nrf_write_tx_pload( data_out, count );
    
    time_out = timeout + 13;
    
    if( ack_enabled )
    {
		while( !success_tx )
		{
		   if( timeout > time_out )
		   {
			   count = 0;
			   break;
		   }
		}
    }
    return count;
}


uint8_t nrf_recieve_data( nrf_address_t *address, uint8_t *data_in )
{
    *address = nrf_get_rx_data_source();
    nrf_read_rx_pload( data_in );

    return nrf_get_rx_pload_width( *address );
}

void nrf_acknowledged()
{
     success_tx = true;
}

bool nrf_is_interrupted( nrf_irq_source_t source )
{
    uint8_t flags = nrf_get_irq_flags();

    if( flags & ( uint8_t )( 1 << source ) )
    {
        nrf_clear_irq_flag( source );
        
        if( source == NRF_MAX_RT )
            nrf_flush_tx();
        return true;
    }
    else
        return false;
}

void nrf_timer_tick()
{
    timeout++;
}



/*************** END OF FUNCTIONS ***************************************************************************/
