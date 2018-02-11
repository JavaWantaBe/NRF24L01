#include "nrf24l01.h"
#include <string.h>


/*** Public function implementation ***/
NRF_RESULT_t NRF_Init( nrf24l01_dev_t* dev )
{
    HAL_Delay(100);
    dev->state = NRF_POWER_DOWN;
    // Setup GPIO and low level hardware
    NRF_SetupGPIO();

    // Power on
    NRF_PowerUp( dev, true );
    NRF_EnableCRC( dev, true );
    NRF_SetCRCWidth( dev, dev->crc_width );
    // Channel and retransmission
    NRF_SetRFChannel( dev, dev->rf_channel );
    NRF_SetDataRate( dev, dev->date_rate );
    NRF_SetRetransmittionCount( dev, dev->retransmit_count );
    NRF_SetRetransmittionDelay( dev, dev->retransmit_delay );
    // Setup pipes
    switch( dev->mode)
    {
        case NRF_PRIMARY_RX:
        {
            NRF_PIPE_t *pipe = &dev->address.P0;

            for( int i = 0; i < 5; i++)
            {
                if( pipe->enabled)
                {
                    NRF_EnablePipe(dev, pipe->pipe_num);

                    if( pipe->auto_ack )
                        NRF_EnableAutoAcknowledgement(dev, pipe->pipe_num);
                    if( pipe->dynamic_payload )
                        NRF_EnableDynamicPayload( dev, pipe->pipe_num );

                }
                pipe++;
            }
            break;
        }
        case NRF_PRIMARY_TX:
            break;
    }

    NRF_RXTXControl( dev, dev->mode );
    // Interrupts
    NRF_ClearInterrupts( dev );
    NRF_EnableRXDataReadyIRQ( dev, true );
    NRF_EnableTXDataSentIRQ( dev, true );
    NRF_EnableMaxRetransmitIRQ( dev, true );
    // Flush TX / RX Buffers
    NRF_FlushRX();
    NRF_FlushTX();

    return NRF_OK;
}

/***** SETTINGS *****/
NRF_RESULT_t NRF_SetDataRate( nrf24l01_dev_t* dev, NRF_DATA_RATE_t rate )
{
    uint8_t reg = 0;

    if( NRF_ReadRegister( NRF_RF_SETUP, &reg ) != NRF_OK )
        return NRF_ERROR;

    reg &= ~(0x28);

    switch( rate )
    {
        case NRF_DATA_RATE_250KBPS:
            reg |= (1 << 5);
            break;
        case NRF_DATA_RATE_1MBPS:
            break;
        case NRF_DATA_RATE_2MBPS:
            reg |= (1 << 3);
            break;
    }

    dev->date_rate = rate;

    return NRF_WriteRegister( NRF_RF_SETUP, &reg );
}

NRF_RESULT_t NRF_SetTXPower( nrf24l01_dev_t* dev, NRF_TX_PWR_t pwr )
{
    uint8_t reg = 0;

    if( NRF_ReadRegister( NRF_RF_SETUP, &reg ) != NRF_OK )
    {
        return NRF_ERROR;
    }

    reg &= ~(0x06);

    switch(pwr)
    {
        case NRF_TX_PWR_M18dBm:
            break;
        case NRF_TX_PWR_M12dBm:
            reg |= (1 << 1);
            break;
        case NRF_TX_PWR_M6dBm:
            reg |= (1 << 2);
            break;
        case NRF_TX_PWR_0dBm:
            reg |= (1 << 1) | (1 << 2);
            break;
    }

    dev->tx_power = pwr;

    return NRF_WriteRegister( NRF_RF_SETUP, &reg );
}

NRF_RESULT_t NRF_SetCCW( nrf24l01_dev_t* dev, bool activate )
{
    uint8_t reg = 0;

    if( NRF_ReadRegister( NRF_RF_SETUP, &reg ) != NRF_OK )
    {
        return NRF_ERROR;
    }

    reg &= ~(1 << 7);

    if( activate )
        reg |= (1 << 7);

    return NRF_WriteRegister( NRF_RF_SETUP, &reg );
}

NRF_RESULT_t NRF_SetRFChannel( nrf24l01_dev_t* dev, uint8_t ch )
{
    uint8_t reg = 0;
    ch &= 0x7F;

    if( NRF_ReadRegister( NRF_RF_CH, &reg ) != NRF_OK )
    {
        return NRF_ERROR;
    }

    reg |= ch;  // setting channel
    dev->rf_channel = ch;

    return NRF_WriteRegister( NRF_RF_CH, &reg );
}

NRF_RESULT_t NRF_SetRetransmittionCount( nrf24l01_dev_t* dev, uint8_t count )
{
    uint8_t reg = 0;
    count &= 0x0F;

    if( NRF_ReadRegister( NRF_SETUP_RETR, &reg ) != NRF_OK )
    {
        return NRF_ERROR;
    }

    reg &= 0xF0;   // clearing bits 0,1,2,3
    reg |= count;  // setting count

    dev->retransmit_count = count;

    return NRF_WriteRegister( NRF_SETUP_RETR, &reg );
}

NRF_RESULT_t NRF_SetRetransmittionDelay( nrf24l01_dev_t* dev, NRF_RETRANS_DELAY_t delay )
{
    uint8_t reg = 0;
    delay &= 0x0F;

    if( NRF_ReadRegister( NRF_SETUP_RETR, &reg ) != NRF_OK )
    {
        return NRF_ERROR;
    }

    reg &= 0x0F;        // clearing bits 4,5,6,7
    reg |= delay << 4;  // setting delay
    dev->retransmit_delay = delay;

    return NRF_WriteRegister( NRF_SETUP_RETR, &reg );
}

NRF_RESULT_t NRF_SetAddressWidth( nrf24l01_dev_t* dev, NRF_ADDR_WIDTH_t width )
{
    uint8_t reg = 0;
    width &= 0x03;

    if( NRF_ReadRegister( NRF_SETUP_AW, &reg ) != NRF_OK )
    {
        return NRF_ERROR;
    }

    reg &= ~0x03;  // clearing bits 0,1
    reg |= width;  // setting delay
    dev->address.addr_width = width;

    return NRF_WriteRegister( NRF_SETUP_AW, &reg );
}

NRF_RESULT_t NRF_EnableCRC( nrf24l01_dev_t* dev, bool activate )
{
    uint8_t reg = 0;

    if( NRF_ReadRegister( NRF_CONFIG, &reg ) != NRF_OK )
    {
        return NRF_ERROR;
    }

    reg &= ~(1 << 3);

    if( activate )
    {
        reg |= (1 << 3);
    }

    return NRF_WriteRegister( NRF_CONFIG, &reg );
}

NRF_RESULT_t NRF_SetCRCWidth( nrf24l01_dev_t* dev, NRF_CRC_WIDTH_t width )
{
    uint8_t reg = 0;

    if( NRF_ReadRegister( NRF_CONFIG, &reg ) != NRF_OK )
    {
        return NRF_ERROR;
    }

    reg &= ~(1 << 2);

    if( width == NRF_CRC_WIDTH_2B )
    {
        reg |= (1 << 2);
    }

    dev->crc_width = width;

    return NRF_WriteRegister( NRF_CONFIG, &reg );
}

NRF_RESULT_t NRF_PowerUp( nrf24l01_dev_t* dev, bool powerUp )
{
    uint8_t reg = 0;
    NRF_RESULT_t result;

    if( NRF_ReadRegister( NRF_CONFIG, &reg ) != NRF_OK )
    {
        return NRF_ERROR;
    }

    if( powerUp )
    {
        reg |= 1 << 1;
        result = NRF_WriteRegister( NRF_CONFIG, &reg );
        HAL_Delay(2);
        dev->state = NRF_STANDBY_I;
    }
    else
    {
        reg &= ~(1 << 1);
        result = NRF_WriteRegister( NRF_CONFIG, &reg );
        dev->state = NRF_POWER_DOWN;
    }

    return result;
}

NRF_RESULT_t NRF_EnableRXDataReadyIRQ( nrf24l01_dev_t* dev, bool activate )
{
    uint8_t reg = 0;

    if( NRF_ReadRegister( NRF_CONFIG, &reg ) != NRF_OK )
    {
        return NRF_ERROR;
    }

    reg &= ~(1 << 6);

    if( !activate )
    {
        reg |= (1 << 6);
    }

    return NRF_WriteRegister( NRF_CONFIG, &reg );
}

NRF_RESULT_t NRF_EnableTXDataSentIRQ( nrf24l01_dev_t* dev, bool activate )
{
    uint8_t reg = 0;

    if( NRF_ReadRegister( NRF_CONFIG, &reg ) != NRF_OK )
    {
        return NRF_ERROR;
    }

    reg &= ~(1 << 5);

    if( !activate )
    {
        reg |= 1 << 5;
    }

    return NRF_WriteRegister( NRF_CONFIG, &reg );
}

NRF_RESULT_t NRF_EnableMaxRetransmitIRQ( nrf24l01_dev_t* dev, bool activate )
{
    uint8_t reg = 0;

    if( NRF_ReadRegister( NRF_CONFIG, &reg ) != NRF_OK )
    {
        return NRF_ERROR;
    }

    reg &= ~(1 << 4);

    if( !activate )
    {
        reg |= 1 << 4;
    }

    return NRF_WriteRegister( NRF_CONFIG, &reg );
}

NRF_RESULT_t NRF_ClearInterrupts( nrf24l01_dev_t* dev )
{
    uint8_t reg = 0;

    if( NRF_ReadRegister( NRF_STATUS, &reg ) != NRF_OK )
    {
        return NRF_ERROR;
    }

    reg |= (7 << 4);  // setting bits 4,5,6

    return NRF_WriteRegister( NRF_STATUS, &reg );
}

NRF_RESULT_t NRF_EnableAutoAcknowledgement( nrf24l01_dev_t* dev, uint8_t pipe )
{
    uint8_t reg = 0;
    pipe &= 0x3F;

    if( NRF_ReadRegister( NRF_EN_AA, &reg ) != NRF_OK )
    {
        return NRF_ERROR;
    }

    reg |= (1 << pipe);

    return NRF_WriteRegister( NRF_EN_AA, &reg );
}

NRF_RESULT_t NRF_EnableDynamicPayload( nrf24l01_dev_t* dev, uint8_t pipe )
{
    uint8_t reg = 0;

    pipe &= 0x3F;

    if( NRF_ReadRegister( NRF_FEATURE, &reg ) != NRF_OK )
        return NRF_ERROR;

    if( !reg & 0x04 )
    {
        reg |= ( 1 << 2 );
        NRF_WriteRegister( NRF_FEATURE, &reg );
    }

    NRF_EnablePipe( dev, pipe );

    if( NRF_ReadRegister( NRF_DYNPD, &reg) != NRF_OK )
    {
        return NRF_ERROR;
    }

    reg |= (1 << pipe );

    return NRF_WriteRegister( NRF_DYNPD, &reg );
}




/***** PIPE Settings *****/
NRF_RESULT_t NRF_EnablePipe( nrf24l01_dev_t* dev, uint8_t pipe )
{
    uint8_t reg = 0;

    if( pipe > 5 )
        return NRF_ERROR;

    if( NRF_ReadRegister( NRF_EN_RXADDR, &reg ) != NRF_OK )
        return NRF_ERROR;

    reg |= ( 1 << pipe );
    if( NRF_WriteRegister( NRF_EN_RXADDR, &reg ) != NRF_OK )
        return NRF_ERROR;
}

NRF_RESULT_t NRF_RXTXControl( nrf24l01_dev_t* dev, NRF_PRIMARY_MODE_t mode )
{
    uint8_t reg = 0;

    if( NRF_ReadRegister( NRF_CONFIG, &reg ) != NRF_OK )
    {
        return NRF_ERROR;
    }

    if( mode == NRF_PRIMARY_RX )
    {
        reg |= (1 << 0);
        dev->state = NRF_STATE_RX;
        NRF24L01_CE_HIGH();
    }
    else
    {
        reg &= ~(1 << 0);
        dev->state = NRF_STATE_TX;
    }

    return NRF_WriteRegister( NRF_CONFIG, &reg );
}

NRF_RESULT_t NRF_SetTXAddress( nrf24l01_dev_t* dev, uint8_t* address )
{
    uint8_t rx[5];

    memcpy( dev->address.TX.addr, address, 5 );

    return NRF_SendCommand(  NRF_CMD_W_REGISTER | NRF_TX_ADDR, address, rx, 5 );
}


/****** SEND / RECEIVE ******/
NRF_RESULT_t NRF_SendPacket( nrf24l01_dev_t* dev, uint8_t* data )
{
    dev->BUSY_FLAG = 1;

    NRF24L01_CE_LOW();
    NRF_RXTXControl( dev, NRF_STATE_TX );
//    NRF_WriteTXPayload( data, dev->payload_len );
    NRF24L01_CE_HIGH();

    while( dev->BUSY_FLAG == 1 ); // wait for end of transmission

    return NRF_OK;
}

NRF_RESULT_t NRF_ReceivePacket( nrf24l01_dev_t* dev, uint8_t* data )
{
    dev->BUSY_FLAG = 1;

    NRF24L01_CE_LOW();
    dev->state = NRF_STANDBY_I;
    NRF_RXTXControl( dev, NRF_STATE_RX );
    NRF24L01_CE_HIGH();
    dev->state = NRF_STATE_RX;

    // TODO get status for pipe number [3:1]
    // TODO get data size

    while( dev->BUSY_FLAG == 1 );  // wait for reception


    return NRF_OK;
}

NRF_RESULT_t NRF_PushPacket( nrf24l01_dev_t* dev, uint8_t* data )
{
    if( dev->BUSY_FLAG == 1 )
    {
        NRF_FlushTX();
    }
    else
    {
        dev->BUSY_FLAG = 1;
    }

    NRF24L01_CE_LOW();
    NRF_RXTXControl( dev, NRF_STATE_TX );
//    NRF_WriteTXPayload( data, dev->payload_len );
    NRF24L01_CE_HIGH();

    return NRF_OK;
}

NRF_RESULT_t NRF_PullPacket( nrf24l01_dev_t* dev, uint8_t* data )
{

    return NRF_OK;
}


void NRF_IRQ_Handler( nrf24l01_dev_t* dev )
{
    uint8_t status = 0;

    if( NRF_ReadRegister( NRF_STATUS, &status ) != NRF_OK )
    {
        return;
    }

    if( ( status & ( 1 << 6 ) ) )
    {
        // RX FIFO Interrupt
        uint8_t fifo_status = 0;

        NRF24L01_CE_LOW();
        NRF_WriteRegister( NRF_STATUS, &status );
        NRF_ReadRegister( NRF_FIFO_STATUS, &fifo_status );
        if( dev->BUSY_FLAG == 1 && ( fifo_status & 1 ) == 0 )
        {
//            NRF_ReadRXPayload( dev->rx_buffer, dev->payload_len );
            status |= 1 << 6;
            NRF_WriteRegister( NRF_STATUS, &status );
            //NRF_FlushRX(dev);
            dev->BUSY_FLAG = 0;
        }
        NRF24L01_CE_HIGH();
    }

    if( ( status & ( 1 << 5 ) ) )
    {  // TX Data Sent Interrupt
        status |= 1 << 5;  // clear the interrupt flag
        NRF24L01_CE_LOW();
        NRF_RXTXControl( dev, NRF_STATE_RX );
        dev->state = NRF_STATE_RX;
        NRF24L01_CE_HIGH();
        NRF_WriteRegister( NRF_STATUS, &status );
        dev->BUSY_FLAG = 0;
    }

    if( ( status & ( 1 << 4 ) ) )
    {  // MaxRetransmits reached
        status |= 1 << 4;

        NRF_FlushTX( );
        NRF_PowerUp( dev, 0 );  // power down
        NRF_PowerUp( dev, 1 );  // power up

        NRF24L01_CE_LOW();
        NRF_RXTXControl( dev, NRF_STATE_RX );
        dev->state = NRF_STATE_RX;
        NRF24L01_CE_HIGH();

        NRF_WriteRegister( NRF_STATUS, &status );
        dev->BUSY_FLAG = 0;
    }
}

__weak void NRF_PacketRecieved_Callback( nrf24l01_dev_t *dev )
{

}

__weak void NRF_PacketTransmitted_Callback( nrf24l01_dev_t *dev )
{

}

__weak void NRF_PacketMaxRetries_Callback( nrf24l01_dev_t *dev )
{

}

/************************************
 ***** Test Functions ***************
 ***********************************/
// PLL Clock Functions
void NRF_SetPLLMode( nrf24l01_dev_t* dev, uint8_t pll_mode )
{
//    if( pll_mode == NRF_PLL_LOCK )
//        nrf_write_reg( RF_SETUP,
//                       ( nrf_hal_read_reg( RF_SETUP ) | ( 1 << PLL_LOCK ) ) );
//    else
//        nrf_write_reg( RF_SETUP,
//                       ( nrf_hal_read_reg( RF_SETUP ) & ~( 1 << PLL_LOCK ) ) );
}

uint8_t NRF_GetPLLMode()
{
//    return ( nrf_pll_mode_t )( ( nrf_hal_read_reg( RF_SETUP ) &
//                                 ( 1 << PLL_LOCK ) ) >> PLL_LOCK );
    return 1;
}

// LNA Gain Functions
void NRF_SetLNAGain( nrf24l01_dev_t* dev, uint8_t lna_gain )
{
//    if( lna_gain == NRF_LNA_HCURR )
//        nrf_write_reg( RF_SETUP,
//                       ( nrf_hal_read_reg( RF_SETUP ) | ( 1 << LNA_HCURR ) ) );
//    else
//        nrf_write_reg( RF_SETUP,
//                       ( nrf_hal_read_reg( RF_SETUP ) & ~( 1 << LNA_HCURR ) ) );
}

// Get LNA gain
uint8_t NRF_GetLNAGain( void )
{
//    return ( nrf_lna_mode_t ) ( ( nrf_hal_read_reg( RF_SETUP ) &
//                                  ( 1 << LNA_HCURR ) ) >> LNA_HCURR );
    return 1;
}


