/****************************************************************************
* Title                 :   ADC Application
* Filename              :   adc_app.h
* Author                :   JWB
* Origin Date           :   06/07/2012
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials   Description
*  08/17/13    XXXXXXXXXXX         JWB      Interface Created.
*
*****************************************************************************/
/** @file file_here.h
 *  @brief What is does
 *
 *  @date 25 Aug 2015
 *  @author Richard Lowe
 *  @copyright GNU Public License
 *
 *  @version .1 - Initial testing and verification
 *
 *  @note Test configuration:
 *   MCU:             STM32F107VC
 *   Dev.Board:       EasyMx Pro v7
 *   Oscillator:      72 Mhz internal
 *   Ext. Modules:    GPS Click
 *   SW:              ARM 4.5.2
 *
 */

#ifndef NRF_DEFS_H_
#define NRF_DEFS_H_

/******************************************************************************
* Includes
*******************************************************************************/


/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
/** @name - Instruction Set - */
#define NRF_WRITE_REG 0x20  /**< Register write command */
#define RD_RX_PLOAD_W 0x60  /**< Read RX payload command */
#define RD_RX_PLOAD   0x61  /**< Read RX payload command */
#define WR_TX_PLOAD   0xA0  /**< Write TX payload command */
#define WR_ACK_PLOAD  0xA8  /**< Write ACK payload command */
#define WR_NAC_TX_PLOAD 0xB0  /**< Write ACK payload command */
#define FLUSH_TX      0xE1  /**< Flush TX register command */
#define FLUSH_RX      0xE2  /**< Flush RX register command */
#define REUSE_TX_PL   0xE3  /**< Reuse TX payload command */
//#define LOCK_UNLOCK   0x50  /**< Lock/unlock exclusive features */

#define NOP           0xFF  /**< No Operation command, used for reading status register */

/** Register Memory Map - */
#define CONFIG        0x00  /**< nRF24L01 config register */
#define EN_AA         0x01  /**< nRF24L01 enable Auto-Acknowledge register */
#define EN_RXADDR     0x02  /**< nRF24L01 enable RX addresses register */
#define SETUP_AW      0x03  /**< nRF24L01 setup of address width register */
#define SETUP_RETR    0x04  /**< nRF24L01 setup of automatic retransmission register */
#define RF_CH         0x05  /**< nRF24L01 RF channel register */
#define RF_SETUP      0x06  /**< nRF24L01 RF setup register */
#define STATUS        0x07  /**< nRF24L01 status register */
#define OBSERVE_TX    0x08  /**< nRF24L01 transmit observe register */
#define CD            0x09  /**< nRF24L01 carrier detect register */
#define RX_ADDR_P0    0x0A  /**< nRF24L01 receive address data pipe0 */
#define RX_ADDR_P1    0x0B  /**< nRF24L01 receive address data pipe1 */
#define RX_ADDR_P2    0x0C  /**< nRF24L01 receive address data pipe2 */
#define RX_ADDR_P3    0x0D  /**< nRF24L01 receive address data pipe3 */
#define RX_ADDR_P4    0x0E  /**< nRF24L01 receive address data pipe4 */
#define RX_ADDR_P5    0x0F  /**< nRF24L01 receive address data pipe5 */
#define TX_ADDR       0x10  /**< nRF24L01 transmit address */
#define RX_PW_P0      0x11  /**< nRF24L01 \# of bytes in rx payload for pipe0 */
#define RX_PW_P1      0x12  /**< nRF24L01 \# of bytes in rx payload for pipe1 */
#define RX_PW_P2      0x13  /**< nRF24L01 \# of bytes in rx payload for pipe2 */
#define RX_PW_P3      0x14  /**< nRF24L01 \# of bytes in rx payload for pipe3 */
#define RX_PW_P4      0x15  /**< nRF24L01 \# of bytes in rx payload for pipe4 */
#define RX_PW_P5      0x16  /**< nRF24L01 \# of bytes in rx payload for pipe5 */
#define FIFO_STATUS   0x17  /**< nRF24L01 FIFO status register */
#define DYNPD         0x1C  /**< nRF24L01 Dynamic payload setup */
#define FEATURE       0x1D  /**< nRF24L01 Exclusive feature setup */

#define MASK_RX_DR    6     /**< CONFIG register bit 6 */
#define MASK_TX_DS    5     /**< CONFIG register bit 5 */
#define MASK_MAX_RT   4     /**< CONFIG register bit 4 */
#define EN_CRC        3     /**< CONFIG register bit 3 */
#define CRCO          2     /**< CONFIG register bit 2 */
#define PWR_UP        1     /**< CONFIG register bit 1 */
#define PRIM_RX       0     /**< CONFIG register bit 0 */

/** @name RF_SETUP register bit definitions */
//@{
#define PLL_LOCK      4     /**< RF_SETUP register bit 4 */
#define RF_DR         3     /**< RF_SETUP register bit 3 */
#define RF_PWR1       2     /**< RF_SETUP register bit 2 */
#define RF_PWR0       1     /**< RF_SETUP register bit 1 */
#define LNA_HCURR     0     /**< RF_SETUP register bit 0 */
//@}

/* STATUS 0x07 */
/** @name STATUS register bit definitions */
//@{
#define RX_DR         6     /**< STATUS register bit 6 */
#define TX_DS         5     /**< STATUS register bit 5 */
#define MAX_RT        4     /**< STATUS register bit 4 */
#define TX_FULL       0     /**< STATUS register bit 0 */
//@}

/* FIFO_STATUS 0x17 */
/** @name FIFO_STATUS register bit definitions */
//@{
#define TX_REUSE      6     /**< FIFO_STATUS register bit 6 */
#define TX_FIFO_FULL  5     /**< FIFO_STATUS register bit 5 */
#define TX_EMPTY      4     /**< FIFO_STATUS register bit 4 */
#define RX_FULL       1     /**< FIFO_STATUS register bit 1 */
#define RX_EMPTY      0     /**< FIFO_STATUS register bit 0 */
//@}

#define NRF_BIT_0 0x01 /**< The value of bit 0 */
#define NRF_BIT_1 0x02 /**< The value of bit 1 */
#define NRF_BIT_2 0x04 /**< The value of bit 2 */
#define NRF_BIT_3 0x08 /**< The value of bit 3 */
#define NRF_BIT_4 0x10 /**< The value of bit 4 */
#define NRF_BIT_5 0x20 /**< The value of bit 5 */
#define NRF_BIT_6 0x40 /**< The value of bit 6 */
#define NRF_BIT_7 0x80 /**< The value of bit 7 */


/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/
/** Swaps the upper byte with the lower byte in a 16 bit variable */
#define NRF_SWAP(x) ((((x)&0xFF)<<8)|(((x)>>8)&0xFF))

#define NRF_SET_BIT(pos) ((uint8_t) (1<<( (uint8_t) (pos) )))

/** The upper 8 bits of a 16 bit value */
#define NRF_MSB(a) ((a & 0xFF00) >> 8)
/** The lower 8 bits (of a 16 bit value) */
#define NRF_LSB(a) ((a & 0xFF))

/** Leaves the minimum of the two arguments */
#define NRF_MIN(a, b) ((a) < (b) ? (a) : (b))
/** Leaves the maximum of the two arguments */
#define NRF_MAX(a, b) ((a) < (b) ? (b) : (a))

#define NRF_FLUSH_TX() nrf_command(FLUSH_TX)
#define NRF_FLUSH_RX() nrf_command(FLUSH_RX)
#define NRF_RESUSE_PL() nrf_command(REUSE_TX_PL)
#define NRF_NOP() nrf_command(NOP)


/******************************************************************************
* Typedefs
*******************************************************************************/
typedef enum
{
    NRF_POWER_DOWN,
    NRF_STANDBY_I,
	NRF_STANDBY_II,
    NRF_TX_MODE,
    NRF_RX_MODE,
	NRF_ERROR
} nrf_status_t;

typedef enum
{
	NRF_SHOCK_BURST,
	NRF_ENHANCED,
	NRF_ENHANCED_BIDIR
} nrf_radio_mode_t;


/**
 * @brief An enum describing the radio's irq sources.
 *
 */
typedef enum
{
    NRF_MAX_RT = 4,     /**< Max retries interrupt */
    NRF_TX_DS,          /**< TX data sent interrupt */
    NRF_RX_DR           /**< RX data received interrupt */
} nrf_irq_source_t;

/* Operation mode definitions */

/**
 * @brief An enum describing the radio's power mode.
 *
 */
typedef enum
{
    NRF_PTX,            /**< Primary TX operation */
    NRF_PRX             /**< Primary RX operation */
} nrf_operation_mode_t;


/**
 * @brief An enum describing the radio's output power mode's.
 *
 */
typedef enum
{
    NRF_18DBM = 0,          /**< Output power set to -18dBm */
    NRF_12DBM = 1,          /**< Output power set to -12dBm */
    NRF_6DBM  = 2,          /**< Output power set to -6dBm  */
    NRF_0DBM  = 4           /**< Output power set to 0dBm   */
} nrf_output_power_t;

/**
 * @brief An enum describing the radio's on-air datarate.
 *
 */
typedef enum
{
    NRF_1MBPS = 0,          /**< Datarate set to 1 Mbps  */
    NRF_2MBPS = 1,           /**< Datarate set to 2 Mbps  */
	NRF_250k  = 2
} nrf_datarate_t;

/**
 * @brief An enum describing the radio's PLL mode.
 *
 */
typedef enum
{
    NRF_PLL_UNLOCK,     /**< PLL unlocked, normal operation  */
    NRF_PLL_LOCK        /**< PLL locked, test mode  */
} nrf_pll_mode_t;

/**
 * @brief An enum describing the radio's LNA mode.
 *
 */
typedef enum
{
    NRF_LNA_LCURR,      /**< LNA set to low current mode */
    NRF_LNA_HCURR       /**< LNA set to high current mode */
} nrf_lna_mode_t;

/**
 * @brief An enum describing the radio's CRC mode.
 *
 */
typedef enum
{
    NRF_CRC_8BIT = 0,   /**< CRC check set to 8-bit */
    NRF_CRC_16BIT = 1       /**< CRC check set to 16-bit */
} nrf_crc_mode_t;

/**
 * @brief An enum describing the read/write payload command.
 *
 */
typedef enum
{
    NRF_TX_PLOAD = 7,   /**< TX payload definition */
    NRF_RX_PLOAD,       /**< RX payload definition */
    NRF_ACK_PLOAD
} nrf_pload_command_t;

/**
 * @brief Structure containing the radio's address map.
 * Pipe0 contains 5 unique address bytes,
 * while pipe[1..5] share the 4 MSB bytes, set in pipe1.
 * <p><b> - Remember that the LSB byte for all pipes have to be unique! -</b>
 */
// nRF24L01 Address struct
typedef struct
{
    uint8_t p0[5];     /**< Pipe0 address, 5 bytes */
    uint8_t p1[5];     /**< Pipe1 address, 5 bytes, 4 MSB bytes shared for pipe1 to pipe5 */
    uint8_t p2[1];     /**< Pipe2 address, 1 byte */
    uint8_t p3[1];     /**< Pipe3 address, 1 byte */
    uint8_t p4[1];     /**< Pipe3 address, 1 byte */
    uint8_t p5[1];     /**< Pipe3 address, 1 byte */
    uint8_t tx[5];     /**< TX address, 5 byte */
} nrf_addr_map_t;


/**
 * @brief An enum describing the nRF24L01 pipe addresses and TX address.
 *
 */
typedef enum
{
    NRF_PIPE0,              /**< Select pipe0 */
    NRF_PIPE1,              /**< Select pipe1 */
    NRF_PIPE2,              /**< Select pipe2 */
    NRF_PIPE3,              /**< Select pipe3 */
    NRF_PIPE4,              /**< Select pipe4 */
    NRF_PIPE5,              /**< Select pipe5 */
    NRF_TX,                 /**< Refer to TX address*/
    NRF_ALL = 0xFF          /**< Close or open all pipes*/
                  /**< @see nrf_set_address @see nrf_get_address
                   @see nrf_open_pipe  @see nrf_close_pipe */
} nrf_address_t;

/**
 * @brief An enum describing the radio's address width.
 *
 */
typedef enum
{
    NRF_AW_3BYTES = 3,      /**< Set address width to 3 bytes */
    NRF_AW_4BYTES,          /**< Set address width to 4 bytes */
    NRF_AW_5BYTES           /**< Set address width to 5 bytes */
} nrf_address_width_t;

/**
 * @brief Enumerates the different states the radio may
 * be in.
 */
typedef enum
{
  NRF_STANDBY,             /**< Radio is idle */
  NRF_POWERDOWN,
  NRF_MAX_RETRIES,      /**< Maximum number of retries have occured */
  NRF_TX_DATA_SENT,     /**< Data is sent */
  NRF_RX_DATA_RECEIVED, /**< Data recieved */
  NRF_TX_ACK_PAYLOAD,   /**< Ack payload recieved */
  NRF_BUSY              /**< Radio is busy */
} nrf_state_t;


/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif



#ifdef __cplusplus
} // extern "C"
#endif

#endif // NRF_REG_H__
/*** End of File **************************************************************/
