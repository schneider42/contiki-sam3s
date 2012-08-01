/*   Copyright (c) 2008, Swedish Institute of Computer Science
 *  All rights reserved. 
 *
 *  Additional fixes for AVR contributed by:
 *
 *	Günter Hildebrandt guenter.hildebrandt@esk.fraunhofer.de
 *	Colin O'Flynn coflynn@newae.com
 *	Eric Gnoske egnoske@gmail.com
 *	Blake Leverett bleverett@gmail.com
 *	Mike Vidales mavida404@gmail.com
 *	Kevin Brown kbrown3@uccs.edu
 *	Nate Bohlmann nate@elfwerks.com
 *
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of the copyright holders nor the names of
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *    \addtogroup hal
 *    @{
 */

/**
 *  \file
 *  \brief This file contains low-level radio driver code.
 *
 *   $Id: hal.h,v 1.1 2009/07/08 16:17:07 dak664 Exp $
*/

#ifndef HAL_SAM3S_H
#define HAL_SAM3S_H
/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include <stdbool.h>
#include "contiki-conf.h"
/*============================ MACROS ========================================*/

/** \name This is the list of pin configurations needed for a given platform.
 * \brief Change these values to port to other platforms.
 * \{
 */
/* Define all possible revisions here */
// RAVEN_D : Raven kit with LCD display
// RAVENUSB_C : used for USB key or Raven card 
// RCB_B : RZ200 kit from Atmel based on 1281V
// ZIGBIT : Zigbit module from Meshnetics
#define RAVEN_D	    0
#define RAVENUSB_C  1
#define RCB_B	    	2
#define ZIGBIT			3
#define HEXABUS_SOCKET	4
#define HEXABUS_USB		5

#if HARWARE_REVISION == ZIGBIT
/* 1281V Zigbit */
#   define SSPORT     B
#   define SSPIN      (0x00)
#   define SPIPORT    B
#   define MOSIPIN    (0x02)
#   define MISOPIN    (0x03)
#   define SCKPIN     (0x01)
#   define RSTPORT    A
#   define RSTPIN     (0x07)
#   define IRQPORT    E
#   define IRQPIN     (0x05)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x04)
#   define TXCWPORT   B
#   define TXCWPIN    (0x07)
#   define USART      1
#   define USARTVECT  USART1_RX_vect
#else

//#error "Platform undefined in hal.h"

#endif

/** \} */

/**
 * \name Pin macros
 * \brief These macros convert the platform-specific pin defines into names and functions
 *       that the source code can directly use.
 * \{
 */
/** \} */


/**< MACRO for pulling SS high. */
#define HAL_SS_HIGH() PIO_Set(&pinSS)
/**< MACRO for pulling SS low. */
#define HAL_SS_LOW() PIO_Clear(&pinSS)

uint8_t hal_get_slptr(void);
void hal_set_slptr_low(void);
void hal_set_slptr_high(void);
void hal_set_rst_high(void);
void hal_set_rst_low(void);
void hal_delay_us(int us);
/** This macro will protect the following code from interrupts.*/
#define ENTER_CRITICAL_REGION( ) { __disable_irq();

/** This macro must always be used in conjunction with AVR_ENTER_CRITICAL_REGION
    so that interrupts are enabled again.*/
#define LEAVE_CRITICAL_REGION( ) __enable_irq();}


/** \brief  Enable the interrupt from the radio transceiver.
 */
//#define hal_enable_trx_interrupt( ) HAL_ENABLE_RADIO_INTERRUPT( )

/** \brief  Disable the interrupt from the radio transceiver.
 *
 *  \retval 0 if the pin is low, 1 if the pin is high.
 */
//#define hal_disable_trx_interrupt( ) HAL_DISABLE_RADIO_INTERRUPT( )
/*============================ TYPDEFS =======================================*/
/*============================ PROTOTYPES ====================================*/
/*============================ MACROS ========================================*/
/** \name Macros for radio operation.
 * \{ 
 */
#define HAL_BAT_LOW_MASK       ( 0x80 ) /**< Mask for the BAT_LOW interrupt. */
#define HAL_TRX_UR_MASK        ( 0x40 ) /**< Mask for the TRX_UR interrupt. */
#define HAL_AMI_MASK		   ( 0x20 ) /**< Mask for the AMI interrupt. */
#define HAL_CCA_ED_DONE		   ( 0x10 ) /**< Mask for the CCA_ED_DONE interrupt. */
#define HAL_TRX_END_MASK       ( 0x08 ) /**< Mask for the TRX_END interrupt. */
#define HAL_RX_START_MASK      ( 0x04 ) /**< Mask for the RX_START interrupt. */
#define HAL_PLL_UNLOCK_MASK    ( 0x02 ) /**< Mask for the PLL_UNLOCK interrupt. */
#define HAL_PLL_LOCK_MASK      ( 0x01 ) /**< Mask for the PLL_LOCK interrupt. */

#define HAL_MIN_FRAME_LENGTH   ( 0x03 ) /**< A frame should be at least 3 bytes. */
#define HAL_MAX_FRAME_LENGTH   ( 0x7F ) /**< A frame should no more than 127 bytes. */
/** \} */
/*============================ TYPDEFS =======================================*/
/** \struct hal_rx_frame_t
 *  \brief  This struct defines the rx data container.
 *
 *  \see hal_frame_read
 */
typedef struct{
    uint8_t length;                       /**< Length of frame. */
    uint8_t data[ HAL_MAX_FRAME_LENGTH ]; /**< Actual frame data. */
    uint8_t lqi;                          /**< LQI value for received frame. */
    bool crc;                             /**< Flag - did CRC pass for received frame? */
    uint8_t ed;							/**< Energy Detection value for received frame. */
} hal_rx_frame_t;


/*============================ PROTOTYPES ====================================*/
void hal_init( void );

uint8_t hal_register_read( uint8_t address );
void hal_register_write( uint8_t address, uint8_t value );
uint8_t hal_subregister_read( uint8_t address, uint8_t mask, uint8_t position );
void hal_subregister_write( uint8_t address, uint8_t mask, uint8_t position,
                            uint8_t value );
void hal_frame_read(hal_rx_frame_t *rx_frame);
void hal_frame_write( uint8_t *write_buffer, uint8_t length );
void hal_sram_read( uint8_t address, uint8_t length, uint8_t *data );
void hal_sram_write( uint8_t address, uint8_t length, uint8_t *data );

#endif
/** @} */
/*EOF*/
