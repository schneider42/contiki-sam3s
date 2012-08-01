/*   Copyright (c) 2009, Swedish Institute of Computer Science
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
 *	David Kopf dak664@embarqmail.com
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
 *
 * 
*/

/**
 *   \addtogroup wireless
 *  @{
*/

/**
 *   \defgroup hal RF212 hardware level drivers
 *   @{
 */

/**
 *  \file
 *  This file contains low-level radio driver code.
 *  This version is optimized for use with the "barebones" RF212bb driver,
 *  which communicates directly with the contiki core MAC layer.
 */



/*============================ INCLUDE =======================================*/
#include <stdlib.h>
#include "board.h"
#include "hal.h"
#include "at86rf212_registermap.h"
/*============================ MACROS ========================================*/

/*
 * Macros defined for the radio transceiver's access modes.
 *
 * These functions are implemented as macros since they are used very often.
 */
#define HAL_DUMMY_READ         (0x00) /**<  Dummy value for the SPI. */

#define HAL_TRX_CMD_RW         (0xC0) /**<  Register Write (short mode). */
#define HAL_TRX_CMD_RR         (0x80) /**<  Register Read (short mode). */
#define HAL_TRX_CMD_FW         (0x60) /**<  Frame Transmit Mode (long mode). */
#define HAL_TRX_CMD_FR         (0x20) /**<  Frame Receive Mode (long mode). */
#define HAL_TRX_CMD_SW         (0x40) /**<  SRAM Write. */
#define HAL_TRX_CMD_SR         (0x00) /**<  SRAM Read. */
#define HAL_TRX_CMD_RADDRM     (0x7F) /**<  Register Address Mask. */

#define HAL_CALCULATED_CRC_OK   (0) /**<  CRC calculated over the frame including the CRC field should be 0. */

volatile int bootloader_mode = 0;
volatile int bootloader_pkt = 0;

#if RAVEN_REVISION == HEXABUS_SOCKET
extern uint8_t forwarding_enabled;
#else
#define forwarding_enabled (0)
#endif

extern uint8_t promiscuous_mode;

/*============================ TYPDEFS =======================================*/
/*============================ VARIABLES =====================================*/
#define PIN_SLP_TR {PIO_PA10, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_SS {PIO_PA11, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_RST {PIO_PA9, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_IRQ {PIO_PA15, PIOA, ID_PIOA, PIO_INPUT, PIO_IT_RISE_EDGE }

static const Pin pinRST = PIN_RST;
static const Pin pinSLP_TR = PIN_SLP_TR;
static const Pin pinSS = PIN_SS;
static const Pin pinsSPI[]  = {PINS_SPI};
static const Pin pinIRQ = PIN_IRQ;


/*Flag section.*/
/*Callbacks.*/
static void RF212IRQ( const Pin* pPin );
/*============================ PROTOTYPES ====================================*/
/*============================ IMPLEMENTATION ================================*/

/** \brief  This function initializes the Hardware Abstraction Layer.
 */
#define DELAY_BEFORE_SPCK          200 /* 2us min (tCSS) <=> 200/100 000 000 = 2us */
#define DELAY_BETWEEN_CONS_COM     0xf /* 5us min (tCSH) <=> (32 * 15) / (100 000 000) = 5us */

void
hal_init(void)
{
    /*IO Specific Initialization.*/
    /* Enable SLP_TR as output. */
    PIO_Configure(&pinSLP_TR, 1);
    PIO_Clear(&pinSLP_TR);
    /* Enable RSTPIN as output. */
    PIO_Configure(&pinRST, 1);
    PIO_Set(&pinRST);

    /*SPI Specific Initialization.*/
    PIO_Configure(pinsSPI, PIO_LISTSIZE(pinsSPI));
    /* Set SS, CLK and MOSI as output. */
    PIO_Configure(&pinSS, 1);
    /* Set SS and CLK high */
    PIO_Set(&pinSS);
    /* Enable SPI module and master operation. */
    SPI_Configure(SPI, ID_SPI,
                  SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_PCS(0));
    /* Run SPI at 1MHz */
    SPI_ConfigureNPCS(SPI, 0,
             SPI_CSR_NCPHA | SPI_CSR_DLYBS(DELAY_BEFORE_SPCK) |
             SPI_CSR_DLYBCT(DELAY_BETWEEN_CONS_COM) | SPI_CSR_SCBR(9) );
    SPI_Enable(SPI);

    /* Enable interrupts from the radio transceiver. */
    PIO_ConfigureIt( &pinIRQ, RF212IRQ ) ; /* Interrupt on falling edge */
    /* Enable PIO controller IRQs. */
    NVIC_EnableIRQ( (IRQn_Type)pinIRQ.id ) ;
    /* Enable PIO line interrupts. */
    PIO_EnableIt( &pinIRQ ) ;

}
uint8_t hal_get_slptr(void)
{
    return PIO_Get(&pinSLP_TR);
}

void hal_set_slptr_low(void)
{
    PIO_Clear(&pinSLP_TR);
}

void hal_set_slptr_high(void)
{
    PIO_Set(&pinSLP_TR);
}

void hal_set_rst_high(void)
{
    PIO_Set(&pinRST);
}

void hal_set_rst_low(void)
{
    PIO_Clear(&pinRST);
}

void hal_delay_us(int us)
{
    while( us-- ){
        volatile int i;
        for(i=0; i<5; i++);
    };
}


/*----------------------------------------------------------------------------*/
/** \brief  This function reads data from one of the radio transceiver's registers.
 *
 *  \param  address Register address to read from. See datasheet for register
 *                  map.
 *
 *  \see Look at the at86rf212_registermap.h file for register address definitions.
 *
 *  \returns The actual value of the read register.
 */
uint8_t
hal_register_read(uint8_t address)
{
    /* Add the register read command to the register address. */
    address &= HAL_TRX_CMD_RADDRM;
    address |= HAL_TRX_CMD_RR;

    uint8_t register_value = 0;

    ENTER_CRITICAL_REGION();

    HAL_SS_LOW(); /* Start the SPI transaction by pulling the Slave Select low. */

    /*Send Register address and read register content.*/
    SPI_Write(SPI, 0, address);
    register_value = SPI_Read(SPI);
    SPI_Write(SPI, 0, register_value);
    register_value = SPI_Read(SPI);

    HAL_SS_HIGH(); /* End the transaction by pulling the Slave Select High. */

    LEAVE_CRITICAL_REGION();

    return register_value;
}

/*----------------------------------------------------------------------------*/
/** \brief  This function writes a new value to one of the radio transceiver's
 *          registers.
 *
 *  \see Look at the at86rf212_registermap.h file for register address definitions.
 *
 *  \param  address Address of register to write.
 *  \param  value   Value to write.
 */
void
hal_register_write(uint8_t address, uint8_t value)
{
    /* Add the Register Write command to the address. */
    address = HAL_TRX_CMD_RW | (HAL_TRX_CMD_RADDRM & address);

    ENTER_CRITICAL_REGION();

    HAL_SS_LOW(); /* Start the SPI transaction by pulling the Slave Select low. */

    /*Send Register address and write register content.*/
    SPI_Write(SPI, 0, address);
    uint8_t dummy_read = SPI_Read(SPI);

    SPI_Write(SPI,0, value);
    dummy_read = SPI_Read(SPI);

    HAL_SS_HIGH(); /* End the transaction by pulling the Slave Slect High. */

    LEAVE_CRITICAL_REGION();
}

/*----------------------------------------------------------------------------*/
/** \brief  This function reads the value of a specific subregister.
 *
 *  \see Look at the at86rf212_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position   Bit position of the subregister
 *  \retval Value of the read subregister.
 */
uint8_t
hal_subregister_read(uint8_t address, uint8_t mask, uint8_t position)
{
    /* Read current register value and mask out subregister. */
    uint8_t register_value = hal_register_read(address);
    register_value &= mask;
    register_value >>= position; /* Align subregister value. */

    return register_value;
}

/*----------------------------------------------------------------------------*/
/** \brief  This function writes a new value to one of the radio transceiver's
 *          subregisters.
 *
 *  \see Look at the at86rf212_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position  Bit position of the subregister
 *  \param  value  Value to write into the subregister.
 */
void
hal_subregister_write(uint8_t address, uint8_t mask, uint8_t position,
                            uint8_t value)
{
    /* Read current register value and mask area outside the subregister. */
    uint8_t register_value = hal_register_read(address);
    register_value &= ~mask;

    /* Start preparing the new subregister value. shift in place and mask. */
    value <<= position;
    value &= mask;

    value |= register_value; /* Set the new subregister value. */

    /* Write the modified register value. */
    hal_register_write(address, value);
}

/*----------------------------------------------------------------------------*/
/** \brief  This function will upload a frame from the radio transceiver's frame
 *          buffer.
 *
 *          If the frame currently available in the radio transceiver's frame buffer
 *          is out of the defined bounds. Then the frame length, lqi value and crc
 *          be set to zero. This is done to indicate an error.
 *          This version is optimized for use with contiki RF212BB driver
 *
 *  \param  rx_frame    Pointer to the data structure where the frame is stored.
 *  \param  rx_callback Pointer to callback function for receiving one byte at a time.
 */
void
hal_frame_read(hal_rx_frame_t *rx_frame)
{
    uint8_t *rx_data=0;

    /*  check that we have a valid frame pointer */
//  if (!rx_frame )
//      return;

    ENTER_CRITICAL_REGION();

    HAL_SS_LOW();

    /*Send frame read command.*/
    SPI_Write(SPI, 0, HAL_TRX_CMD_FR);
    uint8_t frame_length = SPI_Read(SPI);

    /*Read frame length.*/
    SPI_Write(SPI,0, frame_length);
    frame_length = SPI_Read(SPI);

    /*Check for correct frame length.*/
    if ((frame_length >= HAL_MIN_FRAME_LENGTH) && (frame_length <= HAL_MAX_FRAME_LENGTH)){

    	uint8_t rx_status = 0; //variable for RX_STATUS information
        rx_data = (rx_frame->data);
        rx_frame->length = frame_length;
        /*Upload frame buffer to data pointer. Calculate CRC.*/
        SPI_Write(SPI, 0, frame_length);

        do{
            uint8_t tempData = SPI_Read(SPI);
            SPI_Write(SPI, 0, 0);       /*  dummy write */
            *rx_data++ = tempData;
        } while (--frame_length > 0);

        /*Read LQI value for this frame.*/
        rx_frame->lqi = SPI_Read(SPI);

		SPI_Write(SPI, 0, 0);       /*  dummy write */
        //read ED
        rx_frame->ed = SPI_Read(SPI);
		SPI_Write(SPI, 0, 0);       /*  dummy write */
        //read RX_STATUS
        rx_status = SPI_Read(SPI);
		SPI_Write(SPI, 0, 0);       /*  dummy write */
        SPI_Read(SPI);

        HAL_SS_HIGH();

        /*Check calculated crc, and set crc field in hal_rx_frame_t accordingly.*/
        //GH: not necessary crc is checked in transceiver
        rx_frame->crc = (rx_status & 0x80);
    } else {
        HAL_SS_HIGH();
        rx_frame->length = 0;
        rx_frame->lqi    = 0;
        rx_frame->crc    = false;
    }

    LEAVE_CRITICAL_REGION();
}

/*----------------------------------------------------------------------------*/
/** \brief  This function will download a frame to the radio transceiver's frame
 *          buffer.
 *
 *  \param  write_buffer    Pointer to data that is to be written to frame buffer.
 *  \param  length          Length of data. The maximum length is 127 bytes.
 */
void
hal_frame_write(uint8_t *write_buffer, uint8_t length)
{
    length &= HAL_TRX_CMD_RADDRM; /* Truncate length to maximum frame length. */
    ENTER_CRITICAL_REGION();

    HAL_SS_LOW(); /* Initiate the SPI transaction. */

    /*SEND FRAME WRITE COMMAND AND FRAME LENGTH.*/
    SPI_Write(SPI, 0, HAL_TRX_CMD_FW);
    uint8_t dummy_read = SPI_Read(SPI);

    SPI_Write(SPI, 0, length);
    dummy_read = SPI_Read(SPI);

    /* Download to the Frame Buffer. */
    do{
        SPI_Write(SPI, 0, *write_buffer++);
        --length;

        dummy_read = SPI_Read(SPI);
    } while (length > 0);

    HAL_SS_HIGH(); /* Terminate SPI transaction. */

    LEAVE_CRITICAL_REGION();
}

/*----------------------------------------------------------------------------*/
/** \brief Read SRAM
 *
 * This function reads from the SRAM of the radio transceiver.
 *
 * \param address Address in the TRX's SRAM where the read burst should start
 * \param length Length of the read burst
 * \param data Pointer to buffer where data is stored.
 */
void
hal_sram_read(uint8_t address, uint8_t length, uint8_t *data)
{
    ENTER_CRITICAL_REGION();

    HAL_SS_LOW(); /* Initiate the SPI transaction. */

    /*Send SRAM read command.*/
    SPI_Write(SPI, 0, HAL_TRX_CMD_SR);
    uint8_t dummy_read = SPI_Read(SPI);

    /*Send address where to start reading.*/
    SPI_Write(SPI, 0, address);

    dummy_read = SPI_Read(SPI);

    /*Upload the chosen memory area.*/
    do{
        SPI_Write(SPI, 0, HAL_DUMMY_READ);
        *data++ = SPI_Read(SPI);
    } while (--length > 0);

    HAL_SS_HIGH();

    LEAVE_CRITICAL_REGION();
}

/*----------------------------------------------------------------------------*/
/** \brief Write SRAM
 *
 * This function writes into the SRAM of the radio transceiver.
 *
 * \param address Address in the TRX's SRAM where the write burst should start
 * \param length  Length of the write burst
 * \param data    Pointer to an array of bytes that should be written
 */
void
hal_sram_write(uint8_t address, uint8_t length, uint8_t *data)
{
    ENTER_CRITICAL_REGION();

    HAL_SS_LOW();

    /*Send SRAM write command.*/
    SPI_Write(SPI, 0, HAL_TRX_CMD_SW);
    uint8_t dummy_read = SPI_Read(SPI);

    /*Send address where to start writing to.*/
    SPI_Write(SPI, 0 , address);
    dummy_read = SPI_Read(SPI);

    /*Upload the chosen memory area.*/
    do{
        SPI_Write(SPI, 0, *data++);
        dummy_read = SPI_Read(SPI);
    } while (--length > 0);

    HAL_SS_HIGH();

    LEAVE_CRITICAL_REGION();
}

/*----------------------------------------------------------------------------*/
/* This #if compile switch is used to provide a "standard" function body for the */
/* doxygen documentation. */
#if defined(DOXYGEN)
/** \brief ISR for the radio IRQ line, triggered by the input capture.
 *  This is the interrupt service routine for timer1.ICIE1 input capture.
 *  It is triggered of a rising edge on the radio transceivers IRQ line.
 */
void RADIO_VECT(void);
#else  /* !DOXYGEN */
/* These link to the RF212BB driver in rf212.c */
void rf212_interrupt(void);
extern hal_rx_frame_t rxframe;

#define DEBUG 0
#if DEBUG
volatile int rf212_interrupt_flag=0;
#define INTERRUPTDEBUG(arg) rf212_interrupt_flag=arg
#else
#define INTERRUPTDEBUG(arg)
#endif

static void RF212IRQ( const Pin* pPin )
{
    if( pPin != &pinIRQ )
        return;
 
	volatile uint8_t state;

	INTERRUPTDEBUG(1);

	/*Read Interrupt source.*/
	HAL_SS_LOW();

	/*Send Register address and read register content.*/
	SPI_Write(SPI, 0, RG_IRQ_STATUS | HAL_TRX_CMD_RR);

	uint8_t interrupt_source = SPI_Read(SPI); /* The interrupt variable is used as a dummy read. */

	SPI_Write(SPI, 0, interrupt_source);
	interrupt_source = SPI_Read(SPI); /* The interrupt source is read. */

	HAL_SS_HIGH();

	/*Handle the incomming interrupt. Prioritized.*/
	if ((interrupt_source & HAL_RX_START_MASK)) {
		INTERRUPTDEBUG(10);
	}
	if (interrupt_source & HAL_TRX_END_MASK) {
		INTERRUPTDEBUG(11);

		state = hal_subregister_read(SR_TRX_STATUS);

		if ((state == BUSY_RX_AACK) || (state == RX_ON) || (state == BUSY_RX)
				|| (state == RX_AACK_ON)) {
			//protect the frame buffer content against being overwritten
			hal_subregister_write(SR_TRX_CMD, PLL_ON);
			/* Received packet interrupt */
			/* Buffer the frame and call rf212_interrupt to schedule poll for rf212 receive process */
			//         if (rxframe.length) break;			//toss packet if last one not processed yet
			hal_frame_read(&rxframe);
			if(bootloader_mode)
				bootloader_pkt = 1;
			else
				rf212_interrupt();
			/* Enable reception of next packet */

			if (forwarding_enabled || promiscuous_mode)
				hal_subregister_write(SR_TRX_CMD, RX_ON);
			else
				hal_subregister_write(SR_TRX_CMD, RX_AACK_ON);
		} else {
			//transmission ended
			/* Transition to receive mode*/
			if (forwarding_enabled || promiscuous_mode)
				hal_subregister_write(SR_TRX_CMD, RX_ON);
			else
				hal_subregister_write(SR_TRX_CMD, RX_AACK_ON);
		}

	}
	if (interrupt_source & HAL_TRX_UR_MASK) {
		INTERRUPTDEBUG(13);
		;
	}
	if (interrupt_source & HAL_PLL_UNLOCK_MASK) {
		INTERRUPTDEBUG(14);
		;
	}
	if (interrupt_source & HAL_PLL_LOCK_MASK) {
		INTERRUPTDEBUG(15);
	}
	if (interrupt_source & HAL_BAT_LOW_MASK) {
		/*  Disable BAT_LOW interrupt to prevent endless interrupts. The interrupt */
		/*  will continously be asserted while the supply voltage is less than the */
		/*  user-defined voltage threshold. */
		uint8_t trx_isr_mask = hal_register_read(RG_IRQ_MASK);
		trx_isr_mask &= ~HAL_BAT_LOW_MASK;
		hal_register_write(RG_IRQ_MASK, trx_isr_mask);
		//      hal_bat_low_flag++; /* Increment BAT_LOW flag. */
		INTERRUPTDEBUG(16);
		;
	}
	if (interrupt_source & HAL_CCA_ED_DONE) {
		INTERRUPTDEBUG(17);
	}
	if (interrupt_source & HAL_AMI_MASK) {
		INTERRUPTDEBUG(17);
	}
	if (interrupt_source == 0) {
		INTERRUPTDEBUG(99);
	}
}
#   endif /* defined(DOXYGEN) */

/** @} */
/** @} */

/*EOF*/
