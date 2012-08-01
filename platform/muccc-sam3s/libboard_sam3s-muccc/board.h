/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2009, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/**
 * \page sam3s_ek_board_desc SAM3S-EK - Board Description
 *
 * \section Purpose
 *
 * This file is dedicated to describe the SAM3S-EK board.
 *
 * \section Contents
 *
 *  - For SAM3S-EK information, see \subpage sam3s_ek_board_info.
 *  - For operating frequency information, see \subpage sam3s_ek_opfreq.
 *  - For using portable PIO definitions, see \subpage sam3s_ek_piodef.
 *  - For on-board memories, see \subpage sam3s_ek_mem.
 *  - Several USB definitions are included here, see \subpage sam3s_ek_usb.
 *  - For External components, see \subpage sam3s_ek_extcomp.
 *  - For Individual chip definition, see \subpage sam3s_ek_chipdef.
 *
 * To get more software details and the full list of parameters related to the
 * SAM3S-EK board configuration, please have a look at the source file:
 * \ref board.h\n
 *
 * \section Usage
 *
 *  - The code for booting the board is provided by board_cstartup_xxx.c and
 *    board_lowlevel.c.
 *  - For using board PIOs, board characteristics (clock, etc.) and external
 *    components, see board.h.
 *  - For manipulating memories, see board_memories.h.
 *
 * This file can be used as a template and modified to fit a custom board, with
 * specific PIOs usage or memory connections.
 */

/**
 *  \file board.h
 *
 *  Definition of SAM3S-EK characteristics, SAM3S-dependant PIOs and
 *  external components interfacing.
 */

#ifndef _BOARD_
#define _BOARD_

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "chip.h"

#include "include/bitbanding.h"
#include "include/board_lowlevel.h"
#include "include/board_memories.h"
#include "include/clock.h"
#include "include/led.h"
#include "include/math.h"
#include "include/timetick.h"
#include "include/uart_console.h"

/**
 * Libc porting layers
 */
#if defined   ( __CC_ARM   ) /* Keil �Vision 4 */
#    include "include/rand.h"
#elif defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
#    include "include/rand.h"
#elif defined (  __GNUC__  ) /* GCC CS3 2009q3-68/2010q1-188 */
#    include "include/syscalls.h" /** RedHat Newlib minimal stub */
#endif

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/**
 * \page sam3s_ek_board_info "SAM3S-EK - Board informations"
 * This page lists several definition related to the board description.
 *
 * \section Definitions
 * - \ref BOARD_NAME
 */

/** Name of the board */
#define BOARD_NAME "muccc"
/** Board definition */
#define muccc
/** Family definition (already defined) */
#define sam3s
/** Core definition */
#define cortexm3

#define BOARD_REV_A

/*----------------------------------------------------------------------------*/
/**
 *  \page sam3s_ek_opfreq "SAM3S-EK - Operating frequencies"
 *  This page lists several definition related to the board operating frequency
 *  (when using the initialization done by board_lowlevel.c).
 *
 *  \section Definitions
 *  - \ref BOARD_MAINOSC
 *  - \ref BOARD_MCK
 */

/** Frequency of the board main oscillator */
#define BOARD_MAINOSC           16000000

/** Master clock frequency (when using board_lowlevel.c) */
#define BOARD_MCK               64000000

/*----------------------------------------------------------------------------*/
/**
 * \page sam3s_ek_piodef "SAM3S-EK - PIO definitions"
 * This pages lists all the pio definitions contained in board.h. The constants
 * are named using the following convention: PIN_* for a constant which defines
 * a single Pin instance (but may include several PIOs sharing the same
 * controller), and PINS_* for a list of Pin instances.
 *
 * ADC
 * - \ref PIN_ADC0_AD0
 * - \ref PIN_ADC0_AD1
 * - \ref PIN_ADC0_AD2
 * - \ref PIN_ADC0_AD3
 * - \ref PIN_ADC0_AD4
 * - \ref PIN_ADC0_AD5
 * - \ref PIN_ADC0_AD6
 * - \ref PIN_ADC0_AD7
 * - \ref PINS_ADC
 *
 * UART
 * - \ref PINS_UART
 *
 * LEDs
 * - \ref PIN_LED_0
 * - \ref PIN_LED_1
 * - \ref PIN_LED_2
 * - \ref PINS_LEDS
 *
 * MCI
 * - \ref PINS_MCI
 *
 * Push buttons
 * - \ref PIN_PUSHBUTTON_1
 * - \ref PIN_PUSHBUTTON_2
 * - \ref PINS_PUSHBUTTONS
 * - \ref PUSHBUTTON_BP1
 * - \ref PUSHBUTTON_BP2
 *
 * PWMC
 * - \ref PIN_PWMC_PWMH0
 * - \ref PIN_PWMC_PWML0
 * - \ref PIN_PWMC_PWMH1
 * - \ref PIN_PWMC_PWML1
 * - \ref PIN_PWMC_PWMH2
 * - \ref PIN_PWMC_PWML2
 * - \ref PIN_PWMC_PWMH3
 * - \ref PIN_PWMC_PWML3
 * - \ref PIN_PWM_LED0
 * - \ref PIN_PWM_LED1
 * - \ref PIN_PWM_LED2
 * - \ref CHANNEL_PWM_LED0
 * - \ref CHANNEL_PWM_LED1
 * - \ref CHANNEL_PWM_LED2
 *
 * SPI
 * - \ref PIN_SPI_MISO
 * - \ref PIN_SPI_MOSI
 * - \ref PIN_SPI_SPCK
 * - \ref PINS_SPI
 * - \ref PIN_SPI_NPCS0_PA11
 *
 * SSC
 * - \ref PIN_SSC_TD
 * - \ref PIN_SSC_TK
 * - \ref PIN_SSC_TF
 * - \ref PINS_SSC_CODEC
 *
 * PCK0
 * - \ref PIN_PCK0
 *
 * PIO PARALLEL CAPTURE
 * - \ref PIN_PIODCEN1
 * - \ref PIN_PIODCEN2
 *
 * TWI
 * - \ref TWI_V3XX
 * - \ref PIN_TWI_TWD0
 * - \ref PIN_TWI_TWCK0
 * - \ref PINS_TWI0
 * - \ref PIN_TWI_TWD1
 * - \ref PIN_TWI_TWCK1
 * - \ref PINS_TWI1
 *
 * USART0
 * - \ref PIN_USART0_RXD
 * - \ref PIN_USART0_TXD
 * - \ref PIN_USART0_CTS
 * - \ref PIN_USART0_RTS
 * - \ref PIN_USART0_SCK
 *
 * USB
 * - \ref PIN_USB_VBUS
 *
 * PIO definitions for keys
 * \ref KEY_IOMASK_SNS
 * \ref KEY_IOMASK_SNSK
 * \ref PINS_KEY_SNS
 * \ref PINS_KEY_SNSK
 *
 */

/** ADC_AD0 pin definition. */
#define PIN_ADC0_AD0 {1 << 21, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
/** ADC_AD1 pin definition. */
#define PIN_ADC0_AD1 {1 << 30, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
/** ADC_AD2 pin definition. */
#define PIN_ADC0_AD2 {1 << 3, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT}
/** ADC_AD3 pin definition. */
#define PIN_ADC0_AD3 {1 << 4, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT}
/** ADC_AD4 pin definition. */
#define PIN_ADC0_AD4 {1 << 15, PIOC, ID_PIOC, PIO_INPUT, PIO_DEFAULT}
/** ADC_AD5 pin definition. */
#define PIN_ADC0_AD5 {1 << 16, PIOC, ID_PIOC, PIO_INPUT, PIO_DEFAULT}
/** ADC_AD6 pin definition. */
#define PIN_ADC0_AD6 {1 << 17, PIOC, ID_PIOC, PIO_INPUT, PIO_DEFAULT}
/** ADC_AD7 pin definition. */
#define PIN_ADC0_AD7 {1 << 18, PIOC, ID_PIOC, PIO_INPUT, PIO_DEFAULT}

/** Pins ADC */
#define PINS_ADC PIN_ADC0_AD0, PIN_ADC0_AD1, PIN_ADC0_AD2, PIN_ADC0_AD3, PIN_ADC0_AD4, PIN_ADC0_AD5, PIN_ADC0_AD6, PIN_ADC0_AD7

/** Startup time max, return from Idle mode (in �s) */
#define ADC_STARTUP_TIME_MAX       15
/** Track and hold Acquisition Time min (in ns) */
#define ADC_TRACK_HOLD_TIME_MIN  1200
/** ADC clock frequence */
#define BOARD_ADC_FREQ     (6000000)

/** UART pins (UTXD0 and URXD0) definitions, PA9,10. */
#define PINS_UART  { PIO_PA9A_URXD0|PIO_PA10A_UTXD0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}

#define LED_BLUE      0
#define LED_GREEN     1
#define LED_RED       2

/** LED #0 pin definition (BLUE). */
#define PIN_LED_0   {PIO_PC20, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT}
/** LED #1 pin definition (GREEN). */
#define PIN_LED_1   {PIO_PC21, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT}
/** LED #2 pin definition (RED). */
#define PIN_LED_2   {PIO_PC22, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT}


/** List of all LEDs definitions. */
#define PINS_LEDS   PIN_LED_0, PIN_LED_1, PIN_LED_2

/** MCI pins definition. */
#define PINS_MCI   {0x3fUL << 26, PIOA, ID_PIOA, PIO_PERIPH_C, PIO_PULLUP}
/** MCI pin Card Detect. */

#define PIN_MCI_CD {PIO_PA15, PIOA, ID_PIOA, PIO_INPUT, PIO_PULLUP}


/** Push button #0 definition. Attributes = pull-up + debounce + interrupt on rising edge. */
#define PIN_PUSHBUTTON_1    {PIO_PB3, PIOB, ID_PIOB, PIO_INPUT, PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE}
/** Push button #1 definition. Attributes = pull-up + debounce + interrupt on falling edge. */
#define PIN_PUSHBUTTON_2    {PIO_PC12, PIOC, ID_PIOC, PIO_INPUT, PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_FALL_EDGE}
/** List of all push button definitions. */
#define PINS_PUSHBUTTONS    PIN_PUSHBUTTON_1, PIN_PUSHBUTTON_2

/** Push button #1 index. */
#define PUSHBUTTON_BP1   0
/** Push button #2 index. */
#define PUSHBUTTON_BP2   1

/** PWMC PWM0 pin definition: Output High. */
#define PIN_PWMC_PWMH0  {PIO_PC18B_PWMH0, PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
/** PWMC PWM0 pin definition: Output Low. */
#define PIN_PWMC_PWML0  {PIO_PA19B_PWML0, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
/** PWMC PWM1 pin definition: Output High. */
#define PIN_PWMC_PWMH1  {PIO_PC19B_PWMH1, PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
/** PWMC PWM1 pin definition: Output Low. */
#define PIN_PWMC_PWML1  {PIO_PA20B_PWML1, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
/** PWMC PWM2 pin definition: Output High. */
#define PIN_PWMC_PWMH2  {PIO_PC20B_PWMH2, PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
/** PWMC PWM2 pin definition: Output Low. */
#define PIN_PWMC_PWML2  {PIO_PA16C_PWML2, PIOA, ID_PIOA, PIO_PERIPH_C, PIO_DEFAULT}
/** PWMC PWM3 pin definition: Output High. */
#define PIN_PWMC_PWMH3  {PIO_PC21B_PWMH3, PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
/** PWMC PWM3 pin definition: Output Low. */
#define PIN_PWMC_PWML3  {PIO_PA15C_PWML3, PIOA, ID_PIOA, PIO_PERIPH_C, PIO_DEFAULT}
/** PWM pins definition for LED0 */
#define PIN_PWM_LED0 PIN_PWMC_PWMH0, PIN_PWMC_PWML0
/** PWM pins definition for LED1 */
#define PIN_PWM_LED1 PIN_PWMC_PWMH1, PIN_PWMC_PWML1
/** PWM pins definition for LED2 */
#define PIN_PWM_LED2 PIN_PWMC_PWMH2, PIN_PWMC_PWML2
/** PWM channel for LED0 */
#define CHANNEL_PWM_LED0 0
/** PWM channel for LED1 */
#define CHANNEL_PWM_LED1 1
/** PWM channel for LED2 */
#define CHANNEL_PWM_LED2 2

/** SPI MISO pin definition. */
#define PIN_SPI_MISO    {PIO_PA12A_MISO, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** SPI MOSI pin definition. */
#define PIN_SPI_MOSI    {PIO_PA13A_MOSI, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** SPI SPCK pin definition. */
#define PIN_SPI_SPCK    {PIO_PA14A_SPCK, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** SPI chip select pin definition. */
#define PIN_SPI_NPCS0_PA11  {PIO_PA11A_NPCS0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** List of SPI pin definitions (MISO, MOSI & SPCK). */
#define PINS_SPI        PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_SPCK

/** SSC pin Transmitter Data (TD) */
#define PIN_SSC_TD      {PIO_PA17A_TD, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** SSC pin Transmitter Clock (TK) */
#define PIN_SSC_TK      {PIO_PA16A_TK, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** SSC pin Transmitter FrameSync (TF) */
#define PIN_SSC_TF      {PIO_PA15A_TF, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** SSC pins definition for codec. */
#define PINS_SSC_CODEC  PIN_SSC_TD, PIN_SSC_TK, PIN_SSC_TF

/** PCK0 */
#define PIN_PCK0        {PIO_PA6B_PCK0, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
#define PIN_PCK1        {PIO_PA17B_PCK1,PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}

/** PIO PARALLEL CAPTURE */
/** Parallel Capture Mode Data Enable1 */
#define PIN_PIODCEN1    PIO_PA15
/** Parallel Capture Mode Data Enable2 */
#define PIN_PIODCEN2    PIO_PA16

/** TWI ver 3.xx */
#define TWI_V3XX
/** TWI0 data pin */
#define PIN_TWI_TWD0   {PIO_PA3A_TWD0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** TWI0 clock pin */
#define PIN_TWI_TWCK0  {PIO_PA4A_TWCK0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** TWI0 pins */
#define PINS_TWI0      PIN_TWI_TWD0, PIN_TWI_TWCK0
/** TWI1 data pin */
#define PIN_TWI_TWD1   {PIO_PB4A_TWD1, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** TWI1 clock pin */
#define PIN_TWI_TWCK1  {PIO_PB5A_TWCK1, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** TWI1 pins */
#define PINS_TWI1      PIN_TWI_TWD1, PIN_TWI_TWCK1

/** USART0 pin RX */
#define PIN_USART0_RXD    {PIO_PA5A_RXD0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** USART0 pin TX */
#define PIN_USART0_TXD    {PIO_PA6A_TXD0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** USART0 pin CTS */
#define PIN_USART0_CTS    {PIO_PA8A_CTS0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** USART0 pin RTS */
#define PIN_USART0_RTS    {PIO_PA7A_RTS0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** USART0 pin SCK */
#define PIN_USART0_SCK    {PIO_PA2B_SCK0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}

/** USART1 pin RX */
#define PIN_USART1_RXD    {PIO_PA21A_RXD1, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** USART1 pin TX */
#define PIN_USART1_TXD    {PIO_PA22A_TXD1, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** USART1 pin CTS */
#define PIN_USART1_CTS    {PIO_PA25A_CTS1, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** USART1 pin RTS */
#define PIN_USART1_RTS    {PIO_PA24A_RTS1, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** USART1 pin ENABLE */
#define PIN_USART1_EN     {PIO_PA23A_SCK1, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
/** USART1 pin SCK */
#define PIN_USART1_SCK    {PIO_PA23A_SCK1, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}

/** USB VBus monitoring pin definition. */
#define PIN_USB_VBUS    {PIO_PA1, PIOA, ID_PIOA, PIO_INPUT, PIO_PULLUP}


/* PIO definitions for keys */
#define KEY_IOMASK_SNS   (uint32_t)(PIO_PC22 | PIO_PC24 | PIO_PC26 | PIO_PC28 | PIO_PC30)
#define KEY_IOMASK_SNSK  (uint32_t)(PIO_PC23 | PIO_PC25 | PIO_PC27 | PIO_PC29 | PIO_PC31)
#define PINS_KEY_SNS     {KEY_IOMASK_SNS,  PIOC, ID_PIOC, PIO_INPUT, PIO_DEFAULT}
#define PINS_KEY_SNSK    {KEY_IOMASK_SNSK, PIOC, ID_PIOC, PIO_INPUT, PIO_DEFAULT}

/*----------------------------------------------------------------------------*/
/**
 * \page sam3s_ek_usb "SAM3S-EK - USB device"
 *
 * \section Definitions
 * - \ref BOARD_USB_BMATTRIBUTES
 * - \ref CHIP_USB_UDP
 * - \ref CHIP_USB_PULLUP_INTERNAL
 * - \ref CHIP_USB_NUMENDPOINTS
 * - \ref CHIP_USB_ENDPOINTS_MAXPACKETSIZE
 * - \ref CHIP_USB_ENDPOINTS_BANKS
 */

/** USB attributes configuration descriptor (bus or self powered, remote wakeup) */
#define BOARD_USB_BMATTRIBUTES              USBConfigurationDescriptor_SELFPOWERED_RWAKEUP

/** Indicates chip has an UDP Full Speed. */
#define CHIP_USB_UDP

/** Indicates chip has an internal pull-up. */
#define CHIP_USB_PULLUP_INTERNAL

/** Number of USB endpoints */
#define CHIP_USB_NUMENDPOINTS 8

/** Endpoints max paxcket size */
#define CHIP_USB_ENDPOINTS_MAXPACKETSIZE(i) \
   ((i == 0) ? 64 : \
   ((i == 1) ? 64 : \
   ((i == 2) ? 64 : \
   ((i == 3) ? 64 : \
   ((i == 4) ? 512 : \
   ((i == 5) ? 512 : \
   ((i == 6) ? 64 : \
   ((i == 7) ? 64 : 0 ))))))))

/** Endpoints Number of Bank */
#define CHIP_USB_ENDPOINTS_BANKS(i) \
   ((i == 0) ? 1 : \
   ((i == 1) ? 2 : \
   ((i == 2) ? 2 : \
   ((i == 3) ? 1 : \
   ((i == 4) ? 2 : \
   ((i == 5) ? 2 : \
   ((i == 6) ? 2 : \
   ((i == 7) ? 2 : 0 ))))))))

/** USART RX pin for application */
#define BOARD_PIN_USART_RXD        PIN_USART1_RXD
/** USART TX pin for application */
#define BOARD_PIN_USART_TXD        PIN_USART1_TXD
/** USART CTS pin for application */
#define BOARD_PIN_USART_CTS        PIN_USART1_CTS
/** USART RTS pin for application */
#define BOARD_PIN_USART_RTS        PIN_USART1_RTS
/** USART ENABLE pin for application */
#define BOARD_PIN_USART_EN         PIN_USART1_EN
/** USART Base for application */
#define BOARD_USART_BASE           USART1
/** USART ID for application */
#define BOARD_ID_USART             ID_USART1

#endif /* #ifndef _BOARD_ */

