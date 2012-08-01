#include "board.h"
#include "CDCDSerialDriver.h"

#include <stdbool.h>
#include <stdint.h>
#include "lib/ringbuf.h"

/*----------------------------------------------------------------------------
 *      Definitions
 *----------------------------------------------------------------------------*/

/** Size in bytes of the buffer used for reading data from the USB & USART */
//#define DATABUFFERSIZE (64+2)

/** Speed test buffer size */
//#define TEST_BUFFER_SIZE    (2*1024)
/** Speed test loop count */
//#define TEST_COUNT          (30)

/*----------------------------------------------------------------------------
 *      External variables
 *----------------------------------------------------------------------------*/

extern const USBDDriverDescriptors cdcdSerialDriverDescriptors;

/*----------------------------------------------------------------------------
 *      Internal variables
 *----------------------------------------------------------------------------*/
/** Serial Port ON/OFF */
static uint8_t isCdcSerialON = 0;
static struct ringbuf ringoutbuf;
static uint8_t outbuf[128];
static volatile uint8_t usbidle;
#if 0
/** Buffer for storing incoming USB data. */
static uint8_t usbBuffer[DATABUFFERSIZE];

/** TC tick: 1/250 s */
static uint32_t tcTick = 0;

/** USB Tx flag */
static uint8_t txDoneFlag = 0;
#endif

/*----------------------------------------------------------------------------
 *         VBus monitoring
 *----------------------------------------------------------------------------*/
/** VBus pin instance. */
static const Pin pinVbus = PIN_USB_VBUS;

/**
 * Handles interrupts coming from PIO controllers.
 */
static void ISR_Vbus(const Pin *pPin)
{
    /* Check current level on VBus */
    if( PIO_Get(&pinVbus) ){
        //TRACE_INFO("VBUS conn\n\r");
        USBD_Connect();
    }else{
        //TRACE_INFO("VBUS discon\n\r");
        USBD_Disconnect();
    }
}

/**
 * Configures the VBus pin to trigger an interrupt when the level on that pin
 * changes.
 */
static void VBus_Configure( void )
{
    /* Configure PIO */
    PIO_Configure(&pinVbus, 1);
    PIO_ConfigureIt(&pinVbus, ISR_Vbus);
    PIO_EnableIt(&pinVbus);

    /* Check current level on VBus */
    if( PIO_Get(&pinVbus) ){
        /* if VBUS present, force the connect */
        //TRACE_INFO("conn\n\r");
        USBD_Connect();
        usbidle = 1;
    }else{
        USBD_Disconnect();
    }
}

/*-----------------------------------------------------------------------------
 *         Callback re-implementation
 *-----------------------------------------------------------------------------*/

/**
 * Invoked after the USB driver has been initialized. By default, configures
 * the UDP/UDPHS interrupt.
 */
void USBDCallbacks_Initialized(void)
{
    NVIC_EnableIRQ(UDP_IRQn);
}

/**
 * Invoked when the configuration of the device changes. Parse used endpoints.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(unsigned char cfgnum)
{
    CDCDSerialDriver_ConfigurationChangedHandler(cfgnum);
}

/**
 * Invoked when a new SETUP request is received from the host. Forwards the
 * request to the Mass Storage device driver handler function.
 * \param request  Pointer to a USBGenericRequest instance.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
    CDCDSerialDriver_RequestHandler(request);
}

/*----------------------------------------------------------------------------
 *         Internal functions
 *----------------------------------------------------------------------------*/

#if 0
/*----------------------------------------------------------------------------
 * Callback invoked when data has been received on the USB.
 *----------------------------------------------------------------------------*/
static void _UsbDataReceived(uint32_t unused,
                             uint8_t status,
                             uint32_t received,
                             uint32_t remaining)
{
    /* Check that data has been received successfully */
    if (status == USBD_STATUS_SUCCESS) {

        /* Send back CDC data */
        if (isCdcEchoON) {

            CDCDSerialDriver_Write(usbBuffer, received, 0, 0);
        }

        /* Send data through USART */
        if (isCdcSerialON) {

        }

        /* Check if bytes have been discarded */
        if ((received == DATABUFFERSIZE) && (remaining > 0)) {

            //TRACE_WARNING(
            //          "_UsbDataReceived: %u bytes discarded\n\r", (unsigned int)remaining);
        }
    }
    else {

        //TRACE_WARNING( "_UsbDataReceived: Transfer error\n\r");
    }
        CDCDSerialDriver_Read(usbBuffer,
                                      DATABUFFERSIZE,
                                      (TransferCallback) _UsbDataReceived,
                                      0);

}
#endif
/*----------------------------------------------------------------------------
 * Callback invoked when data has been sent.
 *----------------------------------------------------------------------------*/
static void _UsbDataSent( void )
{
    uint8_t buf[64];
    int s = ringbuf_elements(&ringoutbuf);
    int i;
    static volatile int count = 0;
    static volatile int foo;
    if( s == 0 ){
        usbidle = 1;
        return;
    }
    usbidle = 0;
    if( s > 64 )
        s = 64;
    
    for(i=0; i<s; i++)
        buf[i] = ringbuf_get(&ringoutbuf);
    count++;
    if( CDCDSerialDriver_Write(buf, s, (TransferCallback) _UsbDataSent, 0) != USBD_STATUS_SUCCESS ){
        while(1){
            foo++;
        }
    }   
}


/**
 * \brief Configure 48MHz Clock for USB
 */
static void _ConfigureUsbClock(void)
{
    /* Enable PLLB for USB */
    PMC->CKGR_PLLBR = CKGR_PLLBR_DIVB(1)
                    | CKGR_PLLBR_MULB(5)
                    | CKGR_PLLBR_PLLBCOUNT_Msk;
    while((PMC->PMC_SR & PMC_SR_LOCKB) == 0);
    /* USB Clock uses PLLB */
    PMC->PMC_USB = PMC_USB_USBDIV(1)       /* /2   */
                 | PMC_USB_USBS;           /* PLLB */
}
#if 0
/**
 * Configure TC0 to generate an interrupt every 4ms
 */
static void _ConfigureTc0(void)
{
    uint32_t div, tcclks;

    /* Enable TC0 peripheral */
    PMC_EnablePeripheral(ID_TC0);
    /* Configure TC0 for 250Hz frequency and trigger on RC compare */
    TC_FindMckDivisor(250, BOARD_MCK, &div, &tcclks, BOARD_MCK);
    TC_Configure(TC0, 0, tcclks | TC_CMR_CPCTRG);
    TC0->TC_CHANNEL[0].TC_RC = (BOARD_MCK / div) / 250;
    /* Configure and enable interrupt on RC compare */
    NVIC_EnableIRQ(TC0_IRQn);
    TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
    /* Start TC when USB connected & RX enabled */
}
#endif
/**
 * Test USB CDC Serial Speed
 */
#if 0
static void _TestSpeed(void)
{
    uint32_t startT, endT;
    uint32_t i, testCnt;

    if (!isCdcSerialON) {
        //printf("\n\r!! Host serial program not ready!\n\r");
        return;
    }
    //printf("\n\r- USB CDC Serial Speed test:\n\r");

    /* Test data initialize */
    for (i = 0; i < TEST_BUFFER_SIZE; i ++) testBuffer[i] = (i % 10) + '0';

    //printf("- Send 0,1,2 ... to host:\n\r");
    startT = tcTick;
    for (testCnt = 0; testCnt < TEST_COUNT; testCnt ++) {
        txDoneFlag = 0;
        CDCDSerialDriver_Write(testBuffer,
                               TEST_BUFFER_SIZE,
                               (TransferCallback) _UsbDataSent, 0);
        while(!txDoneFlag);
    }
    /* Finish sending */
    CDCDSerialDriver_Write(testBuffer, 0, 0, 0);
    endT = tcTick;
    //printf("- Done: Size %d, Count %d, Time %d ~ %d\n\r", (int)TEST_BUFFER_SIZE, (int)testCnt, (int)startT, (int)endT);
    //printf("- Speed %dKB/s\n\r", (int)((TEST_BUFFER_SIZE*testCnt)/4/(endT-startT)));
}
#endif
/*----------------------------------------------------------------------------
 *          Main
 *----------------------------------------------------------------------------*/
void gpioSetValue2(const Pin *pin, int value)
{
    if( value )
        pin->pio->PIO_SODR = pin->mask;
    else
        pin->pio->PIO_CODR = pin->mask;
}


/**
 * \brief usb_cdc_serial Application entry point.
 *
 * Initializes drivers and start the USB <-> Serial bridge.
 */
int usbmain(void)
{
    /* Disable watchdog */
    WDT_Disable( WDT );

    /* If they are present, configure Vbus & Wake-up pins */
    PIO_InitializeInterrupts(0);

    /* Enable UPLL for USB */
    _ConfigureUsbClock();

    /* CDC serial driver initialization */
    CDCDSerialDriver_Initialize(&cdcdSerialDriverDescriptors);

    /* connect if needed */
    VBus_Configure();
    ringbuf_init(&ringoutbuf, outbuf, sizeof(outbuf));

    //TODO: FAIL this pulls in malloc
    //setbuf(stdout, NULL);
    return 0;
    //setbuf(stdin, NULL);
    volatile int i = 0;
    const Pin SOUT = {PIO_PA8, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT};
    while( i++ < 64000000);
    //printf("hello world");
    //printf("hello world2");
    while (1) {
        if( i++ == 100000 ){
            i = 0;
            gpioSetValue2(&SOUT,1);
            gpioSetValue2(&SOUT,0);
        }
    }
}

extern int _write( int file, char *ptr, int len )
{
    int i;
    for(i=0; i<len; i++)
        ringbuf_put(&ringoutbuf, ptr[i]);
    if( usbidle ){
        if( CDCDSerialDriver_GetControlLineState() & CDCControlLineState_DTR && USBD_GetState() >= USBD_STATE_CONFIGURED )
            _UsbDataSent(); //initiate new transfer
    }
    //int iIndex ;
    //if( isCdcSerialON )
    //if( CDCDSerialDriver_GetControlLineState() & CDCControlLineState_DTR && USBD_GetState() >= USBD_STATE_CONFIGURED)
    //    while (CDCDSerialDriver_Write(ptr,len, 0, 0) != USBD_STATUS_SUCCESS);
        //CDCDSerialDriver_Write(ptr,len, 0, 0);
    return len;
}

