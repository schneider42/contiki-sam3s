#include "board.h"
#include "eeprom.h"

static const Pin pins_eeprom[] = {PINS_TWI0};
static Twid twid;

static uint8_t eui48[6];
static uint8_t eui64[8];

#define TWCK            100000

void eeprom_init()
{
    PIO_Configure(pins_eeprom, PIO_LISTSIZE(pins_eeprom));
    PMC->PMC_PCER0 = 1 << ID_TWI0;
    TWI_ConfigureMaster(TWI0, TWCK, BOARD_MCK);
    TWID_Initialize(&twid, TWI0);
    
    eeprom_readBytes(0x100-6, eui48, sizeof(eui48));
    //TWID_Read(&twid, 0x50, 0x100-6, 0x01, eui48, sizeof(eui48), 0);

    eui64[0] = eui48[0];
    eui64[1] = eui48[1];
    eui64[2] = eui48[2];
    eui64[3] = 0xFF;
    eui64[4] = 0xFE;
    eui64[5] = eui48[3];
    eui64[6] = eui48[4];
    eui64[7] = eui48[5];
    return;
}

uint8_t * eeprom_getEUI48(void)
{
    return eui48;
}

uint8_t * eeprom_getEUI64(void)
{
    return eui64;
}

void eeprom_writeBytes(uint32_t address, uint8_t *buf, uint32_t len)
{
}


void eeprom_readBytes(uint32_t address, uint8_t *buf, uint32_t len)
{
    TWID_Read(&twid, 0x50, address, 0x01, buf, len, 0);
}

uint16_t eeprom_getPanid(void)
{
    return 0xCCCC;
}

uint16_t eeprom_getPanaddr(void)
{
    return 0;
}

