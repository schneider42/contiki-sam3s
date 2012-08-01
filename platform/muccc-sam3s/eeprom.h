#ifndef EEPROM_H_
#define EEPROM_H_
#include "board.h"
#include <stdint.h>

void eeprom_init(void);
void eeprom_writeBytes(uint32_t address, uint8_t *buf, uint32_t len);
void eeprom_readBytes(uint32_t address, uint8_t *buf, uint32_t len);
uint8_t * eeprom_getEUI48(void);
uint8_t * eeprom_getEUI64(void);
uint16_t eeprom_getPanid(void);
uint16_t eeprom_getPanaddr(void);

#endif
