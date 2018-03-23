#ifndef EEPROM_H
#define EEPROM_H

unsigned char EEPROM_read(unsigned int uiAddress);
void EEPROM_write(unsigned int uiAddress, unsigned char ucData);

/**************EECR*************  
 * The EEPROM Control Register
 *****************************/
#define EERE  0
#define EEPE  1
#define EEMPE 2
#define EERIE 3
#define EEPM0 4
#define EEPM1 5

#endif