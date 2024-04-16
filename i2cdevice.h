/* ========================================
 *
 *	i2cdevice.h
 *		description: TouchMidi I2C Device Driver
 *
 *	Copyright(c)2018- Masahiko Hasebe at Kigakudoh
 *  This software is released under the MIT License, see LICENSE.txt
 *
 * ========================================
*/
#ifndef I2CDEVICE_H
#define I2CDEVICE_H

void wireBegin( void );
void initHardware( void );

// USE_CY8CMBR3110
  void MBR3110_resetAll(int maxdevNum);
  int MBR3110_init( int number=0 );
  int MBR3110_originalAdrsExist(void);
  int MBR3110_setup( int number=0 );
  int MBR3110_readData( unsigned char cmd, unsigned char* data, int length, unsigned char i2cAdrs );
  int MBR3110_selfTest( unsigned char* result, int number );
  void MBR3110_changeSensitivity( unsigned char data, int number=0 );
  int MBR3110_readTouchSw( unsigned char* touchSw, int number=0 );
  int MBR3110_checkWriteConfig( unsigned char checksumL, unsigned char checksumH, unsigned char crntI2cAdrs );
  int MBR3110_writeConfig( int number, unsigned char crntI2cAdrs );

// USE_ADA88
	void ada88_init( void );
	void ada88_write( int letter );
  void ada88_writeBit( uint16_t num );
	void ada88_writeNumber( int num );
  void ada88_write_5param(uint8_t prm1, uint8_t prm2, uint8_t prm3, uint8_t prm4, uint8_t prm5);
  void ada88_write_bit(uint16_t num);
  void ada88_write_org_bit(bool* bit);


// USE_PCA9685
	void PCA9685_init( uint8_t chipNumber );
  int PCA9685_write( uint8_t chipNumber, uint8_t cmd1, uint8_t cmd2 );
	int PCA9685_setFullColorLED( uint8_t chipNumber, int ledNum, unsigned short* color  );

#endif
