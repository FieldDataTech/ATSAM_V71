/*
 * TWI_BitBang_B.h
 *
 * Created: 12/2/2017 8:20:44 PM
 *  Author: doug
 */ 


#ifndef TWI_BITBANG_B_H_
#define TWI_BITBANG_B_H_

//#ifndef TWI_MASTER_H_
//#define TWI_MASTER_H_

#include "ioport.h"
#include "pio.h"
#include "pio_handler.h"
#include "delay.h"
//#include "Si114x_defs.h"
// #include "ioavr.h"
// #include "inavr.h"

/*! \brief Definition of pin used as SCL. */
#define SCL_B SCL0_B	//PA4 for camera and RTC

/*! \brief Definition of pin used as SDA. */
#define SDA_B SDA0_B	//PA3 for camera and RTC

/*! \brief Definition of PORT used as SCL. */
//#define PORT_SCL PORTC  //db: covered by ioport function
/*! \brief Definition of DDR used as SCL. */
//#define DDR_SCL	DDRC  //db: covered by ioport function
/*! \brief Definition of PIN used as SCL. */
//#define PIN_SCL 	//db: was PINC, which appears to be the register for PORTC_PIN
/*! \brief Definition of PORT used as SDA. */
//#define PORT_SDA PORTC //db: covered by ioport function
/*! \brief Definition of DDR used as SDA. */
//#define DDR_SDA	DDRC  //db: covered by ioport function
/*! \brief Definition of PIN used as SDA. */
//#define PIN_SDA 	//db: was PINC, which appears to be the register for PORTC_PIN

/*! \brief Slave 8 bit address (shifted). */
//for demo board? #define SLAVE_ADDRESS 0xB4//5A	//Sept 2 definitely works at B4 and not 5A like the datasheet error
#define SLAVE_ADDRESS_B 0x20//for extn RTC P85063 Nov 2017 custom board (datasheet address shifted one bit left)

#define READ_SDA_B(void) ioport_get_pin_level(SDA_L) //(PIN_SDA & (1 << SDA))
#define SET_SDA_OUT_B(void) ioport_set_pin_dir(SDA_L, IOPORT_DIR_OUTPUT)  //DDR_SDA |= (1 << SDA)
#define SET_SDA_IN_B(void) ioport_set_pin_dir(SDA_L, IOPORT_DIR_INPUT)	//DDR_SDA &= ~(1 << SDA)
#define READ_SCL_B(void) ioport_get_pin_level(SCL_L)  //(PIN_SCL & (1 << SCL))?1:0

#define WRITE 0x0
#define READ 0x1

/*! \brief Delay used to generate clock */
#define DELAY 20  //datasheet SI1153 up to 400KHz clock, doesn't work at 0, 1 does 230KHz
//nov 2017 works with DELAY=1 at 230KHz. DOesn't work at 0.  Going with 2 for margin
/*! \brief Delay used for STOP condition */
#define SCL_SDA_DELAY 15

void twi_disable_B(void);
void twi_init_B(void);
void toggle_scl_B(void);
void write_scl_B(char x);
char twi_start_cond_B(void);
char send_slave_address_B(unsigned char read);
char write_data_B(unsigned char* data, char bytes);
char i2c_write_byte_B(unsigned char byte);
char read_bytes_B(unsigned char* data, char bytes);
char i2c_read_byte_B(unsigned char* data, unsigned char bytes, unsigned char index);
void write_sda_B( char x);


//#endif /* TWI_MASTER_H_ */



#endif /* TWI_BITBANG_B_H_ */