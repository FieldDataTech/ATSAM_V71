/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  Bit bang TWI master driver.
 *
 *      This file contains the function prototypes and enumerator definitions
 *      for various configuration parameters for the AVR TWI master driver.
 *
 *      The driver is not intended for size and/or speed critical code, since
 *      most functions are just a few lines of code, and the function call
 *      overhead would decrease code performance. The driver is intended for
 *      rapid prototyping and documentation purposes for getting started with
 *      the AVR TWI master.
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 * 
 * $Date: 2012-06-01 13:03:43 $  \n
 *
 * Copyright (c) 2012, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: gary.grewal
 *****************************************************************************/

#ifndef TWI_MASTER_RTC_
#define TWI_MASTER_RTC_

#include "ioport.h"
#include "pio.h"
#include "pio_handler.h"
#include "delay.h"
//#include "Si114x_defs.h"
// #include "ioavr.h"
// #include "inavr.h"

/*! \brief Definition of pin used as SCL. */
//#define SCL_RTC SCL_RTC	//PC1

/*! \brief Definition of pin used as SDA. */
//#define SDA_RTC SDA0	//PC4

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
//#define SLAVE_ADDRESS_A 0xA6//for SI1153 Nov 2017 custom board (datasheet address shifted one bit left)
/*! \brief Slave 8 bit address (shifted). */
#define SLAVE_ADDRESS_RTC 0xA2	//0xA2 = RTC

#define READ_SDA_RTC(void) ioport_get_pin_level(SDA_RTC) //(PIN_SDA & (1 << SDA))
#define SET_SDA_OUT_RTC(void) ioport_set_pin_dir(SDA_RTC, IOPORT_DIR_OUTPUT)  //DDR_SDA |= (1 << SDA)
#define SET_SDA_IN_RTC(void) ioport_set_pin_dir(SDA_RTC, IOPORT_DIR_INPUT)	//DDR_SDA &= ~(1 << SDA)
#define READ_SCL_RTC(void) ioport_get_pin_level(SCL_RTC)  //(PIN_SCL & (1 << SCL))?1:0

#define WRITE_RTC 0x0
#define READ_RTC 0x1

/*! \brief Delay used to generate clock */
#define DELAY_RTC 20  //increasing speed to 1 does not appear to degrade, but makes no diff in total on time because the 100ms warmup swamps it.
		// Works with 0 and 0 at 142KHz.  1 and 1 is also 142KHz. 2 and 2 is 100KHz.

/*! \brief Delay used for STOP condition */
#define SCL_SDA_DELAY_RTC  20

void twi_disable_RTC(void);
void twi_init_RTC(void);
void toggle_SCL_RTC(void);
void write_SCL_RTC(char x);
char twi_start_cond_RTC(void);
char send_slave_address_RTC(unsigned char read);
char write_data_RTC(unsigned char* data, char bytes);
char i2c_write_byte_RTC(unsigned char byte);
char read_bytes_RTC(unsigned char* data, char bytes);
char i2c_read_byte_RTC(unsigned char* data, unsigned char bytes, unsigned char index);
void write_SDA_RTC( char x);

char i2cAddressTest_RTC(char);
char i2cAddressTestRd_RTC(char);


#endif /* TWI_MASTER_H_ */