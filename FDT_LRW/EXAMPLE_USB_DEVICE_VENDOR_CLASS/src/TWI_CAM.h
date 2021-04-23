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

#ifndef TWI_MASTER_C_
#define TWI_MASTER_C_

#include "ioport.h"
#include "pio.h"
#include "pio_handler.h"
#include "delay.h"
//#include "Si114x_defs.h"
// #include "ioavr.h"
// #include "inavr.h"

/*! \brief Definition of pin used as SCL. */
#define SCL SCL0	//PC1

/*! \brief Definition of pin used as SDA. */
#define SDA SDA0	//PC4

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
//#define SLAVE_ADDRESS 0xA6//for CamCube (I think this worked).  for SI1153 Nov 2017 custom board (datasheet address shifted one bit left)
#define SLAVE_ADDRESS 0x60//for OV2655
//#define SLAVE_ADDRESS 0x6C//for OV2710

#define READ_SDA() ioport_get_pin_level(SDA) //(PIN_SDA & (1 << SDA))
#define SET_SDA_OUT() ioport_set_pin_dir(SDA, IOPORT_DIR_OUTPUT)  //DDR_SDA |= (1 << SDA)
#define SET_SDA_IN() ioport_set_pin_dir(SDA, IOPORT_DIR_INPUT)	//DDR_SDA &= ~(1 << SDA)
#define READ_SCL() ioport_get_pin_level(SCL)  //(PIN_SCL & (1 << SCL))?1:0

#define WRITE 0x0
#define READ 0x1

/*! \brief Delay used to generate clock */
#define DELAY 1  //datasheet SI1153 up to 400KHz clock, doesn't work at 0, 1 does 230KHz
//nov 2017 works with DELAY=1 at 230KHz. DOesn't work at 0.  Going with 2 for margin
/*! \brief Delay used for STOP condition */
#define SCL_SDA_DELAY 1

void twi_disable();
void twi_init();
void toggle_scl();
void write_scl(char x);
char twi_start_cond(void);
char send_slave_address(unsigned char read);
char write_data(unsigned char* data, char bytes);
char i2c_write_byte(unsigned char byte);
char read_bytes(unsigned char* data, char bytes);
char i2c_read_byte(unsigned char* data, unsigned char bytes, unsigned char index);
void write_sda( char x);

char i2cAddressTest(char);
char i2cAddressTestRd(char);


#endif /* TWI_MASTER_H_ */