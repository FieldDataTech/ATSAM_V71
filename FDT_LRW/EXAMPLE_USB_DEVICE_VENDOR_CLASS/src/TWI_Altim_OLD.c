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
#include "TWI_Altim_OLD.h"
//#include "ProxTest.h"

/*! \brief initialize twi master mode
 */ 
void twi_init_A_OLD()
{
// 	DDR_SCL |= (1 << SCL);
// 	DDR_SDA |= (1 << SDA);
	ioport_set_pin_dir(SDA_A, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(SCL_A, IOPORT_DIR_OUTPUT);

	write_sda_A_OLD(1);
        write_scl_A_OLD(1);
	
} 

/*! \brief disables twi master mode
 */
void twi_disable_A_OLD()
{
// 	DDR_SCL &= ~(1 << SCL);
// 	DDR_SDA &= ~(1 << SDA);
	ioport_set_pin_dir(SDA_A, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(SCL_A, IOPORT_DIR_INPUT);

}

/*! \brief Sends start condition
 */
char twi_start_cond_A_OLD(void)
{
        write_sda_A_OLD(0);
	delay_us(DELAY_A_OLD);
	write_scl_A_OLD(0);	
	delay_us(DELAY_A_OLD);
	return 1;
    
}

/*! \brief Sends slave address
 */
char send_slave_address_A_OLD(unsigned char read)
{
 	return i2c_write_byte_A_OLD(SLAVE_ADDRESS_A_OLD | read );
} 
 
/*! \brief Writes data from buffer.
    \param indata Pointer to data buffer
    \param bytes  Number of bytes to transfer
    \return 1 if successful, otherwise 0
 */

char write_data_A_OLD(unsigned char* indata, char bytes)
{
	unsigned char index, ack = 0;
	
	if(!twi_start_cond_A_OLD())
		return 0;
	if(!send_slave_address_A_OLD(WRITE_A_OLD))
		return 0;	
	
	for(index = 0; index < bytes; index++)
	{
		 ack = i2c_write_byte_A_OLD(indata[index]);
		 if(!ack)
			break;		
	}
	//put stop here
	write_scl_A_OLD(1);
	delay_us(SCL_SDA_DELAY_A_OLD);
	write_sda_A_OLD(1);
	return ack;
	
}

char i2cAddressTest_A_OLD(char addrToTest)
{
	unsigned char index, ack = 0;

char timeout, writeRet,readRet;
char data[9];	

	if(!twi_start_cond_A_OLD())
	return 0;
	if(!i2c_write_byte_A_OLD(addrToTest | WRITE_A_OLD )) {
		return 0;
	 }else{
		return addrToTest;
		}
}

char i2cAddressTestRd_A_OLD(char addrToTest)
{
	unsigned char index, ack = 0;

	char timeout, writeRet,readRet;
	char data[9];

	if(!twi_start_cond_A_OLD())
	return 0;
	if(!i2c_write_byte_A_OLD(addrToTest | READ_A_OLD )) {
		return 0;
		}else{
		return addrToTest;
	}
}

/*! \brief Writes a byte on TWI.
    \param byte Data 
    \return 1 if successful, otherwise 0
 */
char i2c_write_byte_A_OLD(unsigned char byte)
{
        char bit;
	for (bit = 0; bit < 8; bit++) 
	{
            write_sda_A_OLD((byte & 0x80) != 0);
            delay_us(DELAY_A_OLD);
            toggle_scl_A_OLD();//goes high
            delay_us(DELAY_A_OLD);
            toggle_scl_A_OLD();//goes low
            byte <<= 1;
            delay_us(DELAY_A_OLD);
        }
	//release SDA
	SET_SDA_IN_A_OLD();
	toggle_scl_A_OLD(); //goes high for the 9th clock
	delay_us(4);
	//Check for acknowledgment
	if(READ_SDA_A_OLD())
	{
		return 0;			
	}
	delay_us(DELAY_A_OLD);
	//Pull SCL low
	toggle_scl_A_OLD(); //end of byte with acknowledgment. 
	//take SDA
	SET_SDA_OUT_A_OLD();
	delay_us(DELAY_A_OLD); 
	return 1;
		
}	
/*! \brief Reads data into buffer.
    \param data Pointer to data buffer
    \param bytes  Number of bytes to read
    \return 1 if successful, otherwise 0
 */
char read_bytes_A_OLD(unsigned char* data, char bytes)
{
	unsigned char index,success = 0;
	if(!twi_start_cond_A_OLD())
		return 0;
	if(!send_slave_address_A_OLD(READ_A_OLD))
		return 0;	
	for(index = 0; index < bytes; index++)
	{
		success = i2c_read_byte_A_OLD(data, bytes, index);//db: always returns 1?
		if(!success)
			break; 
	}
	//put stop here
	write_scl_A_OLD(1);
	delay_us(SCL_SDA_DELAY_A_OLD);
	write_sda_A_OLD(1);
	return success;
	
	
}	

/*! \brief Reads one byte into buffer.
    \param rcvdata Pointer to data buffer
    \param bytes  Number of bytes to read
    \param index Position of the incoming byte in hte receive buffer 
    \return 1 if successful, otherwise 0
 */
char i2c_read_byte_A_OLD(unsigned char* rcvdata, unsigned char bytes, unsigned char index)
{
        unsigned char byte = 0;
	unsigned char bit = 0;
	//release SDA
	SET_SDA_IN_A_OLD();
	for (bit = 0; bit < 8; bit++) 
	{
             toggle_scl_A_OLD();//goes high
             if(READ_SDA_A_OLD())
                     byte|= (1 << (7- bit));
              delay_us(DELAY_A_OLD);
              toggle_scl_A_OLD();//goes low
              delay_us(DELAY_A_OLD);
        }
	rcvdata[index] = byte;
	//take SDA
	SET_SDA_OUT_A_OLD();
	if(index < (bytes-1))
	{
		write_sda_A_OLD(0);
		toggle_scl_A_OLD(); //goes high for the 9th clock
		delay_us(DELAY_A_OLD);
		//Pull SCL low
		toggle_scl_A_OLD(); //end of byte with acknowledgment. 
		//release SDA
		write_sda_A_OLD(1);
		delay_us(DELAY_A_OLD);
	}
	else //send NACK on the last byte
	{
		write_sda_A_OLD(1);
		toggle_scl_A_OLD(); //goes high for the 9th clock
		delay_us(DELAY_A_OLD);
		//Pull SCL low
		toggle_scl_A_OLD(); //end of byte with acknowledgment. 
		//release SDA
		delay_us(DELAY_A_OLD);
	}		
	return 1;
		
}	
	
/*! \brief Writes SCL.
    \param x tristates SCL when x = 1, other wise 0
 */
void write_scl_A_OLD (char x)
{
      if(x)
      {
//             DDR_SCL &= ~(1 << SCL); //tristate it
		ioport_set_pin_dir(SCL_A, IOPORT_DIR_INPUT);
 	    //check clock stretching
 	    while(!READ_SCL_A_OLD());
      }
      else
      {
//             DDR_SCL |= (1 << SCL); //output 
//             PORT_SCL &= ~(1 << SCL); //set it low
			ioport_set_pin_dir(SCL_A, IOPORT_DIR_OUTPUT);
			pio_set_pin_low(SCL_A);
            
      }
}

/*! \brief Writes SDA.
    \param x tristates SDA when x = 1, other wise 0
 */
void write_sda_A_OLD (char x)
{
	if(x)
	{
		///           DDR_SDA &= ~(1 << SDA); //tristate it
		ioport_set_pin_dir(SDA_A, IOPORT_DIR_INPUT);
	}
	else
	{
		///            DDR_SDA |= (1 << SDA); //output
		///            PORT_SDA &= ~(1 << SDA); //set it low
		ioport_set_pin_dir(SDA_A, IOPORT_DIR_OUTPUT);
		pio_set_pin_low(SDA_A);
		
	}
}

void toggle_scl_A_OLD() 
{
	if(ioport_get_pin_level(SCL_A))
	{
// 		DDR_SCL |= (1 << SCL); //output
// 		PORT_SCL &= ~(1 << SCL); //set it low
		ioport_set_pin_dir(SCL_A, IOPORT_DIR_OUTPUT);
		pio_set_pin_low(SCL_A);
	}
	else
	{
// 		DDR_SCL &= ~(1 << SCL); //tristate it
		ioport_set_pin_dir(SCL_A, IOPORT_DIR_INPUT);
 		while(!READ_SCL_A_OLD());
 	}	
}