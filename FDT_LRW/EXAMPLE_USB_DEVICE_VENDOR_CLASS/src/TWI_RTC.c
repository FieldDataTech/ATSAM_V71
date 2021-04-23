#include "TWI_RTC.h"
//#include "ProxTest.h"

/*! \brief initialize twi master mode
 */ 
void twi_init_RTC()
{
	ioport_set_pin_dir(SDA_RTC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(SCL_RTC, IOPORT_DIR_OUTPUT);

	write_SDA_RTC(1);
    write_SCL_RTC(1);
	
} 

/*! \brief disables twi master mode
 */
void twi_disable_RTC()
{
	ioport_set_pin_dir(SDA_RTC, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(SCL_RTC, IOPORT_DIR_INPUT);

}

/*! \brief Sends start condition
 */
char twi_start_cond_RTC(void)
{
    write_SDA_RTC(0);
	delay_us(DELAY_RTC);
	write_SCL_RTC(0);	
	delay_us(DELAY_RTC);
	return 1;
    
}

/*! \brief Sends slave address
 */
char send_slave_address_RTC(unsigned char read)
{
 	return i2c_write_byte_RTC(SLAVE_ADDRESS_RTC | read );
} 
 
/*! \brief Writes data from buffer.
    \param indata Pointer to data buffer
    \param bytes  Number of bytes to transfer
    \return 1 if successful, otherwise 0
 */

char write_data_RTC(unsigned char* indata, char bytes)
{
	unsigned char index, ack = 0;
	
	if(!twi_start_cond_RTC())
		return 0;
	if(!send_slave_address_RTC(WRITE_RTC))
		return 0;	
	
	for(index = 0; index < bytes; index++)
	{
		 ack = i2c_write_byte_RTC(indata[index]);
		 if(!ack)
			break;		
	}
	//put stop here
	write_SCL_RTC(1);
	delay_us(SCL_SDA_DELAY_RTC);
	write_SDA_RTC(1);
	return ack;
	
}

char i2cAddressTest_RTC(char addrToTest)
{
	unsigned char index;

char writeRet,readRet;
char data[9];	

	if(!twi_start_cond_RTC())
	return 0;
	if(!i2c_write_byte_RTC(addrToTest | WRITE_RTC )) {
		return 0;
	 }else{
		return addrToTest;
		}
}

char i2cAddressTestRd_RTC(char addrToTest)
{
	unsigned char index;

	char data[9];

	if(!twi_start_cond_RTC())
	return 0;
	if(!i2c_write_byte_RTC(addrToTest | READ_RTC )) {
		return 0;
		}else{
		return addrToTest;
	}
}

/*! \brief Writes a byte on TWI.
    \param byte Data 
    \return 1 if successful, otherwise 0
 */
char i2c_write_byte_RTC(unsigned char byte)
{
        char bit;
	for (bit = 0; bit < 8; bit++) 
	{
            write_SDA_RTC((byte & 0x80) != 0);
            delay_us(DELAY_RTC);
            toggle_SCL_RTC();//goes high
            delay_us(DELAY_RTC);
            toggle_SCL_RTC();//goes low
            byte <<= 1;
            delay_us(DELAY_RTC);
        }
	//release SDA
	SET_SDA_IN_RTC();
	toggle_SCL_RTC(); //goes high for the 9th clock
	delay_us(16);
	//Check for acknowledgment
	if(READ_SDA_RTC())
	{
		return 0;	//FAILS because SDA is still high		
	}
	delay_us(DELAY_RTC);
	//Pull SCL low
	toggle_SCL_RTC(); //end of byte with acknowledgment. SCL LOW.
	//take SDA
	SET_SDA_OUT_RTC();
	delay_us(DELAY_RTC); 
	return 1;
		
}	
/*! \brief Reads data into buffer.
    \param data Pointer to data buffer
    \param bytes  Number of bytes to read
    \return 1 if successful, otherwise 0
 */
char read_bytes_RTC(unsigned char* data, char bytes)
{
	unsigned char index,success = 0;
	if(!twi_start_cond_RTC())
		return 0;
	if(!send_slave_address_RTC(READ_RTC))
		return 0;	
	for(index = 0; index < bytes; index++)
	{
		success = i2c_read_byte_RTC(data, bytes, index);//db: always returns 1?
		if(!success)
			break; 
	}
	//put stop here
	delay_us(SCL_SDA_DELAY_RTC);
	write_SDA_RTC(0);
	delay_us(SCL_SDA_DELAY_RTC);
	write_SCL_RTC(1);
	delay_us(SCL_SDA_DELAY_RTC);
	write_SDA_RTC(1);
	return success;
	
	
}	

/*! \brief Reads one byte into buffer.
    \param rcvdata Pointer to data buffer
    \param bytes  Number of bytes to read
    \param index Position of the incoming byte in hte receive buffer 
    \return 1 if successful, otherwise 0
 */
char i2c_read_byte_RTC(unsigned char* rcvdata, unsigned char bytes, unsigned char index)
{
        unsigned char byte = 0;
	unsigned char bit = 0;
	//release SDA
	SET_SDA_IN_RTC();
	for (bit = 0; bit < 8; bit++) 
	{
             toggle_SCL_RTC();//goes high
             if(READ_SDA_RTC())
                     byte|= (1 << (7- bit));
              delay_us(DELAY_RTC);
              toggle_SCL_RTC();//goes low
              delay_us(DELAY_RTC);
        }
	rcvdata[index] = byte;
	//take SDA
	SET_SDA_OUT_RTC();
	if(index < (bytes-1))
	{
		write_SDA_RTC(0);
		toggle_SCL_RTC(); //goes high for the 9th clock
		delay_us(DELAY_RTC);
		//Pull SCL low
		toggle_SCL_RTC(); //end of byte with acknowledgment. 
		//release SDA
		write_SDA_RTC(1);
		delay_us(DELAY_RTC);
	}
	else //send NACK on the last byte
	{
		write_SDA_RTC(1);
		toggle_SCL_RTC(); //goes high for the 9th clock
		delay_us(DELAY_RTC);
		//Pull SCL low
		toggle_SCL_RTC(); //end of byte with acknowledgment. 
		//release SDA
		delay_us(DELAY_RTC);
	}		
	return 1;
		
}	
	
/*! \brief Writes SCL.
    \param x tristates SCL when x = 1, other wise 0
 */
void write_SCL_RTC (char x)
{
      if(x)
      {
//             DDR_SCL &= ~(1 << SCL); //tristate it
		ioport_set_pin_dir(SCL_RTC, IOPORT_DIR_INPUT);
 	    //check clock stretching
 	    while(!READ_SCL_RTC());
      }
      else
      {
//             DDR_SCL |= (1 << SCL); //output 
//             PORT_SCL &= ~(1 << SCL); //set it low
			ioport_set_pin_dir(SCL_RTC, IOPORT_DIR_OUTPUT);
			pio_set_pin_low(SCL_RTC);
            
      }
}

/*! \brief Writes SDA.
    \param x tristates SDA when x = 1, other wise 0
 */
void write_SDA_RTC (char x)
{
	if(x)
	{
		///           DDR_SDA &= ~(1 << SDA); //tristate it
		ioport_set_pin_dir(SDA_RTC, IOPORT_DIR_INPUT);
	}
	else
	{
		///            DDR_SDA |= (1 << SDA); //output
		///            PORT_SDA &= ~(1 << SDA); //set it low
		ioport_set_pin_dir(SDA_RTC, IOPORT_DIR_OUTPUT);
		pio_set_pin_low(SDA_RTC);
		
	}
}

void toggle_SCL_RTC() 
{
	if(ioport_get_pin_level(SCL_RTC))
	{
// 		DDR_SCL |= (1 << SCL); //output
// 		PORT_SCL &= ~(1 << SCL); //set it low
		ioport_set_pin_dir(SCL_RTC, IOPORT_DIR_OUTPUT);
		pio_set_pin_low(SCL_RTC);
	}
	else
	{
// 		DDR_SCL &= ~(1 << SCL); //tristate it
		ioport_set_pin_dir(SCL_RTC, IOPORT_DIR_INPUT);
 		while(!READ_SCL_RTC());
 	}	
}