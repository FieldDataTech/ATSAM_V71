/* This file has been prepared for Doxygen automatic documentation generation.*/
#include "TWI_Light.h"

/*** LoRaWAN and Pi functions are in this file below the sensor functions ***/

/*! \brief initialize twi master mode
 */ 
void twi_init_L(void)
{
	SENS_SCL_HI;
	SENS_SDA_HI;
	SENSPWR_OUTPUT;
	SENSPWR_ON;
	SENS_SCL_FLOAT;
	SENS_SDA_FLOAT;
	delay_ms(10);
	SENS_SCL_LO;
	SENS_SDA_LO;
} 

/*! \brief disables twi master mode */
void twi_disable_L(void)
{
	SENS_SCL_FLOAT;
	SENS_SDA_FLOAT;
}

/*! \brief Sends start condition */
char twi_start_cond_L(void)
{
	SENS_SCL_FLOAT;
	SENS_SDA_DRIVE;
	delay_us(DELAY_L);
	SENS_SCL_DRIVE;
	delay_us(DELAY_L);
	return 1;
}

/*! \brief Sends stop condition */
char twi_stop_cond_L(void)
{
	SENS_SDA_DRIVE;
	delay_us(DELAY_L);
	SENS_SCL_FLOAT;
	delay_us(DELAY_L);
	return 1;
}

/*! \brief Sends slave address */
char send_slave_Address_L(unsigned char read, char slaveAddress)
{
 	return i2c_write_byte_L(slaveAddress | read, 0 );
} 
 
/*! \brief Writes data from buffer.
    \param indata Pointer to data buffer
    \param bytes  Number of bytes to transfer
    \return 1 if successful, otherwise 0
 */

char write_data_L(unsigned char* indata, char bytes, char slaveAddress, char stretch)
{
	unsigned char index, ack = 0;
	
	if(!twi_start_cond_L())
		return 0;
	if(!send_slave_Address_L(WRITE_L, slaveAddress))
		return 0;	
	
	for(index = 0; index < bytes; index++)
	{
		if(index==(bytes-1))ack = i2c_write_byte_L(indata[index], stretch);
		 else ack = i2c_write_byte_L(indata[index], 0);
		 if(!ack)
			break;		
	}
	//STOP
	if(stretch==0)SENS_SCL_FLOAT;
	delay_us(SCL_SDA_DELAY_L);
	SENS_SDA_FLOAT;
	return ack;
	
}

char i2cAddressTest_L(char addrToTest)
{
	unsigned char index;

char timeout, writeRet,readRet;

	if(!twi_start_cond_L())
	return 0;
	if(!i2c_write_byte_L(addrToTest | WRITE_L, 0 )) {
		return 0;
	 }else{
		return addrToTest;
		}
}

char i2cAddressTestRd_L(char addrToTest)
{
	unsigned char index;

	char timeout, writeRet,readRet;

	if(!twi_start_cond_L())
	return 0;
	if(!i2c_write_byte_L(addrToTest | READ_L, 0 )) {
		return 0;
		}else{
		return addrToTest;
	}
}

/*! \brief Writes a byte on TWI.
    \param byte Data 
    \return 1 if successful, otherwise 0
 */
char i2c_write_byte_L(unsigned char byte, char stretch)
{
    char bit;
	for (bit = 0; bit < 8; bit++) 
	{
            write_sda_L((byte & 0x80) != 0);
            delay_us(DELAY_L);
            SENS_SCL_FLOAT;//goes high
            delay_us(DELAY_L);
            SENS_SCL_DRIVE;//goes low
            byte <<= 1;
            delay_us(DELAY_L);
        }
	//release SDA
	SENS_SDA_FLOAT;
	SENS_SCL_FLOAT; //goes high for the 9th clock
    delay_us(DELAY_L);
	delay_us(4);
	//Check for acknowledgment
	if(READ_SDA_L())
	{
		return 0;	//If no ACK from slave, return 0.			
	}
	//Pull SCL low
	SENS_SCL_DRIVE; //end of byte with acknowledgment. 
	SENS_SDA_DRIVE;
	delay_us(DELAY_L);
	if(stretch)delay_ms(1);
	return 1;
		
}	
/*! \brief Reads data into buffer.
    \param data Pointer to data buffer
    \param bytes  Number of bytes to read
    \return 1 if successful, otherwise 0
 */
char read_bytes_L(unsigned char* data, char bytes, char slaveAddress)
{
	unsigned char index,success = 0;
	if(!twi_start_cond_L())
		return 0;
	if(!send_slave_Address_L(READ_L, slaveAddress))
		return 0;	
	for(index = 0; index < bytes; index++)
	{
		success = i2c_read_byte_L(data, bytes, index);//db: always returns 1?
		if(!success)
			break; 
	}
	twi_stop_cond_L();
	return success;
}	

/*! \brief Reads one byte into buffer.
    \param rcvdata Pointer to data buffer
    \param bytes  Number of bytes to read
    \param index Position of the incoming byte in hte receive buffer 
    \return 1 if successful, otherwise 0
 */
char i2c_read_byte_L(unsigned char* rcvdata, unsigned char bytes, unsigned char index)
{
    unsigned char byte = 0;
	unsigned char bit = 0;
	//release SDA
	SENS_SDA_FLOAT;
	for (bit = 0; bit < 8; bit++) 
	{
           SENS_SCL_FLOAT;//goes high
           delay_us(DELAY_L);
           if(READ_SDA_L())
                     byte|= (1 << (7- bit));
			SENS_SCL_DRIVE;//goes low
//			if(bit!=7)	
			delay_us(DELAY_L);
        }
	rcvdata[index] = byte;
	if(index < (bytes-1))//if not the last byte, ACK the slave
	{
		SENS_SDA_DRIVE;	//ACK
		SENS_SCL_FLOAT; //SCL high for 9th clock
		delay_us(DELAY_L);
		SENS_SCL_DRIVE; //SCL low after 9th clock
		delay_us(DELAY_L);
		SENS_SDA_FLOAT;//release SDA after ACK
	}
	else //send NACK on the last byte
	{
		SENS_SDA_FLOAT; //NACK
		SENS_SCL_FLOAT; //SCL high for the 9th clock
		delay_us(DELAY_L);
		//Pull SCL low
		SENS_SCL_DRIVE; //SCL low after 9th clock 
		//release SDA
		delay_us(DELAY_L);
		SENS_SCL_FLOAT; //goes high for the 9th clock
		delay_us(DELAY_L);
		while(!READ_SCL_L()){}
	}		
	return 1;
		
}	
	
/*! \brief Writes SCL.
    \param x tristates SCL when x = 1, other wise 0
 */
void write_scl_L (char x)
{
      if(x)
      {
		SENS_SCL_FLOAT;
 	    //check clock stretching
 	    while(!READ_SCL_L());
      }
      else
      {
	     SENS_SCL_DRIVE;
            
      }
}

/*! \brief Writes SDA.
    \param x tristates SDA when x = 1, other wise 0
 */
void write_sda_L (char x)
{
	if(x)
	{
		SENS_SDA_FLOAT;
	}
	else
	{
		SENS_SDA_DRIVE;
		
	}
}

void toggle_scl_L() 
{
	if(ioport_get_pin_level(SCL_L))
	{
		SENS_SCL_DRIVE;
	}
	else
	{
		SENS_SCL_FLOAT;
 		while(!READ_SCL_L());
 	}	
}
/****************************************************************************************/
/*****************************************************************************************/
/**************  LoRaWAN and Pi **********************************************************/
/****************************************************************************************/
/*****************************************************************************************/
/*! \brief initialize twi master mode
 */ 
void twi_init_c(void)
{
	LRWPI_SCL_HI;
	LRWPI_SDA_HI;
// 	SENSPWR_OUTPUT;
// 	SENSPWR_ON;
	LRWPI_SCL_FLOAT;
	LRWPI_SDA_FLOAT;
	delay_ms(10);
	LRWPI_SCL_LO;
	LRWPI_SDA_LO;
} 

/*! \brief disables twi master mode */
void twi_disable_LrwPi(void)
{
	LRWPI_SCL_FLOAT;
	LRWPI_SDA_FLOAT;
}

/*! \brief Sends start condition */
char twi_start_cond_LrwPi(void)
{
	LRWPI_SCL_FLOAT;
	LRWPI_SDA_DRIVE;
	delay_us(DELAY_L);
	LRWPI_SCL_DRIVE;
	delay_us(DELAY_L);
	return 1;
}

/*! \brief Sends stop condition */
char twi_stop_cond_LrwPi(void)
{
	LRWPI_SDA_DRIVE;
	delay_us(DELAY_L);
	LRWPI_SCL_FLOAT;
	delay_us(DELAY_L);
	return 1;
}

/*! \brief Sends slave address */
char send_slave_Address_LrwPi(unsigned char read, char slaveAddress)
{
 	return i2c_write_byte_LrwPi(slaveAddress | read, 0 );
} 
 
/*! \brief Writes data from buffer.
    \param indata Pointer to data buffer
    \param bytes  Number of bytes to transfer
    \return 1 if successful, otherwise 0
 */

char write_data_LrwPi(unsigned char* indata, char bytes, char slaveAddress, char stretch)
{
	unsigned char index, ack = 0;
	
	if(!twi_start_cond_LrwPi())
		return 0;
	if(!send_slave_Address_LrwPi(WRITE_LrwPi, slaveAddress))
		return 0;	
	
	for(index = 0; index < bytes; index++)
	{
		if(index==(bytes-1))ack = i2c_write_byte_LrwPi(indata[index], stretch);
		 else ack = i2c_write_byte_LrwPi(indata[index], 0);
		 if(!ack)
			break;		
	}
	//STOP
	if(stretch==0)SENS_SCL_FLOAT;
	delay_us(SCL_SDA_DELAY_L);
	LRWPI_SDA_FLOAT;
	return ack;
	
}

char i2cAddressTest_LrwPi(char addrToTest)
{
	unsigned char index;

char timeout, writeRet,readRet;

	if(!twi_start_cond_LrwPi())
	return 0;
	if(!i2c_write_byte_LrwPi(addrToTest | WRITE_LrwPi, 0 )) {
		return 0;
	 }else{
		return addrToTest;
		}
}

char i2cAddressTestRd_LrwPi(char addrToTest)
{
	unsigned char index;

	char timeout, writeRet,readRet;

	if(!twi_start_cond_LrwPi())
	return 0;
	if(!i2c_write_byte_LrwPi(addrToTest | READ_LrwPi, 0 )) {
		return 0;
		}else{
		return addrToTest;
	}
}

/*! \brief Writes a byte on TWI.
    \param byte Data 
    \return 1 if successful, otherwise 0
 */
char i2c_write_byte_LrwPi(unsigned char byte, char stretch)
{
    char bit;
	for (bit = 0; bit < 8; bit++) 
	{
            write_sda_LrwPi((byte & 0x80) != 0);
            delay_us(DELAY_L);
            LRWPI_SCL_FLOAT;//goes high
            delay_us(DELAY_L);
            LRWPI_SCL_DRIVE;//goes low
            byte <<= 1;
            delay_us(DELAY_L);
        }
	//release SDA
	LRWPI_SDA_FLOAT;
	LRWPI_SCL_FLOAT; //goes high for the 9th clock
    delay_us(DELAY_L);
	delay_us(4);
	//Check for acknowledgment
	if(READ_SDA_LrwPi())
	{
		return 0;	//If no ACK from slave, return 0.			
	}
	//Pull SCL low
	LRWPI_SCL_DRIVE; //end of byte with acknowledgment. 
	LRWPI_SDA_DRIVE;
	delay_us(DELAY_L);
	if(stretch)delay_ms(1);
	return 1;
		
}	
/*! \brief Reads data into buffer.
    \param data Pointer to data buffer
    \param bytes  Number of bytes to read
    \return 1 if successful, otherwise 0
 */
char read_bytes_LrwPi(unsigned char* data, char bytes, char slaveAddress)
{
	unsigned char index,success = 0;
	if(!twi_start_cond_LrwPi())
		return 0;
	if(!send_slave_Address_LrwPi(READ_LrwPi, slaveAddress))
		return 0;	
	for(index = 0; index < bytes; index++)
	{
		success = i2c_read_byte_LrwPi(data, bytes, index);//db: always returns 1?
		if(!success)
			break; 
	}
	twi_stop_cond_LrwPi();
	return success;
}	

/*! \brief Reads one byte into buffer.
    \param rcvdata Pointer to data buffer
    \param bytes  Number of bytes to read
    \param index Position of the incoming byte in hte receive buffer 
    \return 1 if successful, otherwise 0
 */
char i2c_read_byte_LrwPi(unsigned char* rcvdata, unsigned char bytes, unsigned char index)
{
    unsigned char byte = 0;
	unsigned char bit = 0;
	//release SDA
	LRWPI_SDA_FLOAT;
	for (bit = 0; bit < 8; bit++) 
	{
           LRWPI_SCL_FLOAT;//goes high
           delay_us(DELAY_L);
           if(READ_SDA_LrwPi())
                     byte|= (1 << (7- bit));
			LRWPI_SCL_DRIVE;//goes low
//			if(bit!=7)	
			delay_us(DELAY_L);
        }
	rcvdata[index] = byte;
	if(index < (bytes-1))//if not the last byte, ACK the slave
	{
		LRWPI_SDA_DRIVE;	//ACK
		LRWPI_SCL_FLOAT; //SCL high for 9th clock
		delay_us(DELAY_L);
		LRWPI_SCL_DRIVE; //SCL low after 9th clock
		delay_us(DELAY_L);
		LRWPI_SDA_FLOAT;//release SDA after ACK
	}
	else //send NACK on the last byte
	{
		LRWPI_SDA_FLOAT; //NACK
		LRWPI_SCL_FLOAT; //SCL high for the 9th clock
		delay_us(DELAY_L);
		//Pull SCL low
		LRWPI_SCL_DRIVE; //SCL low after 9th clock 
		//release SDA
		delay_us(DELAY_L);
		LRWPI_SCL_FLOAT; //goes high for the 9th clock
		delay_us(DELAY_L);
		while(!READ_SCL_LrwPi()){}
	}		
	return 1;
		
}	
	
/*! \brief Writes SCL.
    \param x tristates SCL when x = 1, other wise 0
 */
void write_scl_LrwPi (char x)
{
      if(x)
      {
		LRWPI_SCL_FLOAT;
 	    //check clock stretching
 	    while(!READ_SCL_LrwPi());
      }
      else
      {
	     LRWPI_SCL_DRIVE;
            
      }
}

/*! \brief Writes SDA.
    \param x tristates SDA when x = 1, other wise 0
 */
void write_sda_LrwPi (char x)
{
	if(x)
	{
		LRWPI_SDA_FLOAT;
	}
	else
	{
		LRWPI_SDA_DRIVE;
		
	}
}

void toggle_scl_LrwPi() 
{
	if(ioport_get_pin_level(SCL_LrwPi))
	{
		LRWPI_SCL_DRIVE;
	}
	else
	{
		LRWPI_SCL_FLOAT;
 		while(!READ_SCL_LrwPi());
 	}	
}