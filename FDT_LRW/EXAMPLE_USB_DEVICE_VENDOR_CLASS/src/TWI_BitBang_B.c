/*
 * TWI_BitBang_B.c
 *
 * Created: 12/2/2017 8:20:02 PM
 *  Author: doug
 */ 
#include "TWI_Light.h"
#include "TWI_BitBang_B.h"
//#include "ProxTest.h"

/*! \brief initialize twi master mode
 */ 
void twi_init_B()
{
// 	DDR_SCL |= (1 << SCL);
// 	DDR_SDA |= (1 << SDA);
	ioport_set_pin_dir(SDA_L, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(SCL_L, IOPORT_DIR_OUTPUT);

	write_sda_B(1);
        write_scl_B(1);
	
} 

/*! \brief disables twi master mode
 */
void twi_disable_B()
{
// 	DDR_SCL &= ~(1 << SCL);
// 	DDR_SDA &= ~(1 << SDA);
	ioport_set_pin_dir(SDA_L, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(SCL_L, IOPORT_DIR_INPUT);

}

/*! \brief Sends start condition
 */
char twi_start_cond_B(void)
{
        write_sda_B(0);
	delay_us(DELAY);
	write_scl_B(0);	
	delay_us(DELAY);
	return 1;
    
}

/*! \brief Sends slave address
 */
char send_slave_address_B(unsigned char read)
{
 	return i2c_write_byte_B(SLAVE_ADDRESS_B | read );
} 
 
/*! \brief Writes data from buffer.
    \param indata Pointer to data buffer
    \param bytes  Number of bytes to transfer
    \return 1 if successful, otherwise 0
 */

char write_data_B(unsigned char* indata, char bytes)
{
	unsigned char index, ack = 0;
	
	if(!twi_start_cond_B())
		return 0;
	if(!send_slave_address_B(WRITE))
		return 0;	
	
	for(index = 0; index < bytes; index++)
	{
		 ack = i2c_write_byte_B(indata[index]);
		 if(!ack)
			break;		
	}
	//put stop here
	write_scl_B(1);
	delay_us(SCL_SDA_DELAY);
	write_sda_B(1);
	return ack;
	
}

/*! \brief Writes a byte on TWI.
    \param byte Data 
    \return 1 if successful, otherwise 0
 */
char i2c_write_byte_B(unsigned char byte)
{
        char bit;
	for (bit = 0; bit < 8; bit++) 
	{
            write_sda_B((byte & 0x80) != 0);
            delay_us(DELAY);
            toggle_scl_B();//goes high
            delay_us(DELAY);
            toggle_scl_B();//goes low
            byte <<= 1;
            delay_us(DELAY);
        }
	//release SDA
	SET_SDA_IN_B();
	toggle_scl_B(); //goes high for the 9th clock
	delay_us(4);
	//Check for acknowledgment
	if(READ_SDA_B())
	{
		return 0;			
	}
	delay_us(DELAY);
	//Pull SCL low
	toggle_scl_B(); //end of byte with acknowledgment. 
	//take SDA
	SET_SDA_OUT_B();
	delay_us(DELAY); 
	return 1;
		
}	
/*! \brief Reads data into buffer.
    \param data Pointer to data buffer
    \param bytes  Number of bytes to read
    \return 1 if successful, otherwise 0
 */
char read_bytes_B(unsigned char* data, char bytes)
{
	unsigned char index,success = 0;
	if(!twi_start_cond_B())
		return 0;
	if(!send_slave_address_B(READ))
		return 0;	
	for(index = 0; index < bytes; index++)
	{
		success = i2c_read_byte_B(data, bytes, index);//db: always returns 1?
		if(!success)
			break; 
	}
	//put stop here
	write_scl_B(1);
	delay_us(SCL_SDA_DELAY);
	write_sda_B(1);
	return success;
	
	
}	

/*! \brief Reads one byte into buffer.
    \param rcvdata Pointer to data buffer
    \param bytes  Number of bytes to read
    \param index Position of the incoming byte in hte receive buffer 
    \return 1 if successful, otherwise 0
 */
char i2c_read_byte_B(unsigned char* rcvdata, unsigned char bytes, unsigned char index)
{
        unsigned char byte = 0;
	unsigned char bit = 0;
	//release SDA
	SET_SDA_IN_B();
	for (bit = 0; bit < 8; bit++) 
	{
             toggle_scl_B();//goes high
             if(READ_SDA_B())
                     byte|= (1 << (7- bit));
              delay_us(DELAY);
              toggle_scl_B();//goes low
              delay_us(DELAY);
        }
	rcvdata[index] = byte;
	//take SDA
	SET_SDA_OUT_B();
	if(index < (bytes-1))
	{
		write_sda_B(0);
		toggle_scl_B(); //goes high for the 9th clock
		delay_us(DELAY);
		//Pull SCL low
		toggle_scl_B(); //end of byte with acknowledgment. 
		//release SDA
		write_sda_B(1);
		delay_us(DELAY);
	}
	else //send NACK on the last byte
	{
		write_sda_B(1);
		toggle_scl_B(); //goes high for the 9th clock
		delay_us(DELAY);
		//Pull SCL low
		toggle_scl_B(); //end of byte with acknowledgment. 
		//release SDA
		delay_us(DELAY);
	}		
	return 1;
		
}	
	
/*! \brief Writes SCL.
    \param x tristates SCL when x = 1, other wise 0
 */
void write_scl_B (char x)
{
      if(x)
      {
//             DDR_SCL &= ~(1 << SCL); //tristate it
		ioport_set_pin_dir(SCL_L, IOPORT_DIR_INPUT);
 	    //check clock stretching
 	    while(!READ_SCL_B());
      }
      else
      {
//             DDR_SCL |= (1 << SCL); //output 
//             PORT_SCL &= ~(1 << SCL); //set it low
			ioport_set_pin_dir(SCL_L, IOPORT_DIR_OUTPUT);
			pio_set_pin_low(SCL_L);
            
      }
}

/*! \brief Writes SDA.
    \param x tristates SDA when x = 1, other wise 0
 */
void write_sda_B (char x)
{
	if(x)
	{
		///           DDR_SDA &= ~(1 << SDA); //tristate it
		ioport_set_pin_dir(SDA_L, IOPORT_DIR_INPUT);
	}
	else
	{
		///            DDR_SDA |= (1 << SDA); //output
		///            PORT_SDA &= ~(1 << SDA); //set it low
		ioport_set_pin_dir(SDA_L, IOPORT_DIR_OUTPUT);
		pio_set_pin_low(SDA_L);
		
	}
}

void toggle_scl_B() 
{
	if(ioport_get_pin_level(SCL_L))
	{
// 		DDR_SCL |= (1 << SCL); //output
// 		PORT_SCL &= ~(1 << SCL); //set it low
		ioport_set_pin_dir(SCL_L, IOPORT_DIR_OUTPUT);
		pio_set_pin_low(SCL_L);
	}
	else
	{
// 		DDR_SCL &= ~(1 << SCL); //tristate it
		ioport_set_pin_dir(SCL_L, IOPORT_DIR_INPUT);
 		while(!READ_SCL_B());
 	}	
}