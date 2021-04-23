/* This file has been prepared for Doxygen automatic documentation generation.*/

#ifndef TWI_MASTER_L_
#define TWI_MASTER_L_

#include "ioport.h"
#include "pio.h"
#include "pio_handler.h"
#include "delay.h"

#define SENS_SCL_HI 		ioport_set_pin_level(SENS_CLK, 1);
#define SENS_SDA_HI 		ioport_set_pin_level(SENS_SDA, 1);
#define SENS_SCL_LO 		ioport_set_pin_level(SENS_CLK, 0);
#define SENS_SDA_LO 		ioport_set_pin_level(SENS_SDA, 0);
#define SENS_SCL_FLOAT 		ioport_set_pin_dir(SENS_CLK, IOPORT_DIR_INPUT);
#define SENS_SDA_FLOAT 		ioport_set_pin_dir(SENS_SDA, IOPORT_DIR_INPUT);
#define SENS_SCL_DRIVE 		ioport_set_pin_dir(SENS_CLK, IOPORT_DIR_OUTPUT);
#define SENS_SDA_DRIVE 		ioport_set_pin_dir(SENS_SDA, IOPORT_DIR_OUTPUT);
#define SENSPWR_ON		ioport_set_pin_level(SENS_PWRC, 0)
#define SENSPWR_OFF    	ioport_set_pin_level(SENS_PWRC, 1)
#define SENSPWR_OUTPUT 	ioport_set_pin_dir(SENS_PWRC, IOPORT_DIR_OUTPUT)

#define SCL_L SCL0	//PC1
#define SDA_L SDA0	//PC4

#define SLAVE_ADDRESS_L 0x20	//0x20 = Light. 0x80  = Humidity.  0xEE = Altimeter

#define READ_SDA_L() ioport_get_pin_level(SDA_L) //(PIN_SDA & (1 << SDA))
#define SET_SDA_OUT_L() ioport_set_pin_dir(SDA_L, IOPORT_DIR_OUTPUT)  //DDR_SDA |= (1 << SDA)
#define SET_SDA_IN_L() ioport_set_pin_dir(SDA_L, IOPORT_DIR_INPUT)	//DDR_SDA &= ~(1 << SDA)
#define READ_SCL_L() ioport_get_pin_level(SCL_L)  //(PIN_SCL & (1 << SCL))?1:0

#define WRITE_L 0x00
#define READ_L 0x01

/*! \brief Delay used to generate clock */
#define DELAY_L 24  //increasing speed to 1 does not appear to degrade, but makes no diff in total on time because the 100ms warmup swamps it.

/*! \brief Delay used for STOP condition */
#define SCL_SDA_DELAY_L 12

void twi_disable_L(void);
void twi_init_L(void);
void toggle_scl_L(void);
void write_scl_L(char x);
char twi_start_cond_L(void);
char twi_stop_cond_L(void);
char send_slave_address_L(unsigned char, char);
char write_data_L(unsigned char*, char, char, char);
char i2c_write_byte_L(unsigned char, char);
char read_bytes_L(unsigned char*, char, char);
char i2c_read_byte_L(unsigned char*, unsigned char, unsigned char);
void write_sda_L( char x);

/*****************************************************************************************/
/**************  LoRaWAN and Pi **********************************************************/
/****************************************************************************************/
void twi_disable_LrwPi(void);
void twi_init_LrwPi(void);
void toggle_scl_LrwPi(void);
void write_scl_LrwPi(char x);
char twi_start_cond_LrwPi(void);
char twi_stop_cond_LrwPi(void);
char send_slave_address_LrwPi(unsigned char, char);
char write_data_LrwPi(unsigned char*, char, char, char);
char i2c_write_byte_LrwPi(unsigned char, char);
char read_bytes_LrwPi(unsigned char*, char, char);
char i2c_read_byte_LrwPi(unsigned char*, unsigned char, unsigned char);
void write_sda_LrwPi( char x);

char i2cAddressTest_LrwPi(char);
char i2cAddressTestRd_LrwPi(char);

#define LRWPI_SCL_HI 		ioport_set_pin_level(LRWPI_CLK, 1);
#define LRWPI_SDA_HI 		ioport_set_pin_level(LRWPI_SDA, 1);
#define LRWPI_SCL_LO 		ioport_set_pin_level(LRWPI_CLK, 0);
#define LRWPI_SDA_LO 		ioport_set_pin_level(LRWPI_SDA, 0);
#define LRWPI_SCL_FLOAT 		ioport_set_pin_dir(LRWPI_CLK, IOPORT_DIR_INPUT);
#define LRWPI_SDA_FLOAT 		ioport_set_pin_dir(LRWPI_SDA, IOPORT_DIR_INPUT);
#define LRWPI_SCL_DRIVE 		ioport_set_pin_dir(LRWPI_CLK, IOPORT_DIR_OUTPUT);
#define LRWPI_SDA_DRIVE 		ioport_set_pin_dir(LRWPI_SDA, IOPORT_DIR_OUTPUT);
// #define SENSPWR_ON		ioport_set_pin_level(SENS_PWRC, 0)
// #define SENSPWR_OFF    	ioport_set_pin_level(SENS_PWRC, 1)
// #define SENSPWR_OUTPUT 	ioport_set_pin_dir(SENS_PWRC, IOPORT_DIR_OUTPUT)

#define SCL_LrwPi SCL0	//PC1
#define SDA_LrwPi SDA0	//PC4

#define SLAVE_ADDRESS_LrwPi 0x20	//0x20 = Light. 0x80  = Humidity.  0xEE = Altimeter

#define READ_SDA_LrwPi() ioport_get_pin_level(SDA_L) //(PIN_SDA & (1 << SDA))
#define SET_SDA_OUT_LrwPi() ioport_set_pin_dir(SDA_L, IOPORT_DIR_OUTPUT)  //DDR_SDA |= (1 << SDA)
#define SET_SDA_IN_LrwPi() ioport_set_pin_dir(SDA_L, IOPORT_DIR_INPUT)	//DDR_SDA &= ~(1 << SDA)
#define READ_SCL_LrwPi() ioport_get_pin_level(SCL_L)  //(PIN_SCL & (1 << SCL))?1:0

#define WRITE_LrwPi 0x00
#define READ_LrwPi 0x01

#endif /* TWI_MASTER_L_ */