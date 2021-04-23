/**
 * \file
 *
 * \brief User board definition template
 *
 */

 /* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

#include <conf_board.h>

// External oscillator settings.
// Uncomment and set correct values if external oscillator is used.

// External oscillator frequency
#define BOARD_XOSC_HZ          12000000
#define BOARD_FREQ_MAINCK_XTAL    (12000000UL)
#define BOARD_FREQ_MAINCK_BYPASS  (12000000UL)

// External oscillator type.
//!< External clock signal
#define BOARD_XOSC_TYPE        XOSC_TYPE_EXTERNAL
//!< 32.768 kHz resonator on TOSC
//#define BOARD_XOSC_TYPE        XOSC_TYPE_32KHZ
#define BOARD_FREQ_SLCK_XTAL      (32768UL)
#define BOARD_FREQ_SLCK_BYPASS    (32768UL)
//!< 0.4 to 16 MHz resonator on XTALS
//#define BOARD_XOSC_TYPE        XOSC_TYPE_XTAL

// External oscillator startup time
#define BOARD_XOSC_STARTUP_US  500000
/******************************* RTC INT WakeUp definition ******************************/
#define RTC_INT				IOPORT_CREATE_PIN(PIOA, 2)
/******************************* BitBang TWI definition ******************************/
//@{
#define SDA0                  IOPORT_CREATE_PIN(PIOC, 8)  //Altimeter
#define SDA0_ACTIVE            false
#define SDA0_INACTIVE         !SDA0_ACTIVE
#define SCL0                  IOPORT_CREATE_PIN(PIOC, 9)
#define SCL0_ACTIVE            false
#define SCL0_INACTIVE         !SCL0_ACTIVE

#define SDA0_B                  IOPORT_CREATE_PIN(PIOA, 3)  //Camera and RTC
#define SDA0_ACTIVE_B            false
#define SDA0_INACTIVE_B        !SDA0_ACTIVE_B
#define SCL0_B                  IOPORT_CREATE_PIN(PIOA, 4)
#define SCL0_ACTIVE_B            false
#define SCL0_INACTIVE_B         !SCL0_ACTIVE_B

#define SDA_RTC                  IOPORT_CREATE_PIN(PIOC, 10)  //Camera and RTC
#define SDA_RTC_ACTIVE            false
#define SDA_RTC_INACTIVE        !SDA_RTC_ACTIVE
#define SCL_RTC                  IOPORT_CREATE_PIN(PIOC, 13)
#define SCL_RTC_ACTIVE            false
#define SCL_RTC_INACTIVE         !SCL_RTC_ACTIVE

/******************************* TWI definition
 *********************************/
/** TWI interface for OV7740 */
#define OV7740_TWIHS  TWIHS0
/** TWI0 data pin */
//#define PIN_TWIHS0_TWD0                   {PIO_PA3A_TWD0, PIOA, ID_PIOA,  PIO_PERIPH_A, PIO_DEFAULT}

/** TWI0 clock pin */
//#define PIN_TWIHS0_TWCK0                  {PIO_PA4A_TWCK0, PIOA, ID_PIOA,  PIO_PERIPH_A, PIO_DEFAULT}

/** TWI0 Data pins definition */
#define TWIHS0_DATA_GPIO                 PIO_PA3_IDX
#define TWIHS0_DATA_FLAGS                (PIO_PERIPH_A | PIO_DEFAULT)
#define TWIHS0_DATA_MASK                 PIO_PA3
#define TWIHS0_DATA_PIO                  PIOA
#define TWIHS0_DATA_ID                   ID_PIOA
#define TWIHS0_DATA_TYPE                 PIO_PERIPH_A
#define TWIHS0_DATA_ATTR                 PIO_DEFAULT

/** TWI0 clock pins definition */
#define TWIHS0_CLK_GPIO                  PIO_PA4_IDX
#define TWIHS0_CLK_FLAGS                 (PIO_PERIPH_A | PIO_DEFAULT)
#define TWIHS0_CLK_MASK                  PIO_PA4
#define TWIHS0_CLK_PIO                   PIOA
#define TWIHS0_CLK_ID                    ID_PIOA
#define TWIHS0_CLK_TYPE                  PIO_PERIPH_A
#define TWIHS0_CLK_ATTR                  PIO_DEFAULT

/** TWI0 pins */
//#define PINS_TWIHS0                      PIN_TWI_TWD0, PIN_TWI_TWCK0

#define ID_BOARD_TWIHS		               ID_TWIHS0
#define BOARD_TWIHS			                 TWIHS0
#define BOARD_TWIHS_IRQn		             TWIHS0_IRQn
//@}
/*----------------------------------------------------------------------------*/
//! CAM POWER CONTROL
//@{
//#define PIN_CPWRC_ID		ID_PIOA
/** CAM PWDN */
//#define PIN_CPWDNC_ID		ID_PIOA

//#define PIN_ISI_D8  {PIO_PD27D_ISI_D8,  PIOD, ID_PIOD, PIO_PERIPH_D, PIO_PULLUP}
//#define PIN_ISI_D9  {PIO_PD28D_ISI_D9,  PIOD, ID_PIOD, PIO_PERIPH_D, PIO_PULLUP}

//#define BOARD_ISI_PCK   {PIO_PA24D_ISI_PCK, PIOA, ID_PIOA, PIO_PERIPH_D, PIO_DEFAULT}

//#define BOARD_ISI_PCK0  { PIO_PA6B_PCK0,  PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT }
#define BOARD_ISI_RST   { 1 << 13, PIOB, ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT }
#define BOARD_ISI_PWD   { 1 << 19,  PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT }

/** SD CARD PINS */
#define BOARD_SDRAM_ADDR     0x70000000
#define SDCARDPWRC_GPIO            (PIO_PD8_IDX)
#define SDCARDPWRC_POWER_ON			(IOPORT_PIN_LEVEL_HIGH)
#define SDCARDPWRC_POWER_OFF		(IOPORT_PIN_LEVEL_LOW)
/** HSMCI pins definition. */
/*! Number of slot connected on HSMCI interface */
#define SD_MMC_HSMCI_MEM_CNT            1
#define SD_MMC_HSMCI_SLOT_0_SIZE        4
/** HSMCI MCCDA pin definition. */
#define PIN_HSMCI_MCCDA_GPIO            (PIO_PA28_IDX)
#define PIN_HSMCI_MCCDA_FLAGS           (IOPORT_MODE_MUX_C)
/** HSMCI MCCK pin definition. */
#define PIN_HSMCI_MCCK_GPIO             (PIO_PA25_IDX)
#define PIN_HSMCI_MCCK_FLAGS            (IOPORT_MODE_MUX_D)
/** HSMCI MCDA0 pin definition. */
#define PIN_HSMCI_MCDA0_GPIO            (PIO_PA30_IDX)
#define PIN_HSMCI_MCDA0_FLAGS           (IOPORT_MODE_MUX_C)
/** HSMCI MCDA1 pin definition. */
#define PIN_HSMCI_MCDA1_GPIO            (PIO_PA31_IDX)
#define PIN_HSMCI_MCDA1_FLAGS           (IOPORT_MODE_MUX_C)
/** HSMCI MCDA2 pin definition. */
#define PIN_HSMCI_MCDA2_GPIO            (PIO_PA26_IDX)
#define PIN_HSMCI_MCDA2_FLAGS           (IOPORT_MODE_MUX_C)
/** HSMCI MCDA3 pin definition. */
#define PIN_HSMCI_MCDA3_GPIO            (PIO_PA27_IDX)
#define PIN_HSMCI_MCDA3_FLAGS           (IOPORT_MODE_MUX_C)



/** CAM ISI_PINS */
#define ISI_D0_GPIO        (PIO_PD22_IDX)
#define ISI_D1_GPIO        (PIO_PD21_IDX)
#define ISI_D2_GPIO        (PIO_PB3_IDX)
#define ISI_D3_GPIO        (PIO_PA9_IDX)
#define ISI_D4_GPIO        (PIO_PA5_IDX)
#define ISI_D5_GPIO        (PIO_PD11_IDX)
#define ISI_D6_GPIO        (PIO_PD12_IDX)
#define ISI_D7_GPIO        (PIO_PA27_IDX)

#define ISI_HSYNC_GPIO		(PIO_PD24_IDX)
#define ISI_VSYNC_GPIO		(PIO_PD25_IDX)
#define ISI_PCK_PIO			(PIO_PA24_IDX)
#define ISI_MCLK_GPIO		(PIO_PA21_IDX)
#define CPWRC_GPIO          (PIO_PA23_IDX)
#define CPWRC_POWER_ON		(IOPORT_PIN_LEVEL_HIGH)
#define CPWRC_POWER_OFF		(IOPORT_PIN_LEVEL_LOW)
#define CPWDN_GPIO          (PIO_PA10_IDX)
#define CPWDN_POWER_ON		(IOPORT_PIN_LEVEL_LOW)
#define CPWDN_POWER_OFF		(IOPORT_PIN_LEVEL_HIGH)

#define ISI_D0_FLAGS   (PIO_PERIPH_D | PIO_DEFAULT | PIO_PULLUP)
#define ISI_D1_FLAGS   (PIO_PERIPH_D | PIO_DEFAULT | PIO_PULLUP)
#define ISI_D2_FLAGS   (PIO_PERIPH_D | PIO_DEFAULT | PIO_PULLUP)
#define ISI_D3_FLAGS   (PIO_PERIPH_B | PIO_DEFAULT | PIO_PULLUP)
#define ISI_D4_FLAGS   (PIO_PERIPH_B | PIO_DEFAULT | PIO_PULLUP)
#define ISI_D5_FLAGS   (PIO_PERIPH_D | PIO_DEFAULT | PIO_PULLUP)
#define ISI_D6_FLAGS   (PIO_PERIPH_D | PIO_DEFAULT | PIO_PULLUP)
#define ISI_D7_FLAGS   (PIO_PERIPH_D | PIO_DEFAULT | PIO_PULLUP)

#define ISI_HSYNC_FLAGS   (PIO_PERIPH_D | PIO_DEFAULT | PIO_PULLUP)
#define ISI_VSYNC_FLAGS   (PIO_PERIPH_D | PIO_DEFAULT | PIO_PULLUP)
#define ISI_PCK_FLAGS (PIO_PERIPH_D | PIO_DEFAULT | PIO_PULLUP)
#define ISI_MCLK_FLAGS (PIO_PERIPH_B | PIO_DEFAULT | PIO_PULLUP)

#define SRAMPWRC_GPIO          (PIO_PA8_IDX)
#define SRAMPWRC_POWER_ON		(IOPORT_PIN_LEVEL_HIGH)
#define SRAMPWRC_POWER_OFF		(IOPORT_PIN_LEVEL_LOW)

//! TP0
#define TP0_GPIO            (PIO_PD7_IDX)
#define TP0_FLAGS           (0)
#define TP0_ACTIVE_LEVEL    (IOPORT_PIN_LEVEL_LOW)
#define TP0_INACTIVE_LEVEL  (IOPORT_PIN_LEVEL_HIGH)
#define PIN_TP0_ID			ID_PIOD

#define SENS_PWRC				(PIO_PA6_IDX) //Change to D3 for custom board
#define SENS_PWRC_FLAGS           (0)
#define SENS_PWRC_ACTIVE_LEVEL    (IOPORT_PIN_LEVEL_LOW)
#define SENS_PWRC_INACTIVE_LEVEL  (IOPORT_PIN_LEVEL_HIGH)
#define SENS_PWRC_ID				ID_PIOA

#define SENS_SDA					(PIO_PC8_IDX) //Change to D3 for custom board
#define SENS_SDA_FLAGS           (0)
#define SENS_SDA_ACTIVE_LEVEL    (IOPORT_PIN_LEVEL_LOW)
#define SENS_SDA_INACTIVE_LEVEL  (IOPORT_PIN_LEVEL_HIGH)
#define SENS_SDA_ID				ID_PIOC

#define SENS_CLK					(PIO_PC9_IDX) //Change to D3 for custom board
#define SENS_CLK_FLAGS           (0)
#define SENS_CLK_ACTIVE_LEVEL    (IOPORT_PIN_LEVEL_LOW)
#define SENS_CLK_INACTIVE_LEVEL  (IOPORT_PIN_LEVEL_HIGH)
#define SENS_CLK_ID				ID_PIOC

#define LRWPI_SDA					(PIO_PD28_IDX) //Change to D3 for custom board
#define LRWPI_SDA_FLAGS           (0)
#define LRWPI_SDA_ACTIVE_LEVEL    (IOPORT_PIN_LEVEL_LOW)
#define LRWPI_SDA_INACTIVE_LEVEL  (IOPORT_PIN_LEVEL_HIGH)
#define LRWPI_SDA_ID				ID_PIOD

#define LRWPI_CLK					(PIO_PC19_IDX) //Change to D3 for custom board
#define LRWPI_CLK_FLAGS           (0)
#define LRWPI_CLK_ACTIVE_LEVEL    (IOPORT_PIN_LEVEL_LOW)
#define LRWPI_CLK_INACTIVE_LEVEL  (IOPORT_PIN_LEVEL_HIGH)
#define LRWPI_CLK_ID				ID_PIOC

//! TINY COMMS
//@{
#define ToTinyWU            (PIO_PB2_IDX) //Change to D3 for custom board
#define ToTiny_FLAGS           (0)
#define PIN_WUToTiny_ID			ID_PIOB

#define BigTiny_CLK            (PIO_PD3_IDX) //Change to D3 for custom board
#define ToTiny_FLAGS           (0)
#define ToTiny_ACTIVE_LEVEL    (IOPORT_PIN_LEVEL_LOW)
#define ToTiny_INACTIVE_LEVEL  (IOPORT_PIN_LEVEL_HIGH)
#define PIN_CLKToTiny_ID			ID_PIOD

#define BigTiny_DAT            (PIO_PB0_IDX)
#define FromTiny_FLAGS           (0)
#define FromTiny_ACTIVE_LEVEL    (IOPORT_PIN_LEVEL_LOW)
#define FromTiny_INACTIVE_LEVEL  (IOPORT_PIN_LEVEL_HIGH)
#define PIN_FromTiny_ID			 ID_PIOB

#define ToBigWU             IOPORT_CREATE_PIN(PIOA, 1)  //WakeUp to Big from Tiny
#define ToBigWU_GPIO		(PIO_PA1_IDX) //
#define ToBigWU_FLAGS        (0)
#define PIN_ToBigWU_ID		ID_PIOA

#define CONSOLE_UART               UART3
#define CONSOLE_UART_ID            ID_UART3
/** USART1 pins definitions, PA21,PB4. */
//#define USART1_RXD_GPIO   PIO_PA21_IDX
//#define USART1_RXD_FLAGS  IOPORT_MODE_MUX_A
#define UART3_TXD_GPIO   PIO_PD30_IDX
#define UART3_TXD_FLAGS  IOPORT_MODE_MUX_A

/** UART2 pins definitions for XBee3, PD25,PBD26. */
#define UART4_RXD_GPIO   PIO_PD18_IDX
#define UART4_RXD_FLAGS  IOPORT_MODE_MUX_C
#define UART4_TXD_GPIO   PIO_PD19_IDX
#define UART4_TXD_FLAGS  IOPORT_MODE_MUX_C

#define PIN_USART0_SCK_IDX    (PIO_PB13_IDX)
#define PIN_USART0_SCK_FLAGS  (IOPORT_MODE_MUX_C)

/** USART0 pin CTS */
#define PIN_USART0_CTS_IDX    (PIO_PB2_IDX)
#define PIN_USART0_CTS_FLAGS  (IOPORT_MODE_MUX_C)

/** USART0 pin RTS */
#define PIN_USART0_RTS_IDX    (PIO_PB3_IDX)
#define PIN_USART0_RTS_FLAGS  (IOPORT_MODE_MUX_C)


//! \name LED definitions
//@{
#define LED_FLAGS           (0)
#define LED_ACTIVE_LEVEL    (IOPORT_PIN_LEVEL_LOW)
#define LED_INACTIVE_LEVEL  (IOPORT_PIN_LEVEL_HIGH)
#define LED_RED_PIN          LED_RED_GPIO
#define LED_RED_GPIO         (PIO_PD0_IDX)

#define LED_FLAGS           (0)
#define LED_ACTIVE_LEVEL    (IOPORT_PIN_LEVEL_LOW)
#define LED_INACTIVE_LEVEL  (IOPORT_PIN_LEVEL_HIGH)
#define LED_USER_PIN          LED_USER_GPIO
#define LED_USER_GPIO         (PIO_PD9_IDX)

/********** XBEE ****************************/
/*#define XBEE_USART               USART0
#define XBEE_USART_ID            ID_USART0
#define XBEE_USART_IRQn         USART0_IRQn
#define USART_Handler         USART0_Handler
#define XBEEreset_GPIO            (PIO_PD16_IDX)
#define XBEE_TX_GPIO            (PIO_PB1_IDX)
#define XBEE_RX_GPIO            (PIO_PB0_IDX)*/
#define XBEE_UART               UART4
#define XBEE_UART_ID            ID_UART4
#define XBEE_UART_IRQn         UART4_IRQn
#define UART_Handler         UART4_Handler
#define XBEEreset_GPIO            (PIO_PD7_IDX)
#define XBEE_TX_GPIO            (PIO_PD19_IDX)
#define XBEE_RX_GPIO            (PIO_PD18_IDX)
#define XBEE_PWRC_GPIO            (PIO_PD1_IDX)

/********** USB ****************************/
#define USB_ID_FLAGS             (PIO_INPUT | PIO_PULLUP)
#define USB_ID_PIN               PIO_PA31_IDX
#define USB_ID_PIN_IRQn     (PIOA_IRQn)
#define USB_ID_PIO_ID         ID_PIOA
#define USB_ID_PIO_MASK    PIO_PA31

#define USB_VBUS_FLAGS         (PIO_INPUT | PIO_PULLUP)
#define USB_VBUS_PIN             PIO_PA30_IDX
#define USB_VBUS_PIN_IRQn ( PIOA_IRQn)
#define USB_VBUS_PIO_ID       ID_PIOC
#define USB_VBUS_PIO_MASK  PIO_PA30

#define GPIO_PA0            (PIO_PA0_IDX)	//need these defines for all pins?
#define GPIO_PA1            (PIO_PA1_IDX)	//need these defines for all pins?
#define GPIO_PA2            (PIO_PA2_IDX)	//need these defines for all pins?
#define GPIO_PA7            (PIO_PA7_IDX)	//need these defines for all pins?
#define GPIO_PA8            (PIO_PA8_IDX)	//need these defines for all pins?
#define GPIO_PA11            (PIO_PA11_IDX)	//need these defines for all pins?
#define GPIO_PA12            (PIO_PA12_IDX)	//need these defines for all pins?
#define GPIO_PA13            (PIO_PA13_IDX)	//need these defines for all pins?
#define GPIO_PA14            (PIO_PA14_IDX)	//need these defines for all pins?
#define GPIO_PA15            (PIO_PA15_IDX)	//need these defines for all pins?
#define GPIO_PA16            (PIO_PA16_IDX)	//need these defines for all pins?
#define GPIO_PA17            (PIO_PA17_IDX)	//need these defines for all pins?
#define GPIO_PA18            (PIO_PA18_IDX)	//need these defines for all pins?
#define GPIO_PA19            (PIO_PA19_IDX)	//need these defines for all pins?
#define GPIO_PA20            (PIO_PA20_IDX)	//need these defines for all pins?
#define GPIO_PA21            (PIO_PA21_IDX)	//need these defines for all pins?
#define GPIO_PA22            (PIO_PA22_IDX)	//need these defines for all pins?
#define GPIO_PA23            (PIO_PA23_IDX)	//need these defines for all pins?
#define GPIO_PA24            (PIO_PA24_IDX)	//need these defines for all pins?
#define GPIO_PA25            (PIO_PA25_IDX)	//need these defines for all pins?
#define GPIO_PA26            (PIO_PA26_IDX)	//need these defines for all pins?
#define GPIO_PA27            (PIO_PA27_IDX)	//need these defines for all pins?
#define GPIO_PA28            (PIO_PA28_IDX)	//need these defines for all pins?
#define GPIO_PA29            (PIO_PA29_IDX)	//need these defines for all pins?
#define GPIO_PA30            (PIO_PA30_IDX)	//need these defines for all pins?
#define GPIO_PA31            (PIO_PA31_IDX)	//need these defines for all pins?

#define GPIO_PB0            (PIO_PB0_IDX)	//need these defines for all pins?
#define GPIO_PB1            (PIO_PB1_IDX)	//need these defines for all pins?
#define GPIO_PB2            (PIO_PB2_IDX)	//need these defines for all pins?
#define GPIO_PB3            (PIO_PB3_IDX)	//need these defines for all pins?
#define GPIO_PB4            (PIO_PB4_IDX)	//need these defines for all pins?
#define GPIO_PB5            (PIO_PB5_IDX)	//need these defines for all pins?
#define GPIO_PB6            (PIO_PB6_IDX)	//need these defines for all pins?
#define GPIO_PB7            (PIO_PB7_IDX)	//need these defines for all pins?
#define GPIO_PB8            (PIO_PB8_IDX)	//need these defines for all pins?
#define GPIO_PB9            (PIO_PB9_IDX)	//need these defines for all pins?
#define GPIO_PB12           (PIO_PB12_IDX)	//need these defines for all pins?
#define GPIO_PB13           (PIO_PB13_IDX)	//need these defines for all pins?

#define GPIO_PC0            (PIO_PC0_IDX)	//need these defines for all pins?
#define GPIO_PC1            (PIO_PC1_IDX)	//need these defines for all pins?
#define GPIO_PC2            (PIO_PC2_IDX)	//need these defines for all pins?
#define GPIO_PC3            (PIO_PC3_IDX)	//need these defines for all pins?
#define GPIO_PC4            (PIO_PC4_IDX)	//need these defines for all pins?
#define GPIO_PC5            (PIO_PC5_IDX)	//need these defines for all pins?
#define GPIO_PC6            (PIO_PC6_IDX)	//need these defines for all pins?
#define GPIO_PC7            (PIO_PC7_IDX)	//need these defines for all pins?
#define GPIO_PC8            (PIO_PC8_IDX)	//need these defines for all pins?
#define GPIO_PC9            (PIO_PC9_IDX)	//need these defines for all pins?
#define GPIO_PC10            (PIO_PC10_IDX)	//need these defines for all pins?
#define GPIO_PC11            (PIO_PC11_IDX)	//need these defines for all pins?
#define GPIO_PC12            (PIO_PC12_IDX)	//need these defines for all pins?
#define GPIO_PC13            (PIO_PC13_IDX)	//need these defines for all pins?
#define GPIO_PC14            (PIO_PC14_IDX)	//need these defines for all pins?
#define GPIO_PC15            (PIO_PC15_IDX)	//need these defines for all pins?
#define GPIO_PC16            (PIO_PC16_IDX)	//need these defines for all pins?
#define GPIO_PC17            (PIO_PC17_IDX)	//need these defines for all pins?
#define GPIO_PC18            (PIO_PC18_IDX)	//need these defines for all pins?
#define GPIO_PC19            (PIO_PC19_IDX)	//need these defines for all pins?
#define GPIO_PC20            (PIO_PC20_IDX)	//need these defines for all pins?
#define GPIO_PC21            (PIO_PC21_IDX)	//need these defines for all pins?
#define GPIO_PC22            (PIO_PC22_IDX)	//need these defines for all pins?
#define GPIO_PC23            (PIO_PC23_IDX)	//need these defines for all pins?
#define GPIO_PC24            (PIO_PC24_IDX)	//need these defines for all pins?
#define GPIO_PC25            (PIO_PC25_IDX)	//need these defines for all pins?
#define GPIO_PC26            (PIO_PC26_IDX)	//need these defines for all pins?
#define GPIO_PC27            (PIO_PC27_IDX)	//need these defines for all pins?
#define GPIO_PC28            (PIO_PC28_IDX)	//need these defines for all pins?
#define GPIO_PC29            (PIO_PC29_IDX)	//need these defines for all pins?
#define GPIO_PC30            (PIO_PC30_IDX)	//need these defines for all pins?
#define GPIO_PC31            (PIO_PC31_IDX)	//need these defines for all pins?

#define GPIO_PD0            (PIO_PD0_IDX)	//need these defines for all pins?
#define GPIO_PD1            (PIO_PD1_IDX)	//need these defines for all pins?
#define GPIO_PD2            (PIO_PD2_IDX)	//need these defines for all pins?
#define GPIO_PD3            (PIO_PD3_IDX)	//need these defines for all pins?
#define GPIO_PD4            (PIO_PD4_IDX)	//need these defines for all pins?
#define GPIO_PD5            (PIO_PD5_IDX)	//need these defines for all pins?
#define GPIO_PD6            (PIO_PD6_IDX)	//need these defines for all pins?
#define GPIO_PD7            (PIO_PD7_IDX)	//need these defines for all pins?
#define GPIO_PD8            (PIO_PD8_IDX)	//need these defines for all pins?
#define GPIO_PD9            (PIO_PD9_IDX)	//need these defines for all pins?
#define GPIO_PD10            (PIO_PD10_IDX)	//need these defines for all pins?
#define GPIO_PD11            (PIO_PD11_IDX)	//need these defines for all pins?
#define GPIO_PD12            (PIO_PD12_IDX)	//need these defines for all pins?
#define GPIO_PD13            (PIO_PD13_IDX)	//need these defines for all pins?
#define GPIO_PD14            (PIO_PD14_IDX)	//need these defines for all pins?
#define GPIO_PD15            (PIO_PD15_IDX)	//need these defines for all pins?
#define GPIO_PD16            (PIO_PD16_IDX)	//need these defines for all pins?
#define GPIO_PD17            (PIO_PD17_IDX)	//need these defines for all pins?
#define GPIO_PD18            (PIO_PD18_IDX)	//need these defines for all pins?
#define GPIO_PD19            (PIO_PD19_IDX)	//need these defines for all pins?
#define GPIO_PD20            (PIO_PD20_IDX)	//need these defines for all pins?
#define GPIO_PD21            (PIO_PD21_IDX)	//need these defines for all pins?
#define GPIO_PD22            (PIO_PD22_IDX)	//need these defines for all pins?
#define GPIO_PD23            (PIO_PD23_IDX)	//need these defines for all pins?
#define GPIO_PD24            (PIO_PD24_IDX)	//need these defines for all pins?
#define GPIO_PD25            (PIO_PD25_IDX)	//need these defines for all pins?
#define GPIO_PD26            (PIO_PD26_IDX)	//need these defines for all pins?
#define GPIO_PD27            (PIO_PD27_IDX)	//need these defines for all pins?
#define GPIO_PD28            (PIO_PD28_IDX)	//need these defines for all pins?
#define GPIO_PD29            (PIO_PD29_IDX)	//need these defines for all pins?
#define GPIO_PD30            (PIO_PD30_IDX)	//need these defines for all pins?
#define GPIO_PD31            (PIO_PD31_IDX)	//need these defines for all pins?

#define GPIO_PE0            (PIO_PE0_IDX)	//need these defines for all pins?
#define GPIO_PE1            (PIO_PE1_IDX)	//need these defines for all pins?
#define GPIO_PE2            (PIO_PE2_IDX)	//need these defines for all pins?
#define GPIO_PE3            (PIO_PE3_IDX)	//need these defines for all pins?
#define GPIO_PE4            (PIO_PE4_IDX)	//need these defines for all pins?
#define GPIO_PE5            (PIO_PE5_IDX)	//need these defines for all pins?

/**  SDRAM pins definitions */
#define SDRAM_BA0_PIO        PIO_PA20_IDX
#define SDRAM_BA1_PIO        PIO_PA0_IDX

#define SDRAM_SDCK_PIO       PIO_PD23_IDX
#define SDRAM_SDCKE_PIO      PIO_PD14_IDX
#define SDRAM_SDCS_PIO       PIO_PC15_IDX
#define SDRAM_RAS_PIO        PIO_PD16_IDX
#define SDRAM_CAS_PIO        PIO_PD17_IDX
#define SDRAM_SDWE_PIO       PIO_PD29_IDX
#define SDRAM_NBS0_PIO       PIO_PC18_IDX
#define SDRAM_NBS1_PIO       PIO_PD15_IDX
#define SDRAM_A0_PIO         PIO_PC20_IDX	//V71 A2
#define SDRAM_A1_PIO         PIO_PC21_IDX	//V71 A3
#define SDRAM_A2_PIO         PIO_PC22_IDX	//V71 A4
#define SDRAM_A3_PIO         PIO_PC23_IDX	//V71 A5
#define SDRAM_A4_PIO         PIO_PC24_IDX	//V71 A6
#define SDRAM_A5_PIO         PIO_PC25_IDX	//V71 A7
#define SDRAM_A6_PIO         PIO_PC26_IDX	//V71 A8
#define SDRAM_A7_PIO         PIO_PC27_IDX	//V71 A9
#define SDRAM_A8_PIO         PIO_PC28_IDX	//V71 A10
#define SDRAM_A9_PIO         PIO_PC29_IDX	//V71 A11
#define SDRAM_SDA10_PIO      PIO_PD13_IDX	//V71 SDA10
#define SDRAM_A11_PIO        PIO_PC31_IDX	//V71 A13
#define SDRAM_A12_PIO        PIO_PA18_IDX	//V71 A14
#define SDRAM_D0_PIO         PIO_PC0_IDX
#define SDRAM_D1_PIO         PIO_PC1_IDX
#define SDRAM_D2_PIO         PIO_PC2_IDX
#define SDRAM_D3_PIO         PIO_PC3_IDX
#define SDRAM_D4_PIO         PIO_PC4_IDX
#define SDRAM_D5_PIO         PIO_PC5_IDX
#define SDRAM_D6_PIO         PIO_PC6_IDX
#define SDRAM_D7_PIO         PIO_PC7_IDX
#define SDRAM_D8_PIO         PIO_PE0_IDX
#define SDRAM_D9_PIO         PIO_PE1_IDX
#define SDRAM_D10_PIO        PIO_PE2_IDX
#define SDRAM_D11_PIO        PIO_PE3_IDX
#define SDRAM_D12_PIO        PIO_PE4_IDX
#define SDRAM_D13_PIO        PIO_PE5_IDX
#define SDRAM_D14_PIO        PIO_PA15_IDX
#define SDRAM_D15_PIO        PIO_PA16_IDX

#define SDRAM_BA0_FLAGS      PIO_PERIPH_C
#define SDRAM_BA1_FLAGS      PIO_PERIPH_C

#define SDRAM_SDCK_FLAGS     PIO_PERIPH_C
#define SDRAM_SDCKE_FLAGS    PIO_PERIPH_C
#define SDRAM_SDCS_FLAGS     PIO_PERIPH_A
#define SDRAM_RAS_FLAGS      PIO_PERIPH_C
#define SDRAM_CAS_FLAGS      PIO_PERIPH_C
#define SDRAM_SDWE_FLAGS     PIO_PERIPH_C
#define SDRAM_NBS0_FLAGS     PIO_PERIPH_A
#define SDRAM_NBS1_FLAGS     PIO_PERIPH_C
#define SDRAM_A_FLAGS        PIO_PERIPH_A
#define SDRAM_SDA10_FLAGS	 PIO_PERIPH_C
#define SDRAM_A13_FLAGS		 PIO_PERIPH_A
#define SDRAM_A14_FLAGS		 PIO_PERIPH_C
#define SDRAM_D_FLAGS		 PIO_PERIPH_A

#endif // USER_BOARD_H
