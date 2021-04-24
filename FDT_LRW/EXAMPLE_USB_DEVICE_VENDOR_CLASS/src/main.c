
/**********************************************************************
		TO DO
***********************************************************************/
//Dummy_Handler(void) in startup_sam71.c can happen, possibly related to UART
/**********************************************************************
		CRITICAL CONFIGS
***********************************************************************/
#define HAS_CAM
#define RESPOND_TO_TINY
#define USE_TE_ALTIMETER	//uses the old barometer
/*TSHOOT DEFINES*/
#define DO_DIAGS
#define SET_RTC
//#define DO_HOURLY_CSV
//#define DO_LOTS_PICS
#define DO_CONTINUOUS_PICS
//#define DO_SLEEP_TEST
#define DO_TSHOOT_TESTS

#define LRW_XMT_DELAY_OFFSET (8)

#define SET_MONTH 0x06	//BCD
#define SET_DAY 0x02
#define SET_HOUR 0x08
#define SET_MINUTE 0x09
#define SET_SECOND 0x35
#define SET_YEAR 0x55

#define TIMEZONE_OFFSET (-7)
#define HAS_DAYLIGHTSAVINGS

#define	PIC_TIME_LOCAL_A	6	//decimal
#define	PIC_TIME_LOCAL_B	8   //MT avoid 18:00 in summer, 17:00 in winter
#define	PIC_TIME_LOCAL_C	10	//Seattle avoid 17:00 in summer, 16:00 in winter so doesn't interfere with file write.
#define	PIC_TIME_LOCAL_D	12
#define	PIC_TIME_LOCAL_E	14	//Seattle avoid 17:00 in summer, 16:00 in winter so doesn't interfere with file write.
#define	PIC_TIME_LOCAL_F	16
#define	PIC_TIME_LOCAL_G	19	//Seattle avoid 17:00 in summer, 16:00 in winter so doesn't interfere with file write.
#define	PIC_TIME_LOCAL_H	0	//Seattle avoid 17:00 in summer, 16:00 in winter so doesn't interfere with file write.

//GPS when 21 minutes after the hour, and when hour x 2 = date.
// #define	GPS_DATE_B	15
// #define	GPS_TIME_B	2
// #define	GPS_DATE_C	20
// #define	GPS_TIME_C	22
// #define	GPS_DATE_D  6
// #define	GPS_TIME_D  4
// #define	GPS_DATE_E	7
// #define	GPS_TIME_E	5

#define TINY_CLK_DELAY (16)//@50: 20/20 good. @30: 20/20 good.  @20: 24/25 good. @15: 25/25 good. @10:22/25 good. @8: mostly errors
#include <asf.h>
#include <ov2655.h>
#include "ui.h"
#include "main.h"
#include "conf_usb.h"
#include "string.h"
#include "FDT.h"
#include "uart.h"
#ifdef OLD_PVC_BOARD
#include "TWI_Altim_OLD.h"
#else
#include "TWI_Altim.h"
#endif
#include "TWI_Humid.h"
#include "TWI_Light.h"
#include "TWI_RTC.h"
#include "TWI_BitBang_B.h"
#define TWIHS_CLK     (400000UL)

#define  MAIN_LOOPBACK_SIZE   512
#define  USB_COMMAND_PACKET_SIZE    20
COMPILER_WORD_ALIGNED
static uint8_t main_buf_loopback[MAIN_LOOPBACK_SIZE];
static uint8_t main_buf_iso_sel;
//@}

// check configuration
#if UDI_VENDOR_EPS_SIZE_ISO_FS>(MAIN_LOOPBACK_SIZE/2)
# error UDI_VENDOR_EPS_SIZE_ISO_FS must be <= MAIN_LOOPBACK_SIZE/2 in cond_usb.h
#endif
#ifdef USB_DEVICE_HS_SUPPORT
# if UDI_VENDOR_EPS_SIZE_ISO_HS>(MAIN_LOOPBACK_SIZE/2)
#   error UDI_VENDOR_EPS_SIZE_ISO_HS must be <= MAIN_LOOPBACK_SIZE/2 in cond_usb.h
# endif
#endif

void main_vendor_int_in_received(udd_ep_status_t status,
iram_size_t nb_transfered, udd_ep_id_t ep);
void main_vendor_int_out_received(udd_ep_status_t status,
iram_size_t nb_transfered, udd_ep_id_t ep);
void main_vendor_bulk_in_received(udd_ep_status_t status,
iram_size_t nb_transfered, udd_ep_id_t ep);
void main_vendor_bulk_out_received(udd_ep_status_t status,
iram_size_t nb_transfered, udd_ep_id_t ep);
void main_vendor_iso_in_received(udd_ep_status_t status,
iram_size_t nb_transfered, udd_ep_id_t ep);
void main_vendor_iso_out_received(udd_ep_status_t status,
iram_size_t nb_transfered, udd_ep_id_t ep);
static char g_imageSize = 0;

char crapBuf[20];
char* picBuff = (char*)SRAM_BASE;
char* picBuff_2 = (char*)IMAGE_BUFF_A;
//
/****************************************************************************
 *  ISR for ISI interrupt.
 ****************************************************************************/
void ISI_Handler(void)
{
	uint32_t status,imr;
	status = ISI->ISI_SR;
	imr = ISI->ISI_IMR;
	if ((status & ISI_SR_CXFR_DONE) && (imr & ISI_IMR_CXFR_DONE)) {
		ISI->ISI_DMA_CHDR |= ISI_DMA_CHDR_C_CH_DIS;
		ISI->ISI_IDR = ISI_IDR_CXFR_DONE;
		ISI->ISI_DMA_CHER |= ISI_DMA_CHER_C_CH_EN;
		ISI->ISI_IER = ISI_IER_CXFR_DONE;
	}
}
/*****************************************************************************************************************
*****************************************************************************************************************
*****************************************************************************************************************
*****************************************************************************************************************
*************************************** MAIN  *******************************************************************
*****************************************************************************************************************
*****************************************************************************************************************
*****************************************************************************************************************
*****************************************************************************************************************
*****************************************************************************************************************/

/*! \brief Main function. Execution starts here.
 */
int main(void)
{
	unsigned int bootSource = (RSTC->RSTC_SR & RSTC_SR_RSTTYP_Msk);;
	char hexMonthNow,hexDayNow,hexHourNow,hexMinNow;
	int iters;
	unsigned int gotAltitudeTemperature;
	short gotHumidity,gotLight;
	char writeRet,readRet;
	char bigBootSoNeedGPS=0;
	unsigned int archAddrMot,archAddrGPS,archAddrBase;
	char takePicFlag=0;
	uint32_t wdt_mode, timeout_value;
	unsigned short crcrc;
	int spValue;
	char didGoodXfr, didGoodXfrCtr;
	irq_initialize_vectors();//
	cpu_irq_enable();//
	sysclk_init();
	board_init();
	
	// Initialize the sleep manager
	sleepmgr_init();
	sysclk_init();
	board_init();

	/* CONFIG WATCHDOG */
	timeout_value = wdt_get_timeout_value(WDT_PERIOD * 1000,BOARD_FREQ_SLCK_XTAL);
	if (timeout_value == WDT_INVALID_ARGUMENT) {
		while (1) {
			/* Invalid timeout value, error. */
		}
	}
	wdt_mode =
	WDT_MR_WDRSTEN |  /* Enable WDT fault interrupt. */
	WDT_MR_WDDBGHLT  |  /* WDT stops in debug state. */
	WDT_MR_WDIDLEHLT;   /* WDT stops in idle state. */
	wdt_init(WDT, wdt_mode, timeout_value, timeout_value);
	/* Configure and enable WDT interrupt. */
	NVIC_DisableIRQ(WDT_IRQn);
	NVIC_ClearPendingIRQ(WDT_IRQn);
	NVIC_SetPriority(WDT_IRQn, 0);
	NVIC_EnableIRQ(WDT_IRQn);
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
//		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_MR_WDDIS;//WDT_MR_WDDIS  disable watchdog
	sysclk_enable_peripheral_clock(ID_PIOA);
	sysclk_enable_peripheral_clock(ID_PIOB);
	sysclk_enable_peripheral_clock(ID_PIOC);
	sysclk_enable_peripheral_clock(ID_PIOD);

	configure_console();  //pmc_enable_pllack messes up the console speed
#ifdef DO_DIAGS
	configure_console();  //pmc_enable_pllack messes up the console speed
	printf("\r\nBOOT %X ",bootSource);
#endif
//	if(bootSource!=RSTC_SR_RSTTYP_BACKUP_RST){//100 = POWER-UP RESET  
	switch(bootSource){
		case 0x000://PWR UP 
		case 0x400://RST PIN (pgm cable)
/****************************	PWR-UP and RST PIN RESET  ******************************/
/****************************	PWR-UP and RST PIN RESET  ******************************/
/****************************	PWR-UP and RST PIN RESET  ******************************/
		pmc_enable_periph_clk(ID_PIOC);
		unsigned char rtcData[26];// = {0x00,0x00,0xFF,0xFF};//!< Define data buffer
		
		ioport_set_pin_dir(RTC_INT, IOPORT_DIR_INPUT);//RTC INT
		ioport_set_pin_mode(RTC_INT, IOPORT_MODE_PULLUP);//RTC INT
		
		ioport_set_pin_dir(SCL_RTC, IOPORT_DIR_OUTPUT);
		ioport_set_pin_dir(SDA_RTC, IOPORT_DIR_OUTPUT);

		delay_ms(200);//looked fine even at only 1ms (when repeated every 500ms);
		twi_init_RTC();
#ifdef DO_DIAGS
		if(bootSource==0)printf("PWR\r\n");else printf("RST\r\n");
#endif
		delay_ms(500);//
		ioport_set_pin_dir(SDA_RTC, IOPORT_DIR_INPUT);
		ioport_set_pin_mode(SDA_RTC, IOPORT_MODE_PULLUP);//RTC INT
#ifdef SET_RTC
		rtcData[0]=0x00;//write to Address 0x00  SET ALL
		rtcData[1]=0x02;//contents of Addr 0x00 CTL1
		rtcData[2]=0x00;//contents of Addr 0x01 CTL2
		rtcData[3]=0x00;//contents of Addr 0x02 CTL3
		rtcData[4]=SET_SECOND;//contents of Addr 0x03 SECONDS works
		rtcData[5]=SET_MINUTE;//contents of Addr 0x04 MINUTES works
		rtcData[6]=SET_HOUR;//contents of Addr 0x05 HOURS works
		rtcData[7]=SET_DAY;//contents of Addr 0x06 DAYS
		rtcData[8]=0x02;//contents of Addr 0x07 WEEKDAY
		rtcData[9]=SET_MONTH;//contents of Addr 0x08 MONTH
		rtcData[0x0A]=SET_YEAR;//contents of Addr 0x09 YEAR
		rtcData[0x0B]=0x80;//contents of Addr 0x0A  0x80 = second alarm disabled
		rtcData[0x0C]=0x80;//contents of Addr 0x0B  0x80 = minute alarm disabled
		rtcData[0x0D]=0x80;//contents of Addr 0x0C  0x80 = Hour alarm disabled
		rtcData[0x0E]=0x80;//contents of Addr 0x0D  0x80 = day alarm disabled
		rtcData[0x0F]=0x80;//contents of Addr 0x0E  0x80 = weekday alarm disabled
		rtcData[0x10]=0x00;//contents of Addr 0x0F  0x00 = temperature clockout period 4 minutes
		rtcData[0x11]=0x00;//contents of Addr 0x10  20=PULSED INTERRUPT
		writeRet = write_data_RTC(rtcData,18);//!< write sequentially to the slave "1" sends addr plus data[0].
		delay_ms(200);//
		rtcData[0]=0x01;//write to Address 0x00    NEED THIS TO ACK THE RTC INTERRUPT
		rtcData[1]=0x30;//contents of Addr 0x00 CTL1
		write_data_RTC(rtcData,2);//!< write sequentially to the slave "1" sends addr plus data[0].
#else
		rtcData[0]=0x00;//write to Address 0x00 
		rtcData[1]=0x02;//contents of Addr 0x00 CTL1
		rtcData[2]=0x00;//contents of Addr 0x01 CTL2
		rtcData[3]=0x00;//contents of Addr 0x02 CTL3
		writeRet = write_data_RTC(rtcData,4);//!< write sequentially to the slave "1" sends addr plus data[0].
		rtcData[0]=0x0B;//write to Address 0x00
		rtcData[1]=0x80;//contents of Addr 0x0A  0x80 = second alarm disabled
		rtcData[2]=0x80;//contents of Addr 0x0B  0x80 = minute alarm disabled
		rtcData[3]=0x80;//contents of Addr 0x0C  0x80 = Hour alarm disabled
		rtcData[4]=0x80;//contents of Addr 0x0D  0x80 = day alarm disabled
		rtcData[5]=0x80;//contents of Addr 0x0E  0x80 = weekday alarm disabled
		rtcData[6]=0x00;//contents of Addr 0x0F  0x00 = temperature clockout period 4 minutes
		rtcData[7]=0x00;//contents of Addr 0x10  20=PULSED INTERRUPT
		writeRet = write_data_RTC(rtcData,8);//!< write sequentially to the slave "1" sends addr plus data[0].
		delay_ms(200);//
		rtcData[0]=0x01;//write to Address 0x00    NEED THIS TO ACK THE RTC INTERRUPT
		rtcData[1]=0x30;//contents of Addr 0x00 CTL1
		write_data_RTC(rtcData,2);//!< write sequentially to the slave "1" sends addr plus data[0].
#endif
		delay_ms(1000);
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
		rtcData[0]=0x01;//write to Address 0x00    NEED THIS TO ACK THE RTC INTERRUPT
		rtcData[1]=0x30;//contents of Addr 0x00 CTL1
		write_data_RTC(rtcData,2);//!< write sequentially to the slave "1" sends addr plus data[0].
		/*******Read ***/
		rtcData[0]=0x03;//
		write_data_RTC(rtcData,1);//!< write sequentially to the slave "1" sends addr plus data[0].
		delay_ms(1);//
		read_bytes_RTC(rtcData,7);//!< write sequentially to the slave "1" sends addr plus data[0].*/
		delay_ms(1000);
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
#ifdef DO_DIAGS
		printf("%X-%X-%X  %X:%X:%X %X \r\n",rtcData[5],rtcData[3],rtcData[6],rtcData[2],rtcData[1],rtcData[0],rtcData[7]);
#endif
// 	asm volatile ("mov %0, sp\n\t" : "=r" (spValue));
// 	printf("STACK POINTER: %x\r\n",spValue);
/********************************************* TSHOOT TESTS **************************************************************************************/
/********************************************* TSHOOT TESTS **************************************************************************************/
/********************************************* TSHOOT TESTS **************************************************************************************/
/********************************************* TSHOOT TESTS **************************************************************************************/
/********************************************* TSHOOT TESTS **************************************************************************************/
#ifdef DO_CONTINUOUS_PICS
int goodCtr2=0;
int badCtr2=0;
	uint8_t * camBuff = (uint8_t *)SRAM_BASE+0x400;	uint8_t *camBuffPtr;	twihs_options_t opt;	int iters;	camBuffPtr=camBuff;	char ret=0;	Ctrl_status status;
	FRESULT res;
	FATFS fs;
	FIL file_object;
for(;;){
	sleepPins();

	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog;
	redBlink(1);
	SCB_EnableICache();	SCB_EnableDCache();	/* Complete SDRAM configuration */	configPinsSDRAM();	pmc_enable_periph_clk(ID_SDRAMC);	sdramc_init((sdramc_memory_dev_t *)&SDRAM_INSIGNIS_16M,	sysclk_get_cpu_hz());	sdram_enable_unaligned_support();#ifdef DO_DIAGS
	printf("SRAM COMPLETE\r\n");#endif
	MATRIX->CCFG_SMCNFCS = CCFG_SMCNFCS_SDRAMEN;
	/* Configure CAM POWER.*/
	ioport_set_pin_dir(CPWRC_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(SDCARDPWRC_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(CPWDN_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(GPIO_PA8, IOPORT_DIR_OUTPUT);//SRAM PWRC
	ioport_set_pin_dir(ISI_HSYNC_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ISI_VSYNC_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(CPWDN_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CPWDN_GPIO, CPWDN_POWER_OFF);
	ioport_set_pin_level(CPWDN_GPIO, CPWDN_POWER_OFF);
	delay_ms(100);
	ioport_set_pin_level(CPWRC_GPIO, CPWRC_POWER_ON);//needs 30 usec ramp
	ioport_set_pin_level(SDCARDPWRC_GPIO, 1);
	ioport_set_pin_level(GPIO_PA8, 1);//SRAM_PWRC
	delay_ms(100);
	delay_ms(6);//OV2710 datasheet says minimum 5ms between applying power an allowing CPWDN to go low.
	ioport_set_pin_level(CPWDN_GPIO, CPWDN_POWER_ON);
	delay_ms(2);
	delay_ms(100);
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
#ifdef DO_DIAGS
	printf("CAM PWR ON\r\n");//#endif
	gpio_configure_pin(ISI_MCLK_GPIO, ISI_MCLK_FLAGS);
	#define   PMC_PCK_PRES_CLK_4 (0x2u << 7) /**Zoo Board if PLLA_MULT=25=300MHz, then 6 = 33MHz, 7=18Mhz. if PLLA_MULT=20, then 6=27Mhz.  OVM7692 6-27MHz*/
	PMC->PMC_PCK[1] = (PMC_PCK_PRES_CLK_4 | PMC_PCK_CSS_PLLA_CLK);
	PMC->PMC_SCER = PMC_SCER_PCK1;
	iters=500;	while ((!(PMC->PMC_SCSR & PMC_SCSR_PCK1))&&(iters>0)) {
		iters--;
	}
	if(iters==0){ret=1;goto NO_CAMERA_RESPONSE; }
	#ifdef DO_DIAGS
	printf("MCLK OKAY\r\n");//tshooooooooooooooooooooooooooooooooooooooooooooot	#endif
	delay_ms(1);
	// /************************** CAMERA ***********************************/	/* Init TWI peripheral */
	opt.master_clk = sysclk_get_cpu_hz();
	opt.speed      = TWIHS_CLK;
	twihs_master_init(BOARD_TWIHS, &opt);
	/* Configure TWI interrupts */
	NVIC_DisableIRQ(BOARD_TWIHS_IRQn);
	NVIC_ClearPendingIRQ(BOARD_TWIHS_IRQn);
	NVIC_SetPriority(BOARD_TWIHS_IRQn, 0);
	NVIC_EnableIRQ(BOARD_TWIHS_IRQn);
	/* Enable TWI peripheral */
	pmc_enable_periph_clk(ID_BOARD_TWIHS);
	/* OV2655 Initialization */
	pio_configure_pin(ISI_D0_GPIO, ISI_D0_FLAGS);
	pio_configure_pin(ISI_D1_GPIO, ISI_D1_FLAGS);
	pio_configure_pin(ISI_D2_GPIO, ISI_D2_FLAGS);
	pio_configure_pin(ISI_D3_GPIO, ISI_D3_FLAGS);
	pio_configure_pin(ISI_D4_GPIO, ISI_D4_FLAGS);
	pio_configure_pin(ISI_D5_GPIO, ISI_D5_FLAGS);
	pio_configure_pin(ISI_D6_GPIO, ISI_D6_FLAGS);
	pio_configure_pin(ISI_D7_GPIO, ISI_D7_FLAGS);
	pio_configure_pin(ISI_HSYNC_GPIO, ISI_HSYNC_FLAGS);
	pio_configure_pin(ISI_VSYNC_GPIO, ISI_VSYNC_FLAGS);
	pio_configure_pin(ISI_PCK_PIO, ISI_PCK_FLAGS);
	delay_ms(200);
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog;
/*************************** SRAM ***************************************************/
	SCB_CleanInvalidateDCache();
	camBuffPtr=SRAM_BASE;//camBuff=SRAM_BASE+0x400
	// 	for(camCtr=0;camCtr<50000;camCtr+=2){  //This seems to hang the Atmel sometimes.
	// 		*camBuffPtr++=0x05;
	// 		*camBuffPtr++=0x06;
	// 	}
	SCB_CleanInvalidateDCache();
	/************************ ISI *********************************/
	pmc_enable_periph_clk(ID_ISI);//
	_isi_AllocateFBD();//inside here is the address that needs to be sent to isi_set_dma_codec_path() below.
	#ifdef DO_DIAGS
	printf("ISI ALLOCATE COMPLETE\r\n");
	#endif	sdramc_deinit();
	#ifdef DO_DIAGS
	printf("SDRAM DEINIT\r\n");
	delay_ms(100);
	#endif
	SCB_CleanInvalidateDCache();
	SCB_DisableDCache();
	SCB_InvalidateICache();
	SCB_DisableICache();
#ifdef DO_DIAGS
	printf("CACHE DISABLED\r\n");
#endif
	ioport_set_pin_level(CPWDN_GPIO, CPWDN_POWER_OFF);
	ioport_set_pin_level(CPWDN_GPIO, CPWDN_POWER_OFF);
	delay_ms(1);
	ioport_set_pin_level(CPWDN_GPIO, CPWDN_POWER_OFF);
#ifdef DO_DIAGS
	printf("CAM OFF\r\n");
#endif
	NO_CAMERA_RESPONSE:
	resetPinsISI();
	delay_ms(1);
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog;
#ifdef DO_DIAGS
	printf("PINS RESET\r\n");
#endif
	resetSD(300);
	SCB_EnableDCache();//for SD to work twice in a row, have to enable DCache before cleanInvalidate,disable.
	SCB_CleanInvalidateDCache();
	SCB_DisableDCache();
 	sd_mmc_init();
	sd_mmc_test_unit_ready(0);//first electronic signals
	sd_mmc_check(0);

	memset(&fs, 0, sizeof(FATFS));
	res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);

// 	f_close(&file_object);//hangs if try to close file object when it wasn't first opened.
 	closeSDcard();
	ioport_set_pin_level(SRAMPWRC_GPIO, SRAMPWRC_POWER_OFF);
#ifdef DO_DIAGS
	printf("CREATED PIC FILE. FILE CLOSED.\r\n");
#endif
	main_end_of_imgTest:
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog;
	delay_ms(10);
	closeSDcard();//pin cnfig and power off.

	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
		if(takeImage(0,0,1)==0)
		{
			 goodCtr2++;
		storeImage();
		#ifdef DO_DIAGS
			printf("RETURN OK.\r\n");
			WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
			delay_ms(150);
		#endif
		}
		else badCtr2++;
		#ifdef DO_DIAGS
			redBlink(2);
			printf("\r\nCONTINUOUS IMAGE TEST TEST LOOP:    GOOD=%d\r\n\r\n",goodCtr2);
			WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
			delay_ms(150);
		#endif
}
#endif
#ifdef DO_TSHOOT_TESTS
#ifdef DO_DIAGS
		printf("ENTERING TEST LOOP\r\n");
#endif
int testLoopCtr=0;
char fakeBytes[51];
		 sleepPins();
 		delay_ms(1000);

		ioport_set_pin_level(GPIO_PA22,1); //wake-up nRF
		ioport_set_pin_dir(GPIO_PA22, IOPORT_DIR_INPUT);//LRW PWRC
		ioport_set_pin_mode(GPIO_PA22, IOPORT_MODE_PULLUP);//LRW PWRC
// 		ioport_set_pin_level(GPIO_PA22,1); //wake-up nRF
// 		ioport_set_pin_dir(GPIO_PA22, IOPORT_DIR_OUTPUT);//LRW PWRC
 for(;;){
// 		sleepPins();
 		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
 		delay_ms(1000);
		for(iters=58;iters>0;iters--){//DELAY SECONDS
			WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
			delay_ms(1000);
			printf(".");
		}
//		sleepPins();
//		floatCamForSDCard();//Needs this after sleep pins.
		delay_ms(1000);
		
		LRWPI_SCL_FLOAT;
		LRWPI_SDA_FLOAT;
		
		fakeLoRaWAN(fakeBytes,testLoopCtr);
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
		while((uart_write(CONSOLE_UART,0x46))){}//start chars
		while((uart_write(CONSOLE_UART,0x44))){}
		while((uart_write(CONSOLE_UART, LRW_XMT_DELAY_OFFSET))){}
		for(iters=0;iters<51;iters++){
			while((uart_write(CONSOLE_UART,fakeBytes[iters]))){}
		}
		for(iters=0;iters<51;iters++){
			while((uart_write(CONSOLE_UART,fakeBytes[iters]))){}
		}
		while((uart_write(CONSOLE_UART,0x54))){}//end char
		
		//		printf("DONE\r\n");
		//		printf("ABCDEFGHIJKLMNOPQRS ABCDEFGHIJKLMNOPQRS ABCDEFGHIJKLMNOPQRS ABCDEFGHIJKLMNOPQRS ABCDEFGHIJKLMNOPQRS ABCD",testLoopCtr++);
		redBlink(5);
		userBlink(5);
		// 		ioport_set_pin_mode(GPIO_PA22, IOPORT_MODE_OPEN_DRAIN);//LRW PWRC
		// 		ioport_set_pin_level(GPIO_PA22,0); //wake-up nRF
		// 		ioport_set_pin_dir(GPIO_PA22, IOPORT_DIR_OUTPUT);//LRW PWRC
		//  		delay_ms(1);//2000 causes two interrupts
		ioport_set_pin_level(GPIO_PA22,1); //wake-up nRF
		ioport_set_pin_dir(GPIO_PA22, IOPORT_DIR_INPUT);//LRW PWRC
		ioport_set_pin_mode(GPIO_PA22, IOPORT_MODE_PULLUP);//LRW PWRC
		
	printf("TEST LOOP COUNTER = %d\r\n",testLoopCtr);
	testLoopCtr++;
 }
//		printf("\r\n");
//		printf("TEST LOOP\r\n");


// for(;;){
// 		sleepPins();
// 		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
// 		delay_ms(1000);
// 		SDRAMtest();
//  		floatCamForSDCard();//Needs this after sleep pins.
//  		oneDayToSD(testLoopCtr,0);
// 		closeSDcard();
// 		for(iters=2;iters>0;iters--){//DELAY SECONDS
// 			WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
// 			delay_ms(1000);
// 			printf(".");
// 		}
// 	testLoopCtr++;
// 	if(testLoopCtr>16)testLoopCtr=1;
// }
//		printf("\r\n");
//		printf("TEST LOOP\r\n");
		sleepPins();
		floatCamForSDCard();//Needs this after sleep pins.
 		delay_ms(1000);
		
	LRWPI_SCL_FLOAT;
	LRWPI_SDA_FLOAT;
	
		fakeLoRaWAN(fakeBytes,testLoopCtr);
 		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
			while((uart_write(CONSOLE_UART,0x46))){}//start chars
			while((uart_write(CONSOLE_UART,0x44))){}
			while((uart_write(CONSOLE_UART, LRW_XMT_DELAY_OFFSET))){}
		for(iters=0;iters<51;iters++){
			while((uart_write(CONSOLE_UART,fakeBytes[iters]))){}
		}
		for(iters=0;iters<51;iters++){
			while((uart_write(CONSOLE_UART,fakeBytes[iters]))){}
		}
			while((uart_write(CONSOLE_UART,0x54))){}//end char
				
//		printf("DONE\r\n");
//		printf("ABCDEFGHIJKLMNOPQRS ABCDEFGHIJKLMNOPQRS ABCDEFGHIJKLMNOPQRS ABCDEFGHIJKLMNOPQRS ABCDEFGHIJKLMNOPQRS ABCD",testLoopCtr++);
		redBlink(5);
		userBlink(5);
// 		ioport_set_pin_mode(GPIO_PA22, IOPORT_MODE_OPEN_DRAIN);//LRW PWRC
// 		ioport_set_pin_level(GPIO_PA22,0); //wake-up nRF
// 		ioport_set_pin_dir(GPIO_PA22, IOPORT_DIR_OUTPUT);//LRW PWRC
//  		delay_ms(1);//2000 causes two interrupts
		ioport_set_pin_level(GPIO_PA22,1); //wake-up nRF
		ioport_set_pin_dir(GPIO_PA22, IOPORT_DIR_INPUT);//LRW PWRC
		ioport_set_pin_mode(GPIO_PA22, IOPORT_MODE_PULLUP);//LRW PWRC
// 		printf("PWRC HIGH\r\n");

 		sleepPins();
		 
	LRWPI_SCL_FLOAT;
	LRWPI_SDA_FLOAT;
		 
//		 sendLoRaWAN(0x44);
		 
 		delay_ms(1000);
 		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
//////}
#endif  //END DO TSHOOT TESTS
/********************************************* END TSHOOT TESTS  **************************************************************************************/
/********************************************* KILL CURRENT TEST **************************************************************************************/
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
#ifdef DO_SLEEP_TEST
		redBlink(5);
		userBlink(5);
		delay_ms(2000);
		NVIC_DisableIRQ(RTC_IRQn);
		NVIC_ClearPendingIRQ(RTC_IRQn);

		supc_set_wakeup_inputs(SUPC,SUPC_WUIR_WKUPEN1_ENABLE|SUPC_WUIR_WKUPEN2_ENABLE,SUPC_WUIR_WKUPT1_LOW|SUPC_WUIR_WKUPT2_LOW);//A1=1 from TINY.
		sleepmgr_init();
		sleepmgr_lock_mode(SLEEPMGR_BACKUP);
		pmc_enable_periph_clk(ID_PIOA);
		pmc_enable_periph_clk(ID_PIOB);
		pmc_enable_periph_clk(ID_PIOD);
		supc_set_wakeup_inputs(SUPC,SUPC_WUIR_WKUPEN1_ENABLE|SUPC_WUIR_WKUPEN2_ENABLE,SUPC_WUIR_WKUPT1_LOW|SUPC_WUIR_WKUPT2_LOW);//A1=1 from TINY.
		sleepmgr_init();
		sleepmgr_lock_mode(SLEEPMGR_BACKUP);
#ifdef DO_DIAGS
		configure_console();  //pmc_enable_pllack messes up the console speed
		printf("SLEEP\r\n");
		delay_ms(100);
#endif
		sleepPins();//2019 eNest: writing once gets 100uA, twice gets 44 uA (after 3.6VReg).
		sleepPins();//2019 eNest: writing once gets 100uA, twice gets 44 uA (after 3.6VReg).
		sleepmgr_enter_sleep();
#endif
/********************************************* END KILL CURRENT TEST **************************************************************************************/
/********************************************* END KILL CURRENT TEST **************************************************************************************/
		/******* RTC READ ******/
		for(iters=0;iters<20;iters++)rtcData[iters]=0;
		rtcData[0]=0x00;//
		writeRet = write_data_RTC(rtcData,1);//!< write sequentially to the slave "1" sends addr plus data[0].
		delay_ms(1);//
		readRet = read_bytes_RTC(rtcData,7);//!< write sequentially to the slave "1" sends addr plus data[0].*/
		delay_ms(1);
		write_data_RTC(rtcData,2);//!< write sequentially to the slave "1" sends addr plus data[0].
		if((rtcData[1]&0x80)==0x80){
		rtcData[0]=0x01;//write to Address 0x00
		rtcData[1]=0x30;//contents of Addr 0x00 CTL1
		writeRet = write_data_RTC(rtcData,2);//!< write sequentially to the slave "1" sends addr plus data[0].
		printf("%X-%X-%X  %X:%X:%X %X \r\n",rtcData[8],rtcData[6],rtcData[9],rtcData[5],rtcData[4],rtcData[3],rtcData[7]);
		delay_ms(1000);
		}
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
		delay_s(2);
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
		delay_s(1);
		bigBootSoNeedGPS=1;
/*****************************	NOT PWR-UP RESET  ***************************/
/*****************************	NOT PWR-UP RESET  ***************************/
		break;//from PWR-UP or RST case in bootSource switch case
	case 0x100://bootSource switch case 100=WAKEUP pin (RTC or TINY)
	case 0x200://bootSource switch case 200=WATCHDOG Reset. For some goofy reason, when erasesector() is included in the record32() function, we get WATCHDOG resets instead of RTC
		pmc_enable_periph_clk(ID_PIOB);
		pmc_enable_periph_clk(ID_PIOD);
		pmc_enable_periph_clk(ID_PIOA);//for RTC INT pin
		ioport_set_pin_dir(ToBigWU, IOPORT_DIR_INPUT);
		//	ioport_set_pin_dir(ToBigSLEEP, IOPORT_DIR_INPUT);
		takePicFlag=0;
		bigBootSoNeedGPS=0;
		delay_us(1);
		if(ioport_get_pin_level(ToBigWU)==0){//IF THIS WAKEUP IS FROM TINY WITH A GPS DATE/TIME
/***************************	GPS INTERRUPT FROM TINY  ****************************/
/***************************	GPS INTERRUPT FROM TINY  ****************************/
			getGPSdateTimeFrmTiny();	//RUN TINY STUFF
			WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
#ifdef DO_DIAGS
			configure_console();
			delay_ms(1);
			printf("TINY\r\n");
#endif
/***************************	RTC WAKE-UP  ***********************************/
/***************************	RTC WAKE-UP   **********************************/
		}else if (ioport_get_pin_level(RTC_INT)==0)	{
			pmc_enable_periph_clk(ID_PIOC);
#ifdef DO_DIAGS
			if(bootSource==0x100)printf("RTC\r\n");
			else printf("WATCHDOG? MORE LIKELY RTC\r\n");
#endif
/**** ALL RTC: GET ALTITUDE, SEND ALTITUDE */
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
		pmc_enable_periph_clk(ID_PIOA);
		pmc_enable_periph_clk(ID_PIOB);
		pmc_enable_periph_clk(ID_PIOD);
		unsigned char rtcData[26];// = {0x00,0x00,0xFF,0xFF};//!< Define data buffer
		//	delay_ms(20);//
		rtcData[0]=0x01;//write to Address 0x00    NEED THIS TO ACK THE RTC INTERRUPT
		rtcData[1]=0x30;//contents of Addr 0x00 CTL1
		write_data_RTC(rtcData,2);//!< write sequentially to the slave "1" sends addr plus data[0].
		/*******Read ***/
		rtcData[0]=0x03;//
		write_data_RTC(rtcData,1);//!< write sequentially to the slave "1" sends addr plus data[0].
		delay_ms(1);//
		read_bytes_RTC(rtcData,7);//!< write sequentially to the slave "1" sends addr plus data[0].*/
#ifdef DO_DIAGS
		printf("%X-%X-%X %X:%X:%X\r\n",rtcData[5],rtcData[3],rtcData[6],rtcData[2],rtcData[1],secNow);//tshooooooooooooooooooooooooooooooooooot
#endif
		yearNow=rtcData[6];
		hexMonthNow=bcdToHex(rtcData[5]);
		hexDayNow=bcdToHex(rtcData[3]);
		hexHourNow=bcdToHex(rtcData[2]);
		hexMinNow=bcdToHex(rtcData[1]);
		secNow=rtcData[0];
		hexHourNow=hexHourNow+24+TIMEZONE_OFFSET;
#ifdef HAS_DAYLIGHTSAVINGS
		char beforeNov2 = 0;
		char afterMar7 = 0;
		if(hexMonthNow<11)beforeNov2=1;
		if((hexMonthNow==11)&&(hexDayNow>1))beforeNov2=0;
		if(hexMonthNow>3)afterMar7=1;
		if((hexMonthNow==3)&&(hexDayNow<8))afterMar7=0;
		if((beforeNov2)&&(afterMar7))hexHourNow++;//(18 is BCD 12)
#endif
		if(hexHourNow>23)hexHourNow-=24;
/**** TEST IF NEED TO REQUEST GPS ********************/   ////TODOOOOOOOO  deal with the possibility that Big woke up a second 58 or 59 sp that minNow is behind.
		if((bigBootSoNeedGPS||((yearNow==99)&&((hexHourNow%3)==0)&&(hexMinNow==0)))
			||(((hexDayNow>>2)==(hexHourNow))&&(hexMinNow==21))){//Always 21 minutes after an hour.
			minPacketForTiny[0]=0x10; //set flag to take GPS
		}else minPacketForTiny[0]=0;

			if((minPacketForTiny[0]==0)&&(((hexMinNow==0)))&&(
			(hexHourNow==PIC_TIME_LOCAL_A)
			||(hexHourNow==PIC_TIME_LOCAL_B)
			||(hexHourNow==PIC_TIME_LOCAL_C)
			||(hexHourNow==PIC_TIME_LOCAL_D)
			||(hexHourNow==PIC_TIME_LOCAL_E)
			||(hexHourNow==PIC_TIME_LOCAL_F)
			||(hexHourNow==PIC_TIME_LOCAL_G)
			||(hexHourNow==PIC_TIME_LOCAL_H)
			)
			)takePicFlag=1;else takePicFlag=0;

#ifdef DO_LOTS_PICS
			if((minPacketForTiny[0]==0)&&((hexMinNow%5)==0))takePicFlag=1;
#endif
#ifdef USE_TE_ALTIMETER
		gotAltitudeTemperature=getAltimeterTemperatureTE(0xEE);
		#else
	//	gotAltitudeTemperature=getAltimeterTemperature();
#endif
		gotHumidity=getHumidity(0x80);
		gotLight=getALSfromVEML(0x20);
#ifdef DO_DIAGS
		printf("TEMP %X BAROM %X HUMIDITY %X LIGHT %X MOTION %X\r\n",(gotAltitudeTemperature>>16),(gotAltitudeTemperature&0x0000FFFF),gotHumidity,gotLight,1);
#endif

// 		sleepPins();
//		 SDRAMtest();
//  		floatCamForSDCard();//Needs this after sleep pins.
//  		oneDayToSD(22,0);
// 		 closeSDcard();

	/**** STORE MOTION ********************/
#ifdef RESPOND_TO_TINY
		char rcvFromTinyS[28];
		didGoodXfr=0;
		didGoodXfrCtr = 0;
		while((didGoodXfr==0)&&(didGoodXfrCtr<22)){
			WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
			sendPacketToTiny32(bcdToHex(rtcData[1])&0x1F);//send minute bits to tell Tiny which portion of Loc to send as partial param.
			ioport_set_pin_dir(BigTiny_CLK, IOPORT_DIR_OUTPUT);
			ioport_set_pin_level(BigTiny_CLK, 1);
			delay_ms(5);//needs to wait for Tiny to calc CRC for 56 bytes. Making longer than 5 didn't help.
			ioport_set_pin_level(BigTiny_CLK, 0);//start condition
			for(iters=0;iters<28;iters++){
				rcvFromTinyS[iters]=getByteFromTiny();
			}
			ioport_set_pin_level(BigTiny_CLK,1);//return to high when done

			ioport_set_pin_dir(BigTiny_DAT, IOPORT_DIR_OUTPUT);//use this to ack a good CRC from Tiny
			ioport_set_pin_level(BigTiny_DAT,1);//

			crcrc=calcCRC(rcvFromTinyS,26);
			if(((crcrc>>8)==rcvFromTinyS[26])&&((crcrc&0x00FF)==rcvFromTinyS[27])){
				ioport_set_pin_level(BigTiny_DAT,0);//
				//need an ack here so Tiny can clear motParams
				didGoodXfr=1;
#ifdef DO_DIAGS
				configure_console();
				delay_ms(1);
				printf("LOG MOTION FUNCTION THAT CAUSES WATCHDOG FAILURE\r\n");
#endif
 				logMotionRecord32byte(rcvFromTinyS,0,NULL);//
 				ioport_set_pin_dir(BigTiny_DAT, IOPORT_DIR_INPUT);//use this to ack a good CRC from Tiny
#ifdef DO_DIAGS
				configure_console();
				delay_ms(1);
				printf("BIGTINY CRC GOOD %d %X %X%X\r\n",didGoodXfrCtr,crcrc,rcvFromTinyS[26],rcvFromTinyS[27]);
				printf("MOTION: %X %X %X %X %X %X %X %X  MIN_TOTAL: %X  PROX: %X %X %X %X %X %X %X %X %X %X %X %X BASELINE PROX: %X\r\n RTC_MIN: %X LIGHT: %X LOC BYTE: %X\r\n",
				rcvFromTinyS[1],rcvFromTinyS[2],rcvFromTinyS[3],rcvFromTinyS[4],rcvFromTinyS[5],rcvFromTinyS[6],rcvFromTinyS[7],rcvFromTinyS[8],  rcvFromTinyS[9], rcvFromTinyS[10],rcvFromTinyS[11],rcvFromTinyS[12],rcvFromTinyS[13],rcvFromTinyS[14],rcvFromTinyS[15],rcvFromTinyS[16],rcvFromTinyS[17],rcvFromTinyS[18],rcvFromTinyS[19],rcvFromTinyS[20],rcvFromTinyS[21],rcvFromTinyS[22],rtcData[1]&0x0F,(((short)(rcvFromTinyS[0])&0x06)<<7)+rcvFromTinyS[23],rcvFromTinyS[24]);
				delay_ms(10);
#endif
			}else{//bad CRC packet from Tiny
				ioport_set_pin_level(BigTiny_DAT,1);//
#ifdef DO_DIAGS
				configure_console();
				delay_ms(1);
				printf("BIGTINY CRC ERROR %d %X %X%X\r\n",didGoodXfrCtr,crcrc,rcvFromTinyS[26],rcvFromTinyS[27]);
				delay_ms(10);
#endif
				delay_ms(400+(didGoodXfrCtr>>5));//needs adjusting
				ioport_set_pin_dir(BigTiny_DAT, IOPORT_DIR_INPUT);//use this to ack a good CRC from Tiny
			}//end of packet CRC tests from Tiny
			didGoodXfrCtr++;
		}//end while not yet didGoodXfrCtr
#endif  //END RESPOND TO TINY
		}else printf("DIDN'T DETECT RTC PULSE\r\n");//END OF RTC WAKEUPS
	break;//bootSource case 400 (wakeup pin)
	}//end of bootSource switch case
/****************** ALL ***********************/
/****************** ALL ***********************/
#ifdef DO_DIAGS
	printf("ALL\r\n");
	delay_ms(100);
#endif
#ifdef DO_CONTINUOUS_PICS
	int goodTestImageCtr=0;
	int badTestImageCtr=0;
	for(;;){
		sleepPins();
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
		if(takeImage(0,0,1)==0)
			{goodTestImageCtr++;
			storeImage();}
			else badTestImageCtr++;
		#ifdef DO_DIAGS
		printf("\r\nCONTINUOUS IMAGE TEST LOOP. GOT INTO RTC LOOP:          GOOD=%d  BAD=%d\r\n\r\n",goodTestImageCtr,badTestImageCtr);
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
		delay_ms(500);
		#endif
	}
#endif
	if(takePicFlag==1){
		#ifdef HAS_CAM
		if(takeImage(0,0,1)==0)storeImage();
		#endif
		takePicFlag=0;
	}

	archAddrBase = 0x480000 + ((monthNow-1)*0x20000) + ((dayNow-1)*0x1000);
	archAddrMot = archAddrBase + ((dayMin/6)<<4);
	archAddrMot &= 0xFFFFFFF0;
	archAddrGPS = archAddrBase + 3840 + (hourNow<<3);
	archAddrGPS &= 0xFFFFFFF0;
/****************** USB ***************************************/
// 	ioport_set_pin_dir(SRAMPWRC_GPIO, IOPORT_DIR_OUTPUT);
// 	ioport_set_pin_level(SRAMPWRC_GPIO, SRAMPWRC_POWER_OFF);
// 	configure_console();  //pmc_enable_pllack messes up the console speed
// 	configure_console();  //pmc_enable_pllack messes up the console speed
// 	tttt = 0;
// 	SDRAMtest();
// 	redBlink(20);
	
//SCB_DisableDCache();//for SD to work twice in a row, have to enable DCache before cleanInvalidate,disable.
//floatCamForSDCard();
//fileTest();
//oneDayToSD(44,0);

// #ifdef DO_DIAGS
// 	printf("LAUNCH USB STACK\r\n")	;
// #endif
// 	SCB_EnableDCache();//for SD to work twice in a row, have to enable DCache before cleanInvalidate,disable.
// 	SCB_CleanInvalidateDCache();
// 	ioport_set_pin_level(CPWRC_GPIO,1);
// 	ioport_set_pin_dir(GPIO_PA8, IOPORT_DIR_OUTPUT);//SRAM PWRC
// 	ioport_set_pin_level(GPIO_PA8, IOPORT_PIN_LEVEL_HIGH);//SRAM PWRC
// 	delay_ms(1);
// 	SCB_DisableDCache();//for SD to work twice in a row, have to enable DCache before cleanInvalidate,disable.
// 
// 	cpu_irq_enable();
// 	udc_start();// Start USB stack to authorize VBus monitoring
// 
// 	SCB_EnableDCache();//for SD to work twice in a row, have to enable DCache before cleanInvalidate,disable.
// 	SCB_CleanInvalidateDCache();
// 	// 	delay_ms(1);//Second byte read after cache clean is zero unless a delay is here.
// 	while (true) {
// 		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
// 		printf("x");
// 		delay_ms(100);
// 		sleepmgr_enter_sleep();
// 	}
// 	while (true) {
// 		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
// 		printf("MAIN LOOP: %d %d %d %d\r\n",g_usbFlagA,g_usbFlagB,g_usbFlagC,g_usbFlagD);
// 		delay_ms(1000);
// 		//		sleepmgr_enter_sleep();
// 	}

/****************** END USB ***********************************/
	
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);

	supc_set_wakeup_inputs(SUPC,SUPC_WUIR_WKUPEN1_ENABLE|SUPC_WUIR_WKUPEN2_ENABLE,SUPC_WUIR_WKUPT1_LOW|SUPC_WUIR_WKUPT2_LOW);//A1=1 from TINY.
	//	supc_set_wakeup_inputs(SUPC,SUPC_WUIR_WKUPEN2_ENABLE,SUPC_WUIR_WKUPT2_LOW);//PA2 = New RTC Interrupt
	sleepmgr_init();
	sleepmgr_lock_mode(SLEEPMGR_BACKUP);
	pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOD);
	supc_set_wakeup_inputs(SUPC,SUPC_WUIR_WKUPEN1_ENABLE|SUPC_WUIR_WKUPEN2_ENABLE,SUPC_WUIR_WKUPT1_LOW|SUPC_WUIR_WKUPT2_LOW);//A1=1 from TINY.
	//	supc_set_wakeup_inputs(SUPC,SUPC_WUIR_WKUPEN2_ENABLE,SUPC_WUIR_WKUPT2_LOW);//PA2 = New RTC Interrupt
	sleepmgr_init();
	sleepmgr_lock_mode(SLEEPMGR_BACKUP);
#ifdef DO_DIAGS
	configure_console();  //pmc_enable_pllack messes up the console speed
	printf("SLEEP\r\n");
	delay_ms(100);
#endif
	sleepPins();//2019 eNest: writing once gets 100uA, twice gets 44 uA (after 3.6VReg).
	sleepPins();//2019 eNest: writing once gets 100uA, twice gets 44 uA (after 3.6VReg).
	sleepmgr_enter_sleep();

}
/********** END MAIN ******************************************************************************************************
*****************************************************************************************************************
*****************************************************************************************************************
*****************************************************************************************************************
*****************************************************************************************************************/
/****************************************************************************************************************
 * USB FUNCTIONS
 ****************************************************************************************************************/
void main_suspend_action(void)
{
	ui_powerdown();
	g_usbFlagA=1;
}

void main_resume_action(void)
{
	ui_wakeup();
	g_usbFlagB=2;
}

void main_sof_action(void)
{
	if (!main_b_vendor_enable)
		return;
	ui_process(udd_get_frame_number());
}

bool main_vendor_enable(void)
{
	main_b_vendor_enable = true;
	// Start data reception on OUT endpoints
#if UDI_VENDOR_EPS_SIZE_INT_FS
	main_vendor_int_in_received(UDD_EP_TRANSFER_OK, 0, 0);
	g_usbFlagC=3;

#endif
#if UDI_VENDOR_EPS_SIZE_BULK_FS
	main_vendor_bulk_in_received(UDD_EP_TRANSFER_OK, 0, 0);
#endif
#if UDI_VENDOR_EPS_SIZE_ISO_FS
	main_buf_iso_sel=0;
	main_vendor_iso_out_received(UDD_EP_TRANSFER_OK, 0, 0);
#endif
	return true;
}

void main_vendor_disable(void)
{
	main_b_vendor_enable = false;
//	g_usbFlagE=5;
}
/****************************************************************************************************************
 * USB SETUP FUNCTIONS
 ****************************************************************************************************************/
bool main_setup_out_received(void)
{
//	ui_loop_back_state(true);
	udd_g_ctrlreq.payload = main_buf_loopback;
	udd_g_ctrlreq.payload_size = min(
			udd_g_ctrlreq.req.wLength,
			sizeof(main_buf_loopback));
	return true;
}

bool main_setup_in_received(void)
{
//	ui_loop_back_state(false);
	udd_g_ctrlreq.payload = main_buf_loopback;
	udd_g_ctrlreq.payload_size =
			min( udd_g_ctrlreq.req.wLength,
			sizeof(main_buf_loopback) );
	return true;
}
/****************************************************************************************************************
 * USB INTERRUPT IN
 ****************************************************************************************************************/
#if UDI_VENDOR_EPS_SIZE_INT_FS
void main_vendor_int_in_received(udd_ep_status_t status,
iram_size_t nb_transfered, udd_ep_id_t ep)
{
	UNUSED(nb_transfered);
	UNUSED(ep);
	if (UDD_EP_TRANSFER_OK != status) {
		return; // Transfer aborted, then stop loopback
	}
	// Wait a full buffer
//		SCB_CleanInvalidateDCache();
	udi_vendor_interrupt_out_run(
	main_buf_loopback,
	sizeof(main_buf_loopback),
	main_vendor_int_out_received);
	if((main_buf_loopback[0]=='W')&&(main_buf_loopback[1]=='D')&&(main_buf_loopback[2]=='S')){
		switch(main_buf_loopback[5]){
		case 2:
			redBlink(main_buf_loopback[17]);
			printf("Blinked\r\n");
			break;
		case 1:
			g_imageSize = main_buf_loopback[17];
			takeImage(0, main_buf_loopback[14], main_buf_loopback[17]);			
			break;
		case 5:
			receiveUSBcamConfig();
			break;
		}
	}
	printf("IN: %d %d %d %d %d %d %d\r\n",main_buf_loopback[0],main_buf_loopback[1],main_buf_loopback[2],main_buf_loopback[3],main_buf_loopback[4],main_buf_loopback[5],main_buf_loopback[6]);
}
/****************************************************************************************************************
 * USB INTERRUPT OUT
 ****************************************************************************************************************/
void main_vendor_int_out_received(udd_ep_status_t status,
iram_size_t nb_transfered, udd_ep_id_t ep)
{
	UNUSED(ep);
	if (UDD_EP_TRANSFER_OK != status) {
		return; // Transfer aborted, then stop loopback
	}
	// Send on IN endpoint the data received on endpoint OUT
//		SCB_CleanInvalidateDCache();
	udi_vendor_interrupt_in_run(
	main_buf_loopback,
	nb_transfered,
	main_vendor_int_in_received);
	printf("OUT: %d %d %d %d %d %d %d\r\n",main_buf_loopback[0],main_buf_loopback[1],main_buf_loopback[2],main_buf_loopback[3],main_buf_loopback[4],main_buf_loopback[5],main_buf_loopback[6]);
	
}

#endif

#if UDI_VENDOR_EPS_SIZE_BULK_FS
/****************************************************************************************************************
 * USB BULK IN FUNCTION (happens first, calls BULK OUT which will have the image data)
 ****************************************************************************************************************/
void main_vendor_bulk_in_received(udd_ep_status_t status,	//BULK IN: This one hits first
		iram_size_t nb_transfered, udd_ep_id_t ep)
{
	UNUSED(nb_transfered);
	UNUSED(ep);
	if (UDD_EP_TRANSFER_OK != status) {
		return; // Transfer aborted, then stop loopback
	}
	// Wait a full buffer
	udi_vendor_bulk_out_run(
			crapBuf,//Buffer on Internal RAM to send or fill. It must be align, then use COMPILER_WORD_ALIGNED.
			20,//
			main_vendor_bulk_out_received);//param callback: NULL or function to call at the end of transfer
	 printf("BULK OUT RCVD: %x %x %x %x %x %x %x %x\r\n",crapBuf[0],crapBuf[1],crapBuf[2],crapBuf[3],crapBuf[4],crapBuf[5],crapBuf[6],crapBuf[7]);
}
/****************************************************************************************************************
 * USB BULK OUT FUNCTION (happens second and contains the image data, called by BULK IN.
 ****************************************************************************************************************/
void main_vendor_bulk_out_received(udd_ep_status_t status,		//BULK OUT: This one hits second. TO DEVICE, but DATA from device
		iram_size_t nb_transfered, udd_ep_id_t ep)
{
	UNUSED(ep);
	if (UDD_EP_TRANSFER_OK != status) {
		return; // Transfer aborted, then stop loopback
	}
	// Send on IN endpoint 
	if(g_imageSize==0){
	udi_vendor_bulk_in_run(
			picBuff,//picBuff//Buffer on Internal RAM to send or fill. It must be align, then use COMPILER_WORD_ALIGNED.
			IMAGE_BYTE_SIZE_565,//num bytes transferred from device to host during
			main_vendor_bulk_in_received);//param callback: NULL or function to call at the end of transfer
 	 printf("BULK IN DATA SENT 565: %x %x %x %x %x %x %x %x\r\n",picBuff[0],picBuff[1],picBuff[2],picBuff[3],picBuff[4],picBuff[5],picBuff[6],picBuff[7]);
	}else{
	udi_vendor_bulk_in_run(
			picBuff_2,//picBuff//Buffer on Internal RAM to send or fill. It must be align, then use COMPILER_WORD_ALIGNED.
			IMAGE_BYTE_SIZE_888,//num bytes transferred from device to host during
			main_vendor_bulk_in_received);//param callback: NULL or function to call at the end of transfer
			printf("BULK IN DATA SENT 888: %x %x %x %x %x %x %x %x\r\n",picBuff[0],picBuff[1],picBuff[2],picBuff[3],picBuff[4],picBuff[5],picBuff[6],picBuff[7]);
		
	}
}
#endif

#if UDI_VENDOR_EPS_SIZE_ISO_FS
void main_vendor_iso_in_received(udd_ep_status_t status,
		iram_size_t nb_transfered, udd_ep_id_t ep)
{
	UNUSED(status);
	UNUSED(nb_transfered);
	UNUSED(ep);
	ui_loop_back_state(false);
}

void main_vendor_iso_out_received(udd_ep_status_t status,
		iram_size_t nb_transfered, udd_ep_id_t ep)
{
	uint8_t *buf_ptr;
	UNUSED(ep);

	if (UDD_EP_TRANSFER_OK != status) {
		return; // Transfer aborted, then stop loopback
	}

	if (nb_transfered) {
		ui_loop_back_state(true);
		// Send on IN endpoint the data received on endpoint OUT
		buf_ptr = &main_buf_loopback[ main_buf_iso_sel
				*(sizeof(main_buf_loopback)/2) ];
		udi_vendor_iso_in_run(
				buf_ptr,
				nb_transfered,
				main_vendor_iso_in_received);
	}

	// Switch of buffer
	main_buf_iso_sel = main_buf_iso_sel? 0:1;

	// Immediately enable a transfer on next USB isochronous OUT packet
	// to avoid to skip a USB packet.
	// NOTE:
	// Here the expected buffer size is equal to endpoint size.
	// Thus, this transfer request will end after reception of
	// one USB packet.
	//
	// When using buffer size larger than endpoint size,
	// the requested transfer is stopped when the buffer is = full*.
	// *on USBC and XMEGA USB driver, the buffer is full
	// when "number of data transfered" > "buffer size" - "endpoint size".
	buf_ptr = &main_buf_loopback[ main_buf_iso_sel
			*(sizeof(main_buf_loopback)/2) ];

	// Send on IN endpoint the data received on endpoint OUT
	udi_vendor_iso_out_run(
			buf_ptr,
			udd_is_high_speed()?
				UDI_VENDOR_EPS_SIZE_ISO_HS:UDI_VENDOR_EPS_SIZE_ISO_FS,
			main_vendor_iso_out_received);
}
#endif
/****************************************************************************************************************
 * USB RECEIVE CAMERA CONFIG FILE
 ****************************************************************************************************************/
void receiveUSBcamConfig(void){
	short iters;
	for(iters=0;iters<120;iters++){
		printf("%x",main_buf_loopback[iters]);
	}
}
/****************************************************************************************************************
 * SD Card Test
 ****************************************************************************************************************/
void oneDayToSD(unsigned int archiveDay,unsigned int test){
	/**********************************************************/
	/*  DON'T RUN SLEEP PINS BEFORE CALLING THIS.             */
	/*  (ISI PIN RESET OR D11,12,21 MESS UP SDCARD FUNCTIONS) */
	/**********************************************************/
	Ctrl_status status;
	FRESULT res;
	FATFS fs;
	FIL file_object;
	DIR myDir;
	char errorLoopCtr,isCard;
	char fakeMSec;
	unsigned char rtcData[7];// = {0x00,0x00,0xFF,0xFF};//!< Define data buffer
	unsigned int appendJump;
	int spValue;
	
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
	twi_init_RTC();
	delay_ms(100);//
	rtcData[0]=0x03;//
	write_data_RTC(rtcData,1);//!< write sequentially to the slave "1" sends addr plus data[0].
	delay_ms(1);//
 	read_bytes_RTC(rtcData,7);//!< write sequentially to the slave "1" sends addr plus data[0].*/
 	getSetDeviceID();
 	rtc_set_date(RTC,(unsigned int)(bcdToHex(rtcData[0])+19),bcdToHex(rtcData[5]),bcdToHex(rtcData[3]),5);//Flag to avoid bogus downloads or other interruptions from interfering with initial GPS)
 	rtc_set_time(RTC,bcdToHex(rtcData[2]),bcdToHex(rtcData[1]),bcdToHex(rtcData[0]));//[RTC,hourNow,minsNow,secNow] Set these as a counter to determine if stuck in a stupid boot-up loop. )

	const char fdtFolderName[] = {'F','D','T','_','D','A','T',0};
	char csvFileName[] = {'F','D','T','_','D','A','T','/',
	DEVICEID[0],DEVICEID[1],DEVICEID[2],DEVICEID[3],DEVICEID[4],DEVICEID[5],DEVICEID[6],DEVICEID[7],
	DEVICEID[8],DEVICEID[9],DEVICEID[10],DEVICEID[11],DEVICEID[12],DEVICEID[13],DEVICEID[14],DEVICEID[15],
	'_',(char)(hex1ToAscii((bcdToHex(rtcData[6]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[6]))%10)),
	(char)(hex1ToAscii((bcdToHex(rtcData[5]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[5]))%10)),
	(char)(hex1ToAscii((bcdToHex(rtcData[3]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[3]))%10)),
	(char)(hex1ToAscii((bcdToHex(rtcData[2]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[2]))%10)),
	(char)(hex1ToAscii((bcdToHex(rtcData[1]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[1]))%10)),
	'_',0x30,0x30,'.','C','S','V',0
	};
	

#ifdef DO_DIAGS
	configure_console();
	delay_ms(10);
	printf("BEGIN ONE DAY APPEND TO CSV %X.\r\n",archiveDay);
#endif
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
	errorLoopCtr=50;
	resetSD(300);
	SCB_EnableDCache();//for SD to work twice in a row, have to enable DCache before cleanInvalidate,disable.
	SCB_CleanInvalidateDCache();
	SCB_DisableDCache();
	sd_mmc_init();
	do {
		status = sd_mmc_test_unit_ready(0);//first electronic signals
		printf("MMC %d %d\r\n",status, errorLoopCtr);
		isCard = sd_mmc_check(0);
		if  ((status == CTRL_FAIL)&&((errorLoopCtr%4)==0)) {
			printf("MMC RESET %d %d\r\n",status, errorLoopCtr);
			if(errorLoopCtr<20)resetSD(2000);else resetSD(300);//number of msec to remain off
			SCB_EnableDCache();//for SD to work twice in a row, have to enable DCache before cleanInvalidate,disable.
			SCB_CleanInvalidateDCache();
			SCB_DisableDCache();
			delay_ms(10);
			sd_mmc_init();
			WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
		}
		delay_ms(100);
		errorLoopCtr--;
	} while ((status != CTRL_GOOD)&&(errorLoopCtr>0));
	
#ifdef DO_DIAGS
	printf("MMC INIT %d\r\n",status);
	printf("MOUNT DISK...\r\n");
#endif
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
	memset(&fs, 0, sizeof(FATFS));
	res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
#ifdef DO_DIAGS
	printf("MOUNT DISK RESULT %d\r\n",res);
#endif
	if (FR_INVALID_DRIVE == res) {
		goto oneDay_end_of_test;
	}
#ifdef DO_DIAGS
//	printf("TRYING F_STAT()\r\n");
	printf("TRYING F_OPENDIR()\r\n");
#endif
//	res = f_stat(fdtFolderName,&fileInfo);
	res = f_opendir(&myDir,fdtFolderName);
#ifdef DO_DIAGS
//	printf("F_STAT() RES = %d\r\n",res);
	printf("F_OPENDIR() RES = %d\r\n",res);
#endif
	switch (res){
		case FR_OK:
#ifdef DO_DIAGS
		printf("FOUND FDT FOLDER\r\n");
#endif
		break;
		case FR_NO_FILE:
		printf("CREATING FDT FOLDER\r\n");
		f_mkdir((char const *)fdtFolderName);
		break;
		case FR_NO_PATH:
		printf("CREATING FDT FOLDER\r\n");
		f_mkdir((char const *)fdtFolderName);
		break;
		default:
		printf("SOME OTHER DIRECTORY ERROR %X \r\n",res);
	}
#ifdef DO_DIAGS	
	if(res==0)printf("F_MOUNT GOOD FDT\r\n");  
#endif	
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
	fakeMSec=0;res=0;
	while((fakeMSec<100)&&(res!=FR_NO_FILE)){

		res = f_open(&file_object,
		(char const *)csvFileName,
		FA_OPEN_EXISTING);
		//	printf("\r\nSEARCH RESULT = ");
		if(res==FR_NO_FILE)fakeMSec=101;
		else{
#ifdef DO_DIAGS
//			printf("FOUND %d\r\n",fakeMSec);
#endif					
			delay_ms(100);//March 2021 discovered that without these delays it sometimes runs through all 100 MSec filenames while failing.
			f_close(&file_object);
			delay_ms(100);//March 2021 discovered that without these delays it sometimes runs through all 100 MSec filenames while failing.
			fakeMSec++;
			csvFileName[36]=(char)(hex1ToAscii(fakeMSec/10));
			csvFileName[37]=(char)(hex1ToAscii(fakeMSec%10));
		}
	}
 	res = f_open(&file_object, (char const *)csvFileName, FA_CREATE_ALWAYS | FA_WRITE);
	if (res != FR_OK) {
		printf("F_OPEN FAIL res %d\r\n", res);
		goto oneDay_end_of_test;
	}
#ifdef DO_DIAGS
	printf("CREATED CSV FILE RES= %d\r\n",res);
	printf("%s\r\n",csvFileName);
#endif
//	if(appendJump==0){
		f_puts(oneDayHeader,&file_object);
// 		for(picBuffCtr=0;picBuffCtr<ONEDAYHEADERLEN;picBuffCtr++){
// 			f_putc(oneDayHeader[picBuffCtr],&file_object);
// 		}
//	}
	char *archiveBuffPtr,paramCtr,recFormatVers;
	int recordCtr;
	archiveBuffPtr=ARCHIVEADDR + ((bcdToHex(archiveDay)-1)*46080);//0x480020;
	int temperatureHigh,temperatureLow;
	int barom,humidityHB,humidityVal,lightHB,lightVal;
#ifdef DO_DIAGS
	printf("ARCHIVE FROM: %X\r\n",archiveBuffPtr);
#endif
	char latSign=0;
	char latWhole=0;
	unsigned int latFrac=0;
	unsigned int latFrac6=0;
	char longSign=0;
	char longWhole=0;
	unsigned int longFrac=0;
	unsigned int longFrac6=0;
	char batt=0;
	char asciiChars[5];
	char minCycle;
	char GPSyear=0;
	char GPSmonth=0;
	char GPSday=0;
	char GPShour=0;
	char GPSminute=0;
	char GPSsecond=0;
	unsigned short GPSttff=0;
	char GPSnumSats=0;
	char magnetometer=0;
	char hexLocHour,thisDay;
	char temp;
	for(recordCtr=0;recordCtr<1440;recordCtr++){//should be 1440
		temperatureHigh=((int)((*archiveBuffPtr)&0x00E0))<<3;
		humidityHB=((int)((*archiveBuffPtr)&0x0018))<<5;
		lightHB=((int)((*archiveBuffPtr)&0x0006))<<7;
	//Record Format Version
		recFormatVers=(*archiveBuffPtr)&0x01;
		hex2ToAscii(hexToBCD(recFormatVers),asciiChars);
		temp=asciiChars[0];
		asciiChars[0]=asciiChars[2];
		asciiChars[2]=temp;
		asciiChars[3]=',';
		asciiChars[4]=0;
		f_puts((const char*)asciiChars,&file_object);
		archiveBuffPtr++;
	//Date GMT
		f_putc('2',&file_object);//
		f_putc('0',&file_object);//
		f_putc((char)(hex1ToAscii((bcdToHex(rtcData[6]))/10)),&file_object);//Year HB from RTC
		f_putc((char)(hex1ToAscii((bcdToHex(rtcData[6]))%10)),&file_object);//Year LB from RTC
		f_putc('-',&file_object);//
		f_putc((char)(hex1ToAscii((bcdToHex(rtcData[5]))/10)),&file_object);//Month HB from RTC
		f_putc((char)(hex1ToAscii((bcdToHex(rtcData[5]))%10)),&file_object);//Month LB from RTC
		f_putc('-',&file_object);//
		thisDay=*archiveBuffPtr++;
		hex2ToAscii(thisDay,asciiChars);//Date is already in BCD
		f_putc(asciiChars[1],&file_object);//Date from flash archive
		f_putc(asciiChars[0],&file_object);////Date from flash archive
		f_putc(',',&file_object);//THIS BYTE IS READ FIRST BY WINDOWS.
	//Time GMT
		hexLocHour=*archiveBuffPtr++;
		hex2ToAscii(hexLocHour,asciiChars);//hexLocHour is still BCD at this line
		hexLocHour=bcdToHex(hexLocHour);
		f_putc(asciiChars[1],&file_object);//Hour HB from flash archive
		f_putc(asciiChars[0],&file_object);//Hour LB from flash archive
		f_putc(':',&file_object);//THIS BYTE IS READ FIRST BY WINDOWS.
		minCycle=(bcdToHex(*archiveBuffPtr))&0x1F;
		hex2ToAscii(*archiveBuffPtr,asciiChars);//date and time is already in BCD
		f_putc(asciiChars[1],&file_object);//Minute HB from flash archive
		f_putc(asciiChars[0],&file_object);//Minute LB from flash archive
		f_putc(',',&file_object);//THIS BYTE IS READ FIRST BY WINDOWS.
	//Time Local
		hexLocHour=hexLocHour+24+TIMEZONE_OFFSET;
#ifdef HAS_DAYLIGHTSAVINGS
		char beforeNov2 = 0;
		char afterMar7 = 0;
		if(rtcData[5]<18)beforeNov2=1;//(18 is BCD 12)
		if((rtcData[5]==11)&&(thisDay>1))beforeNov2=0;
		if(rtcData[5]>3)afterMar7=1;
		if((rtcData[5]==3)&&(thisDay<8))afterMar7=0;
		if((beforeNov2)&&(afterMar7))hexLocHour++;//(18 is BCD 12)
#endif
		if(hexLocHour>23)hexLocHour-=24;
		hexLocHour=hexToBCD(hexLocHour);
		hex2ToAscii(hexLocHour,asciiChars);//hexLocalHour is now BCD again.
		f_putc(asciiChars[1],&file_object);//Hour HB from flash archive
		f_putc(asciiChars[0],&file_object);//Hour LB from flash archive
		f_putc(':',&file_object);//THIS BYTE IS READ FIRST BY WINDOWS.
		minCycle=(bcdToHex(*archiveBuffPtr))&0x1F;
		hex2ToAscii(*archiveBuffPtr++,asciiChars);//date and time is already in BCD
		f_putc(asciiChars[1],&file_object);//Minute HB from flash archive
		f_putc(asciiChars[0],&file_object);//Minute LB from flash archive
		f_putc(',',&file_object);//THIS BYTE IS READ FIRST BY WINDOWS.
	//Motion Total
		hex2ToAscii(hexToBCD(*archiveBuffPtr++),asciiChars);
		f_putc(asciiChars[2],&file_object);//THIS BYTE IS READ SECOND BY WINDOWS. Two bytes per pixel
		f_putc(asciiChars[1],&file_object);//THIS BYTE IS READ SECOND BY WINDOWS. Two bytes per pixel
		f_putc(asciiChars[0],&file_object);//THIS BYTE IS READ SECOND BY WINDOWS. Two bytes per pixel
		f_putc(',',&file_object);//THIS BYTE IS READ FIRST BY WINDOWS.
	//Raw Motion
		f_putc(0x27,&file_object);//apostrophe forces Excel to display as a string
		hex2ToAscii(*archiveBuffPtr++,asciiChars);//Seconds 0..15
		f_putc(asciiChars[1],&file_object);
		f_putc(asciiChars[0],&file_object);
		hex2ToAscii(*archiveBuffPtr++,asciiChars);
		f_putc(asciiChars[1],&file_object);
		f_putc(asciiChars[0],&file_object);
		hex2ToAscii(*archiveBuffPtr++,asciiChars);
		f_putc(asciiChars[1],&file_object);
		f_putc(asciiChars[0],&file_object);
		hex2ToAscii(*archiveBuffPtr++,asciiChars);
		f_putc(asciiChars[1],&file_object);
		f_putc(asciiChars[0],&file_object);
		hex2ToAscii(*archiveBuffPtr++,asciiChars);
		f_putc(asciiChars[1],&file_object);
		f_putc(asciiChars[0],&file_object);
		hex2ToAscii(*archiveBuffPtr++,asciiChars);
		f_putc(asciiChars[1],&file_object);
		f_putc(asciiChars[0],&file_object);
		hex2ToAscii(*archiveBuffPtr++,asciiChars);
		f_putc(asciiChars[1],&file_object);
		f_putc(asciiChars[0],&file_object);
		hex2ToAscii(*archiveBuffPtr++,asciiChars);//Seconds 54..59
		f_putc(asciiChars[1],&file_object);
		f_putc(asciiChars[0],&file_object);
		f_putc(',',&file_object);//THIS BYTE IS READ FIRST BY WINDOWS.
	//Temperature
		temperatureLow=*archiveBuffPtr;
		if((temperatureHigh&0x0400)==0x0400){
			f_putc('-',&file_object);
			temperatureHigh=0x0700-temperatureHigh;
			temperatureLow=0x100 - temperatureLow;
		}
		hex2ToAscii(hexToBCD((short)(temperatureHigh+temperatureLow)),asciiChars);
		f_putc(asciiChars[3],&file_object);//
		f_putc(asciiChars[2],&file_object);//
		f_putc(asciiChars[1],&file_object);//
		f_putc('.',&file_object);//
		f_putc(asciiChars[0],&file_object);//
		f_putc(',',&file_object);//
		archiveBuffPtr++;
	//Humidity
		humidityVal=(*archiveBuffPtr)+humidityHB;
		humidityVal=(humidityVal + (humidityVal>>2))-60;//from datasheet RH = (125*val)/1024 - 6.  which is about (1.25*val) - 60
		hex2ToAscii(hexToBCD(humidityVal),asciiChars);
		f_putc(asciiChars[3],&file_object);//THIS BYTE IS READ SECOND BY WINDOWS. Two bytes per pixel
		f_putc(asciiChars[2],&file_object);//THIS BYTE IS READ SECOND BY WINDOWS. Two bytes per pixel
		f_putc(asciiChars[1],&file_object);//THIS BYTE IS READ SECOND BY WINDOWS. Two bytes per pixel
		f_putc('.',&file_object);
		f_putc(asciiChars[0],&file_object);//THIS BYTE IS READ SECOND BY WINDOWS. Two bytes per pixel
		f_putc(',',&file_object);//THIS BYTE IS READ FIRST BY WINDOWS.
		archiveBuffPtr++;
	//Barometric Pressure
		barom=(((int)(*archiveBuffPtr))&0x000000FF)<<8;
		archiveBuffPtr++;
		barom+=*archiveBuffPtr;
		barom<<=1;
		barom&=0x00003FFF;
		if(barom>9999){
			barom-=10000;
			f_putc('1',&file_object);//
		}
		hex2ToAscii(hexToBCD(barom),asciiChars);
		f_putc(asciiChars[3],&file_object);//
		f_putc(asciiChars[2],&file_object);//
		f_putc(asciiChars[1],&file_object);//
		f_putc('.',&file_object);//
		f_putc(asciiChars[0],&file_object);//
		f_putc(',',&file_object);//
		archiveBuffPtr++;
	//Light
		lightVal=(*archiveBuffPtr)+lightHB;
		hex2ToAscii(hexToBCD(lightVal),asciiChars);
		f_putc(asciiChars[3],&file_object);//
		f_putc(asciiChars[2],&file_object);//THIS BYTE IS READ SECOND BY WINDOWS. Two bytes per pixel
		f_putc(asciiChars[1],&file_object);//THIS BYTE IS READ SECOND BY WINDOWS. Two bytes per pixel
		f_putc(asciiChars[0],&file_object);//THIS BYTE IS READ SECOND BY WINDOWS. Two bytes per pixel
		f_putc(',',&file_object);//THIS BYTE IS READ FIRST BY WINDOWS.
		archiveBuffPtr++;
	//Trip Durations
		for(paramCtr=0;paramCtr<12;paramCtr++){//should be 1440
			if((*archiveBuffPtr)==255){f_putc('-',&file_object);}
			else{
				hex2ToAscii(hexToBCD(*archiveBuffPtr),asciiChars);
				f_putc(asciiChars[2],&file_object);//THIS BYTE IS READ SECOND BY WINDOWS. Two bytes per pixel
				f_putc(asciiChars[1],&file_object);//THIS BYTE IS READ SECOND BY WINDOWS. Two bytes per pixel
				f_putc(asciiChars[0],&file_object);//THIS BYTE IS READ SECOND BY WINDOWS. Two bytes per pixel
			}
			f_putc(',',&file_object);//THIS BYTE IS READ FIRST BY WINDOWS.
			archiveBuffPtr++;
		}
	//Number of Trips
//		if((*archiveBuffPtr)==0){f_putc('0',&file_object);}
//		else{
			hex2ToAscii(hexToBCD(*archiveBuffPtr),asciiChars);
			f_putc(asciiChars[2],&file_object);//THIS BYTE IS READ SECOND BY WINDOWS. Two bytes per pixel
			f_putc(asciiChars[1],&file_object);//THIS BYTE IS READ SECOND BY WINDOWS. Two bytes per pixel
			f_putc(asciiChars[0],&file_object);//THIS BYTE IS READ SECOND BY WINDOWS. Two bytes per pixel
//		}
		f_putc(',',&file_object);//
		archiveBuffPtr++;
		
		switch (minCycle){
			case 0:
				if(((*archiveBuffPtr)|0x80)==0x80)latSign='-';
				else latSign='+';
				latWhole=(*archiveBuffPtr)&0x7F;
				break;
			case 1:
				latFrac=((unsigned int)(*archiveBuffPtr))*10000;
				break;
			case 2:
				latFrac+=((unsigned int)(*archiveBuffPtr))*100;
				break;
			case 3:
				latFrac+=(unsigned int)(*archiveBuffPtr);
				latFrac6=latFrac/6;
				break;
			case 4:
				longSign=*archiveBuffPtr;
				break;
			case 5:
				longWhole=*archiveBuffPtr;
				break;
			case 6:
				longFrac=((unsigned int)(*archiveBuffPtr))*10000;
				break;
			case 7:
				longFrac+=((unsigned int)(*archiveBuffPtr))*100;
				break;
			case 8:
				longFrac+=(unsigned int)(*archiveBuffPtr);
				longFrac6=longFrac/6;
				break;
			case 9:
				GPSyear=*archiveBuffPtr;
				break;
			case 10:
				GPSmonth=*archiveBuffPtr;
				break;
			case 11:
				GPSday=*archiveBuffPtr;
				break;
			case 12:
				GPShour=*archiveBuffPtr;
				break;
			case 13:
				GPSminute=*archiveBuffPtr;
				break;
			case 14:
				GPSsecond=*archiveBuffPtr;
				break;
			case 15:
				batt=*archiveBuffPtr;
				break;
			case 16: batt=*archiveBuffPtr;
				GPSnumSats=*archiveBuffPtr;
				break;
			case 17:
				GPSttff=((unsigned short)(*archiveBuffPtr))<<8;
				break;
			case 18:
				GPSttff+=(unsigned short)(*archiveBuffPtr);
				break;
			case 19:
				magnetometer=*archiveBuffPtr;
				break;
			default: break;
		}
		
	//Longitude
		f_putc(longSign,&file_object);//
		hex2ToAscii(hexToBCD(longWhole),asciiChars);
		f_putc(asciiChars[2],&file_object);//
		f_putc(asciiChars[1],&file_object);//
		f_putc(asciiChars[0],&file_object);//
		f_putc('.',&file_object);//
		hex2ToAscii(hexToBCD((unsigned int)(longFrac6)),asciiChars);
		f_putc(asciiChars[4],&file_object);
		f_putc(asciiChars[3],&file_object);
		f_putc(asciiChars[2],&file_object);
		f_putc(asciiChars[1],&file_object);
		f_putc(asciiChars[0],&file_object);
		f_putc(',',&file_object);
	//Latitude
		f_putc(latSign,&file_object);//
		hex2ToAscii(hexToBCD(latWhole),asciiChars);//
		f_putc(asciiChars[2],&file_object);//
		f_putc(asciiChars[1],&file_object);//
		f_putc(asciiChars[0],&file_object);//
		f_putc('.',&file_object);//.
		hex2ToAscii(hexToBCD((unsigned int)(latFrac6)),asciiChars);
		f_putc(asciiChars[4],&file_object);
		f_putc(asciiChars[3],&file_object);
		f_putc(asciiChars[2],&file_object);
		f_putc(asciiChars[1],&file_object);
		f_putc(asciiChars[0],&file_object);
		f_putc(',',&file_object);
	//GPS Date
		f_putc('2',&file_object);//
		f_putc('0',&file_object);//
		hex2ToAscii(hexToBCD(GPSyear),asciiChars);		
		f_putc(asciiChars[1],&file_object);//
		f_putc(asciiChars[0],&file_object);//
		f_putc('-',&file_object);//
		hex2ToAscii(hexToBCD(GPSmonth),asciiChars);
		f_putc(asciiChars[1],&file_object);//
		f_putc(asciiChars[0],&file_object);//
		f_putc('-',&file_object);//
		hex2ToAscii(hexToBCD(GPSday),asciiChars);
		f_putc(asciiChars[1],&file_object);//
		f_putc(asciiChars[0],&file_object);//
		f_putc(',',&file_object);//
	//GPS Time
		hex2ToAscii(hexToBCD(GPShour),asciiChars);
		f_putc(asciiChars[1],&file_object);//
		f_putc(asciiChars[0],&file_object);//
		f_putc(':',&file_object);//
		hex2ToAscii(hexToBCD(GPSminute),asciiChars);
		f_putc(asciiChars[1],&file_object);//
		f_putc(asciiChars[0],&file_object);//
		f_putc(':',&file_object);//
		hex2ToAscii(hexToBCD(GPSsecond),asciiChars);
		f_putc(asciiChars[1],&file_object);//
		f_putc(asciiChars[0],&file_object);//
		f_putc(',',&file_object);//
	//Battery
		hex2ToAscii(hexToBCD(batt),asciiChars);//placeholder
		f_putc(asciiChars[2],&file_object);//
		f_putc(asciiChars[1],&file_object);//
		f_putc(asciiChars[0],&file_object);//
		f_putc(',',&file_object);//
	//GPS Num Sats
		hex2ToAscii(hexToBCD(GPSnumSats),asciiChars);//placeholder
		f_putc(asciiChars[2],&file_object);//
		f_putc(asciiChars[1],&file_object);//
		f_putc(asciiChars[0],&file_object);//
		f_putc(',',&file_object);//
	//GPS TTFF
		hex2ToAscii(hexToBCD(GPSttff),asciiChars);//placeholder
		f_putc(asciiChars[3],&file_object);//
		f_putc(asciiChars[2],&file_object);//
		f_putc(asciiChars[1],&file_object);//
		f_putc(asciiChars[0],&file_object);//
		f_putc(',',&file_object);//
	//Magnetometer
		hex2ToAscii(hexToBCD(magnetometer),asciiChars);//placeholder
		f_putc(asciiChars[2],&file_object);//
		f_putc(asciiChars[1],&file_object);//
		f_putc(asciiChars[0],&file_object);//
	//End of Line
		f_putc(0x0D,&file_object);
#ifdef DO_DIAGS
		if((recordCtr%50)==0)printf(".");
#endif
		if((recordCtr%200)==0)WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog;
		archiveBuffPtr++;
	}
	f_close(&file_object);
	closeSDcard();
	ioport_set_pin_level(SDCARDPWRC_GPIO, SDCARDPWRC_POWER_OFF);
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
#ifdef DO_DIAGS	
	printf("\r\nMain Branch. GOOD. FILE CLOSED.\r\n");
#endif	
	oneDay_end_of_test:
	delay_ms(10);
	closeSDcard();
}
/****************************************************************************************************************
 * SD Card STORE IMAGE
 ****************************************************************************************************************/
void storeImage(void){
	Ctrl_status status;
	FRESULT res;
	FATFS fs;
	FIL file_object;
	DIR myDir;
	char fakeMSec;
	unsigned char rtcData[7];// = {0x00,0x00,0xFF,0xFF};//!< Define data buffer
	char errorLoopCtr;
	char oneLine[4800];
	char* oneLinePtr;
	char isCard;

	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
	twi_init_RTC();
	delay_ms(100);//
	rtcData[0]=0x03;//
	write_data_RTC(rtcData,1);//!< write sequentially to the slave "1" sends addr plus data[0].
	delay_ms(1);//
	read_bytes_RTC(rtcData,7);//!< write sequentially to the slave "1" sends addr plus data[0].*/
	rtc_set_date(RTC,(unsigned int)(bcdToHex(rtcData[0])+19),bcdToHex(rtcData[5]),bcdToHex(rtcData[3]),5);//Flag to avoid bogus downloads or other interruptions from interfering with initial GPS)
	rtc_set_time(RTC,bcdToHex(rtcData[2]),bcdToHex(rtcData[1]),bcdToHex(rtcData[0]));//[RTC,hourNow,minsNow,secNow] Set these as a counter to determine if stuck in a stupid boot-up loop. )
	getSetDeviceID();
	
	printf(" %d%d\r\n",rtcData[2],rtcData[1]);
	
	const char fdtFolderName[] = {'F','D','T','_','I','M','G',0};
	char picFileName[] = {'F','D','T','_','I','M','G','/',
	DEVICEID[0],DEVICEID[1],DEVICEID[2],DEVICEID[3],DEVICEID[4],DEVICEID[5],DEVICEID[6],DEVICEID[7],
	DEVICEID[8],DEVICEID[9],DEVICEID[10],DEVICEID[11],DEVICEID[12],DEVICEID[13],DEVICEID[14],DEVICEID[15],
	'_',(char)(hex1ToAscii((bcdToHex(rtcData[6]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[6]))%10)),
	(char)(hex1ToAscii((bcdToHex(rtcData[5]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[5]))%10)),
	(char)(hex1ToAscii((bcdToHex(rtcData[3]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[3]))%10)),
	(char)(hex1ToAscii((bcdToHex(rtcData[2]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[2]))%10)),
	(char)(hex1ToAscii((bcdToHex(rtcData[1]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[1]))%10)),
	(char)(hex1ToAscii((bcdToHex(rtcData[0]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[0]))%10)),'_',0x30,0x30,'.','B','M','P',0
	};

#ifdef DO_DIAGS
	configure_console();
	delay_ms(10);
	printf("BEGIN STORE IMAGE\r\n");
#endif
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
	errorLoopCtr=50;
	resetSD(300);
	SCB_EnableDCache();//for SD to work twice in a row, have to enable DCache before cleanInvalidate,disable.
	SCB_CleanInvalidateDCache();
	SCB_DisableDCache();
	sd_mmc_init();
	do {
		status = sd_mmc_test_unit_ready(0);//first electronic signals
		printf("MMC %d %d\r\n",status, errorLoopCtr);
		isCard = sd_mmc_check(0);
		if  ((status == CTRL_FAIL)&&((errorLoopCtr%4)==0)) {
			printf("MMC RESET %d %d\r\n",status, errorLoopCtr);
			if(errorLoopCtr<20)resetSD(2000);else resetSD(300);//number of msec to remain off
			SCB_EnableDCache();//for SD to work twice in a row, have to enable DCache before cleanInvalidate,disable.
			SCB_CleanInvalidateDCache();
			SCB_DisableDCache();
			delay_ms(10);
			sd_mmc_init();
			WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
		}
		delay_ms(100);
		errorLoopCtr--;
	} while ((status != CTRL_GOOD)&&(errorLoopCtr>0));
	
#ifdef DO_DIAGS
	printf("MMC INIT %d\r\n",status);
	printf("MOUNT DISK...\r\n");
#endif
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
	memset(&fs, 0, sizeof(FATFS));
	res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
#ifdef DO_DIAGS
	printf("MOUNT DISK RESULT %d\r\n",res);
#endif
	if (FR_INVALID_DRIVE == res) {
		goto main_end_of_imgTest;
	}
#ifdef DO_DIAGS
	//	printf("TRYING F_STAT()\r\n");
	printf("TRYING F_OPENDIR()\r\n");
#endif
	//	res = f_stat(fdtFolderName,&fileInfo);
	res = f_opendir(&myDir,fdtFolderName);
#ifdef DO_DIAGS
	//	printf("F_STAT() RES = %d\r\n",res);
	printf("F_OPENDIR() RES = %d\r\n",res);
#endif
	switch (res){
		case FR_OK:
		#ifdef DO_DIAGS
		printf("FOUND FDT_IMG FOLDER\r\n");
		#endif
		break;
		case FR_NO_FILE:
		printf("CREATING FDT_IMG FOLDER\r\n");
		f_mkdir((char const *)fdtFolderName);
		break;
		case FR_NO_PATH:
		printf("CREATING FDT_IMG FOLDER\r\n");
		f_mkdir((char const *)fdtFolderName);
		break;
		default:
		printf("SOME OTHER DIRECTORY ERROR %X \r\n",res);
	}
#ifdef DO_DIAGS
	if(res==0)printf("F_MOUNT GOOD FDT\r\n");
#endif
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
	fakeMSec=0;res=0;
	while((fakeMSec<100)&&(res!=FR_NO_FILE)){
		res = f_open(&file_object,
		(char const *)picFileName,
		FA_OPEN_EXISTING);
		if(res==FR_NO_FILE)fakeMSec=101;
		else{
			//		printf("FOUND THAT ONE\r\n");
			delay_ms(100);
			f_close(&file_object);
			delay_ms(100);
			fakeMSec++;
			picFileName[38]=(char)(hex1ToAscii(fakeMSec/10));
			picFileName[39]=(char)(hex1ToAscii(fakeMSec%10));
		}
	}
	res = f_open(&file_object, (char const *)picFileName, FA_CREATE_ALWAYS | FA_WRITE);
	res = f_open(&file_object,
	(char const *)picFileName,
	FA_CREATE_ALWAYS | FA_WRITE);
	if (res != FR_OK) {
		printf("F_OPEN FAIL RES %d\r\n", res);
		goto main_end_of_imgTest;
	}
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
#ifdef DO_DIAGS
	printf("CREATED PIC FILE %s\r\n",picFileName);
#endif

	int picBuffCtr,picBlockCtr;
	for(picBuffCtr=0;picBuffCtr<PIC_HEADER_LEN;picBuffCtr++){
	f_putc(picHeader_2655[picBuffCtr],&file_object);
	}
#ifdef DO_DIAGS
	printf("WROTE FILE NAME\r\n",picFileName);
#endif

	char *camBuffPtrA,*camBuffPtrB;
	camBuffPtrA=SRAM_BASE;//
	char firstByte,secondByte,redByte,greenByte,blueByte;
	/******* RGB565 Interlaced****************/
	camBuffPtrA=SRAM_BASE;//
//	camBuffPtrB=SRAM_BASE+1920000;//
    camBuffPtrB = SRAM_BASE + 1913600;//used to be 1920000 during 2020 nest season
	char fwret;
// #ifdef DO_CONTINUOUS_PICS
// 	for(picBlockCtr=0;picBlockCtr<3;picBlockCtr++){
// #else
	for(picBlockCtr=0;picBlockCtr<600;picBlockCtr++){
//#endif		
		oneLinePtr=oneLine;
		for(picBuffCtr=0;picBuffCtr<1600;picBuffCtr++){
			firstByte=*camBuffPtrA++;
			secondByte=*camBuffPtrA++;
			blueByte=(firstByte&0xF8);
			greenByte=((firstByte&0x07)<<5)+((secondByte&0xE0)>>3);
			redByte=(secondByte&0x1F)<<3;
			*oneLinePtr++=blueByte;
			*oneLinePtr++=greenByte;
			*oneLinePtr++=redByte;
		}
		f_write(&file_object,oneLine,4800,&fwret);
		oneLinePtr=oneLine;
		for(picBuffCtr=0;picBuffCtr<1600;picBuffCtr++){
			firstByte=*camBuffPtrB++;
			secondByte=*camBuffPtrB++;
			blueByte=(firstByte&0xF8);
			greenByte=((firstByte&0x07)<<5)+((secondByte&0xE0)>>3);
			redByte=(secondByte&0x1F)<<3;
			*oneLinePtr++=blueByte;
			*oneLinePtr++=greenByte;
			*oneLinePtr++=redByte;
	}
	f_write(&file_object,oneLine,4800,&fwret);
#ifdef DO_DIAGS
	if((picBlockCtr%30)==0)printf(".");
#endif

	if((picBlockCtr%30)==0)	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog

	}
	f_close(&file_object);
	closeSDcard();
	ioport_set_pin_level(SRAMPWRC_GPIO, SRAMPWRC_POWER_OFF);
#ifdef DO_DIAGS
	printf("\r\nCREATED PIC FILE. FILE CLOSED.\r\n");
#endif
	main_end_of_imgTest:
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog;
	delay_ms(10);
	closeSDcard();//pin cnfig and power off.
#ifdef DO_DIAGS
	printf("\r\CLOSED SD CARD.\r\n");
	delay_ms(200);
#endif
	}	
	/*********************************************/
	/* DON'T RUN SLEEP PINS BEFORE CALLING THIS. */
	/*********************************************/
// 	Ctrl_status status;
// 	FRESULT res;
// 	FATFS fs;
// 	FIL file_object;
// 	FILINFO* fileInfo;
// 	DIR* myDir;
// 	char fakeYr,fakeMo,fakeDay,fakeHr,fakeMin,fakeSec,fakeMSec;
// 	unsigned char rtcData[7];// = {0x00,0x00,0xFF,0xFF};//!< Define data buffer
// 	char errorLoopCtr;
// 	char oneLine[4800];
// 	char* oneLinePtr;
// 
// 	twi_init_RTC();
// 	delay_ms(100);//
// 	rtcData[0]=0x03;//
// 	write_data_RTC(rtcData,1);//!< write sequentially to the slave "1" sends addr plus data[0].
// 	delay_ms(1);//
// 	read_bytes_RTC(rtcData,7);//!< write sequentially to the slave "1" sends addr plus data[0].*/
// 	rtc_set_date(RTC,(unsigned int)(bcdToHex(rtcData[0])+19),bcdToHex(rtcData[5]),bcdToHex(rtcData[3]),5);//Flag to avoid bogus downloads or other interruptions from interfering with initial GPS)
// 	rtc_set_time(RTC,bcdToHex(rtcData[2]),bcdToHex(rtcData[1]),bcdToHex(rtcData[0]));//[RTC,hourNow,minsNow,secNow] Set these as a counter to determine if stuck in a stupid boot-up loop. )
// 	getSetDeviceID();
// 	
// 	printf(" %d%d\r\n",rtcData[2],rtcData[1]);
// 	
// 	
// 	
// 	char picFileName[] = {'A','n','i','m','a','l','D','S','/',
// 		DEVICEID[0],DEVICEID[1],DEVICEID[2],DEVICEID[3],DEVICEID[4],DEVICEID[5],DEVICEID[6],DEVICEID[7],
// 		DEVICEID[8],DEVICEID[9],DEVICEID[10],DEVICEID[11],DEVICEID[12],DEVICEID[13],DEVICEID[14],DEVICEID[15],
// 		'_',(char)(hex1ToAscii((bcdToHex(rtcData[6]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[6]))%10)),
// 		(char)(hex1ToAscii((bcdToHex(rtcData[5]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[5]))%10)),
// 		(char)(hex1ToAscii((bcdToHex(rtcData[3]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[3]))%10)),
// 		(char)(hex1ToAscii((bcdToHex(rtcData[2]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[2]))%10)),
// 		(char)(hex1ToAscii((bcdToHex(rtcData[1]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[1]))%10)),
// 		(char)(hex1ToAscii((bcdToHex(rtcData[0]))/10)),(char)(hex1ToAscii((bcdToHex(rtcData[0]))%10)),'_',0x30,0x30,'.','B','M','P',0
// 	};
// 
// // 	ioport_set_pin_dir(SRAMPWRC_GPIO, IOPORT_DIR_OUTPUT);
// // 	ioport_set_pin_level(SRAMPWRC_GPIO, SRAMPWRC_POWER_ON);
// #ifdef DO_DIAGS	
// 	printf("BEGIN SD CARD TEST \r\n");
// #endif
// 	errorLoopCtr=5;
// 	do {
// 		ioport_set_pin_dir(SDCARDPWRC_GPIO, IOPORT_DIR_OUTPUT);
// 		ioport_set_pin_level(SDCARDPWRC_GPIO, SDCARDPWRC_POWER_ON);
// 		resetSD();
// 		SCB_EnableDCache();//for SD to work twice in a row, have to enable DCache before cleanInvalidate,disable.
// 		SCB_CleanInvalidateDCache();
// 		SCB_DisableDCache();
// 		sd_mmc_init();
// //		status = sd_mmc_test_unit_ready(0);//first electronic signals
// 		// 		isCard = sd_mmc_check(0);
// 		if  (status == CTRL_FAIL) {
// 			closeSDcard();
// 			ioport_set_pin_level(SDCARDPWRC_GPIO, SDCARDPWRC_POWER_OFF);
// 			WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
// 			delay_ms(2000);
// 			printf("SD CARD ERR LOOP: %d STATUS: %d\r\n", errorLoopCtr,status);
// 		}else errorLoopCtr=1;
// 		errorLoopCtr--;
// 	} while ((status != CTRL_GOOD)&&(errorLoopCtr>0));
// 	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
// 
// 	memset(&fs, 0, sizeof(FATFS));
// 	res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
// //	printFileError(res);
// 	if (FR_INVALID_DRIVE == res) {
// 		printf("f_mount FAIL res %d\r\n", res);
// 		goto main_end_of_test;
// 	}/*else	printf("f_mount GOOD ENEST\r\n");*/  //WORKS
// 	/* Display basic card information */
// 	main_display_info_card(0);		//WORKS
// // 	delay_ms(500);
// // 	printf("%s\r\n",picFileName);//	res = f_stat("AnimalDS",fileInfo);
// 	switch (res){
// 		case FR_OK:
// //			printf("Found folder OKAY\r\n");
// 			break;
// 		case FR_NO_FILE:
// 			printf("creating AnimalDS \r\n");
// 			f_mkdir("AnimalDS");
// 			break;
// 		default:
// 			printf("some other directory error %X \r\n",res);
// 	}
// 	fakeMSec=0;res=0;
// 	while((fakeMSec<100)&&(res!=FR_NO_FILE)){
// 
// 		res = f_open(&file_object,
// 		(char const *)picFileName,
// 		FA_OPEN_EXISTING);
// 
// 	//	printf("\r\nSEARCH RESULT = ");
// 	//	printFileError(res);
// //		printf("%d %s\r\n",fakeMSec,picFileName);
// 		if(res==FR_NO_FILE)fakeMSec=101;
// 		else{
// 	//		printf("FOUND THAT ONE\r\n");
// 			f_close(&file_object);
// 			fakeMSec++;
// 			picFileName[39]=(char)(hex1ToAscii(fakeMSec/10));
// 			picFileName[40]=(char)(hex1ToAscii(fakeMSec%10));
// 		}
// 	}
// 	//	printf("Create a file (f_open)...\r\n");
// 		res = f_open(&file_object,
// 		(char const *)picFileName,
// 		FA_CREATE_ALWAYS | FA_WRITE);
// 	//	printFileError(res);
// 		if (res != FR_OK) {
// 			printf("f_open A FAIL res %d\r\n", res);
// 			goto main_end_of_test;
// 		}
// #ifdef DO_DIAGS
// 	printf("%s\r\n",picFileName);
// #endif	
// 
//  	int picBuffCtr,picBlockCtr;
// 		for(picBuffCtr=0;picBuffCtr<PIC_HEADER_LEN;picBuffCtr++){
// 			f_putc(picHeader_2655[picBuffCtr],&file_object);
// 		}
// 
// 	char *camBuffPtrA,*camBuffPtrB;
// 	camBuffPtrA=SRAM_BASE;//
// 	char colorCtr=0;
// 	char testPixA=0x1f;
// 	char testPixB=0x00;
// 	char firstByte,secondByte,redByte,greenByte,blueByte;
// /******* RGB565 Interlaced****************/
// camBuffPtrA=SRAM_BASE;//
// camBuffPtrB = SRAM_BASE + 1913600;//used to be 1920000 during 2020 nest season
// 	char fwret;
// 	for(picBlockCtr=0;picBlockCtr<600;picBlockCtr++){
// 		oneLinePtr=oneLine;
// 		for(picBuffCtr=0;picBuffCtr<1600;picBuffCtr++){
// 			firstByte=*camBuffPtrA++;
// 			secondByte=*camBuffPtrA++;
// 			blueByte=(firstByte&0xF8);
// 			greenByte=((firstByte&0x07)<<5)+((secondByte&0xE0)>>3);
// 			redByte=(secondByte&0x1F)<<3;
// 			*oneLinePtr++=blueByte;
// 			*oneLinePtr++=greenByte;
// 			*oneLinePtr++=redByte;
// 		}
// 		f_write(&file_object,oneLine,4800,&fwret);
// 		oneLinePtr=oneLine;
// 		for(picBuffCtr=0;picBuffCtr<1600;picBuffCtr++){
// 			firstByte=*camBuffPtrB++;
// 			secondByte=*camBuffPtrB++;
// 			blueByte=(firstByte&0xF8);
// 			greenByte=((firstByte&0x07)<<5)+((secondByte&0xE0)>>3);
// 			redByte=(secondByte&0x1F)<<3;
// 			*oneLinePtr++=blueByte;
// 			*oneLinePtr++=greenByte;
// 			*oneLinePtr++=redByte;
// 		}
// 		f_write(&file_object,oneLine,4800,&fwret);
// #ifdef DO_DIAGS
// 		if((picBlockCtr%30)==0)printf(".");
// #endif		
// 
// 	if((picBlockCtr%30)==0)	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
// 
// 	}
// 	f_close(&file_object);
// 	f_utime(picFileName,fileInfo);
// 	closeSDcard();
// 	ioport_set_pin_level(SDCARDPWRC_GPIO, SDCARDPWRC_POWER_OFF);
// #ifdef DO_DIAGS
// 	printf("\r\nMain Branch. GOOD. FILE CLOSED.\r\n");
// #endif	
// main_end_of_test:
// 	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog;
// 	delay_ms(10);
// 	closeSDcard();

/****************************************************************************************************************
 * ACQUIRE IMAGE
 *Cache:
 *	Transfer data in multiples of 32 bytes.
 *	Options:  
 *		0 = no flash
 ****************************************************************************************************************/
 char takeImageTest(char flash, char configSource, char formatSize){
 uint8_t * camBuff = (uint8_t *)SRAM_BASE+0x400; uint8_t *camBuffPtr; twihs_options_t opt; int iters; camBuffPtr=camBuff; char ret=0;  ioport_set_pin_dir(GPIO_PD7, IOPORT_DIR_OUTPUT);//TP #ifdef DO_DIAGS printf("TAKE IMAGE. ConfigSource: %d. ImageSize: %d \r\n", configSource, formatSize); #endif SCB_EnableICache(); SCB_EnableDCache(); /* Complete SDRAM configuration */ configPinsSDRAM();
 pmc_enable_periph_clk(ID_SDRAMC);
  sdramc_init((sdramc_memory_dev_t *)&SDRAM_INSIGNIS_16M,	sysclk_get_cpu_hz());
  sdram_enable_unaligned_support();
 #ifdef DO_DIAGS
 printf("SRAM COMPLETE\r\n"); #endif
	sdramc_deinit();

	SCB_CleanInvalidateDCache();
	SCB_DisableDCache();
	//	SCB_CleanInvalidateICache();
	SCB_InvalidateICache();
	SCB_DisableICache();
//	ov_configure(BOARD_TWIHS, 2);//OV2710 RESET. Required to free up ISID7_SDdat3 for SDcard to work.  OV2655 = 2 for Reset (not needed for 2655??)
	//	ioport_set_pin_level(CAM_FLASH_GPIO, CAM_FLASH_OFF);// CAMERA FLASH OFF
//	delay_ms(1000);
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog;

	return 0;

 }

/*******************************************************************************************/
/*******************************************************************************************/


char takeImage(char flash, char configSource, char formatSize){
	uint8_t * camBuff = (uint8_t *)SRAM_BASE+0x400;	uint8_t *camBuffPtr;	twihs_options_t opt;	int iters;	camBuffPtr=camBuff;	char ret=0;		ioport_set_pin_dir(GPIO_PD7, IOPORT_DIR_OUTPUT);//TP#ifdef DO_DIAGS	printf("TAKE IMAGE. CONFIG SOURCE: %d. IMAGE SIZE: %d \r\n", configSource, formatSize);#endif	SCB_EnableICache();	SCB_EnableDCache();	/* Complete SDRAM configuration */	configPinsSDRAM();	pmc_enable_periph_clk(ID_SDRAMC);	sdramc_init((sdramc_memory_dev_t *)&SDRAM_INSIGNIS_16M,	sysclk_get_cpu_hz());	sdram_enable_unaligned_support();#ifdef DO_DIAGS
	printf("SRAM COMPLETE\r\n");#endif
	MATRIX->CCFG_SMCNFCS = CCFG_SMCNFCS_SDRAMEN;
	/* Configure CAM POWER.*/
	ioport_set_pin_dir(CPWRC_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(SDCARDPWRC_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(CPWDN_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(GPIO_PA8, IOPORT_DIR_OUTPUT);//SRAM PWRC
	ioport_set_pin_dir(ISI_HSYNC_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ISI_VSYNC_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(CPWDN_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CPWDN_GPIO, CPWDN_POWER_OFF);
	ioport_set_pin_level(CPWDN_GPIO, CPWDN_POWER_OFF);
	delay_ms(100);
	ioport_set_pin_level(CPWRC_GPIO, CPWRC_POWER_ON);//needs 30 usec ramp
	ioport_set_pin_level(SDCARDPWRC_GPIO, 1);
	ioport_set_pin_level(GPIO_PA8, 1);//SRAM_PWRC
	delay_ms(100);
	delay_ms(6);//OV2710 datasheet says minimum 5ms between applying power an allowing CPWDN to go low.
	ioport_set_pin_level(CPWDN_GPIO, CPWDN_POWER_ON);
	delay_ms(2);
	delay_ms(100);
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
#ifdef DO_DIAGS
	printf("CAM PWR ON\r\n");//#endif
	
	gpio_configure_pin(ISI_MCLK_GPIO, ISI_MCLK_FLAGS);
	#define   PMC_PCK_PRES_CLK_4 (0x2u << 7) /**Zoo Board if PLLA_MULT=25=300MHz, then 6 = 33MHz, 7=18Mhz. if PLLA_MULT=20, then 6=27Mhz.  OVM7692 6-27MHz*/
	PMC->PMC_PCK[1] = (PMC_PCK_PRES_CLK_4 | PMC_PCK_CSS_PLLA_CLK);
	PMC->PMC_SCER = PMC_SCER_PCK1;
	iters=500;	while ((!(PMC->PMC_SCSR & PMC_SCSR_PCK1))&&(iters>0)) {
		iters--;
	}
	if(iters==0){ret=1;goto NO_CAMERA_RESPONSE; }
#ifdef DO_DIAGS
	printf("MCLK OKAY\r\n");//tshooooooooooooooooooooooooooooooooooooooooooooot#endif
	delay_ms(1);
	// /************************** CAMERA ***********************************/	/* Init TWI peripheral */
	opt.master_clk = sysclk_get_cpu_hz();
	opt.speed      = TWIHS_CLK;
	twihs_master_init(BOARD_TWIHS, &opt);
	/* Configure TWI interrupts */
	NVIC_DisableIRQ(BOARD_TWIHS_IRQn);
	NVIC_ClearPendingIRQ(BOARD_TWIHS_IRQn);
	NVIC_SetPriority(BOARD_TWIHS_IRQn, 0);
	NVIC_EnableIRQ(BOARD_TWIHS_IRQn);
	/* Enable TWI peripheral */
	pmc_enable_periph_clk(ID_BOARD_TWIHS);
	/* OV2655 Initialization */
	pio_configure_pin(ISI_D0_GPIO, ISI_D0_FLAGS);
	pio_configure_pin(ISI_D1_GPIO, ISI_D1_FLAGS);
	pio_configure_pin(ISI_D2_GPIO, ISI_D2_FLAGS);
	pio_configure_pin(ISI_D3_GPIO, ISI_D3_FLAGS);
	pio_configure_pin(ISI_D4_GPIO, ISI_D4_FLAGS);
	pio_configure_pin(ISI_D5_GPIO, ISI_D5_FLAGS);
	pio_configure_pin(ISI_D6_GPIO, ISI_D6_FLAGS);
	pio_configure_pin(ISI_D7_GPIO, ISI_D7_FLAGS);
	pio_configure_pin(ISI_HSYNC_GPIO, ISI_HSYNC_FLAGS);
	pio_configure_pin(ISI_VSYNC_GPIO, ISI_VSYNC_FLAGS);
	pio_configure_pin(ISI_PCK_PIO, ISI_PCK_FLAGS);
	delay_ms(200);
	iters=10;	while ((ov_init(BOARD_TWIHS)==1)&&(iters>0)) {
		delay_ms(200);
#ifdef DO_DIAGS
		printf(".");
#endif
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog;
	iters--;	}	//WORKS for OV2655 (after changing one address byte for OV7692 to two address bytes for OV2655)
	if(iters==0){ret=1;goto NO_CAMERA_RESPONSE; }
	delay_ms(100);//CRITICAL DELAY, THIS NEEDS TO BE HERE OR OV_CONFIG() HANGS when no DO_DIAGS. March 2020. 50 seemed to work at least most of the time.
#ifdef DO_DIAGS
	printf("CAM INIT OKAY\r\n");
#endif
	ov_configure(BOARD_TWIHS, 4);//OV2655 RESET=4
#ifdef DO_DIAGS
	printf("CAM RESET OKAY\r\n");
#endif
	/********************************** OV2655 CONFIG ************************************************************************/	if(configSource == 0){		ov_configure(BOARD_TWIHS, 2);//OV2655 CONFIG=2
#ifdef DO_DIAGS
		printf("CAM CONFIG FROM HARD CODE DEFAULT.\r\n");
#endif
	}
	else {
		ov_reg ov2655FromUSB[] = {
	{0x308C, 0x80},//BaseExample 80
	{0x308D, 0x10},//BaseExample 0E   (10=NO MIPI)
	{0x360B, 0x00},//BaseExample 00.  TEST BITS
	{0x30B0, 0xFC},//BaseExample FF (FC=my output pins)
	{0x30B1, 0x2F},//BaseExample FF (2F=my output pins)
	{0x30B2, 0x24},//BaseExample 04  ??
	
	{0x3601, 0x00},//DVP Pins
	{0x3308, 0x01},//3801,01 = COLOR BAR TEST PATTERN

	{0x3012, 0x10},//BaseExample 10		CROPPING and AVERAGING
	{0x3011, 0x02},//BaseExample 01  MY PINS=02
	//D00 CLK  40=external clock from XCLK? Datasheet seems messed up here. Example_1,3,4 = 01. 00=52MHz (still works). 01=27MHz. WORKS AT 07=6MHz
	//
	// 	{0x300E, 0x34},//BaseExample 34
	// 	{0x300F, 0xA6},//BaseExample A6
	// 	{0x3010, 0x81},//BaseExample 81
	// 	{0x3082, 0x01},//BaseExample 01
	// 	{0x30F4, 0x01},//BaseExample 01
	// 	{0x3090, 0x33},//Example_2,3,4 33 43
	// 	{0x3091, 0xC0},//BaseExample C0
	// 	{0x30AC, 0x42},//BaseExample 42
	//
	// 	{0x30D1, 0x08},//BaseExample 08
	// 	{0x30A8, 0x55},//BaseExample 55
	// 	{0x3015, 0x02},//BaseExample 02  ALSO 41
	{0x3093, 0x00},//BaseExample 00   //THIS IS THE CRITICAL REGISTER FOR IMAGES WITHOUT VERT STRIPES
	
	//AEC
	{0x3013, 0xF7},//DE7 BaseExample F7  17,37,77,C7,D7=black
	{0x3014, 0x8C},//D04 BaseExample 84 08 had the least streaking in low light (inside birdhouse with flashlight). Night Mode on.
	{0x3018, 0x88},//D78 BaseExample 80  AEC gap (stability)
	{0x3019, 0x70},//D68 BaseExample 70  AEC gap (stability)
	{0x301A, 0xD4},//DD4 BaseExample D4  AEC step
	// Considered, but decided not needed:  3047,3038,3039,303A,303B,303C,303D,303E,303F,3030,3031,3032,3033,3002,3003,3008,3009,3040,3041,3042,3043,3044,303A,303B,303E,303F
	
	//IMAGE SIZE  1600x1200
	{0x3020, 0x01},//D01 BaseExample 01  H.START 0x118=280
	{0x3021, 0x18},//D18 BaseExample 18
	{0x3022, 0x00},//D00 BaseExample 00  V.START 0x00A=10
	{0x3023, 0x0A},//D0A BaseExample 06
	{0x3024, 0x06},//D06 BaseExample 06  H.WIDTH
	{0x3025, 0x40},//D58 BaseExample 58  ???
	{0x3026, 0x02},//D04 BaseExample 02  V.HEIGHT
	{0x3027, 0x58},//DBC BaseExample 5E  ???
	{0x3028, 0x07},//D07 BaseExample=07  H.TOTAL SIZE 0x793=1939  1624=658
	{0x3029, 0x93},//D93 BaseExample=93
	{0x302A, 0x04},//D04 BaseExample=02  V.TOTOAL SIZE 0x04 D4=1236  1224=4C8
	{0x302B, 0xD4},//DD4 BaseExample=E6
	{0x3088, 0x06},//D06 BaseExample 03  OUTPUT SIZE
	{0x3089, 0x40},//D40 BaseExample 20  OUTPUT SIZE
	{0x308A, 0x02},//D00 BaseExample 02  OUTPUT SIZE
	{0x308B, 0x58},//D00 BaseExample 58  OUTPUT SIZE
	{0x3316, 0x64},//BaseExample 64  NEW
	{0x3317, 0x25},//BaseExample 25  NEW
	{0x3318, 0x80},//BaseExample 80  NEW
	{0x3319, 0x08},//BaseExample 08  NEW
	{0x331A, 0x64},//BaseExample 64  NEW
	{0x331B, 0x4B},//BaseExample 4B  NEW
	{0x331C, 0x00},//BaseExample 00  NEW
	{0x331D, 0x38},//BaseExample 38  NEW
	
	
	//AWB
	{0x3320, 0xFA},//BaseExample FA
	{0x3321, 0x11},//BaseExample 11
	{0x3322, 0x92},//BaseExample 92
	{0x3323, 0x01},//BaseExample 01
	{0x3324, 0x97},//BaseExample 97
	{0x3325, 0x02},//BaseExample 02
	{0x3326, 0xFF},//BaseExample FF
	{0x3327, 0x0C},//BaseExample 0C
	{0x3328, 0x10},//BaseExample 10
	{0x3329, 0x10},//BaseExample 10
	{0x332A, 0x58},//BaseExample 58
	{0x332B, 0x56},//BaseExample 56
	{0x332C, 0xBE},//BaseExample BE
	{0x332D, 0xE1},//BaseExample E1
	{0x332E, 0x3A},//BaseExample 3A
	{0x332F, 0x36},//BaseExample 36
	{0x3330, 0x4D},//BaseExample 4D
	{0x3331, 0x44},//BaseExample 44
	{0x3332, 0xF8},//BaseExample F8
	{0x3333, 0x0A},//BaseExample 0A
	{0x3334, 0xF0},//BaseExample F0
	{0x3335, 0xF0},//BaseExample F0
	{0x3336, 0xF0},//BaseExample F0
	{0x3337, 0x40},//BaseExample 40
	{0x3338, 0x40},//BaseExample 40
	{0x3339, 0x40},//BaseExample 40
	{0x333A, 0x00},//BaseExample 00
	{0x333B, 0x00},//BaseExample 00
	
	//COLOR MATRIX  (RGB to YUV)

	{0x3380, 0x28},//D28
	{0x3381, 0x48},//D48
	{0x3382, 0x10},//D10
	{0x3383, 0x18},//D18
	{0x3384, 0x28},//D28
	{0x3385, 0x40},//D40
	{0x3386, 0x40},//D40
	{0x3387, 0x34},//D34
	{0x3388, 0x0C},//D0C
	{0x3389, 0x98},//D98
	{0x338A, 0x01},//01 CRITICAL: 01=Works for color bar March15 2020.
	
	//GAMMA
	{0x3340, 0x04},//BaseExample 04  NO BIGGIE
	{0x3341, 0x07},//BaseExample 07
	{0x3342, 0x19},//BaseExample 19
	{0x3343, 0x34},//BaseExample 34
	{0x3344, 0x4A},//BaseExample 4A
	{0x3345, 0x5A},//BaseExample 5A
	{0x3346, 0x6A},//BaseExample 6A
	{0x3347, 0x71},//BaseExample 71
	{0x3348, 0x7C},//BaseExample 7C
	{0x3349, 0x8C},//BaseExample 8C
	{0x334A, 0x9B},//BaseExample 98
	{0x334B, 0xA9},//BaseExample A9
	{0x334C, 0xC0},//BaseExample C0
	{0x334D, 0xD5},//BaseExample D5
	{0x334E, 0xE8},//BaseExample E8
	{0x334F, 0x20},//BaseExample 20
	
	//Special Digital Effects  DEFAULTS DATASHEET Mar2020. This stuff can have big effects.
	{0x3090, 0x41},//D41 BaseExample 03,0B 33 43
	//  	{0x3091, 0x00},//D00  SOMETHING IN THIS SECTION MAKES THE IMAGE GO BLACK
	//  	{0x3092, 0x80},//D80
	//  	{0x3093, 0x00},//D00
	//  	{0x3094, 0x40},//D40
	//  	{0x3095, 0x40},//D40
	//  	{0x3096, 0x80},//D80
	//  	{0x3097, 0x80},//D80
	//  	{0x3098, 0x00},//D00
	//  	{0x3099, 0x20},//D20
	//  	{0x309A, 0x00},//D00
	
	//ISP_TOP (New Datasheet)
	{0x3300, 0xB0},//DF0 80 CRITICAL: 80=Works for color bar March15 2020. 81,82,83 produce RAW.
	//80=ISP ENABLE. 40=RAW GAMMA ENABLE. 20=AWB_STAT ENABLE. 10=AWB_GAIN ENABLE. 0C=LENC ENABLES.
	
	//	{0x3302, 0x11},//BaseExample 11  This one scrambles the colors. Undocumented.


	//UV ADJUST
	{0x3301, 0xFF},//BaseExample FF	   NO BIGGIE
	{0x338B, 0x11},//BaseExample 11
	{0x338C, 0x10},//BaseExample 10
	{0x338D, 0x40},//BaseExample 40
	
	//SHARPNESS DEFAULTS PER Mar2020 DATASHEET
	{0x3370, 0xFF},//DFF  BaseExample D0
	{0x3371, 0x00},//D00  BaseExample 00
	{0x3372, 0x17},//D17  BaseExample 00
	{0x3373, 0x20},//D20  BaseExample 30
	{0x3374, 0x00},//D00  BaseExample 10
	{0x3375, 0x10},//D10  BaseExample 10
	{0x3376, 0x10},//D10  BaseExample 07 BITS 1&2:
	{0x3377, 0x00},//D00  BaseExample 00
	{0x3378, 0x10},//D10  BaseExample 04
	{0x3379, 0x80},//D80  BaseExample 40
	
	//BLC
	{0x3069, 0x86},//BaseExample 86  NO BIGGIE  3:0=target black level used in algorithm
	//	{0x306E, 0x},//New Datasheet bit 1: 1=BLC manually for 32 frames.
	//	{0x306F, 0x},//New Datasheet 3:0= HDR short exposure. Target black level used in algorithm
	{0x3087, 0x02},//BaseExample 02
	
	//DIGITAL GAIN
	{0x307C, 0x10},//BaseExample 13,10   ALSO BLC New Datasheet: Digital gain enable bit5 1=enable
	//	{0x307E, 0x},// New Datasheet: Digital gain range

	//STROBE PULSE
	{0x307A, 0x00},// New Datasheet: bit 7 0=disable


	//ISP SYSTEM CONTROL
	{0x3100, 0x02},//02 CRITICAL: 02=Works for color bar March15 2020.
	//	{0x3105, 0x00},//New Datasheet ISP Clock Enables

	//OTHER FUNCTIONS
	{0x3400, 0x40},//D00 40=RGB565 50=RGB555 60=RGB444(XXXX_XXX X__XXXX_) 70=RGB444(XXXXXXXX XXXX.XXXX) 80=RGB444(____XXXX XXXXXXXX) 00=422 90=RAW
	//    	{0x3606, 0x00},//BaseExample 20 Example_1,2,3,4//This wreck H.Sync
	{0x3601, 0x00},//BaseExample=30. DVP pin order D9..D2 connected to my D7 to D0
	{0x30F3, 0x83},//BaseExample 83
	{0x304E, 0x88},//BaseExample 88
	{0x3015, 0x02},//BaseExample 02
	{0x302D, 0x00},//BaseExample 00
	{0x302E, 0x00},//BaseExample 00
	{0x3306, 0x00},//BaseExample 00  //from online driver for RGB "|02" for manual WB. also set in AWB
	{0x363B, 0x01},//BaseExample 01
	{0x363C, 0xF2},//BaseExample F2
	//   	{0x3086, 0x0F},//Example_1,2,3,4// These two configs at 0x3086 make the camera stall. Sleep mode?
	//  	{0x3086, 0x00},//Example_1,2,3,4// These two configs at 0x3086 make the camera stall. Sleep mode?
	{0x30A1, 0x41},//BaseExample 41
	{0x30A3, 0x80},//BaseExample 80 ALSO ABOVE
	{0x30A8, 0x56},//BaseExample 56
	{0x30AA, 0x72},//BaseExample 72
	{0x30AF, 0x10},//BaseExample 10  BIT0=1=disable AGC histogram calc
	{0x30B2, 0x2C},//BaseExample 2C
	{0x30D9, 0x8C},//BaseExample 8C
	// END OF BASELINE EXAMPLE

	//		{0x3094, 0x80},//D80 Power up sequence
	//MY STUFF FROM DATA SHEET
	{0x3000, 0x00},//Example_1
	
	{0xFFFF, 0xFF}};//STOPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
		ov_write_regs(BOARD_TWIHS, ov2655FromUSB);
#ifdef DO_DIAGS
		printf("CAM CONFIG FROM HOST PC\r\n");
#endif
		}
	/*************************************************************************************************************************/	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
#ifdef DO_DIAGS
	printf("CAM CONFIG OKAY\r\n");
#endif
	char regVal = 0x40;
#ifndef DO_CONTINUOUS_PICS
	delay_ms(2000);			// IMAGE SENSOR DELAY TO ADJUST TO LIGHT 500ms,200ms get bad.
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog;
	delay_ms(1000);			// IMAGE SENSOR DELAY TO ADJUST TO LIGHT 500ms,200ms get bad.
#endif
	delay_ms(100);
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog;
	regVal+=0x11;
	/*  PRINT SOME REGISTERS  */
	// 	ov_dump_registers(BOARD_TWIHS, camRegDump,0x33);

	/*************************** SRAM ***************************************************/
	SCB_CleanInvalidateDCache();
	camBuffPtr=SRAM_BASE;//camBuff=SRAM_BASE+0x400
	// 	for(camCtr=0;camCtr<50000;camCtr+=2){  //This seems to hang the Atmel sometimes.
	// 		*camBuffPtr++=0x05;
	// 		*camBuffPtr++=0x06;
	// 	}
	SCB_CleanInvalidateDCache();
	/************************ ISI *********************************/
	pmc_enable_periph_clk(ID_ISI);//
	_isi_AllocateFBD();//inside here is the address that needs to be sent to isi_set_dma_codec_path() below.
#ifdef DO_DIAGS
	printf("ISI ALLOCATE COMPLETE\r\n");
#endif	iters=1500;	ISI->ISI_CR = ISI_CR_ISI_SRST | ISI_CR_ISI_DIS;
	while (((ISI->ISI_SR & ISI_SR_SRST) != ISI_SR_SRST)&&(iters>0)) {  //early boards stalled here
		delay_ms(1);
		iters--;
	}
	if(iters==0){ret=1;goto NO_CAMERA_RESPONSE; }
	ISI->ISI_CR |= ISI_CR_ISI_EN;
	delay_ms(10);
	iters=1500;	while (((ISI->ISI_SR & ISI_SR_ENABLE) != ISI_SR_ENABLE)&&(iters>0)) {  //early boards stalled here
		delay_ms(1);
		iters--;
	}
	if(iters==0){ret=1;goto NO_CAMERA_RESPONSE; }
#ifdef DO_DIAGS
	printf("ISI ENABLED\r\n");
#endif
	isi_config adsIsiConfig;
	adsIsiConfig.crc_sync=0;//0=no crc on embedded sync
	adsIsiConfig.emb_sync=0;//0=sync on vsync/hsync signals
	adsIsiConfig.hpol=0;//0=sync active high
	adsIsiConfig.vpol=0;//0=sync active high
	adsIsiConfig.image_fmt=ISI_INPUT_YUV;//ISI_INPUT_GS_8BIT;//need 8-bit "grayscale" ("grayscale is deceiving term, it means raw unprocessed, it can mean color)
	adsIsiConfig.image_hsize=1600;//OV2655 color bar ~works with 1600.
	adsIsiConfig.image_vsize=1200;//480;//960;//480;
	adsIsiConfig.sld=0;//h blanking
	adsIsiConfig.sfd=0;//v blanking
	adsIsiConfig.pck_pol=0;//0=data sampled on rising edge
	adsIsiConfig.thmask=0;//only four beats of AHB burst allowed
	isi_init(ISI, &adsIsiConfig);
	isi_set_dma_codec_path(ISI,1, 1000,&preBufDescList[0], 1, SRAM_BASE);//Use codec path for capturing in "grayscale" (raw color) mode.
	//'FULL' (2nd param) parameter means both Codec and Preview DMAs are operating simultaneiously)
	ISI->ISI_IDR = 0xFFFFFFFF;
	ISI->ISI_IER = ISI_IER_CXFR_DONE;
	ISI->ISI_DMA_CHDR |= ISI_DMA_CHDR_P_CH_DIS;//disable Preview Mode
	ISI->ISI_DMA_CHDR |= ISI_DMA_CHDR_C_CH_DIS;//disable Codec Mode
	SCB_CleanDCache();
	ISI->ISI_DMA_CHER |= ISI_DMA_CHER_C_CH_EN;//enable Codec Mode. MUST USE CODEC PATH BECAUSE PREVIEW IS ONLY 640 X 480
#ifdef DO_DIAGS
	printf("ISI INIT DONE A\r\n");
#endif
// 	ioport_set_pin_dir(CAM_FLASH_GPIO, IOPORT_DIR_OUTPUT);
// 	ioport_set_pin_level(CAM_FLASH_GPIO, CAM_FLASH_OFF);// CAMERA FLASH
	// for(iterssx=0;iterssx<10;iterssx++){//Loop some captures to watch on the PC
// 	twi_packet_regs.addr[0] = (0x33);
// 	twi_packet_regs.addr[1] = (0x70);
// 	twi_packet_regs.addr_length = 2;
// 	twi_packet_regs.chip = (0x60u >> 1);
// 	twi_packet_regs.length = 1;
// 	twi_packet_regs.buffer = &testReg;
	//	ov_write_reg(BOARD_TWIHS, &twi_packet_regs);
	delay_ms(5);

	NVIC_ClearPendingIRQ(ISI_IRQn);
	NVIC_SetPriority(ISI_IRQn, 7);
	NVIC_EnableIRQ(ISI_IRQn);
	
	char capIters=3;	capIters = isi_capture(ISI);#ifdef DO_DIAGS
	printf("ISI CAPTURE DONE. CAP;sjd;lskdjf;lksjf;sldjkf;sldj AAA= %d\r\n",capIters);
//	delay_ms(200);
#endif
	if(capIters>190)goto NO_CAMERA_RESPONSE;	delay_ms(1200);//100ms left first row of memory wrong. Need even longer delay if use slower PLL divide on camera.
	sdramc_deinit();
#ifdef DO_DIAGS
	printf("SDRAM DEINIT\r\n");
	delay_ms(100);
#endif
	SCB_CleanInvalidateDCache();
	SCB_DisableDCache();
	SCB_InvalidateICache();
	SCB_DisableICache();
#ifdef DO_DIAGS
	printf("CACHE DISABLED\r\n");
#endif

	ov_configure(BOARD_TWIHS, 2);//OV2710 RESET. Required to free up ISID7_SDdat3 for SDcard to work.  OV2655 = 2 for Reset (not needed for 2655??)
#ifdef DO_DIAGS
	printf("OV CONFIGURED\r\n");
#endif
#ifndef DO_CONTINUOUS_PICS
	delay_ms(1000);
#endif	
	delay_ms(100);
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog;
	if(formatSize)RGB565toRGB888();
	/* CAM OFF */
	ioport_set_pin_level(CPWDN_GPIO, CPWDN_POWER_OFF);
	ioport_set_pin_level(CPWDN_GPIO, CPWDN_POWER_OFF);
	delay_ms(1);
	ioport_set_pin_level(CPWDN_GPIO, CPWDN_POWER_OFF);
#ifdef DO_DIAGS
	printf("CAM OFF\r\n");
	delay_ms(100);
#endif
	NO_CAMERA_RESPONSE:
	resetPinsISI();
	delay_ms(1);
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog;
#ifdef DO_DIAGS
	printf("PINS RESET\r\n");
	delay_ms(100);
#endif
	
	//	SDcardTest();
	// sleepPins();
	// 	for(;;){redBlink(5);delay_ms(500);}
	return ret;
}

/***********************************************************************/
/*  ISI AllocateFDB
***********************************************************************/
static void _isi_AllocateFBD(void)
{
uint32_t i;
for(i = 0; i < ISI_MAX_PREV_BUFFER; i++) {
preBufDescList[i].Current = (uint32_t)SRAM_BASE;
preBufDescList[i].Control = ISI_DMA_C_CTRL_C_FETCH;
preBufDescList[i].Next = (uint32_t)&preBufDescList[0];
}
}
/***********************************************************************/
/*  RGB565 TO RGB888 TO IMAGE_BUFF_A
***********************************************************************/
void RGB565toRGB888 (void){
 	int picBuffCtr,picBlockCtr;
	char *camBuffPtrA,*camBuffPtrB,*camBuffPtrC;
	camBuffPtrA=SRAM_BASE;//
	char firstByte,secondByte,redByte,greenByte,blueByte;
	char oneLine[4800];
	char* oneLinePtr;

	camBuffPtrA=SRAM_BASE;//
	camBuffPtrB = SRAM_BASE + 1913600;//used to be 1920000 during 2020 nest season
	camBuffPtrC = IMAGE_BUFF_A;
	for(picBlockCtr=0;picBlockCtr<600;picBlockCtr++){
		oneLinePtr=oneLine;
		for(picBuffCtr=0;picBuffCtr<1600;picBuffCtr++){
			firstByte=*camBuffPtrA++;
			secondByte=*camBuffPtrA++;
			blueByte=(firstByte&0xF8);
			greenByte=((firstByte&0x07)<<5)+((secondByte&0xE0)>>3);
			redByte=(secondByte&0x1F)<<3;
			*camBuffPtrC++=blueByte;
			*camBuffPtrC++=greenByte;
			*camBuffPtrC++=redByte;
		}
		oneLinePtr=oneLine;
		for(picBuffCtr=0;picBuffCtr<1600;picBuffCtr++){
			firstByte=*camBuffPtrB++;
			secondByte=*camBuffPtrB++;
			blueByte=(firstByte&0xF8);
			greenByte=((firstByte&0x07)<<5)+((secondByte&0xE0)>>3);
			redByte=(secondByte&0x1F)<<3;
			*camBuffPtrC++=blueByte;
			*camBuffPtrC++=greenByte;
			*camBuffPtrC++=redByte;
		}
	}
}
/***********************************************************************
 *    LCD Functions
 ***********************************************************************/
void LCDclear (void){
	printf("%c%c%c%c%c%c%c%c",0xFE,0x46,0xFE,0x48,0xFE,0x4C,0xFE,0x51);//clear display, cursor home
	delay_ms(10);
}
void LCDbottomLine (void){
	printf("%c%c%c",0xFE,0x45,0x40);//go to bottom line
}
/**************************************************************************
 *  Configure UART console.
 **************************************************************************/
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

   ioport_set_pin_peripheral_mode(UART3_TXD_GPIO, UART3_TXD_FLAGS);
   sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
   stdio_serial_init(CONF_UART, &uart_serial_options);
   uart_enable_tx(CONF_UART);
   sam_uart_opt_t uart_settings;
   uart_settings.ul_mck = sysclk_get_peripheral_hz();
   uart_settings.ul_baudrate = CONF_UART_BAUDRATE;
   uart_settings.ul_mode = CONF_UART_PARITY;
   uart_init(CONF_UART, &uart_settings);
}
/**************************************************************************
 *  Configure UART console.
 **************************************************************************/
static void configure_debugUART(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = (57600UL),
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

   ioport_set_pin_peripheral_mode(UART3_TXD_GPIO, UART3_TXD_FLAGS);
   sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
   stdio_serial_init(CONF_UART, &uart_serial_options);
   uart_enable_tx(CONF_UART);
   sam_uart_opt_t uart_settings;
   uart_settings.ul_mck = sysclk_get_peripheral_hz();
   uart_settings.ul_baudrate = (57600UL);
   uart_settings.ul_mode = CONF_UART_PARITY;
   uart_init(CONF_UART, &uart_settings);
}
/***********************************************************************
 *  TOG TP ATSAM
 ***********************************************************************/
 void togTP (char n) {
   togPortApin(TP0_GPIO);
   togPortApin(TP0_GPIO);
   while (n>0){
   togPortApin(TP0_GPIO);
       n--;
   };
 }
 void togPortApin (unsigned int tpin){
 	if (PIOD->PIO_ODSR & (1<<7)) {
	 	ioport_set_pin_level(TP0_GPIO,0);//works
	 	} else {
	 	ioport_set_pin_level(TP0_GPIO,1);//works
 	}
 }
/***********************************************************************
*  Fake Motion Records
Byte 0								1								2								3                         Followed by 12 bytes of motion or other sensor data
7	6	5	4	3	2	1	0	7	6	5	4	3	2	1	0	7	6	5	4	3	2	1	0	7	6	5	4	3	2	1	0
YEAR				MONTH				DAY					HOUR					MINUTE						DataType
Y	Y	Y	Y	M	M	M	M	D	D	D	D	D	H	H	H	H	H	M	M	M	M	M	M
*****************************************************************************************************************/
// void fakeNestMotionsRealistic (void){
// 	unsigned int params[]={0,0,0,0,0,0,0,0};
// 	unsigned int i,j,k,m,n,y;
// 	unsigned int numBlocks=1;
// 	g_motionPosition=0;
// 	unsigned char mm;
// 	unsigned short tt,xx,yy,zz;
// 	char rcvFromTiny[56];
// 	params[0]=0;params[1]=0;params[2]=0;params[3]=0;params[4]=0;params[5]=0;params[6]=0;params[7]=0;
// 
// 	mm=1;
// 
// 	 y=9 & 0x0F;	//Year  0-15  (+2018)
// 	 j=0 & 0x0F;	//Month
// 	 k=0 & 0x1F;	//Day
// 	 m=0 & 0x1F;	//Hour
// 	 n=0xFFFFFFFF;	//Minute
// 
// 	tt=1;//barometer
// 	xx=1020;//temperature
// 	yy=1;//humidity
// 	zz=1020;//light
// 
// 	 #define BLOCKS_TO_FILL 5
// 
// 	 archPtr=ARCHIVEADDR;
//      for (numBlocks=0;numBlocks<BLOCKS_TO_FILL;numBlocks++){
// 		 printf("E");
// 		 eraseSector(ARCHIVEADDR+(i*FLASHSECTORSIZE));
// 	 }
// 	numBlocks--;
// //   for (i=(ARCHIVEADDR);i<(ARCHIVEADDR+(/*numBlocks*/FLASHSECTORSIZE));i+=(ARCHRECORDLEN))  {	//219136= block[D6 214], fills through 213 (D5)
//    for (i=(ARCHIVEADDR);i<(ARCHIVEADDR+(46080*1));i+=(ARCHRECORDLEN))  {	//219136= block[D6 214], fills through 213 (D5)
// 	   printf(".");
// 	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
// 			params[0]=0;params[1]=0;params[2]=0;params[3]=0;params[4]=0;params[5]=0;params[6]=0;params[7]=0;
// 			n+=1;//minute
// 			if(n>59){
// 				n=0;
// 				m++;
// 				if(m>23){
// 					m=0;
// 					k++;
// 					if(k>31){
// 						k=1;
// 						j++;
// 					}
// 
// 				}
// 			}
// 			 params[0]=
// 			 (y<<28)	//Year
// 			 +(j<<24)	//Month
// 			 +(k<<19)	//Day
// 			 +(m<<14)	//Hour
// 			 +(n<<8);	//Min
// 
// 	mm++;
// 	if(mm>59)mm=0;
// //	params[0]+=mm<<2;//mm0
// 	rcvFromTiny[0]=mm;
// 	mm++;if(mm>59)mm=0;
// 	rcvFromTiny[1]=mm;
// //	params[0]+=(mm>>4);	//mm1
// //	params[1]+=((unsigned int)mm)<<28;	//mm1
// 	mm++;if(mm>59)mm=0;
// 	rcvFromTiny[2]=mm;
// //	params[1]+=((unsigned int)mm)<<22;//mm2
// 	mm++;if(mm>59)mm=0;
// 	rcvFromTiny[3]=mm;
// //	params[1]+=((unsigned int)mm)<<16;//mm3
// 	mm++;if(mm>59)mm=0;
// 	rcvFromTiny[4]=mm;
// //	params[1]+=((unsigned int)mm)<<10;//mm4
// 	mm++;if(mm>59)mm=0;
// 	rcvFromTiny[5]=mm;
// //	params[1]+=((unsigned int)mm)<<4;//mm5
// 
// 	tt++;if(tt>1023)tt=0;//barometer
// 	rcvFromTiny[6]=(char)(tt>>8);
// 	rcvFromTiny[7]=(char)(tt&0x00FF);
// //	params[1]+=((unsigned int)tt)>>6;//tt0
// //	params[2]+=((unsigned int)tt)<<26;//tt0
// 	tt++;if(tt>1023)tt=0;//barometer
// 	rcvFromTiny[8]=(char)(tt>>8);
// 	rcvFromTiny[9]=(char)(tt&0x00FF);
// 	tt++;if(tt>1023)tt=0;//barometer
// 	rcvFromTiny[10]=(char)(tt>>8);
// 	rcvFromTiny[11]=(char)(tt&0x00FF);
// 	tt++;if(tt>1023)tt=0;//barometer
// 	rcvFromTiny[12]=(char)(tt>>8);
// 	rcvFromTiny[13]=(char)(tt&0x00FF);
// 	tt++;if(tt>1023)tt=0;//barometer
// 	rcvFromTiny[14]=(char)(tt>>8);
// 	rcvFromTiny[15]=(char)(tt&0x00FF);
// 	tt++;if(tt>1023)tt=0;//barometer
// 	rcvFromTiny[16]=(char)(tt>>8);
// 	rcvFromTiny[17]=(char)(tt&0x00FF);
// /*	dd=1;
// 	params[2]+=((unsigned int)dd)<<16;//tt1
// 	params[2]+=((unsigned int)dd)<<6;//tt2
// 	params[2]+=((unsigned int)dd)>>4;//tt3
// 	params[3]+=((unsigned int)dd)<<28;//tt3
// 	params[3]+=((unsigned int)dd)<<18;//tt4
// 	params[3]+=((unsigned int)dd)<<8;//tt5		*/
// 
// 	xx+=1;if(xx>1023)xx=0;//temperature
// 	rcvFromTiny[18]=(char)(xx>>8);
// 	rcvFromTiny[19]=(char)(xx&0x00FF);
// //	xx+=5;if(xx>1023)xx=0;//temperature
// 	rcvFromTiny[20]=(char)(xx>>8);
// 	rcvFromTiny[21]=(char)(xx&0x00FF);
// //	xx+=5;if(xx>1023)xx=0;//temperature
// 	rcvFromTiny[22]=(char)(xx>>8);
// 	rcvFromTiny[23]=(char)(xx&0x00FF);
// //	xx+=5;if(xx>1023)xx=0;//temperature
// 	rcvFromTiny[24]=(char)(xx>>8);
// 	rcvFromTiny[25]=(char)(xx&0x00FF);
// //	xx+=5;if(xx>1023)xx=0;//temperature
// 	rcvFromTiny[26]=(char)(xx>>8);
// 	rcvFromTiny[27]=(char)(xx&0x00FF);
// //	xx+=5;if(xx>1023)xx=0;//temperature
// 	rcvFromTiny[28]=(char)(xx>>8);
// 	rcvFromTiny[29]=(char)(xx&0x00FF);
// 
// 	yy+=1;if(yy>1023)yy=0;//humidity
// 	rcvFromTiny[30]=(char)(yy>>8);
// 	rcvFromTiny[31]=(char)(yy&0x00FF);
// //	yy+=10;if(yy>1023)yy=0;//humidity
// 	rcvFromTiny[32]=(char)(yy>>8);
// 	rcvFromTiny[33]=(char)(yy&0x00FF);
// //	yy+=10;if(yy>1023)yy=0;//humidity
// 	rcvFromTiny[34]=(char)(yy>>8);
// 	rcvFromTiny[35]=(char)(yy&0x00FF);
// //	yy+=10;if(yy>1023)yy=0;//humidity
// 	rcvFromTiny[36]=(char)(yy>>8);
// 	rcvFromTiny[37]=(char)(yy&0x00FF);
// //	yy+=10;if(yy>1023)yy=0;//humidity
// 	rcvFromTiny[38]=(char)(yy>>8);
// 	rcvFromTiny[39]=(char)(yy&0x00FF);
// //	yy+=10;if(yy>1023)yy=0;//humidity
// 	rcvFromTiny[40]=(char)(yy>>8);
// 	rcvFromTiny[41]=(char)(yy&0x00FF);
// 
// 	zz+=1;if(zz>1023)zz=0;//light
// 	rcvFromTiny[42]=(char)(zz>>8);
// 	rcvFromTiny[43]=(char)(zz&0x00FF);
// //	zz+=15;if(zz>1023)zz=0;//light
// 	rcvFromTiny[44]=(char)(zz>>8);
// 	rcvFromTiny[45]=(char)(zz&0x00FF);
// //	zz+=15;if(zz>1023)zz=0;//light
// 	rcvFromTiny[46]=(char)(zz>>8);
// 	rcvFromTiny[47]=(char)(zz&0x00FF);
// //	zz+=15;if(zz>1023)zz=0;//light
// 	rcvFromTiny[48]=(char)(zz>>8);
// 	rcvFromTiny[49]=(char)(zz&0x00FF);
// //	zz+=15;if(zz>1023)zz=0;//light
// 	rcvFromTiny[50]=(char)(zz>>8);
// 	rcvFromTiny[51]=(char)(zz&0x00FF);
// //	zz+=15;if(zz>1023)zz=0;//light
// 	rcvFromTiny[52]=(char)(zz>>8);
// 	rcvFromTiny[53]=(char)(zz&0x00FF);
// 
// 
// 
// 		logMotionRecord32byte(rcvFromTiny,1,&params[0]);
// //			writeRecToFlash(i, params, ARCHRECORDLEN);
// 			incArchive();
// //			break;
// 		}
// 	g_motionPosition++;
// 	if(g_motionPosition>=12)g_motionPosition=0;
// }
/***********************************************************************
*  Fake Motion Records
Byte 0								1								2								3                         Followed by 12 bytes of motion or other sensor data
7	6	5	4	3	2	1	0	7	6	5	4	3	2	1	0	7	6	5	4	3	2	1	0	7	6	5	4	3	2	1	0
YEAR				MONTH				DAY					HOUR					MINUTE						DataType
Y	Y	Y	Y	M	M	M	M	D	D	D	D	D	H	H	H	H	H	M	M	M	M	M	M
*****************************************************************************************************************/
void fakeNestMotions (void){
	unsigned int params[]={0,0,0,0,0,0,0,0};
	unsigned int i,j,k,m,n,y;
	unsigned int numBlocks=40;
	g_motionPosition=0;
	unsigned char mm;
	params[0]=0;params[1]=0;params[2]=0;params[3]=0;params[4]=0;params[5]=0;params[6]=0;params[7]=0;

	mm=1;

	y=19; //Year  0-99 one byte
	j=12;	//Month
	k=25;	//Day
	m=17;	//Hour
	n=110;	//Minute

	archPtr=ARCHIVEADDR;
//	for (i=0;numBlocks<20;i++)eraseSector(ARCHIVEADDR+(i*FLASHSECTORSIZE));

	for (i=(ARCHIVEADDR);i<(ARCHIVEADDR+(/*numBlocks**/FLASHSECTORSIZE)/*219136*/);i+=(ARCHRECORDLEN))  {	//219136= block[D6 214], fills through 213 (D5)
		params[0]=0;params[1]=0;params[2]=0;params[3]=0;params[4]=0;params[5]=0;params[6]=0;params[7]=0;
// 		n+=6;
// 		if(n>56){
// 			n=0;
// 			m++;
// 			if(m>23){
// 				m=0;
// 				k++;
// 				if(k>31){
// 					k=1;
// 					j++;
// 				}
// 
// 			}
// 		}
n+=32;
		params[0]=
			((0x26)<<24)		//Day
			+((0x12)<<16)		//Month
			+((0x19)<<8)		//Year
			+(0X01);			//Data Format Version
		params[1]=
			((0xFE)<<24)		//MotionRaw0
			+((0x32)<<16)		//MotionTotal
			+((0x53)<<8)		//Minute
			+((0x07));			//Hour
		params[2]=
			((0x76)<<24)		//MotionRaw4
			+((0x98)<<16)		//MotionRaw3
			+((0xBA)<<8)		//MotionRaw2
			+((0xDC));			//MotionRaw1
		params[3]=
			((n+4)<<24)		//
			+((0x10)<<16)		//MotionRaw7
			+((0x32)<<8)		//MotionRaw6
			+((0x54));			//MotionRaw5
		params[4]=
			((n+8)<<24)		//
			+((n+9)<<16)		//
			+((n+10)<<8)		//
			+((n+11));		//
		params[5]=
			((n+12)<<24)		//
			+((n+13)<<16)		//
			+((n+14)<<8)		//
			+((n+15));		//
		params[6]=
			((n+16)<<24)		//
			+((n+17)<<16)		//
			+((n+18)<<8)		//
			+((n+19));		//
		params[7]=
			((n+20)<<24)		//
			+((n+21)<<16)		//
			+((n+22)<<8)		//
			+((n+23));		//


		writeRecToFlash(i, params, ARCHRECORDLEN);
		incArchive();
		//			break;
	}
	delay_ms(500);//500 got partial pgming
// 	g_motionPosition++;
// 	if(g_motionPosition>=12)g_motionPosition=0;
delay_ms(10);
}

/***********************************************************************
*  Fake Motion Records
Byte 0								1								2								3                         Followed by 12 bytes of motion or other sensor data
7	6	5	4	3	2	1	0	7	6	5	4	3	2	1	0	7	6	5	4	3	2	1	0	7	6	5	4	3	2	1	0
YEAR				MONTH				DAY					HOUR					MINUTE						DataType
Y	Y	Y	Y	M	M	M	M	D	D	D	D	D	H	H	H	H	H	M	M	M	M	M	M
*****************************************************************************************************************/
void fakeLoRaWAN (char fakeLRW[51], char seed){
	unsigned int params[]={0,0,0,0,0,0,0,0};
	unsigned int i,j,k,m,n,y;
	unsigned int numBlocks=40;
	g_motionPosition=0;
	unsigned char mm;
	unsigned short crcrc;
	params[0]=0;params[1]=0;params[2]=0;params[3]=0;params[4]=0;params[5]=0;params[6]=0;params[7]=0;

	mm=1;

	y=0x0E; //Year  0-F
	j=0x0C;	//Month 0-F
	k=0x1F;	//Day 0-31
	m=0x17;	//Hour 0-63
	n=(seed&0x3F);	//Minute 0-63

		params[0]=0;params[1]=0;params[2]=0;params[3]=0;params[4]=0;params[5]=0;params[6]=0;params[7]=0;
		
		params[0]=
		((y)<<28)		//Year
		+((j)<<24)		//Month
		+((k)<<19)		//Day
		+((m)<<14)		//Hour
		+(n<<8)			//Minute
		+0xA5;			//Motion Pattern
		
		params[1]=
		((51)<<26)		//Motion Total During Minute
		+((1003)<<16)	//Temperature 10 bits
		+(10362);		//Barometer 16 bits
		
		params[2]=
		((234)<<24)		//Humidity	8 bits
		+((198)<<16)	//Light 8 bits
		+((0xBA)<<8)	//Partial LatLong and Battery
		+((0xDC));		//Pic Score 1
		
		params[3]=
		((n+4)<<24)		//Pic Score 2
		+((0x10)<<16)	//Pic Score 3
		+((0x32)<<8)	//Pic Score 4
		+((0x54));		//Pic Score 5
		params[4]=
		((n+8)<<24)		//Pic Score 6
		+((n+9)<<16)	//Pic Score 7
		+((n+10)<<8)	//Pic Score 8
		+((n+11));		//Pic Score 9
		params[5]=
		((n+12)<<24)	//Pic Score 10
		+((n+13)<<16)	//Pic Score 11
		+((n+14)<<8)	//Pic Score 12
		+((n+15));		//Pic Score 
		params[6]=
		((n+16)<<24)		//
		+((n+17)<<16)		//
		+((n+18)<<8)		//
		+((n+19));		//
		params[7]=
		((n+20)<<24)		//
		+((n+21)<<16)		//
		+((n+22)<<8)		//
		+((n+23));		//

		fakeLRW[0] = (char)(params[0]>>24);
		fakeLRW[1] = (char)(params[0]>>16);
		fakeLRW[2] = (char)(params[0]>>8);
		fakeLRW[3] = (char)(params[0]>>0);
		fakeLRW[4] = (char)(params[1]>>24);
		fakeLRW[5] = (char)(params[1]>>16);
		fakeLRW[6] = (char)(params[1]>>8);
		fakeLRW[7] = (char)(params[1]>>0);
		fakeLRW[8] = (char)(params[2]>>24);
		fakeLRW[9] = (char)(params[2]>>16);
		fakeLRW[10] = (char)(params[2]>>8);
		fakeLRW[11] = (char)(params[2]>>0);
		fakeLRW[12] = (char)(params[3]>>24);
		fakeLRW[13] = (char)(params[3]>>16);
		fakeLRW[14] = (char)(params[3]>>8);
		fakeLRW[15] = (char)(params[3]>>0);
		fakeLRW[16] = (char)(params[4]>>24);
		fakeLRW[17] = (char)(params[4]>>16);
		fakeLRW[18] = (char)(params[4]>>8);
		fakeLRW[19] = (char)(params[4]>>0);
		fakeLRW[20] = (char)(params[5]>>24);
		fakeLRW[21] = (char)(params[5]>>16);
		fakeLRW[22] = (char)(params[5]>>8);
		fakeLRW[23] = (char)(params[5]>>0);
		fakeLRW[24] = (char)(params[6]>>24);
		fakeLRW[25] = (char)(params[6]>>16);
		fakeLRW[26] = (char)(params[6]>>8);
		fakeLRW[27] = (char)(params[6]>>0);
		fakeLRW[28] = (char)(params[7]>>24);
		fakeLRW[29] = (char)(params[7]>>16);
		fakeLRW[30] = (char)(params[7]>>8);
		fakeLRW[31] = (char)(params[7]>>0);
		fakeLRW[32] = 43;
		fakeLRW[33] = 44;
		fakeLRW[34] = 45;
		fakeLRW[35] = 45;
		fakeLRW[36] = 45;
		fakeLRW[37] = 45;
		fakeLRW[38] = 45;
		fakeLRW[39] = 45;
		fakeLRW[40] = 45;
		fakeLRW[41] = 45;
		fakeLRW[42] = 45;
		fakeLRW[43] = 45;
		fakeLRW[44] = 45;
		fakeLRW[45] = 45;
		fakeLRW[46] = 45;
		fakeLRW[47] = 46;
		fakeLRW[48] = 47;
		crcrc=calcCRC(fakeLRW, 49);
		fakeLRW[49] = (char)(crcrc>>8);
		fakeLRW[50] = (char)(crcrc&0x00FF);
}
/***********************************************************************
*  Increment Archive Pointers, erase next block if needed
***********************************************************************/
void incArchive(void){
	archPtr+=ARCHRECORDLEN;
	if(archPtr>=PASTARCHIVEADDR){/*If next ptr is at the end of the archive, wrap both pointers and erase the first block*/
		archPtr=ARCHIVEADDR;
		archErasePtr=ARCHIVEADDR;
		eraseSector(archErasePtr);
		archErasePtr += FLASHSECTORSIZE;
		}
	if (archPtr>=archErasePtr){/*If next ptr is at the beginning of the next block, erase the next block*/
		eraseSector(archErasePtr);
		archErasePtr += FLASHSECTORSIZE;
	}
}
/***********************************************************************
*  Erase Entire Archive
***********************************************************************/
void eraseEntireArchive(void){
	unsigned int erasePtr = ARCHIVEADDR;
	for (erasePtr=ARCHIVEADDR;erasePtr<PASTARCHIVEADDR;erasePtr+=0x0800){
		eraseSector(erasePtr);
	};
}
/***********************************************************************
*  Erase Sector  (K70S Sector is 128K)  can it also erase by page?
***********************************************************************/
void eraseSector(unsigned int addr) {
	unsigned int ul_rc;
	ul_rc = flash_erase_sector(addr);
	if (ul_rc != FLASH_RC_OK) {
		LCDclear();
		printf("-F- Flash programming error %lu\n\r", (unsigned long)ul_rc);
	}
}
/***********************************************************************
*  Write Record to Flash
***********************************************************************/
void writeRecToFlash (unsigned int larchPtr, unsigned int* params, unsigned int l_recLen){
	unsigned int ul_rc;
	ul_rc =  flash_write(larchPtr, (unsigned int*)params, l_recLen, 0);
	if (ul_rc != FLASH_RC_OK) {
		LCDclear();
		printf("ERROR Flash pgm %lu\n\r", (unsigned long)ul_rc);
	}
}
/***********************************************************************
*  Log 32-Byte Record to Archive
***********************************************************************/
char logMotionRecord32byte(char* rcvd, char ifFake, unsigned int fakeDate[8]){
#define RECORD_FORMAT_VERSION 1
unsigned int archMon,archDay,archHour,archMin,nnYear;
unsigned int params[]={0,0,0,0,0,0,0,0};
unsigned int addrToUse;
unsigned char rtcData[7];
unsigned int gotAltitudeTemperature,gotHumidity,gotLight;
unsigned int testForBlank;
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
if(ifFake){
	nnYear=fakeDate[0]>>28;
	archMon=hexToBCD((fakeDate[0]>>24)&0x0000000F);
	archDay=hexToBCD((fakeDate[0]>>19)&0x0000001F);
	archHour=hexToBCD((fakeDate[0]>>14)&0x0000001F);
	archMin=hexToBCD((fakeDate[0]>>8)&0x0000003F);
}else{
	twi_init_RTC();
	delay_ms(100);//
	rtcData[0]=0x01;//write to Address 0x00    NEED THIS TO ACK THE RTC INTERRUPT
	rtcData[1]=0x30;//contents of Addr 0x00 CTL1
	write_data_RTC(rtcData,2);//!< write sequentially to the slave "1" sends addr plus data[0].
	/*******Read ***/
	rtcData[0]=0x03;//
	write_data_RTC(rtcData,1);//!< write sequentially to the slave "1" sends addr plus data[0].
	delay_ms(1);//
	read_bytes_RTC(rtcData,7);//!< write sequentially to the slave "1" sends addr plus data[0].*/
	archHour=rtcData[2];//new RTC
	archMin=rtcData[1];//new RTC
	nnYear=rtcData[6];//new RTC
	archMon=rtcData[5];//new RTC
	archDay=rtcData[3];//new RTC
	dayMin = (hourNow*60) + minsNow;
#ifdef USE_TE_ALTIMETER
	gotAltitudeTemperature=getAltimeterTemperatureTE(0xEE);
#else
//	gotAltitudeTemperature=getAltimeterTemperature();
#endif
	gotHumidity=getHumidity(0x80);
	gotLight=getALSfromVEML(0x20);
#ifdef DO_DIAGS
// 	configure_console();
// 	delay_ms(10);
	printf("LOG MOTION  %X-%X-20%X  %X:%X\r\n",archMon,archDay,nnYear,archHour,archMin);
	printf("TEMP %X BAROM %X HUMIDITY %X\r\n",(gotAltitudeTemperature>>16),(gotAltitudeTemperature&0x0000FFFF),gotHumidity);
// 	delay_ms(100);
#endif
}
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
	addrToUse = ARCHIVEADDR + ((bcdToHex(archDay)-1)*46080) + ((bcdToHex(archHour))*1920) + ((bcdToHex(archMin))*32);

	int temperatureLB, humidityLB, /*lightLB,*/ maskedAltitudeHB, maskedAltitudeLB;
	int temperatureHB,humidityHB/*,lightHB*/;
	humidityHB=(gotHumidity&0x0300)>>5;
	humidityLB=(gotHumidity&0x00FF)<<16;
	temperatureHB=(gotAltitudeTemperature&0x07000000)>>19;
	temperatureLB=(gotAltitudeTemperature&0x00FF0000)>>8;
	maskedAltitudeHB=(gotAltitudeTemperature&0x00001F00)<<16;
	maskedAltitudeLB=(gotAltitudeTemperature&0x0000000FF);

	params[0]=(rcvd[0]&0x06) + RECORD_FORMAT_VERSION + temperatureHB + humidityHB + (archDay<<8) + (archHour<<16) + (archMin<<24);
	params[1]=rcvd[9] + (rcvd[1]<<8) + (rcvd[2]<<16) + (rcvd[3]<<24);
	params[2]=rcvd[4]	+ (rcvd[5]<<8) + (rcvd[6]<<16) + (rcvd[7]<<24);
	params[3]=rcvd[8] +	temperatureLB + humidityLB + maskedAltitudeHB;
	params[4]=(maskedAltitudeLB + (rcvd[23]<<8) + (rcvd[10]<<16) + (rcvd[11]<<24));
	params[5]=rcvd[12]	+ (rcvd[13]<<8) + (rcvd[14]<<16) + (rcvd[15]<<24);
	params[6]=rcvd[16]	+ (rcvd[17]<<8) + (rcvd[18]<<16) + (rcvd[19]<<24);
	params[7]=rcvd[20]	+ (rcvd[21]<<8) + (rcvd[22]<<16) + (rcvd[24]<<24);

#ifdef DO_DIAGS
 	printf("%X %X %X %X %X\r\n",addrToUse,*((int*)addrToUse),archMon,archDay,archHour);
// 	delay_ms(100);
#endif

	testForBlank=addrToUse;
	if((*((int*)testForBlank))==0xFFFFFFFF){
	writeRecToFlash(addrToUse, params, ARCHRECORDLEN);
	} else {
	if((addrToUse&0xFFFE0000)==addrToUse){//if beginning of a new sector
	eraseSector(addrToUse);
	writeRecToFlash(addrToUse, params, ARCHRECORDLEN);
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
	}
	}
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog

#ifdef DO_DIAGS
 	printf("PGM FINISHED\r\n");
// 	delay_ms(100);
#endif
#ifdef DO_HOURLY_CSV
	if(archMin==0x59){
		floatCamForSDCard();//Needs this after sleep pins.
		oneDayToSD(archDay,0);
	}
#else
	if(((archHour==0x23)&&(archMin==0x59))||((archHour==0x23)&&(archMin==0x57))){
		floatCamForSDCard();//Needs this after sleep pins.
		oneDayToSD(archDay,0);
	}
#endif	
	return 0;
}
/***********************************************************************
*  Calc 10bit Delta
***********************************************************************/
unsigned short calc10bitDelta (unsigned short baseP, unsigned short nextP){
	unsigned short deltaP;
	if(nextP>=baseP){
		deltaP = nextP - baseP;
		//		deltaP >>= ALTITUDE_DIV;
		if(deltaP>0x01FF){
			deltaP=0x01FF;
			altOverFlowFlag=1;
		}else altOverFlowFlag=0;
		}else{
		deltaP = baseP - nextP;
		//		deltaP >>= ALTITUDE_DIV;
		if(deltaP>0x1FF){
			deltaP=0x1FF;
			altOverFlowFlag=1;
		}else altOverFlowFlag=0;
		deltaP|=0x200;
	}
	return deltaP;
}
/***********************************************************************
*  Calc 6bit Delta
***********************************************************************/
unsigned short calc6bitDelta (unsigned short baseP, unsigned short nextP){
	unsigned short deltaP;
	if(nextP>=baseP){
		deltaP = nextP - baseP;
		if(deltaP>0x001F){
			deltaP=0x001F;
			altOverFlowFlag=1;
		}else altOverFlowFlag=0;
		}else{
		deltaP = baseP - nextP;
		if(deltaP>0x001F){
			deltaP=0x001F;
			altOverFlowFlag=1;
		}else altOverFlowFlag=0;
		deltaP|=0x0020;
	}
	return deltaP;
}
/***********************************************************************
*  Log a Motion Record in Archive
***********************************************************************/
char logGPSRecord(char archMon,char archDay,char archHour,char archMin){
	unsigned int params[]={0,0,0,0};
	unsigned int addrToUse;
	char hrPos;
	if (archHour < 2) hrPos=0;
	else if (archHour < 6) hrPos=1;
	else if (archHour < 10) hrPos=2;
	else if (archHour < 14) hrPos=3;
	else if (archHour < 18) hrPos=4;
	else if (archHour < 22) hrPos=5;
	else hrPos = 0;
	addrToUse = ARCHIVEADDR + ((archMon-1)*131072) + ((archDay-1)*4096) + 3840 + ARCHRECORDLEN/*skip a rec for GPS flag*/ + hrPos*16;
	params[0]=0;
	params[0]+=(8<<28);//YEAR  y
	params[0]+=(archMon<<24);//MONTH  mo
	params[0]+=(archDay<<19);//DAY
	params[0]+=(archHour<<14);//HOUR
	params[0]+=(archMin<<8);//MIN

	//after calling gpsReqLocRtry();
	unsigned int j,k,m,n,p;
	j=1;				//Lat sign  (1=positive)
	params[0]+=(j<<7)	/*Lat sign*/
	+(asciiDecToHex(0x34,0x36));			/*Lat whole*/

	k=((asciiDecToHex(0x30,0x39))*10000)	//Lat fraction
	+((asciiDecToHex(0x39,0x39))*100)
	+(asciiDecToHex(0x39,0x39));

	j=0;				//Long sign  (1=positive)

	m=100;				/*high digit of Long whole*/
	n=m+(unsigned short)(asciiDecToHex(0x31,0x32)); /*long whole*/

	p=((asciiDecToHex(0x30,0x38))*10000)	//Lat fraction
	+((asciiDecToHex(0x30,0x30))*100)
	+(asciiDecToHex(0x30,0x31));

	params[1]=((k<<15)+(j<<14)+(n<<6)+(p>>11));
	params[2]=(p<<21);

	j=(unsigned int)((asciiDecToHex(0x31,0x32))*100)//elevation
	+(asciiDecToHex(0x33,0x34));

	//	if (0x2a==0x2D)j=8192-j;//Elevation sign
	params[2]+=(j<<8);//elevation

	params[3]=0xC6F1BC00;	/*expansion*/

	writeRecToFlash(addrToUse, params, ARCHRECORDLEN);
	return 0;
}
/***********************************************************************
*  Log GPS Flag
***********************************************************************/
char logGPSflag(char archMon,char archDay){
	unsigned int params[]={0,0,0,0};
	unsigned int addrToUse;
	addrToUse = ARCHIVEADDR + ((archMon-1)*131072) + ((archDay-1)*4096) + 3840;
	params[1]=0x22222222;
	params[2]=0x22222222;
	params[3]=0x22222222;
	params[0]=0x22222222;
	writeRecToFlash(addrToUse, params, ARCHRECORDLEN);
	return 0;
}
/***********************************************************************
*  Init Flash Archive
***********************************************************************/
void initFlashStuff(void){
	/* FLASH PGM STUFF Initialize flash: 6 wait states for flash writing. */
	unsigned int ul_rc;
	ul_rc = flash_init(FLASH_ACCESS_MODE_128, 6);
	if (ul_rc != FLASH_RC_OK) {
//		LCDclear();
//		printf("ERROR Init %lu\n\r", (unsigned long)ul_rc);
		delay_s(1);
	}
	/*Unlock Archive*/
//	LCDclear();
//	printf("Unlock: 0x%08x\r\n", 0x500);
	ul_rc = flash_unlock(ARCHIVEADDR, PASTARCHIVEADDR-1, 0, 0);
//	ul_rc = flash_unlock(CALENDAR_USER_ADDR, PASTARCHIVEADDR-1, 0, 0);
if (ul_rc != FLASH_RC_OK) {
//		LCDbottomLine();
//		printf("-F- Unlock error %lu\n\r", (unsigned long)ul_rc);
		delay_s(1);
	}
}

/****************************************************************************************************************
 * SDRAM TEST
 ****************************************************************************************************************/
void SDRAMtest (void){
	char *camBuffPtr;
	camBuffPtr=SRAM_BASE;//camBuff;//camBuff=SRAM_BASE+0x400
	int barCtr,rowCtr;
	SCB_EnableDCache();//for SD to work twice in a row, have to enable DCache before cleanInvalidate,disable.
	SCB_CleanInvalidateDCache();
	ioport_set_pin_level(CPWRC_GPIO,1);
	pmc_enable_periph_clk(ID_SDRAMC);
	/* Complete SDRAM configuration */
	configPinsSDRAM();
	printf("SRAM\r\n");
 	ioport_set_pin_dir(GPIO_PA8, IOPORT_DIR_OUTPUT);//SRAM PWRC
 	ioport_set_pin_level(GPIO_PA8, IOPORT_PIN_LEVEL_HIGH);//SRAM PWRC
	sdramc_init((sdramc_memory_dev_t *)&SDRAM_INSIGNIS_16M,	sysclk_get_cpu_hz());
	sdram_enable_unaligned_support();
	printf("SRAM COMPLETE\r\n");
	delay_ms(200);

	SCB_CleanInvalidateDCache();
	camBuffPtr=SRAM_BASE;//camBuff=SRAM_BASE+0x400   SRAM_BASE = 0x71000000 to 72FFFFFF
 	for(rowCtr=0;rowCtr<1200;rowCtr++){
	for(barCtr=0;barCtr<400;barCtr+=2){
		*camBuffPtr++=0xFF;//WHITE
		*camBuffPtr++=0xFF;
	}
	for(barCtr=0;barCtr<400;barCtr+=2){
		*camBuffPtr++=0x07;//YEL
		*camBuffPtr++=0xFF;
	}
	for(barCtr=0;barCtr<400;barCtr+=2){
		*camBuffPtr++=0xFF;//CYAN
		*camBuffPtr++=0xE0;
	}
	for(barCtr=0;barCtr<400;barCtr+=2){
		*camBuffPtr++=0x07;//GREEN
		*camBuffPtr++=0xE0;
	}
	for(barCtr=0;barCtr<400;barCtr+=2){
		*camBuffPtr++=0xF8;//MAGENTA
		*camBuffPtr++=0x1F;
	}
	for(barCtr=0;barCtr<400;barCtr+=2){
		*camBuffPtr++=0x00;//RED
		*camBuffPtr++=0x1F;
	}
	for(barCtr=0;barCtr<400;barCtr+=2){
		*camBuffPtr++=0xF8;//BLUE
		*camBuffPtr++=0x00;
	}
	for(barCtr=0;barCtr<400;barCtr+=2){
		*camBuffPtr++=0x00;//BLACK
		*camBuffPtr++=0x00;
	}
	}
	
// 	for(camCtrOut=0;camCtrOut<512;camCtrOut++){
// 	for(camCtr=0;camCtr<0x10000/*CAMLEN_2MPIX*/;camCtr+=16){
// 		*camBuffPtr++=(char)(camCtrOut>>8);
// 		*camBuffPtr++=(char)(camCtrOut&0x00FF);
// 		*camBuffPtr++=(char)(camCtr>>8);
//  		*camBuffPtr++=(char)(camCtr&0x00FF);
//  		*camBuffPtr++=0x44;
//  		*camBuffPtr++=0x55;
//  		*camBuffPtr++=0x66;
//  		*camBuffPtr++=0x77;
//  		*camBuffPtr++=0x88;
//  		*camBuffPtr++=0x99;
//  		*camBuffPtr++=0xAA;
//  		*camBuffPtr++=0xBB;
//  		*camBuffPtr++=0xCC;
//  		*camBuffPtr++=0xDD;
//  		*camBuffPtr++=0xEE;
//  		*camBuffPtr++=0xFF;
// 	}
// 	}
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
	SCB_CleanInvalidateDCache();
	camBuffPtr=0x710000EE;
	printf("%X %X\r\n",camBuffPtr,*camBuffPtr++);
	camBuffPtr=0x710000EE;
	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
	camBuffPtr=0x711000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x712000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
//	camBuffPtr=0x713000EE;
//	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
//	camBuffPtr=0x714000EE;
//	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x715000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x716000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x717000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x718000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x719000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x71A000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x71B000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x71C000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x71D000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x71E000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x71F000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x720000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
 	camBuffPtr=0x721000EE;
 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
//  	camBuffPtr=0x722000EE;
//  	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x723000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x724000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x725000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x726000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x727000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x728000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x729000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x72A000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
//  	camBuffPtr=0x72B000EE;
//  	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
//  	camBuffPtr=0x72C000EE;
//  	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x72D000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x72E000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
// 	camBuffPtr=0x72F000EE;
// 	printf("%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X\r\n",camBuffPtr,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++,*camBuffPtr++);
	
	delay_ms(100);//
	SCB_DisableDCache();
// 	ioport_set_pin_level(GPIO_PA8, IOPORT_PIN_LEVEL_LOW);//SRAM PWRC
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
//	resetPinsSDRAM();
}
/*********************************************************
 * SEND CMD TO TINY
 *********************************************************/

 void sendCmdToTiny(char cmdToSend){

	 if((cmdToSend&0x80)==0)ioport_set_pin_level(BigTiny_DAT,0);
	 else ioport_set_pin_level(BigTiny_DAT,1);
	 delay_us(TINY_CLK_DELAY);//likely need longer to wake Tiny from sleep
	 ioport_set_pin_level(BigTiny_CLK,0);
	 delay_us(TINY_CLK_DELAY);
	 ioport_set_pin_level(BigTiny_CLK,1);

	 if((cmdToSend&0x40)==0)ioport_set_pin_level(BigTiny_DAT,0);
	 else ioport_set_pin_level(BigTiny_DAT,1);
	 delay_us(TINY_CLK_DELAY);//likely need longer to wake Tiny from sleep
	 ioport_set_pin_level(BigTiny_CLK,0);
	 delay_us(TINY_CLK_DELAY);
	 ioport_set_pin_level(BigTiny_CLK,1);

	 if((cmdToSend&0x20)==0)ioport_set_pin_level(BigTiny_DAT,0);
	 else ioport_set_pin_level(BigTiny_DAT,1);
	 delay_us(TINY_CLK_DELAY);//likely need longer to wake Tiny from sleep
	 ioport_set_pin_level(BigTiny_CLK,0);
	 delay_us(TINY_CLK_DELAY);
	 ioport_set_pin_level(BigTiny_CLK,1);

	 if((cmdToSend&0x10)==0)ioport_set_pin_level(BigTiny_DAT,0);
	 else ioport_set_pin_level(BigTiny_DAT,1);
	 delay_us(TINY_CLK_DELAY);//likely need longer to wake Tiny from sleep
	 ioport_set_pin_level(BigTiny_CLK,0);
	 delay_us(TINY_CLK_DELAY);
	 ioport_set_pin_level(BigTiny_CLK,1);

	 if((cmdToSend&0x08)==0)ioport_set_pin_level(BigTiny_DAT,0);
	 else ioport_set_pin_level(BigTiny_DAT,1);
	 delay_us(TINY_CLK_DELAY);//likely need longer to wake Tiny from sleep
	 ioport_set_pin_level(BigTiny_CLK,0);
	 delay_us(TINY_CLK_DELAY);
	 ioport_set_pin_level(BigTiny_CLK,1);

	 if((cmdToSend&0x04)==0)ioport_set_pin_level(BigTiny_DAT,0);
	 else ioport_set_pin_level(BigTiny_DAT,1);
	 delay_us(TINY_CLK_DELAY);
	 ioport_set_pin_level(BigTiny_CLK,0);
	 delay_us(TINY_CLK_DELAY);
	 ioport_set_pin_level(BigTiny_CLK,1);

	 if((cmdToSend&0x02)==0)ioport_set_pin_level(BigTiny_DAT,0);
	 else ioport_set_pin_level(BigTiny_DAT,1);
	 delay_us(TINY_CLK_DELAY);
	 ioport_set_pin_level(BigTiny_CLK,0);
	 delay_us(TINY_CLK_DELAY);
	 ioport_set_pin_level(BigTiny_CLK,1);

	 if((cmdToSend&0x01)==0)ioport_set_pin_level(BigTiny_DAT,0);
	 else ioport_set_pin_level(BigTiny_DAT,1);
	 delay_us(TINY_CLK_DELAY);
	 ioport_set_pin_level(BigTiny_CLK,0);
	 delay_us(TINY_CLK_DELAY);
	 ioport_set_pin_level(BigTiny_CLK,1);

 }
/*********************************************************
 * GET BYTE FROM TINY
 *********************************************************/
char getByteFromTiny(void){
	char ret=0;

	ioport_set_pin_level(BigTiny_CLK,1);
	delay_us(TINY_CLK_DELAY);//likely need longer to wake Tiny from sleep. Making only this one longer doesn't help.
	ioport_set_pin_level(BigTiny_CLK,0);
	if(ioport_get_pin_level(BigTiny_DAT))ret|=0x80;
	delay_us(TINY_CLK_DELAY);

	ioport_set_pin_level(BigTiny_CLK,1);
	delay_us(TINY_CLK_DELAY);
	ioport_set_pin_level(BigTiny_CLK,0);
	if(ioport_get_pin_level(BigTiny_DAT))ret|=0x40;
	delay_us(TINY_CLK_DELAY);

	ioport_set_pin_level(BigTiny_CLK,1);
	delay_us(TINY_CLK_DELAY);
	ioport_set_pin_level(BigTiny_CLK,0);
	if(ioport_get_pin_level(BigTiny_DAT))ret|=0x20;
	delay_us(TINY_CLK_DELAY);

	ioport_set_pin_level(BigTiny_CLK,1);
	delay_us(TINY_CLK_DELAY);
	ioport_set_pin_level(BigTiny_CLK,0);
	if(ioport_get_pin_level(BigTiny_DAT))ret|=0x10;
	delay_us(TINY_CLK_DELAY);

	ioport_set_pin_level(BigTiny_CLK,1);
	delay_us(TINY_CLK_DELAY);
	ioport_set_pin_level(BigTiny_CLK,0);
	if(ioport_get_pin_level(BigTiny_DAT))ret|=0x08;
	delay_us(TINY_CLK_DELAY);

	ioport_set_pin_level(BigTiny_CLK,1);
	delay_us(TINY_CLK_DELAY);
	ioport_set_pin_level(BigTiny_CLK,0);
	if(ioport_get_pin_level(BigTiny_DAT))ret|=0x04;
	delay_us(TINY_CLK_DELAY);

	ioport_set_pin_level(BigTiny_CLK,1);
	delay_us(TINY_CLK_DELAY);
	ioport_set_pin_level(BigTiny_CLK,0);
	if(ioport_get_pin_level(BigTiny_DAT))ret|=0x02;
	delay_us(TINY_CLK_DELAY);

	ioport_set_pin_level(BigTiny_CLK,1);
	delay_us(TINY_CLK_DELAY);
	ioport_set_pin_level(BigTiny_CLK,0);
	if(ioport_get_pin_level(BigTiny_DAT))ret|=0x01;
	delay_us(TINY_CLK_DELAY);

	return ret;
}
/***********************************************************************/
/*  CRC   tag IDs took 550usec at intern ref clk,0div
***********************************************************************/
unsigned short calcCRC(char cbuff[], int LEN) {
   int i,j;
   unsigned short X = 0xFFFF;
   unsigned short Y = 0x0080;
   unsigned short Z;
   for (i=0;i<LEN;i++){       //for each element
     Y = 0x0080;
     for (j=0;j<8;j++){
       Z = X;
       X <<= 1;
       if((Y & cbuff[i]) != 0){ X++;};
       Y >>= 1;
       if ((Z & 0x8000) != 0) {X ^= 0x1021; };
    };   //end 8x
//    __RESET_WATCHDOG();	/*needed Jan 2014*/
   };    // end for each element
  for (i=0;i<16;i++){
    if ((X & 0x8000) != 0) { X<<=1; X ^= 0x1021; } else X <<= 1;
  };     //end 16x
  return X;
}
/***********************************************************************/
/*  hex2ToAscii 2 bytes hex to 4 bytes Ascii, plus an optional high char of ascii
***********************************************************************/
void hex2ToAscii(int hexx, char* asciiChars){
   hexx &= 0x000FFFFF;
   asciiChars[0] = hex1ToAscii(hexx);
   hexx >>= 4;
   asciiChars[1] = hex1ToAscii(hexx);
   hexx >>= 4;
   asciiChars[2] = hex1ToAscii(hexx);
   hexx >>= 4;
   asciiChars[3] = hex1ToAscii(hexx);
   hexx >>= 4;
   asciiChars[4] = hex1ToAscii(hexx);
}
 /***********************************************************************/
 /*  hex1ToAscii nibble to 2-byte Ascii
 ***********************************************************************/
 static unsigned int hex1ToAscii(int hex){
   int a;
   hex &= 0x000F;
     switch (hex) {
       case 0: a = 0x30;
       break;
       case 1: a = 0x31;
       break;
       case 2: a = 0x32;
       break;
       case 3: a = 0x33;
       break;
       case 4: a = 0x34;
       break;
       case 5: a = 0x35;
       break;
       case 6: a = 0x36;
       break;
       case 7: a = 0x37;
       break;
       case 8: a = 0x38;
       break;
       case 9: a = 0x39;
       break;
       case 10: a = 0x41;
       break;
       case 11: a = 0x42;
       break;
       case 12: a = 0x43;
       break;
       case 13: a = 0x44;
       break;
       case 14: a = 0x45;
       break;
       case 15: a = 0x46;
       break;
     };
    return a;
 }
/***********************************************************************/
/*  Ascii to Hex.  Converts two ascii HEX digits to hex integer.
***********************************************************************/
int asciiHexToHex(char hiDig, char loDig){
	int hexTotal=0;
	switch (loDig){
	case 0x30: hexTotal=0;break;
	case 0x31: hexTotal=1;break;
	case 0x32: hexTotal=2;break;
	case 0x33: hexTotal=3;break;
	case 0x34: hexTotal=4;break;
	case 0x35: hexTotal=5;break;
	case 0x36: hexTotal=6;break;
	case 0x37: hexTotal=7;break;
	case 0x38: hexTotal=8;break;
	case 0x39: hexTotal=9;break;
	case 0x41: hexTotal=10;break;
	case 0x42: hexTotal=11;break;
	case 0x43: hexTotal=12;break;
	case 0x44: hexTotal=13;break;
	case 0x45: hexTotal=14;break;
	case 0x46: hexTotal=15;break;
	default: hexTotal=0;break;
	};
	switch (hiDig){
	case 0x30: hexTotal+=0;break;
	case 0x31: hexTotal+=16;break;
	case 0x32: hexTotal+=32;break;
	case 0x33: hexTotal+=48;break;
	case 0x34: hexTotal+=64;break;
	case 0x35: hexTotal+=80;break;
	case 0x36: hexTotal+=96;break;
	case 0x37: hexTotal+=112;break;
	case 0x38: hexTotal+=128;break;
	case 0x39: hexTotal+=144;break;
	case 0x41: hexTotal+=160;break;
	case 0x42: hexTotal+=176;break;
	case 0x43: hexTotal+=192;break;
	case 0x44: hexTotal+=208;break;
	case 0x45: hexTotal+=224;break;
	case 0x46: hexTotal+=240;break;
	default: hexTotal=+0;break;
	};
return hexTotal;
}
/***********************************************************************/
/*  Ascii to Hex.  Convertst two ascii DECIMAL digits to hex integer.
***********************************************************************/
int asciiDecToHex(char hiDig, char loDig){
int hexTotal=0;
switch (loDig){
case 0x30: hexTotal=0;break;
case 0x31: hexTotal=1;break;
case 0x32: hexTotal=2;break;
case 0x33: hexTotal=3;break;
case 0x34: hexTotal=4;break;
case 0x35: hexTotal=5;break;
case 0x36: hexTotal=6;break;
case 0x37: hexTotal=7;break;
case 0x38: hexTotal=8;break;
case 0x39: hexTotal=9;break;
default: hexTotal=0;break;
};
switch (hiDig){
case 0x30: hexTotal+=0;break;
case 0x31: hexTotal+=10;break;
case 0x32: hexTotal+=20;break;
case 0x33: hexTotal+=30;break;
case 0x34: hexTotal+=40;break;
case 0x35: hexTotal+=50;break;
case 0x36: hexTotal+=60;break;
case 0x37: hexTotal+=70;break;
case 0x38: hexTotal+=80;break;
case 0x39: hexTotal+=90;break;
default: hexTotal=+0;break;
};
return hexTotal;
}
/***********************************************************************/
/*  Converts BCD to Hex.
***********************************************************************/
char bcdToHex (char toConvert){
char inHex;
inHex = ((toConvert>>4)*10)+(toConvert%16);
return inHex;
}
/***********************************************************************/
/*  Converts Hex to BCD. Works up to 99,999.
***********************************************************************/
static int hexToBCD (int toConvert){
	char tenThous=0;
	char thousands=0;
	char hundreds=0;
	char tens=0;
	if(toConvert>89999){tenThous=9;toConvert-=90000;}
	else if(toConvert>79999){tenThous=8;toConvert-=80000;}
	else if(toConvert>69999){tenThous=7;toConvert-=70000;}
	else if(toConvert>59999){tenThous=6;toConvert-=60000;}
	else if(toConvert>49999){tenThous=5;toConvert-=50000;}
	else if(toConvert>39999){tenThous=4;toConvert-=40000;}
	else if(toConvert>29999){tenThous=3;toConvert-=30000;}
	else if(toConvert>19999){tenThous=2;toConvert-=20000;}
	else if(toConvert>9999){tenThous=1;toConvert-=10000;}

	if(toConvert>8999){thousands=9;toConvert-=9000;}
	else if(toConvert>7999){thousands=8;toConvert-=8000;}
	else if(toConvert>6999){thousands=7;toConvert-=7000;}
	else if(toConvert>5999){thousands=6;toConvert-=6000;}
	else if(toConvert>4999){thousands=5;toConvert-=5000;}
	else if(toConvert>3999){thousands=4;toConvert-=4000;}
	else if(toConvert>2999){thousands=3;toConvert-=3000;}
	else if(toConvert>1999){thousands=2;toConvert-=2000;}
	else if(toConvert>999){thousands=1;toConvert-=1000;}

	if(toConvert>899){hundreds=9;toConvert-=900;}
	else if(toConvert>799){hundreds=8;toConvert-=800;}
	else if(toConvert>699){hundreds=7;toConvert-=700;}
	else if(toConvert>599){hundreds=6;toConvert-=600;}
	else if(toConvert>499){hundreds=5;toConvert-=500;}
	else if(toConvert>399){hundreds=4;toConvert-=400;}
	else if(toConvert>299){hundreds=3;toConvert-=300;}
	else if(toConvert>199){hundreds=2;toConvert-=200;}
	else if(toConvert>99){hundreds=1;toConvert-=100;}

	if(toConvert>89){tens=9;toConvert-=90;}
	else if(toConvert>79){tens=8;toConvert-=80;}
	else if(toConvert>69){tens=7;toConvert-=70;}
	else if(toConvert>59){tens=6;toConvert-=60;}
	else if(toConvert>49){tens=5;toConvert-=50;}
	else if(toConvert>39){tens=4;toConvert-=40;}
	else if(toConvert>29){tens=3;toConvert-=30;}
	else if(toConvert>19){tens=2;toConvert-=20;}
	else if(toConvert>9){tens=1;toConvert-=10;}

	return (tenThous*65536)+(thousands*4096)+(hundreds*256)+(tens*16)+toConvert;
}
/***********************************************************************/
/*  Interrupt from Tiny
***********************************************************************/
void getGPSdateTimeFrmTiny(void){
	char rcvFromTinyGG[56];
	char writeRet,iters;
	unsigned short crcrc;
	int yearNow;

	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOD);
	ioport_set_pin_dir(BigTiny_CLK, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(BigTiny_CLK, 1);
	ioport_set_pin_dir(BigTiny_DAT, IOPORT_DIR_INPUT);
	ioport_set_pin_level(BigTiny_CLK, 0);//start condition for big-to-tiny
	delay_us(150);
	for(iters=0;iters<25;iters++){
		rcvFromTinyGG[iters]=getByteFromTiny();
	}
	ioport_set_pin_level(BigTiny_CLK, 1);//start condition for tiny-to-big
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
	crcrc=calcCRC(rcvFromTinyGG,23);
	if(((crcrc>>8)!=rcvFromTinyGG[23])||((crcrc&0x00FF)!=rcvFromTinyGG[24])){
		ioport_set_pin_dir(BigTiny_CLK, IOPORT_DIR_OUTPUT);//WakeUp Tiny
		ioport_set_pin_level(BigTiny_CLK, 1);//start condition
		ioport_set_pin_dir(BigTiny_DAT, IOPORT_DIR_OUTPUT);//WakeUp Tiny
		sendCmdToTiny('b');//potential addition tshoooooooooooot
		ioport_set_pin_dir(BigTiny_DAT, IOPORT_DIR_INPUT);//WakeUp Tiny
#ifdef DO_DIAGS
		configure_console();  //TSHOOOOOOOOOOOOOOT
		printf("\r\nFAIL CRC TO TINY\r\n");
#endif
	}else{	//Good GPS from Tiny
		ioport_set_pin_dir(BigTiny_CLK, IOPORT_DIR_OUTPUT);//WakeUp Tiny
		ioport_set_pin_level(BigTiny_CLK, 1);//start condition for tiny-to-big
		ioport_set_pin_dir(BigTiny_DAT, IOPORT_DIR_OUTPUT);//WakeUp Tiny
		sendCmdToTiny('G');//potential addition tshoooooooooooot
		ioport_set_pin_dir(BigTiny_DAT, IOPORT_DIR_INPUT);//WakeUp Tiny
		yearNow = rcvFromTinyGG[9];
		monthNow = rcvFromTinyGG[10];
		dayNow = rcvFromTinyGG[11];
		hourNow = rcvFromTinyGG[12];
		minsNow = rcvFromTinyGG[13];
		secNow = rcvFromTinyGG[14];
//		dayMin = (hourNow*60) + minsNow;

	pmc_enable_periph_clk(ID_PIOC);
	unsigned char rtcData[26];// = {0x00,0x00,0xFF,0xFF};//!< Define data buffer
	
	ioport_set_pin_dir(RTC_INT, IOPORT_DIR_INPUT);//RTC INT
	ioport_set_pin_mode(RTC_INT, IOPORT_MODE_PULLUP);//RTC INT
	
	ioport_set_pin_dir(SCL_RTC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(SDA_RTC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(SCL_RTC, 1);
	ioport_set_pin_level(SDA_RTC, 1);
	delay_ms(200);//looked fine even at only 1ms (when repeated every 500ms);
	twi_init_RTC();
	delay_ms(500);//
	ioport_set_pin_dir(SDA_RTC, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(SDA_RTC, IOPORT_MODE_PULLUP);//RTC INT
	rtcData[0]=0x00;//write to Address 0x00  SET ALL
	rtcData[1]=0x02;//contents of Addr 0x00 CTL1
	rtcData[2]=0x00;//contents of Addr 0x01 CTL2
	rtcData[3]=0x00;//contents of Addr 0x02 CTL3
	rtcData[4]=hexToBCD(secNow);//contents of Addr 0x03 SECONDS works
	rtcData[5]=hexToBCD(minsNow);//contents of Addr 0x04 MINUTES works
	rtcData[6]=hexToBCD(hourNow);//contents of Addr 0x05 HOURS works
	rtcData[7]=hexToBCD(dayNow);//contents of Addr 0x06 DAYS
	rtcData[8]=0x02;//contents of Addr 0x07 WEEKDAY
	rtcData[9]=hexToBCD(monthNow);//contents of Addr 0x08 MONTH
	rtcData[0x0A]=hexToBCD(yearNow);//contents of Addr 0x09 YEAR
	rtcData[0x0B]=0x80;//contents of Addr 0x0A  0x80 = second alarm disabled
	rtcData[0x0C]=0x80;//contents of Addr 0x0B  0x80 = minute alarm disabled
	rtcData[0x0D]=0x80;//contents of Addr 0x0C  0x80 = Hour alarm disabled
	rtcData[0x0E]=0x80;//contents of Addr 0x0D  0x80 = day alarm disabled
	rtcData[0x0F]=0x80;//contents of Addr 0x0E  0x80 = weekday alarm disabled
	rtcData[0x10]=0x00;//contents of Addr 0x0F  0x00 = temperature clockout period 4 minutes
	rtcData[0x11]=0x00;//contents of Addr 0x10  20=PULSED INTERRUPT
	writeRet = write_data_RTC(rtcData,18);//!< write sequentially to the slave "1" sends addr plus data[0].
	delay_ms(200);//
	rtcData[0]=0x01;//write to Address 0x00    NEED THIS TO ACK THE RTC INTERRUPT
	rtcData[1]=0x30;//contents of Addr 0x00 CTL1
	write_data_RTC(rtcData,2);//!< write sequentially to the slave "1" sends addr plus data[0].
#ifdef DO_DIAGS
		configure_console();  //TSHOOOOOOOOOOOOOOT
		printf("\r\nGOOD DATA RCVD FROM TINY: %d-%d-%d %d:%d \r\n",monthNow, dayNow, yearNow, hourNow, minsNow);
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
		delay_s(2);
#endif

	}
	ioport_set_pin_dir(BigTiny_CLK, IOPORT_DIR_OUTPUT);//WakeUp Tiny
	ioport_set_pin_level(BigTiny_CLK, 1);//start condition for tiny-to-big
	ioport_set_pin_dir(BigTiny_DAT, IOPORT_DIR_OUTPUT);//WakeUp Tiny
	ioport_set_pin_dir(BigTiny_DAT, IOPORT_DIR_INPUT);//WakeUp Tiny
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
	delay_ms(1000);
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
}
/***********************************************************************/
/*  Get Humidity
***********************************************************************/
short getHumidity(char slaveAddress){
	unsigned char data[5];
	unsigned char readRet,writeRet = 0;
	char iters;
	unsigned short humidityMeasurement;
	char errCtr;
		SENS_SCL_FLOAT;
		SENS_SDA_FLOAT;
		SENSPWR_OUTPUT;
		SENSPWR_ON;
		delay_ms(10);
 		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
		SENS_SCL_LO;//Make them both low so that when the code drives them they will be low.
		SENS_SDA_LO;//Make them both low so that when the code drives them they will be low.
		data[0]=0xF5;//Config
		write_data_L(data,1,0x80,0);
		delay_us(1);
		errCtr=20;
		data[0]=0;data[1]=0;data[2]=0;
		while((errCtr>0)&&(read_bytes_L(data,3,0x80)==0)){
			errCtr--;
			delay_ms(2);
		}
		humidityMeasurement=(((unsigned int)data[0])<<2)+(((unsigned int)data[1])>>6);//only want the upper ten bits
		SENS_SCL_FLOAT;
		SENS_SDA_FLOAT;
	return(humidityMeasurement);
}
/***********************************************************************/
 /*  Get Altimeter
 ***********************************************************************/
// unsigned int getAltimeterTemperature(void){
// 		unsigned int temperature,pressure;
// 		pmc_enable_periph_clk(ID_PIOD);
// 		SENSPWR_OUTPUT;
// 		SENSPWR_ON;
// 		ioport_set_pin_dir(SENS_CLK, IOPORT_DIR_OUTPUT);
// 		ioport_set_pin_dir(SENS_SDA, IOPORT_DIR_OUTPUT);
// 		ioport_set_pin_level(SENS_CLK, 1);
// 		ioport_set_pin_level(SENS_SDA, 1);
// 		char data[6];
// #ifndef USE_TE_ALTIMETER		
// 		delay_ms(5);//
// 
// 		data[0]=0xAC;//Altimeter Command initiates measurement
// 		write_data_A(data,1);//!< write sequentially to the slave "1" sends addr plus data[0].
// 		char altTimeout = 30;
// 		while((altTimeout>0)&&(data[0]!=0x44)){  //typicall does 9 iterations before data ready
// 			altTimeout--;
// 			delay_ms(1);
//   			read_bytes_A(data,5);//!< write sequentially to the slave "1" sends addr plus data[0].
// 		}
// 		ioport_set_pin_dir(SENS_SDA, IOPORT_DIR_INPUT);//SENS_SDA
// 		ioport_set_pin_dir(SENS_CLK, IOPORT_DIR_INPUT);//ATL_CLK
// 		ioport_set_pin_mode(SENS_SDA, IOPORT_MODE_PULLUP);//SENS_SDA
// 		ioport_set_pin_mode(SENS_CLK, IOPORT_MODE_PULLUP);//ATL_CLK
// 		SENSPWR_OUTPUT;
// 		SENSPWR_OFF;
// 		
// 		temperature = ((data[3]<<8)+(data[4]));
// 		pressure = ((data[1]<<8)+(data[2]));
// 		
// 		temperature = (temperature>>8)*125;
// 		temperature = (temperature>>7)*5;
// 		temperature -= 400;
// 
// 		pressure = (pressure>>6)*25;
// 		pressure = (pressure>>6)*25;
// 		pressure += 2600;
// #endif
// #ifdef DO_DIAGS
// //  	printf("%d  %d\r\n",temperature, pressure);//tshoooooooooooooooooot
// #endif
// 		return ((temperature<<16)+pressure);
//  }
/***********************************************************************/
/*  Get Ambient Light from VEML Sensor
***********************************************************************/
short getALSfromVEML(char slaveAddress){
	unsigned char data[4];
	char iters;
	unsigned short lightMeasurement;
		SENS_SCL_LO;
		SENS_SDA_LO;
		data[0]=0x01;//Register Address 01
		data[1]=0x00;//Config:  High threshold window
		data[2]=0x00;
		write_data_L(data,3,0x20,0);
		data[0]=0x02;//Register Address 02
		data[1]=0x00;//Config:  Low threshold window
		data[2]=0x00;
		write_data_L(data,3,0x20,0);
		data[0]=0x03;//Register Address 03
		data[1]=0x00;//Power saving mode
		data[2]=0x00;
		write_data_L(data,3,0x20,0);
		data[0]=0x00;//Register Address 00
		data[1]=0x00;//Config:  0x000 00(sensitivity=1) 0 00(integration time=100ms)
		data[2]=0x00;//00(integration time) 00(persistence) 00 0(no interrupt) 0(power on)
		write_data_L(data,3,0x20,0);
		delay_ms(100);
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
		data[0]=0x04;//
		write_data_L(data,1,0x20,1);
		SENS_SCL_FLOAT; 		SENS_SDA_FLOAT;
		for(iters=0;iters<3;iters++)data[iters]=0;//ZERO the buffer
 		read_bytes_L(data,2,0x20);
		SENS_SCL_FLOAT; 		SENS_SDA_FLOAT;
		lightMeasurement = (data[1] << 4) + (data[0]>>4);
	return(lightMeasurement);
}
/***********************************************************************/
/*  Send LoRaWAN
***********************************************************************/
unsigned int sendLoRaWAN(char slaveAddr){
	pmc_enable_periph_clk(ID_PIOD);
	pmc_enable_periph_clk(ID_PIOC);
	LRWPI_SCL_FLOAT;
	LRWPI_SDA_FLOAT;
// 	SENSPWR_OUTPUT;
// 	SENSPWR_ON;
	delay_ms(20);//looked fine even at only 1ms (when repeated every 500ms);
	unsigned char data[16];// = {0x00,0x00,0xFF,0xFF};//!< Define data buffer
	unsigned char readRet,writeRet = 0;
	/*******RESET ALTIMETER***/
	data[0]=0x1E;//Altimeter RESET
	writeRet = write_data_LrwPi(data,1,0xEE,0);
	delay_ms(4);//datasheet says 2.8ms needed after reset
	/*******Read Calibration Coefs***/
// 	data[0]=0xA2;//Ax=read PROM
// 	writeRet = write_data_LrwPi(data,1,0xEE,0);
// 	data[0]=0;data[1]=0;
// 	readRet = read_bytes_LrwPi(data,2,0xEE);
// 	C1=(((unsigned int)data[0])<<8)+(((unsigned int)data[1]));
	/**********/

	LRWPI_SCL_FLOAT;
	LRWPI_SDA_FLOAT;
}

/***********************************************************************/
/*  Get Altimeter
***********************************************************************/
unsigned int getAltimeterTemperatureTE(char slaveAddr){
		unsigned int HH64,LL64;
		unsigned int OFF_H,OFF_LL,OFF_M,OFF_L;
		unsigned int SENS_H,SENS_LL,SENS_M,SENS_L;
		signed int compTemperature;
		pmc_enable_periph_clk(ID_PIOD);
		SENS_SCL_FLOAT;
		SENS_SDA_FLOAT;
		SENSPWR_OUTPUT;
		SENSPWR_ON;
		delay_ms(20);//looked fine even at only 1ms (when repeated every 500ms);
		unsigned int rawTemperature;
		unsigned int rawPressure,compPressure=0;
		unsigned int dT=0;
		unsigned int C1=0;unsigned int C2=0;unsigned int C3=0;unsigned int C4=0;unsigned int C5=0;unsigned int C6=0;
		unsigned char data[16];// = {0x00,0x00,0xFF,0xFF};//!< Define data buffer
		unsigned char readRet,writeRet = 0;
		/*******RESET ALTIMETER***/
		data[0]=0x1E;//Altimeter RESET
		writeRet = write_data_L(data,1,0xEE,0);
		delay_ms(4);//datasheet says 2.8ms needed after reset
		/*******Read Calibration Coefs***/
		data[0]=0xA2;//Ax=read PROM
		writeRet = write_data_L(data,1,0xEE,0);
		data[0]=0;data[1]=0;
		readRet = read_bytes_L(data,2,0xEE);
		C1=(((unsigned int)data[0])<<8)+(((unsigned int)data[1]));
		/**********/
		data[0]=0xA4;//Ax=read PROM
		writeRet = write_data_L(data,1,0xEE,0);
		data[0]=0;data[1]=0;
		readRet = read_bytes_L(data,2,0xEE);
		C2=(((unsigned int)data[0])<<8)+(((unsigned int)data[1]));
		/**********/
		data[0]=0xA6;//Ax=read PROM
		writeRet = write_data_L(data,1,0xEE,0);
		data[0]=0;data[1]=0;
		readRet = read_bytes_L(data,2,0xEE);
		C3=(((unsigned int)data[0])<<8)+(((unsigned int)data[1]));
		/**********/
		data[0]=0xA8;//Ax=read PROM
		writeRet = write_data_L(data,1,0xEE,0);
		data[0]=0;data[1]=0;
		readRet = read_bytes_L(data,2,0xEE);
		C4=(((unsigned int)data[0])<<8)+(((unsigned int)data[1]));
		/**********/
		data[0]=0xAA;//Ax=read PROM
		writeRet = write_data_L(data,1,0xEE,0);
		data[0]=0;data[1]=0;
		readRet = read_bytes_L(data,2,0xEE);
		C5=(((unsigned int)data[0])<<8)+(((unsigned int)data[1]));
		/**********/
		data[0]=0xAC;//Ax=read PROM
		writeRet = write_data_L(data,1,0xEE,0);
		data[0]=0;data[1]=0;
		readRet = read_bytes_L(data,2,0xEE);
		C6=(((unsigned int)data[0])<<8)+(((unsigned int)data[1]));
		/*******Read PRESSURE***/
		rawPressure=0;
		rawTemperature=0;

			data[0]=0x48;//48=start conversion of 24 bit pressure
			writeRet = write_data_L(data,1,0xEE,0);
			delay_ms(9);//datasheet says 8ms delay for 4096 OverSampling (OSR) bits
			data[0]=0x00;//00=Read Command
			writeRet = write_data_L(data,1,0xEE,0);
			data[0]=0;data[1]=0;data[2]=0;
			readRet = read_bytes_L(data,3,0xEE);
			rawPressure+=(((int)data[0])<<16)+(((int)data[1])<<8)+(((int)data[2]));

			data[0]=0x58;//58=start conversion of 24 bit temperature
			writeRet = write_data_L(data,1,0xEE,0);
			delay_ms(9);
			data[0]=0x00;//00=Read Command
			writeRet = write_data_L(data,1,0xEE,0);
			data[0]=0;data[1]=0;data[2]=0;
			readRet = read_bytes_L(data,3,0xEE);
			rawTemperature+=(((unsigned int)data[0])<<16)+(((unsigned int)data[1])<<8)+(((unsigned int)data[2]));

#ifdef DO_DIAGS
//printf("%X %X ",rawPressure,rawTemperature);//tshoooooooooooooooooot
#endif

			SENS_SCL_FLOAT;
			SENS_SDA_FLOAT;
			SENSPWR_OUTPUT;
			SENSPWR_OFF;
			C5=C5<<8;

		unsigned int below20;

			if(rawTemperature>C5){
				dT=rawTemperature-C5;//dT is positive
				mult64bit(dT,C6,&HH64,&LL64);
				compTemperature=((int)((LL64>>23)+(HH64<<9)))+2000;

				mult64bit(C4,dT,&HH64,&LL64);
				OFF_L=(LL64>>7)&0x01FFFFFF;
				OFF_L+=(HH64<<25);
				OFF_LL=OFF_L&0x0000FFFF;
				OFF_H=(HH64>>7)&0x01FFFFFF;
				OFF_M=(OFF_H<<16)+(OFF_L>>16)+C2;
				OFF_H=(OFF_M>>16);
				OFF_L=(OFF_M<<16)+OFF_LL;

				mult64bit(C3,dT,&HH64,&LL64);
				SENS_L=(LL64>>8)&0x00FFFFFF;
				SENS_L+=(HH64<<24);
				SENS_LL=SENS_L&0x00007FFF;
				SENS_H=(HH64>>8)&0x00FFFFFF;
				SENS_M=(SENS_H<<17)+(SENS_L>>15)+C1;
				SENS_H=(SENS_M>>17);
				SENS_L=(SENS_M<<15)+SENS_LL;

				mult64bit(rawPressure,SENS_L,&HH64,&LL64);
				compPressure=(((LL64>>21)+(HH64<<11))-(OFF_L))>>15;//assumes OFF_H==0
			}
			else{
					dT=C5-rawTemperature;//dT is negative
					mult64bit(dT,C6,&HH64,&LL64);//28325
					below20 = ((int)((LL64>>23)+(HH64<<9)));
					if(below20<=2000)//dt is negative and between 0C and +20.00C
						compTemperature=2000-below20;
					else
						compTemperature = 0-(below20-2000);

						mult64bit(C4,dT,&HH64,&LL64);
						OFF_L=(LL64>>7)&0x01FFFFFF;
						OFF_L+=(HH64<<25);
						OFF_LL=OFF_L&0x0000FFFF;
						OFF_H=(HH64>>7)&0x01FFFFFF;
						OFF_M=(OFF_H<<16)+(OFF_L>>16);
						OFF_M=C2-OFF_M;
						OFF_H=(OFF_M>>16);
						OFF_L=(OFF_M<<16)+OFF_LL;

						mult64bit(C3,dT,&HH64,&LL64);
						SENS_L=(LL64>>8)&0x00FFFFFF;
						SENS_L+=(HH64<<24);
						SENS_LL=SENS_L&0x00007FFF;
						SENS_H=(HH64>>8)&0x00FFFFFF;
						SENS_M=(SENS_H<<17)+(SENS_L>>15);
						SENS_M=C1-SENS_M;
						SENS_H=(SENS_M>>17);
						SENS_L=(SENS_M<<15)+SENS_LL;

						mult64bit(rawPressure,SENS_L,&HH64,&LL64);
						compPressure=(((LL64>>21)+(HH64<<11))-(OFF_L))>>15;//assumes OFF_H==0
					}
#ifdef DO_DIAGS
// 	printf("%X cP ",compPressure);//tshoooooooooooooooooot
// 	printf("%X %X %X %X %X %X\r\n",C1,C2,C3,C4,C5,C6);//tshoooooooooooooooooot
#endif
		compPressure/=10;
		compTemperature/=10;
		return (((compTemperature<<16)&0xFFFF0000)+(compPressure&0x0000FFFF));
 }
/***********************************************************************/
/*  32 x 32 = 64 bit MULTIPLY
***********************************************************************/
void mult64bit (unsigned int A32,unsigned int B32,unsigned int* resultHigh, unsigned int* resultLow) {
	unsigned int Ahigh,Alow,Bhigh,Blow,BLALL,BLALH,BLAHL,BLAHH,BHALL,BHALH,BHAH,carry;
	Ahigh=(A32>>16)&0x0000FFFF;
	Alow=A32&0x0000FFFF;
	Bhigh=(B32>>16)&0x0000FFFF;
	Blow=B32&0x0000FFFF;

	BLALH=Blow*Alow;
	BLALL=BLALH;
	BLALH=(BLALH>>16)&0x0000FFFF;
	BLALL&=0x0000FFFF;

	BLAHH=Blow*Ahigh;
	BLAHL=BLAHH;
	BLAHH=(BLAHH>>16)&0x0000FFFF;
	BLAHL&=0x0000FFFF;
	BHALH=Bhigh*Alow;
	BHALL=BHALH;
	BHALH=(BHALH>>16)&0x0000FFFF;
	BHALL&=0x0000FFFF;
	BHAH=Bhigh*Ahigh;
	carry=((BLALH+BLAHL+BHALL)&0xFFFF0000)>>16;

	*resultLow=((BLALH+BLAHL+BHALL)<<16)+BLALL;
	*resultHigh=BLAHH+BHALH+BHAH + carry;
}
/*********************************************************
 * SEND PACKET TO TINY
 *********************************************************/
void sendPacketToTiny32(char partialMin){
	short crcrcrc;
	ioport_set_pin_dir(BigTiny_CLK, IOPORT_DIR_OUTPUT);//WakeUp Tiny
	ioport_set_pin_level(BigTiny_CLK, 1);//start condition
	ioport_set_pin_dir(BigTiny_DAT, IOPORT_DIR_OUTPUT);//WakeUp Tiny
	ioport_set_pin_dir(ToTinyWU, IOPORT_DIR_OUTPUT);//WakeUp Tiny
	ioport_set_pin_level(ToTinyWU,0);//WakeUp Tiny
	minPacketForTiny[0]|=(char)((minsNow%6)&0x07);
	minPacketForTiny[1]=partialMin;//Derived from RTC minutes, used by Tiny to determine which portion of Loc to return.
	crcrcrc=calcCRC(minPacketForTiny,2);
	minPacketForTiny[2]=crcrcrc>>8;
	minPacketForTiny[3]=crcrcrc&0x00FF;
	delay_us(50);//was 50us for Tiny817, which didn't always work.
	sendCmdToTiny(minPacketForTiny[0]);
	ioport_set_pin_level(ToTinyWU,1);//UnWakeUp Tiny
	sendCmdToTiny(minPacketForTiny[1]);
	sendCmdToTiny(minPacketForTiny[2]);
	sendCmdToTiny(minPacketForTiny[3]);
	ioport_set_pin_dir(BigTiny_DAT, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(BigTiny_DAT, 0);//BigTiny DAT
}
 /***********************************************************************/
 /*  READ ATMEL'S UNIQUE CHIP ID
 ***********************************************************************/
void getSetDeviceID(void){
	char rdIDerr;
	int unique_id[4];
	rdIDerr = flash_read_unique_id(unique_id, 4);
	if (rdIDerr != FLASH_RC_OK) {
		LCDclear();
		printf("UniqueID error");
	}
	DEVICEID[0]=(char)(unique_id[0]>>24);
	DEVICEID[1]=(char)((unique_id[0]>>16)&0x00FF);
	DEVICEID[2]=(char)((unique_id[0]>>8)&0x00FF);
	DEVICEID[3]=0x30;//(char)(unique_id[0]&0x00FF);
	DEVICEID[4]=(char)(unique_id[1]>>24);
	DEVICEID[5]=(char)((unique_id[1]>>16)&0x00FF);
	DEVICEID[6]=(char)((unique_id[1]>>8)&0x00FF);
	DEVICEID[7]=(char)(unique_id[1]&0x00FF);
	DEVICEID[8]=(char)(unique_id[2]>>24);
	DEVICEID[9]=(char)((unique_id[2]>>16)&0x00FF);
	DEVICEID[10]=(char)((unique_id[2]>>8)&0x00FF);
	DEVICEID[11]=(char)(unique_id[2]&0x00FF);
	DEVICEID[12]=(char)(unique_id[3]>>24);
	DEVICEID[13]=(char)((unique_id[3]>>16)&0x00FF);
	DEVICEID[14]=(char)((unique_id[3]>>8)&0x00FF);
	DEVICEID[15]=(char)(unique_id[3]&0x00FF);
}
/***********************************************************************/
/*  sleep for seconds
***********************************************************************/
void sleepForSecs(unsigned int iters) {
	unsigned int i;
	for(i=iters;i>0;i--){
		//__RESET_WATCHDOG();
		//SRTISC |= 0x40;    /*clear the RTI interrupt flag. Discovered during GPSPTT development, this interrupt gets set while uP is doing stuff*/
		//SRTISC|=0x10;        //enable RTI interrupt
		//asm { stop #0x2000;}  //16 bits loaded into SR. See _Stop() in derivative.h.
		//SRTISC&=~0x10;    //disable RTC interrupt
		delay_ms(1000);
	}
}
/***********************************************************************
 *    CONFIGURE SDRAM PINS
 ***********************************************************************/
void configPinsSDRAM(void){
	pio_configure_pin(SDRAM_A0_PIO, SDRAM_A_FLAGS);	//V71 A2
	pio_configure_pin(SDRAM_A1_PIO, SDRAM_A_FLAGS);	//V71 A3
	pio_configure_pin(SDRAM_A2_PIO, SDRAM_A_FLAGS);	//V71 A4
	pio_configure_pin(SDRAM_A3_PIO, SDRAM_A_FLAGS);  //V71 A5
	pio_configure_pin(SDRAM_A4_PIO, SDRAM_A_FLAGS);  //V71 A6
	pio_configure_pin(SDRAM_A5_PIO, SDRAM_A_FLAGS);  //V71 A7
	pio_configure_pin(SDRAM_A6_PIO, SDRAM_A_FLAGS);  //V71 A8
	pio_configure_pin(SDRAM_A7_PIO, SDRAM_A_FLAGS);  //V71 A9
	pio_configure_pin(SDRAM_A8_PIO, SDRAM_A_FLAGS);	 //V71 A10
	pio_configure_pin(SDRAM_A9_PIO, SDRAM_A_FLAGS);  //V71 A11
	pio_configure_pin(SDRAM_SDA10_PIO, SDRAM_SDA10_FLAGS); //V71 SDA10
	pio_configure_pin(SDRAM_A11_PIO, SDRAM_A13_FLAGS); //V71 A13
	pio_configure_pin(SDRAM_A12_PIO, SDRAM_A14_FLAGS); //V71 A14
	pio_configure_pin(SDRAM_BA0_PIO, SDRAM_BA0_FLAGS);
	pio_configure_pin(SDRAM_BA1_PIO, SDRAM_BA1_FLAGS);
	pio_configure_pin(SDRAM_SDCK_PIO, SDRAM_SDCK_FLAGS);
	pio_configure_pin(SDRAM_SDCKE_PIO, SDRAM_SDCKE_FLAGS);
	pio_configure_pin(SDRAM_SDCS_PIO, SDRAM_SDCS_FLAGS);
	pio_configure_pin(SDRAM_RAS_PIO, SDRAM_RAS_FLAGS);
	pio_configure_pin(SDRAM_CAS_PIO, SDRAM_CAS_FLAGS);
	pio_configure_pin(SDRAM_SDWE_PIO, SDRAM_SDWE_FLAGS);
	pio_configure_pin(SDRAM_NBS0_PIO, SDRAM_NBS0_FLAGS);
	pio_configure_pin(SDRAM_NBS1_PIO, SDRAM_NBS1_FLAGS);
	pio_configure_pin(SDRAM_D0_PIO, SDRAM_D_FLAGS);
	pio_configure_pin(SDRAM_D1_PIO, SDRAM_D_FLAGS);
	pio_configure_pin(SDRAM_D2_PIO, SDRAM_D_FLAGS);
	pio_configure_pin(SDRAM_D3_PIO, SDRAM_D_FLAGS);
	pio_configure_pin(SDRAM_D4_PIO, SDRAM_D_FLAGS);
	pio_configure_pin(SDRAM_D5_PIO, SDRAM_D_FLAGS);
	pio_configure_pin(SDRAM_D6_PIO, SDRAM_D_FLAGS);
	pio_configure_pin(SDRAM_D7_PIO, SDRAM_D_FLAGS);
	pio_configure_pin(SDRAM_D8_PIO, SDRAM_D_FLAGS);
	pio_configure_pin(SDRAM_D9_PIO, SDRAM_D_FLAGS);
	pio_configure_pin(SDRAM_D10_PIO, SDRAM_D_FLAGS);
	pio_configure_pin(SDRAM_D11_PIO, SDRAM_D_FLAGS);
	pio_configure_pin(SDRAM_D12_PIO, SDRAM_D_FLAGS);
	pio_configure_pin(SDRAM_D13_PIO, SDRAM_D_FLAGS);
	pio_configure_pin(SDRAM_D14_PIO, SDRAM_D_FLAGS);
	pio_configure_pin(SDRAM_D15_PIO, SDRAM_D_FLAGS);
}
 /***********************************************************************/
 /*    Dev Board BLINK ATSAM
 ***********************************************************************/
 void redBlink (int numBlinks) {
	 int i;
	 ioport_set_pin_dir(LED_RED_GPIO, IOPORT_DIR_OUTPUT);
	 for (i=numBlinks; i>0; i--){
		 ioport_set_pin_level(LED_RED_GPIO, LED_ACTIVE_LEVEL); //LED ON
		 delay_ms(20);
		 ioport_set_pin_level(LED_RED_GPIO, LED_INACTIVE_LEVEL); //LED OFF
		 delay_ms(50);
	 };
	 ioport_set_pin_dir(LED_RED_GPIO, IOPORT_DIR_INPUT);
 }
 /***********************************************************************/
 /*    Dev Board BLINK ATSAM
 ***********************************************************************/
 void userBlink (int numBlinks) {
	 int i;
	 ioport_set_pin_dir(LED_USER_GPIO, IOPORT_DIR_OUTPUT);
	 for (i=numBlinks; i>0; i--){
		 ioport_set_pin_level(LED_USER_GPIO, LED_ACTIVE_LEVEL); //LED ON
		 delay_ms(20);
		 ioport_set_pin_level(LED_USER_GPIO, LED_INACTIVE_LEVEL); //LED OFF
		 delay_ms(50);
	 };
	 ioport_set_pin_dir(LED_USER_GPIO, IOPORT_DIR_INPUT);
 }
/**************************************************************************
 *  Configure Pins for Sleep.
 **************************************************************************/
void sleepPins(void){
	pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOC);
	pmc_enable_periph_clk(ID_PIOD);
	pmc_enable_periph_clk(ID_PIOE);

	ioport_set_pin_dir(GPIO_PA0, IOPORT_DIR_INPUT);//User Pushbutton, extern pullup stuffed?
	ioport_set_pin_dir(GPIO_PA1, IOPORT_DIR_INPUT);//WakeUP from Tiny
	ioport_set_pin_dir(GPIO_PA2, IOPORT_DIR_INPUT);//RTC INT
	ioport_set_pin_dir(SENS_PWRC, IOPORT_DIR_INPUT);//A6=Barometer PWRC
	ioport_set_pin_dir(GPIO_PA7, IOPORT_DIR_INPUT);//FLIR Pwr Ctrl
	ioport_set_pin_dir(GPIO_PA11, IOPORT_DIR_INPUT);//Flash CS
	ioport_set_pin_dir(GPIO_PA12, IOPORT_DIR_INPUT);//Flash MISO
	ioport_set_pin_dir(GPIO_PA13, IOPORT_DIR_INPUT);//Flash MOSI
	ioport_set_pin_dir(GPIO_PA14, IOPORT_DIR_INPUT);//Flash CLK
	ioport_set_pin_dir(GPIO_PA17, IOPORT_DIR_INPUT);//NC
	ioport_set_pin_dir(GPIO_PA19, IOPORT_DIR_INPUT);//NC
	ioport_set_pin_dir(GPIO_PA22, IOPORT_DIR_INPUT);//LRW PWRC
	ioport_set_pin_dir(GPIO_PA25, IOPORT_DIR_OUTPUT);//SD CLK
	ioport_set_pin_dir(GPIO_PA26, IOPORT_DIR_OUTPUT);//SD D2
	ioport_set_pin_dir(GPIO_PA27, IOPORT_DIR_OUTPUT);//SD D3 - ISI D7
	ioport_set_pin_dir(GPIO_PA28, IOPORT_DIR_OUTPUT);//SD CMD
	ioport_set_pin_dir(GPIO_PA29, IOPORT_DIR_INPUT);//User Pushbutton
	ioport_set_pin_dir(GPIO_PA30, IOPORT_DIR_OUTPUT);//SD D0
	ioport_set_pin_dir(GPIO_PA31, IOPORT_DIR_OUTPUT);//SD D1

	ioport_set_pin_mode(GPIO_PA0, IOPORT_MODE_PULLUP);//User Pushbutton, extern pullup stuffed?
	ioport_set_pin_mode(GPIO_PA1, IOPORT_MODE_OPEN_DRAIN | IOPORT_MODE_GLITCH_FILTER);//DON'T MESS WITH PULLUP ON WAKEUP PIN FROM TINY CUZ IT PULLS SO HIGH TINY CAN'T PULL IT DOWN.
	ioport_set_pin_mode(GPIO_PA2, IOPORT_MODE_PULLUP);//RTC INT	ioport_set_pin_mode(GPIO_PA2, IOPORT_MODE_PULLUP | IOPORT_MODE_GLITCH_FILTER);//RTC INT
	ioport_set_pin_mode(SENS_PWRC, IOPORT_MODE_PULLUP);//A6=Barometer and Humidity PWRC (NO EXTERN PULLUP)
	ioport_set_pin_mode(GPIO_PA7, IOPORT_MODE_OPEN_DRAIN);//FLIR PWRC
	ioport_set_pin_mode(GPIO_PA11, IOPORT_MODE_PULLUP);//Flash CS
	ioport_set_pin_mode(GPIO_PA12, IOPORT_MODE_PULLUP);//Flash MISO
	ioport_set_pin_mode(GPIO_PA13, IOPORT_MODE_PULLUP);//Flash MOSI
	ioport_set_pin_mode(GPIO_PA14, IOPORT_MODE_PULLUP);//Flash CLK
	ioport_set_pin_mode(GPIO_PA17, IOPORT_MODE_PULLUP);//NC
	ioport_set_pin_mode(GPIO_PA19, IOPORT_MODE_PULLUP);//NC
	ioport_set_pin_mode(GPIO_PA22, IOPORT_MODE_PULLUP);//LRW PWRC
	ioport_set_pin_mode(GPIO_PA25, 0);//SD CLK
	ioport_set_pin_mode(GPIO_PA26, 0);//SD D2
	ioport_set_pin_mode(GPIO_PA27, 0);//SD D3 - ISI D7
	ioport_set_pin_level(GPIO_PA28, 0);//SD CMD
	ioport_set_pin_mode(GPIO_PA29, IOPORT_MODE_PULLUP);//User Pushbutton
	ioport_set_pin_mode(GPIO_PA30, 0);//SD D0
	ioport_set_pin_mode(GPIO_PA31, 0);//SD D1
	
	ioport_set_pin_dir(BigTiny_DAT, IOPORT_DIR_INPUT);//BigTiny DAT
	ioport_set_pin_dir(GPIO_PB1, IOPORT_DIR_INPUT);//TINY RESET.  This pin dips during pwr state changes, not from Big, appears internal to Tiny
	ioport_set_pin_dir(GPIO_PB2, IOPORT_DIR_INPUT);//WU TINY
	ioport_set_pin_dir(GPIO_PB4, IOPORT_DIR_INPUT);//TDI
	ioport_set_pin_dir(GPIO_PB5, IOPORT_DIR_INPUT);//BIG_TDO
	ioport_set_pin_dir(GPIO_PB6, IOPORT_DIR_INPUT);//DBG
	//  	ioport_set_pin_dir(GPIO_PB7, IOPORT_DIR_INPUT);//TCLK
	//  	ioport_set_pin_dir(GPIO_PB8, IOPORT_DIR_INPUT);//XIN
	ioport_set_pin_dir(GPIO_PB9, IOPORT_DIR_INPUT);//XOUT
	ioport_set_pin_dir(GPIO_PB12, IOPORT_DIR_INPUT);//CHIP ERASE
	ioport_set_pin_dir(GPIO_PB13, IOPORT_DIR_INPUT);//FLIR MCLK

	ioport_set_pin_mode(BigTiny_DAT, IOPORT_MODE_PULLUP);//BigTiny DAT
	ioport_set_pin_mode(GPIO_PB1, IOPORT_MODE_PULLUP);//TINY RESET.  This pin dips during pwr state changes, not from Big, appears internal to Tiny
	ioport_set_pin_mode(GPIO_PB2, IOPORT_MODE_PULLUP);//WU TINY
	ioport_set_pin_mode(GPIO_PB4, IOPORT_MODE_PULLUP);//TDI
	ioport_set_pin_mode(GPIO_PB5, IOPORT_MODE_PULLUP);//BIG_TDO
	ioport_set_pin_mode(GPIO_PB6, IOPORT_MODE_PULLUP);//DBG
	//  	ioport_set_pin_mode(GPIO_PB7, IOPORT_MODE_PULLUP);//TCLK
	//  	ioport_set_pin_mode(GPIO_PB8, IOPORT_MODE_PULLUP);//XIN
	ioport_set_pin_mode(GPIO_PB9, IOPORT_MODE_PULLUP);//XOUT
	ioport_set_pin_mode(GPIO_PB12, IOPORT_MODE_PULLDOWN);//CHIP ERASE
	ioport_set_pin_mode(GPIO_PB13, IOPORT_MODE_PULLUP);//FLIR MCLK

	ioport_set_pin_dir(GPIO_PC8, IOPORT_DIR_OUTPUT);//Sensors SDA
	ioport_set_pin_dir(GPIO_PC9, IOPORT_DIR_OUTPUT);//Sensors SCL
	ioport_set_pin_dir(GPIO_PC10, IOPORT_DIR_INPUT);//RTC SDA
	ioport_set_pin_dir(GPIO_PC11, IOPORT_DIR_INPUT);//NC
	ioport_set_pin_dir(GPIO_PC12, IOPORT_DIR_INPUT);//USB5V AND PA30
	ioport_set_pin_dir(GPIO_PC13, IOPORT_DIR_INPUT);//RTC SCL
	ioport_set_pin_dir(GPIO_PC14, IOPORT_DIR_INPUT);//nc
	ioport_set_pin_dir(GPIO_PC16, IOPORT_DIR_INPUT);//TP
	ioport_set_pin_dir(GPIO_PC17, IOPORT_DIR_INPUT);//NC
	ioport_set_pin_dir(GPIO_PC19, IOPORT_DIR_INPUT);//Host to LoRaWAN-Pi I2C CLK
	ioport_set_pin_dir(GPIO_PC30, IOPORT_DIR_INPUT);//TP

	ioport_set_pin_level(GPIO_PC8, IOPORT_PIN_LEVEL_LOW);//Sensors SDA
	ioport_set_pin_level(GPIO_PC9, IOPORT_PIN_LEVEL_LOW);//Sensors SCL
	ioport_set_pin_mode(GPIO_PC10, 0);//RTC SDA
	ioport_set_pin_mode(GPIO_PC11, IOPORT_MODE_PULLUP);//NC
	ioport_set_pin_mode(GPIO_PC12, IOPORT_MODE_PULLUP);//USB5V AND PA30
	ioport_set_pin_mode(GPIO_PC13, 0);//RTC SCL
	ioport_set_pin_mode(GPIO_PC14, IOPORT_MODE_PULLUP);//nc
	ioport_set_pin_mode(GPIO_PC16, IOPORT_MODE_PULLUP);//TP
	ioport_set_pin_mode(GPIO_PC17, IOPORT_MODE_PULLUP);//NC
	ioport_set_pin_mode(GPIO_PC19, IOPORT_MODE_PULLUP);//Host to LoRaWAN-Pi I2C CLK
	ioport_set_pin_mode(GPIO_PC30, IOPORT_MODE_PULLUP);//TP

	ioport_set_pin_dir(GPIO_PD0, IOPORT_DIR_INPUT);//Dev board LED
	ioport_set_pin_dir(GPIO_PD1, IOPORT_DIR_INPUT);//nc
	ioport_set_pin_dir(GPIO_PD2, IOPORT_DIR_INPUT);//nc
	ioport_set_pin_dir(GPIO_PD3, IOPORT_DIR_INPUT);//TINY CLK
	ioport_set_pin_dir(GPIO_PD4, IOPORT_DIR_INPUT);//
	ioport_set_pin_dir(GPIO_PD5, IOPORT_DIR_INPUT);//
	ioport_set_pin_dir(GPIO_PD6, IOPORT_DIR_INPUT);//
	ioport_set_pin_dir(GPIO_PD7, IOPORT_DIR_INPUT);//TP
	ioport_set_pin_dir(GPIO_PD8, IOPORT_DIR_OUTPUT);//SD Card PWRC
	ioport_set_pin_dir(GPIO_PD9, IOPORT_DIR_INPUT);//User LED
	ioport_set_pin_dir(GPIO_PD10, IOPORT_DIR_INPUT);//NC
	ioport_set_pin_dir(GPIO_PD18, IOPORT_DIR_INPUT);//FROM XBee
	ioport_set_pin_dir(GPIO_PD19, IOPORT_DIR_INPUT);//TO XBee
	ioport_set_pin_dir(GPIO_PD20, IOPORT_DIR_INPUT);//NC
	ioport_set_pin_dir(GPIO_PD26, IOPORT_DIR_INPUT);//NC
	ioport_set_pin_dir(GPIO_PD27, IOPORT_DIR_INPUT);//NC
	ioport_set_pin_dir(GPIO_PD28, IOPORT_DIR_INPUT);//Host to LoRaWAN-Pi I2C DAT
	ioport_set_pin_dir(GPIO_PD30, IOPORT_DIR_INPUT);//TP
	ioport_set_pin_dir(GPIO_PD31, IOPORT_DIR_INPUT);//NC

	ioport_set_pin_mode(GPIO_PD0, IOPORT_MODE_PULLUP);//Dev board LED
	ioport_set_pin_mode(GPIO_PD1, IOPORT_MODE_PULLUP);//
	ioport_set_pin_mode(GPIO_PD2, IOPORT_MODE_PULLUP);//
	ioport_set_pin_mode(GPIO_PD3, IOPORT_MODE_PULLUP);//TINY CLK
	ioport_set_pin_mode(GPIO_PD4, IOPORT_MODE_PULLUP);//
	ioport_set_pin_mode(GPIO_PD5, IOPORT_MODE_PULLUP);//
	ioport_set_pin_mode(GPIO_PD6, IOPORT_MODE_PULLUP);//
	ioport_set_pin_mode(GPIO_PD7, IOPORT_MODE_PULLUP);//TP
	ioport_set_pin_level(GPIO_PD8, IOPORT_PIN_LEVEL_LOW);//SD PWRC
	ioport_set_pin_mode(GPIO_PD9, IOPORT_MODE_PULLUP);//User LED
	ioport_set_pin_mode(GPIO_PD10, IOPORT_MODE_PULLUP);//NC
	ioport_set_pin_mode(GPIO_PD18, IOPORT_MODE_PULLUP);//NC
	ioport_set_pin_mode(GPIO_PD19, IOPORT_MODE_PULLUP);//NC
	ioport_set_pin_mode(GPIO_PD20, IOPORT_MODE_PULLUP);//NC
	ioport_set_pin_mode(GPIO_PD26, IOPORT_MODE_PULLUP);//NC
	ioport_set_pin_mode(GPIO_PD27, IOPORT_MODE_PULLUP);//NC
	ioport_set_pin_mode(GPIO_PD28, IOPORT_MODE_PULLUP);//Host to LoRaWAN-Pi I2C DAT
	ioport_set_pin_mode(GPIO_PD30, IOPORT_MODE_PULLUP);//TP
	ioport_set_pin_mode(GPIO_PD31, IOPORT_MODE_PULLUP);//NC

	/************* CAMERA ***************/
 	resetPinsISI();
	ioport_set_pin_dir(SDA0_B, IOPORT_DIR_INPUT);//PA3=Camera SDA
	ioport_set_pin_dir(SCL0_B, IOPORT_DIR_INPUT);//PA4=Camera SCL
	ioport_set_pin_dir(ISI_D4_GPIO, IOPORT_DIR_OUTPUT);//PA5=ISI D5
	ioport_set_pin_dir(ISI_D3_GPIO, IOPORT_DIR_OUTPUT);//PA9=ISI D3
	ioport_set_pin_dir(CPWDN_GPIO, IOPORT_DIR_OUTPUT);//A10=Cam PWRDN
	ioport_set_pin_dir(GPIO_PA21, IOPORT_DIR_OUTPUT);//ISI MCK
	ioport_set_pin_dir(ISI_PCK_PIO, IOPORT_DIR_OUTPUT);//CAM PCLK (w series resistor R13)
	
	ioport_set_pin_mode(SDA0_B, IOPORT_MODE_OPEN_DRAIN);//PA3=Camera SDA
	ioport_set_pin_mode(SCL0_B, IOPORT_MODE_OPEN_DRAIN);//PA4=Camera SCL
	ioport_set_pin_level(ISI_D4_GPIO, IOPORT_PIN_LEVEL_LOW);//PA5=ISI D5
	ioport_set_pin_level(ISI_D3_GPIO, IOPORT_PIN_LEVEL_LOW);//PA9=ISI D3
	ioport_set_pin_level(CPWDN_GPIO, IOPORT_PIN_LEVEL_LOW);//PA10=Cam PWRDN
	ioport_set_pin_level(GPIO_PA21, IOPORT_PIN_LEVEL_LOW);//ISI MCK
	ioport_set_pin_level(ISI_PCK_PIO, IOPORT_PIN_LEVEL_LOW);//CAM PCLK (w series resistor R13)

	ioport_set_pin_dir(GPIO_PB3, IOPORT_DIR_OUTPUT);//ISI D3
	ioport_set_pin_level(GPIO_PB3, IOPORT_PIN_LEVEL_LOW);//ISI D3
	
 	ioport_set_pin_dir(GPIO_PD11, IOPORT_DIR_OUTPUT);//ISI D5
 	ioport_set_pin_dir(GPIO_PD12, IOPORT_DIR_OUTPUT);//ISI D6
	ioport_set_pin_dir(GPIO_PD21, IOPORT_DIR_OUTPUT);//ISI_D1
	ioport_set_pin_dir(GPIO_PD22, IOPORT_DIR_OUTPUT);//ISI_D0
	ioport_set_pin_dir(GPIO_PD24, IOPORT_DIR_OUTPUT);//CAM HSYNC
	ioport_set_pin_dir(GPIO_PD25, IOPORT_DIR_OUTPUT);//CAM VSYNC
 	ioport_set_pin_level(GPIO_PD11, IOPORT_PIN_LEVEL_LOW);//ISI D5
 	ioport_set_pin_level(GPIO_PD12, IOPORT_PIN_LEVEL_LOW);//ISI D6
	ioport_set_pin_level(GPIO_PD21, IOPORT_PIN_LEVEL_LOW);//ISI_D1			//These two ISI pins get pulled low if this line is executed
	ioport_set_pin_level(GPIO_PD22, IOPORT_PIN_LEVEL_LOW);//ISI_D0			//These two ISI pins get pulled low if this line is executed
	ioport_set_pin_level(GPIO_PD24, IOPORT_PIN_LEVEL_LOW);//CAM HSYNC
	ioport_set_pin_level(GPIO_PD25, IOPORT_PIN_LEVEL_LOW);//CAM VSYNC
	/************* SRAM ***************/
	resetPinsSDRAM();
	ioport_set_pin_dir(GPIO_PA8, IOPORT_DIR_OUTPUT);//SRAM PWRC
	ioport_set_pin_dir(GPIO_PA15, IOPORT_DIR_OUTPUT);//SRAM D14
	ioport_set_pin_dir(GPIO_PA16, IOPORT_DIR_OUTPUT);//SRAM D15
	ioport_set_pin_dir(GPIO_PA18, IOPORT_DIR_OUTPUT);//SRAM A12
	ioport_set_pin_dir(GPIO_PA20, IOPORT_DIR_OUTPUT);//SRAM A20
	ioport_set_pin_dir(GPIO_PA23, IOPORT_DIR_OUTPUT);//CAM PWRC (TEMPORARILY FOR SRAM)
	ioport_set_pin_level(GPIO_PA8, IOPORT_PIN_LEVEL_LOW);//SRAM PWRC
	ioport_set_pin_level(GPIO_PA15, IOPORT_PIN_LEVEL_LOW);//SRAM D14
	ioport_set_pin_level(GPIO_PA16, IOPORT_PIN_LEVEL_LOW);//SRAM D15
	ioport_set_pin_level(GPIO_PA18, IOPORT_PIN_LEVEL_LOW);//SRAM A12
	ioport_set_pin_level(GPIO_PA20, IOPORT_PIN_LEVEL_LOW);//SRAM A20
	ioport_set_pin_level(GPIO_PA23, IOPORT_PIN_LEVEL_LOW);//CAM PWRC (TEMPORARILY FOR SRAM)

	ioport_set_pin_dir(GPIO_PC0, IOPORT_DIR_OUTPUT);//SRAM D0
	ioport_set_pin_dir(GPIO_PC1, IOPORT_DIR_OUTPUT);//SRAM D1
	ioport_set_pin_dir(GPIO_PC2, IOPORT_DIR_OUTPUT);//SRAM D2
	ioport_set_pin_dir(GPIO_PC3, IOPORT_DIR_OUTPUT);//SRAM D3
	ioport_set_pin_dir(GPIO_PC4, IOPORT_DIR_OUTPUT);//SRAM D4
	ioport_set_pin_dir(GPIO_PC5, IOPORT_DIR_OUTPUT);//SRAM D5
	ioport_set_pin_dir(GPIO_PC6, IOPORT_DIR_OUTPUT);//SRAM D6
	ioport_set_pin_dir(GPIO_PC7, IOPORT_DIR_OUTPUT);//SRAM D7
	ioport_set_pin_dir(GPIO_PC15, IOPORT_DIR_OUTPUT);//SRAM CS
	ioport_set_pin_dir(GPIO_PC18, IOPORT_DIR_OUTPUT);//SRAM LDM
	ioport_set_pin_dir(GPIO_PC20, IOPORT_DIR_OUTPUT);//SRAM A0
	ioport_set_pin_dir(GPIO_PC21, IOPORT_DIR_OUTPUT);//SRAM A1
	ioport_set_pin_dir(GPIO_PC22, IOPORT_DIR_OUTPUT);//SRAM A2
	ioport_set_pin_dir(GPIO_PC23, IOPORT_DIR_OUTPUT);//SRAM A3
	ioport_set_pin_dir(GPIO_PC24, IOPORT_DIR_OUTPUT);//SRAM A4 AND FLIR CLK
	ioport_set_pin_dir(GPIO_PC25, IOPORT_DIR_OUTPUT);//SRAM A5 AND FLIR CSL
	ioport_set_pin_dir(GPIO_PC26, IOPORT_DIR_OUTPUT);//SRAM A6 AND FLIR MISO
	ioport_set_pin_dir(GPIO_PC27, IOPORT_DIR_OUTPUT);//SRAM A7 AND FLIR MOSI
	ioport_set_pin_dir(GPIO_PC28, IOPORT_DIR_OUTPUT);//SRAM A8
	ioport_set_pin_dir(GPIO_PC29, IOPORT_DIR_OUTPUT);//SRAM A9
	ioport_set_pin_dir(GPIO_PC31, IOPORT_DIR_OUTPUT);//SRAM A11
	ioport_set_pin_level(GPIO_PC0, IOPORT_PIN_LEVEL_LOW);//SRAM D0   USE THIS SECTION FOR REAL BOARD (pc0..pc7)
	ioport_set_pin_level(GPIO_PC1, IOPORT_PIN_LEVEL_LOW);//SRAM D1
	ioport_set_pin_level(GPIO_PC2, IOPORT_PIN_LEVEL_LOW);//SRAM D2
	ioport_set_pin_level(GPIO_PC3, IOPORT_PIN_LEVEL_LOW);//SRAM D3
	ioport_set_pin_level(GPIO_PC4, IOPORT_PIN_LEVEL_LOW);//SRAM D4
	ioport_set_pin_level(GPIO_PC5, IOPORT_PIN_LEVEL_LOW);//SRAM D5
	ioport_set_pin_level(GPIO_PC6, IOPORT_PIN_LEVEL_LOW);//SRAM D6
	ioport_set_pin_level(GPIO_PC7, IOPORT_PIN_LEVEL_LOW);//SRAM D7
	ioport_set_pin_level(GPIO_PC15, IOPORT_PIN_LEVEL_LOW);//SRAM CS
	ioport_set_pin_level(GPIO_PC18, IOPORT_PIN_LEVEL_LOW);//SRAM DML
	ioport_set_pin_level(GPIO_PC20, IOPORT_PIN_LEVEL_LOW);//SRAM A0	USE THIS SECTION INSTEAD (PC20..PC29)
	ioport_set_pin_level(GPIO_PC21, IOPORT_PIN_LEVEL_LOW);//SRAM A1
	ioport_set_pin_level(GPIO_PC22, IOPORT_PIN_LEVEL_LOW);//SRAM A2
	ioport_set_pin_level(GPIO_PC23, IOPORT_PIN_LEVEL_LOW);//SRAM A3
	ioport_set_pin_level(GPIO_PC24, IOPORT_PIN_LEVEL_LOW);//SRAM A4 AND FLIR CLK
	ioport_set_pin_level(GPIO_PC25, IOPORT_PIN_LEVEL_LOW);//SRAM A5 AND FLIR CSL
	ioport_set_pin_level(GPIO_PC26, IOPORT_PIN_LEVEL_LOW);//SRAM A6 AND FLIR MISO
	ioport_set_pin_level(GPIO_PC27, IOPORT_PIN_LEVEL_LOW);//SRAM A7 AND FLIR MOSI
	ioport_set_pin_level(GPIO_PC28, IOPORT_PIN_LEVEL_LOW);//SRAM A8
	ioport_set_pin_level(GPIO_PC29, IOPORT_PIN_LEVEL_LOW);//SRAM A9
	ioport_set_pin_level(GPIO_PC31, IOPORT_PIN_LEVEL_LOW);//SRAM A11

	ioport_set_pin_dir(GPIO_PD13, IOPORT_DIR_OUTPUT);//SRAM A10
	ioport_set_pin_dir(GPIO_PD14, IOPORT_DIR_OUTPUT);//SRAM CKE
	ioport_set_pin_dir(GPIO_PD15, IOPORT_DIR_OUTPUT);//SRAM UDM
	ioport_set_pin_dir(GPIO_PD16, IOPORT_DIR_OUTPUT);//SRAM RAS		//SDRAM DOESN'T WORK WHEN SLEEPPINS TURNS THIS OFF
	ioport_set_pin_dir(GPIO_PD17, IOPORT_DIR_OUTPUT);//SRAM CAS		//SDRAM DOESN'T WORK WHEN SLEEPPINS TURNS THIS OFF
	ioport_set_pin_dir(GPIO_PD23, IOPORT_DIR_OUTPUT);//SRAM CLK		//SDRAM DOESN'T WORK WHEN SLEEPPINS TURNS THIS OFF
	ioport_set_pin_dir(GPIO_PD29, IOPORT_DIR_OUTPUT);//SRAM WE
	ioport_set_pin_level(GPIO_PD13, IOPORT_PIN_LEVEL_LOW);//SRAM A10
	ioport_set_pin_level(GPIO_PD14, IOPORT_PIN_LEVEL_LOW);//SRAM CKE
	ioport_set_pin_level(GPIO_PD15, IOPORT_PIN_LEVEL_LOW);//SRAM UDM
	ioport_set_pin_level(GPIO_PD16, IOPORT_PIN_LEVEL_LOW);//SRAM RAS
	ioport_set_pin_level(GPIO_PD17, IOPORT_PIN_LEVEL_LOW);//SRAM CAS
	ioport_set_pin_level(GPIO_PD23, IOPORT_PIN_LEVEL_LOW);//SRAM CLK
	ioport_set_pin_level(GPIO_PD29, IOPORT_PIN_LEVEL_LOW);//SRAM WE

	ioport_reset_pin_mode(GPIO_PE0);//Turn off the SDRAM function
	ioport_enable_pin(GPIO_PE0);//Enable normal SDRAM function
	ioport_set_pin_dir(GPIO_PE0, IOPORT_DIR_OUTPUT);//SRAM D8		USE THIS SECTION FOR ALL OF PORT E
	ioport_set_pin_dir(GPIO_PE1, IOPORT_DIR_OUTPUT);//SRAM D9
	ioport_set_pin_dir(GPIO_PE2, IOPORT_DIR_OUTPUT);//SRAM D10
	ioport_set_pin_dir(GPIO_PE3, IOPORT_DIR_OUTPUT);//SRAM D11
	ioport_set_pin_dir(GPIO_PE4, IOPORT_DIR_OUTPUT);//SRAM D12
	ioport_set_pin_dir(GPIO_PE5, IOPORT_DIR_OUTPUT);//SRAM D13
	ioport_set_pin_level(GPIO_PE0, IOPORT_PIN_LEVEL_LOW);//SRAM D8
	ioport_set_pin_level(GPIO_PE1, IOPORT_PIN_LEVEL_LOW);//SRAM D9
	ioport_set_pin_level(GPIO_PE2, IOPORT_PIN_LEVEL_LOW);//SRAM D10
	ioport_set_pin_level(GPIO_PE3, IOPORT_PIN_LEVEL_LOW);//SRAM D11
	ioport_set_pin_level(GPIO_PE4, IOPORT_PIN_LEVEL_LOW);//SRAM D12
	ioport_set_pin_level(GPIO_PE5, IOPORT_PIN_LEVEL_LOW);//SRAM D13
}
/***********************************************************************
 *    DISPLAY SD ERROR
 ***********************************************************************/
void printFileError(char res){
	switch (res){
		case 0:
		printf ("FR_OK");
		break;
		case 1:
		printf ("FR_DISK_ERR");
		break;
		case 2:
		printf ("FR_INT_ERR");
		break;
		case 3:
		printf ("FR_NOT_READY");
		break;
		case 4:
		printf ("FR_NO_FILE");
		break;
		case 5:
		printf ("FR_NO_PATH");
		break;
		case 6:
		printf ("FR_INVALID_NAME");
		break;
		case 7:
		printf ("FR_DENIED");
		break;
		case 8:				/* (8) Acces denied due to prohibited access */
		printf ("FR_EXIST");
		break;
		case 9:
		printf ("FR_INVALID_OBJECT");
		break;
		case 10:
		printf ("FR_WRITE_PROTECTED");
		break;
		case 11:
		printf ("FR_INVALID_DRIVE");
		break;
		case 12:
		printf ("FR_NOT_ENABLED");
		break;
		case 13:
		printf ("FR_NO_FILESYSTEM");
		break;
		case 14:
		printf ("FR_MKFS_ABORTED");
		break;
		case 15:
		printf ("FR_TIMEOUT");
		break;
		case 16:
		printf ("FR_LOCKED");
		break;
		case 17:
		printf ("FR_NOT_ENOUGH_CORE");
		break;
		case 18:
		printf ("FR_TOO_MANY_OPEN_FILES");
		break;
		case 19:
		printf ("FR_INVALID_PARAMETER");
		break;
		DEFAULT: printf ("FR_INVALID_PARAMETER");
	};
	printf("\r\n");
}
/***********************************************************************
 *    DISPLAY SD CARD INFO
 ***********************************************************************/
static void main_display_info_card(uint8_t slot)
{
	printf("Card information:\r\n");

	printf("    ");
	switch (sd_mmc_get_type(slot)) {
		case CARD_TYPE_SD | CARD_TYPE_HC:
		printf("SDHC");
		break;
		case CARD_TYPE_SD:
		printf("SD");
		break;
		case CARD_TYPE_MMC | CARD_TYPE_HC:
		printf("MMC High Density");
		break;
		case CARD_TYPE_MMC:
		printf("MMC");
		break;
		case CARD_TYPE_SDIO:
		printf("SDIO\r\n");
		return;
		case CARD_TYPE_SD_COMBO:
		printf("SD COMBO");
		break;
		case CARD_TYPE_UNKNOWN:
		default:
		printf("Unknown\r\n");
		return;
	}
	printf("\r\n    %d MB\r\n", (uint16_t)(sd_mmc_get_capacity(slot) / 1024));
}
/***********************************************************************
 *    RESET SDRAM PINS
 ***********************************************************************/
void resetPinsSDRAM(void){
	ioport_reset_pin_mode(GPIO_PA15);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PA15);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PA16);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PA16);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PA18);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PA18);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PA20);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PA20);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC0);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC0);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC1);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC1);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC2);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC2);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC3);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC3);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC4);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC4);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC5);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC5);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC6);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC6);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC7);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC7);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC15);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC15);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC18);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC18);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC20);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC20);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC21);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC21);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC22);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC22);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC23);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC23);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC24);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC24);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC25);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC25);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC26);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC26);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC27);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC27);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC28);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC28);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC29);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC29);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PC31);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PC31);//Enable normal PIO function

	ioport_reset_pin_mode(GPIO_PD13);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PD13);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PD14);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PD14);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PD15);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PD15);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PD16);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PD16);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PD17);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PD17);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PD23);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PD23);//Enable normal PIO function
	ioport_reset_pin_mode(GPIO_PD29);//Turn off the HCSMI function
	ioport_enable_pin(GPIO_PD29);//Enable normal PIO function

	ioport_reset_pin_mode(GPIO_PE0);//Turn off the SDRAM function
	ioport_enable_pin(GPIO_PE0);//Enable normal SDRAM function
	ioport_reset_pin_mode(GPIO_PE1);//Turn off the SDRAM function
	ioport_enable_pin(GPIO_PE1);//Enable normal SDRAM function
	ioport_reset_pin_mode(GPIO_PE2);//Turn off the SDRAM function
	ioport_enable_pin(GPIO_PE2);//Enable normal SDRAM function
	ioport_reset_pin_mode(GPIO_PE3);//Turn off the SDRAM function
	ioport_enable_pin(GPIO_PE3);//Enable normal SDRAM function
	ioport_reset_pin_mode(GPIO_PE4);//Turn off the SDRAM function
	ioport_enable_pin(GPIO_PE4);//Enable normal SDRAM function
	ioport_reset_pin_mode(GPIO_PE5);//Turn off the SDRAM function
	ioport_enable_pin(GPIO_PE5);//Enable normal SDRAM function
}
/***********************************************************************
 *    CLOSE SD CARD
 ***********************************************************************/
void closeSDcard(void){
	ioport_reset_pin_mode(PIN_HSMCI_MCCDA_GPIO);//Turn off the HCSMI function
	ioport_reset_pin_mode(PIN_HSMCI_MCCK_GPIO);
	ioport_reset_pin_mode(PIN_HSMCI_MCDA0_GPIO);
	ioport_reset_pin_mode(PIN_HSMCI_MCDA1_GPIO);
	ioport_reset_pin_mode(PIN_HSMCI_MCDA2_GPIO);
	ioport_reset_pin_mode(PIN_HSMCI_MCDA3_GPIO);
	ioport_enable_pin(PIN_HSMCI_MCCDA_GPIO);//Enable normal PIO function
	ioport_enable_pin(PIN_HSMCI_MCCK_GPIO);
	ioport_enable_pin(PIN_HSMCI_MCDA0_GPIO);
	ioport_enable_pin(PIN_HSMCI_MCDA1_GPIO);
	ioport_enable_pin(PIN_HSMCI_MCDA2_GPIO);
	ioport_enable_pin(PIN_HSMCI_MCDA3_GPIO);
	ioport_set_pin_dir(PIN_HSMCI_MCCDA_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PIN_HSMCI_MCCK_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PIN_HSMCI_MCDA0_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PIN_HSMCI_MCDA1_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PIN_HSMCI_MCDA2_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PIN_HSMCI_MCDA3_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_HSMCI_MCCDA_GPIO,0);
	ioport_set_pin_level(PIN_HSMCI_MCCK_GPIO,0);
	ioport_set_pin_level(PIN_HSMCI_MCDA0_GPIO,0);
	ioport_set_pin_level(PIN_HSMCI_MCDA1_GPIO,0);
	ioport_set_pin_level(PIN_HSMCI_MCDA2_GPIO,0);
	ioport_set_pin_level(PIN_HSMCI_MCDA3_GPIO,0);
	delay_ms(10);
	ioport_set_pin_level(SDCARDPWRC_GPIO, 0);
	ioport_set_pin_level(CPWRC_GPIO,0);
	delay_ms(10);
}
/***********************************************************************
 *    SSD Card Hardware Reset
 ***********************************************************************/
void resetSD (int offDelay){
	ioport_set_pin_dir(CPWDN_GPIO, IOPORT_DIR_OUTPUT);//A10=Cam PWRDN
	ioport_set_pin_level(CPWDN_GPIO,1);//high is camera powered down.
	ioport_set_pin_level(CPWRC_GPIO,0);
	ioport_set_pin_dir(SDCARDPWRC_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(SDCARDPWRC_GPIO, 0);
	ioport_reset_pin_mode(PIN_HSMCI_MCCDA_GPIO);//Turn off the HCSMI function
	ioport_reset_pin_mode(PIN_HSMCI_MCCK_GPIO);
	ioport_reset_pin_mode(PIN_HSMCI_MCDA0_GPIO);
	ioport_reset_pin_mode(PIN_HSMCI_MCDA1_GPIO);
	ioport_reset_pin_mode(PIN_HSMCI_MCDA2_GPIO);
	ioport_reset_pin_mode(PIN_HSMCI_MCDA3_GPIO);
	ioport_enable_pin(PIN_HSMCI_MCCDA_GPIO);//Enable normal PIO function
	ioport_enable_pin(PIN_HSMCI_MCCK_GPIO);
	ioport_enable_pin(PIN_HSMCI_MCDA0_GPIO);
	ioport_enable_pin(PIN_HSMCI_MCDA1_GPIO);
	ioport_enable_pin(PIN_HSMCI_MCDA2_GPIO);
	ioport_enable_pin(PIN_HSMCI_MCDA3_GPIO);
	ioport_set_pin_dir(PIN_HSMCI_MCCDA_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PIN_HSMCI_MCCK_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PIN_HSMCI_MCDA0_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PIN_HSMCI_MCDA1_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PIN_HSMCI_MCDA2_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PIN_HSMCI_MCDA3_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_HSMCI_MCCDA_GPIO,0);
	ioport_set_pin_level(PIN_HSMCI_MCCK_GPIO,0);
	ioport_set_pin_level(PIN_HSMCI_MCDA0_GPIO,0);
	ioport_set_pin_level(PIN_HSMCI_MCDA1_GPIO,0);
	ioport_set_pin_level(PIN_HSMCI_MCDA2_GPIO,0);
	ioport_set_pin_level(PIN_HSMCI_MCDA3_GPIO,0);
	ioport_set_pin_level(CPWRC_GPIO,0);
	ioport_set_pin_dir(SRAMPWRC_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(SRAMPWRC_GPIO, SRAMPWRC_POWER_OFF);
	delay_ms(offDelay);//needs 300 to drift down to zero
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCCDA_GPIO, PIN_HSMCI_MCCDA_FLAGS);
 	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCCK_GPIO, PIN_HSMCI_MCCK_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCDA0_GPIO, PIN_HSMCI_MCDA0_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCDA1_GPIO, PIN_HSMCI_MCDA1_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCDA2_GPIO, PIN_HSMCI_MCDA2_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCDA3_GPIO, PIN_HSMCI_MCDA3_FLAGS);
// 	ioport_set_pin_dir(SRAMPWRC_GPIO, IOPORT_DIR_OUTPUT);
// 	ioport_set_pin_level(SRAMPWRC_GPIO, SRAMPWRC_POWER_ON);
	ioport_set_pin_level(CPWRC_GPIO,1);
	ioport_set_pin_level(SDCARDPWRC_GPIO,1);
}
/***********************************************************************
 *    RESET ISI PINS
 ***********************************************************************/
void resetPinsISI(void){
	
	ioport_reset_pin_mode(ISI_D0_GPIO);
	ioport_reset_pin_mode(ISI_D1_GPIO);
	ioport_reset_pin_mode(ISI_D2_GPIO);
	ioport_reset_pin_mode(ISI_D3_GPIO);
	ioport_reset_pin_mode(ISI_D4_GPIO);
	ioport_reset_pin_mode(ISI_D5_GPIO);
	ioport_reset_pin_mode(ISI_D6_GPIO);
	ioport_reset_pin_mode(ISI_D7_GPIO);
	ioport_reset_pin_mode(ISI_HSYNC_GPIO);
	ioport_reset_pin_mode(ISI_VSYNC_GPIO);
	ioport_reset_pin_mode(ISI_MCLK_GPIO);
	ioport_reset_pin_mode(ISI_PCK_PIO);
	
	ioport_enable_pin(ISI_D0_GPIO);
	ioport_enable_pin(ISI_D1_GPIO);
	ioport_enable_pin(ISI_D2_GPIO);
	ioport_enable_pin(ISI_D3_GPIO);
	ioport_enable_pin(ISI_D4_GPIO);
	ioport_enable_pin(ISI_D5_GPIO);
	ioport_enable_pin(ISI_D6_GPIO);
	ioport_enable_pin(ISI_D7_GPIO);
	ioport_enable_pin(ISI_HSYNC_GPIO);
	ioport_enable_pin(ISI_VSYNC_GPIO);
	ioport_enable_pin(ISI_MCLK_GPIO);
	ioport_enable_pin(ISI_PCK_PIO);
}
/***********************************************************************
 *    FLOAT ISI PINS
 ***********************************************************************/
//NOT YET SURE WHAT CAM PINS MIGHT SOMETIMES MESS UP SD CARD !!!!!!!!!!!!!!!
void floatCamForSDCard(void){
	resetPinsISI();
	ioport_set_pin_dir(ISI_D0_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ISI_D1_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ISI_D2_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ISI_D3_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ISI_D4_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ISI_D5_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ISI_D6_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ISI_D7_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ISI_HSYNC_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ISI_VSYNC_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ISI_MCLK_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ISI_PCK_PIO, IOPORT_DIR_INPUT);

	ioport_set_pin_mode(ISI_D0_GPIO, IOPORT_MODE_PULLUP);
	ioport_set_pin_mode(ISI_D1_GPIO, IOPORT_MODE_PULLUP);
	ioport_set_pin_mode(ISI_D2_GPIO, IOPORT_MODE_PULLUP);
	ioport_set_pin_mode(ISI_D3_GPIO, IOPORT_MODE_PULLUP);
	ioport_set_pin_mode(ISI_D4_GPIO, IOPORT_MODE_PULLUP);
	ioport_set_pin_mode(ISI_D5_GPIO, IOPORT_MODE_PULLUP);
	ioport_set_pin_mode(ISI_D6_GPIO, IOPORT_MODE_PULLUP);
	ioport_set_pin_mode(ISI_D7_GPIO, IOPORT_MODE_PULLUP);
	ioport_set_pin_mode(ISI_HSYNC_GPIO, IOPORT_MODE_PULLUP);
	ioport_set_pin_mode(ISI_VSYNC_GPIO, IOPORT_MODE_PULLUP);
	ioport_set_pin_mode(ISI_MCLK_GPIO, IOPORT_MODE_PULLUP);
	ioport_set_pin_mode(ISI_PCK_PIO, IOPORT_MODE_PULLUP);
}
/***********************************************************************
 *    WATCHDOG INIT
 ***********************************************************************/
void wdt_init(Wdt *p_wdt, uint32_t ul_mode, uint16_t us_counter,
uint16_t us_delta)
{
	p_wdt->WDT_MR = ul_mode |
	WDT_MR_WDV(us_counter) | WDT_MR_WDD(us_delta);
}
uint32_t wdt_get_timeout_value(uint32_t ul_us, uint32_t ul_sclk)
{
	uint32_t max, min;

	min = WDT_SLCK_DIV * 1000000 / ul_sclk;
	max = min * WDT_MAX_VALUE;

	if ((ul_us < min) || (ul_us > max)) {
		return WDT_INVALID_ARGUMENT;
	}

	return WDT_MR_WDV(ul_us / min);
}
