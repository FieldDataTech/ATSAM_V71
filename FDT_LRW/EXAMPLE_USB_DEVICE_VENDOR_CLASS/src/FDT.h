/*
 * PSG.h
 *
 * Created: 8/15/2016 5:14:31 PM
 *  Author: doug
 */


#ifndef PSG_H_
#define PSG_H_

void store_bmp(void);

#ifndef CONF_EXAMPLE_H_INCLUDED
#define CONF_EXAMPLE_H_INCLUDED


/** USART Interface  : Console UART */
#define CONF_TEST_USART      CONSOLE_UART
/** Baudrate setting : 115200 */
#define CONF_TEST_BAUDRATE   9600
/** Char setting     : 8-bit character length (don't care for UART) */
#define CONF_TEST_CHARLENGTH 0
/** Parity setting   : No parity check */
#define CONF_TEST_PARITY     UART_MR_PAR_NO
/** Stopbit setting  : No extra stopbit, i.e., use 1 (don't care for UART) */
#define CONF_TEST_STOPBITS   false
#define sizeOfConfigFile 500
#define WDT_PERIOD                        8000


/* LCD board defines. */
//#define ILI9325_LCD_CS                 (2UL) // Chip select number
//#define IMAGE_WIDTH                    (320UL)
//#define IMAGE_HEIGHT                   (240UL)

/* Image sensor board defines. */
/** Frame Buffer Descriptors */
typedef struct {
	/** Address of the Current FrameBuffer */
	uint32_t Current;
	/** Address of the Control */
	uint32_t Control;
	/** Address of the Next FrameBuffer */
	uint32_t Next;
} ISI_FrameBufferDescriptors;


/** Frame Buffer Descriptors */
#define ISI_MAX_PREV_BUFFER     1

/* ISI DMA descriptor for preview path */
COMPILER_ALIGNED(32) ISI_FrameBufferDescriptors preBufDescList[ISI_MAX_PREV_BUFFER];

static void _isi_AllocateFBD(void);

// Image sensor Power pin.
//#define OV_POWER_PIO                   OV_SW_OVT_PIO
//#define OV_POWER_MASK                  OV_SW_OVT_MASK
// Image sensor VSYNC pin.
//#define OV7740_VSYNC_PIO	           OV_VSYNC_PIO
//#define OV7740_VSYNC_ID		           OV_VSYNC_ID
//#define OV7740_VSYNC_MASK              OV_VSYNC_MASK
//#define OV7740_VSYNC_TYPE              OV_VSYNC_TYPE
// Image sensor data pin.
//#define OV7740_DATA_BUS_PIO            OV_DATA_BUS_PIO
//#define OV7740_DATA_BUS_ID             OV_DATA_BUS_ID


#define SPI_BUFF_LEN (256)



#define ARCHRECORDLEN 32
#define RFWINDOWMAX (100)
#define XBSENDTIMES (10)
/* SDRAM IS42S16100E configuration */
const sdramc_memory_dev_t SDRAM_INSIGNIS_16M = {
	22, /* Block1 is at the bit 22, 2+8+11+1. */
	0x30,  /*
	     * This configures the SDRAM with the following parameters in the
	     *mode register:
	     * - bits 0 to 2: burst length: 1 (000b);
	     * - bit 3: burst type: sequential (0b);
	     * - bits 4 to 6: CAS latency;
	     * - bits 7 to 8: operating mode: standard operation (00b);
	     * - bit 9: write burst mode: programmed burst length (0b);
	     * - all other bits: reserved: 0b.
	     */
	{
		SDRAMC_CR_NC_COL9      | /* 9 column bits.    same for Insignis */
		SDRAMC_CR_NR_ROW13     | /* 13 row bits    (2K). */
		SDRAMC_CR_NB_BANK4     | /* SDRAM 4 bank for Insignis. */
		SDRAMC_CR_CAS_LATENCY3 | /* CAS Latency 3. Insignis says 2 or 3*/
		SDRAMC_CR_DBW          | /* Data bus width 16 bits, good for Insignis. */
		SDRAMC_CR_TWR(5)       | /* Write Recovery Delay. */
		SDRAMC_CR_TRC_TRFC(13) | /* Row Cycle Delay and Row Refresh Cycle. */
		SDRAMC_CR_TRP(5)       | /* Row Precharge Delay. */
		SDRAMC_CR_TRCD(5)      | /* Row to Column Delay. */
		SDRAMC_CR_TRAS(9)      | /* Active to Precharge Delay. */
		SDRAMC_CR_TXSR(15)       /* Exit from Self Refresh to Active Delay. */
	},
};


/* SDRAM IS42S16100E configuration */
const sdramc_memory_dev_t SDRAM_ISSI_IS42S16100E = {
	22, /* Block1 is at the bit 22, 2+8+11+1. */
	0x30,  /*
	     * This configures the SDRAM with the following parameters in the
	     *mode register:
	     * - bits 0 to 2: burst length: 1 (000b);
	     * - bit 3: burst type: sequential (0b);
	     * - bits 4 to 6: CAS latency;
	     * - bits 7 to 8: operating mode: standard operation (00b);
	     * - bit 9: write burst mode: programmed burst length (0b);
	     * - all other bits: reserved: 0b.
	     */
	{
		SDRAMC_CR_NC_COL8      | /* 8 column bits. */
		SDRAMC_CR_NR_ROW11     | /* 11 row bits    (2K). */
		SDRAMC_CR_NB_BANK2     | /* SDRAM 2 bank. */
		SDRAMC_CR_CAS_LATENCY3 | /* CAS Latency 3. */
		SDRAMC_CR_DBW          | /* Data bus width 16 bits. */
		SDRAMC_CR_TWR(5)       | /* Write Recovery Delay. */
		SDRAMC_CR_TRC_TRFC(13) | /* Row Cycle Delay and Row Refresh Cycle. */
		SDRAMC_CR_TRP(5)       | /* Row Precharge Delay. */
		SDRAMC_CR_TRCD(5)      | /* Row to Column Delay. */
		SDRAMC_CR_TRAS(9)      | /* Active to Precharge Delay. */
		SDRAMC_CR_TXSR(15)       /* Exit from Self Refresh to Active Delay. */
	},
};
char mState;

//#define MAX_NUM_SDFILES 99999
uint32_t 	gnextFileNumToStore;
char	bmpFileName[12];
char ertcSec,ertcMin,ertcHr,ertcDay,ertcMo,ertcYr;

//USB Stuff
#define IMAGE_PIX_WIDTH 1600
#define IMAGE_PIX_HEIGHT 1200
#define IMAGE_BYTE_WIDTH_565 (IMAGE_PIX_WIDTH * 2)
#define IMAGE_BYTE_WIDTH_888 (IMAGE_PIX_WIDTH * 3)
#define IMAGE_BYTE_SIZE_565 (IMAGE_BYTE_WIDTH_565 * IMAGE_PIX_HEIGHT)  //3840000 //OV2655 2 bytes per pixel
#define IMAGE_BYTE_SIZE_888	(IMAGE_BYTE_WIDTH_888 * IMAGE_PIX_HEIGHT)
#define SRAM_BASE            BOARD_SDRAM_ADDR	//(0x70000000UL) // SRAM adress
#define IMAGE_BUFF_A		SRAM_BASE + IMAGE_BYTE_SIZE_565
#define IMAGE_BUFF_B	(IMAGE_BUFF_A + IMAGE_BYTE_SIZE_888)
#define SRAM_CS              (0UL)

char g_usbFlagA = 0;
char g_usbFlagB = 0;
char g_usbFlagC = 0;
char g_usbFlagD = 0;

static volatile bool main_b_vendor_enable = false;
void sendPic(char, char);
void receiveUSBcamConfig(void);
//END USB STUFF

unsigned int sendLoRaWAN(char);
void fakeLoRaWAN (char*, char);

void RGB565toRGB888 (void);

void floatCamForSDCard(void);
void configPinsSDRAM(void);
void resetPinsSDRAM(void);
void configPinsSDCard(void);
void resetPinsSDCard(void);
void wdt_init(Wdt *, uint32_t, uint16_t,uint16_t);
uint32_t wdt_get_timeout_value(uint32_t, uint32_t);
#define WDT_INVALID_ARGUMENT 0xFFFF
#define WDT_KEY_PASSWORD  0xA5000000
#define WDT_SLCK_DIV      128
#define WDT_MAX_VALUE     4095

void resetPinsISI(void);

void SDRAMtest(void);

void storeImage(void);
void printFileError(char);
void resetSD (int);
static void main_display_info_card(uint8_t);

void genFileName (uint32_t, char*);
char digToChar(uint8_t);
char initSI1153(void);
void getSetDeviceID(void);
void sleepForSecs(unsigned int);
void togPortApin (unsigned int);
void LCDclear(void);
void LCDbottomLine(void);

void si115xParamSet(unsigned char, unsigned char);
char initRTCext(void);
void displayDateTime(void);
void setExternRTC(char,char,char,char,char,char);
void retrieveExternRTC(void);

void readConfigSD (void);
void TC_Handler(void);
unsigned int tcCtr;
void storeTextFile(void);
void writeFileSD(void);

/* Test page start address. */
char DEVICEID[16];
#define TEST_PAGE_ADDRESS (IFLASH_ADDR + IFLASH_SIZE - IFLASH_PAGE_SIZE * 4)
#define ARCHIVEADDR (0x480000)
#define PASTARCHIVEADDR (0x600000)	
		//1,572,864 bytes. 32-byte records. 8192 per day, 32 days per month, 6 months = 0x180000 
		//1,572,864 bytes. 16-byte records. 4096 per day, 32 days per month, 12 months = 0x180000
#define FLASHSECTORSIZE (0x020000)
#define XBEEBLOCKSIZE (0x400)
#define ARCH_MAR_SEP (0x480000)
#define ARCH_APR_OCT (0x)
#define ARCH_MAY_NOV (0x)
#define ARCH_JUN_DEC (0x)
#define ARCH_JLY_JAN (0x)
#define ARCH_AUG_FEB (0x)


/* XBee DEFINES from example */
#define TIMEOUT             (1000)
/** Character to synchronize with the other end. */
//#define SYNC_CHAR            0x11
/** Character to acknowledge receipt of the sync char. */
//#define ACK_CHAR             0x13
/** All interrupt mask. */
#define ALL_INTERRUPT_MASK   0xffffffff
/** System tick frequency in Hz. */
#define SYS_TICK_FREQ        1000

#define doGPS_MASK 0x01;
#define doXBEE_MASK 0x02;
#define doALTITUDE_MASK 0x04;

void oneDayToSD(unsigned int, unsigned int);

static void configure_debugUART (void);
void incArchive (void);
unsigned int archPtr=ARCHIVEADDR;
unsigned int archErasePtr=ARCHIVEADDR;
void initFlashStuff (void);
void eraseSector (unsigned int);
void writeRecToFlash(unsigned int ,unsigned int*, unsigned int);
void eraseEntireArchive(void);
void clearCalendar(char);
void initDefaultCalendar(char);
void overwriteCalendar(char/*typeCalendar*/, unsigned int);
void eraseCalendar(unsigned int);

char logGPSflag(char,char);
char logGPSRecord(char,char,char,char);
unsigned short calc10bitDelta (unsigned short, unsigned short);
unsigned short calc6bitDelta (unsigned short, unsigned short);
char logMotionRecord32byte(char*,char,unsigned int*);
char altOverFlowFlag=0;
int asciiDecToHex(char, char);

#define REQ_ALL_MOTION_SECONDS (1)
#define REQ_ONE_MINUTE_TOTALS (0)

/** XBee USART */
typedef enum st_usart_state {
	INITIALIZED,
	TRANSMITTING,
	RECEIVING,
	RECEIVED,
	TRANSMITTED
} usart_state_t;
volatile usart_state_t g_state = INITIALIZED;
volatile uint32_t g_ul_tick_count;
// char g_transmitBuff[BUFFER_SIZE];//2K
// char g_receiveBuff[BUFFER_SIZE];
// char *p_rcvData = &g_receiveBuff[0];
/** count number for received data. */
uint32_t g_ulcount = 0;
char gc_test=0;
int g_gotMinTotal;
int g_gotSecsA,g_gotSecsB,g_gotSecsC,g_gotSecsD,g_gotSecsE,g_gotSecsF,g_gotSecsG,g_gotSecsH;
char g_RTCflag=0;
uint32_t yearNow,monthNow,dayNow,weekNow,hourNow,secNow,minsNow;
int intYearNow;
unsigned int dayMin=0;
// unsigned int getAltimeterTemperature(void);
unsigned int getAltimeterTemperatureTE(char);
short getHumidity(char);
short getALSfromVEML(char);
void mult64bit (unsigned int,unsigned int,unsigned int*, unsigned int*);
void sendPacketToTiny32(char);
char minPacketForTiny[9];

#define SENSPWR_ON		ioport_set_pin_level(SENS_PWRC, 0)
#define SENSPWR_OFF    	ioport_set_pin_level(SENS_PWRC, 1)
#define SENSPWR_OUTPUT 	ioport_set_pin_dir(SENS_PWRC, IOPORT_DIR_OUTPUT)
#define SENS_SCL_HI 		ioport_set_pin_level(SENS_CLK, 1);
#define SENS_SDA_HI 		ioport_set_pin_level(SENS_SDA, 1);
#define SENS_SCL_LO 		ioport_set_pin_level(SENS_CLK, 0);
#define SENS_SDA_LO 		ioport_set_pin_level(SENS_SDA, 0);
#define SENS_SCL_FLOAT 		ioport_set_pin_dir(SENS_CLK, IOPORT_DIR_INPUT);
#define SENS_SDA_FLOAT 		ioport_set_pin_dir(SENS_SDA, IOPORT_DIR_INPUT);
#define SENS_SCL_DRIVE 		ioport_set_pin_dir(SENS_CLK, IOPORT_DIR_OUTPUT);
#define SENS_SDA_DRIVE 		ioport_set_pin_dir(SENS_SDA, IOPORT_DIR_OUTPUT);

#define ioport_set_pin_peripheral_mode(pin, mode) \
do {\
	ioport_set_pin_mode(pin, mode);\
	ioport_disable_pin(pin);\
} while (0)

char getByteFromTiny(void);
void sendCmdToTiny(char);

void redBlink (int);
void userBlink (int);
void togTP (char);
void ioPinTest(void);

unsigned short calcCRC(char*, int);
static unsigned int hex1ToAscii(int);
void hex2ToAscii(int, char*);
static int asciiHexToHex(char,char);
char sndArchiveBlock(unsigned int);
void sndArchiveBlockRetries(unsigned int);
void initDnldTable(void);

void fakeNestMotions(void);
void fakeNestMotionsRealistic(void);
char g_motionPosition;

void sendMotion(short,char);
void sendMotionDummy(short,char,char);

void sleepPins(void);
void getGPSdateTimeFrmTiny(void);

#define PIC_HEADER_LEN 54
char takeImage(char, char, char);
char takeImageTest(char, char, char);

void closeSDcard(void);

static char bcdToHex(char);
static int hexToBCD (int);

//RGB888
const char picHeader_2655[PIC_HEADER_LEN]  =
{	0x42,0x4D,
	0x36,0xE4,0x57,0x00,//file size
	0x00,0x00,0x00,0x00,//reserved
	0x36,0x00,0x00,0x00,//start of image
	0x28,0x00,0x00,0x00,//header structure, always 0x28
	0x40,0x06,0x00,0x00,//width in pixels
	0xB0,0x04,0x00,0x00,//height in pixels
	0x01,0x00,//num of planes
	0x18,0x00,//bits per pixel, 1,4,8,24
	0x00,0x00,0x00,0x00,//compression 00
	0x00,0xE4,0x57,0x00,//size of image data in bytes
	0x00,0x00,0x00,0x00,//H.resolution
	0x00,0x00,0x00,0x00,//V.resolution
	0x00,0x00,0x00,0x00,//num colors or 00
	0x00,0x00,0x00,0x00	};//num important colors or00
 const char oneDayHeader[]  = "Version,Date(GMT),Time(GMT),Time(Loc),MotionPerMinute,MotionSeq,Temperature,Humidity,AirPressure,Light,Dur1,Dur2,Dur3,Dur4,Dur5,Dur6,Dur7,Dur8,Dur9,Dur10,Dur11,Dur12,NumTrips,Longitude,Latitude,LastGPSdate,LastGPStime,Battery,GPSNumSats,GPSTTFF,Magnetometer\r\n\0";
unsigned int pirConfig;

#endif /* CONF_EXAMPLE_H_INCLUDED */


#endif /* PSG_H_ */