/**
 * \file
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "board.h"
#include "ov2655.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 */
const ov_reg MY_OV2710_BASIC_CONFIG[] = {
	{0xFFFF, 0xFF}};//STOPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
const ov_reg MY_OV2710_BASIC_RESET[] = {
	{0x3008, 0x82},//2710 D00 82=Reset
 	{0x3017, 0x00},//2710 D02 SYS CONTROL00
 	{0x3018, 0x00},//2710 D02 SYS CONTROL00
	{0xFFFF, 0xFF}//STOPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
};
const ov_reg MY_OV2655_BASIC_34[] = {
	{0x3400, 0x40},//D00 40=RGB565  00=422
	// 	{0x01, 0x40},//
	// 	{0x02, 0x40},//
	// 	{0x02, 0x40},//
	{0xFFFF, 0xFF}};//STOPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
const ov_reg MY_OV2655_BASIC_RESET[] = {
	{0x3012, 0x80},//D00 80=Reset
	// 	{0x01, 0x40},//
	// 	{0x02, 0x40},//
	// 	{0x02, 0x40},//
	{0xFFFF, 0xFF}};//STOPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
const ov_reg MY_OV2655_BASIC_TESTPATTERN33[] = {
	{0x3308, 0x01},//=01 for test pattern
	{0xFFFF, 0xFF}};//STOPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
const ov_reg MY_OV2655_BASIC_FALLBACK[] = {
	
	{0xFFFF, 0xFF}};//STOPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
const ov_reg MY_OV2655_BASIC_CONFIG[] = {
//BEGIN BASELINE EXAMPLE
//IO, CLOCK & ANALOG	
	{0x308C, 0x80},//BaseExample 80
	{0x308D, 0x10},//BaseExample 0E   (10=NO MIPI)
	{0x360B, 0x00},//BaseExample 00.  TEST BITS
	{0x30B0, 0xFC},//BaseExample FF (FC=my output pins)
	{0x30B1, 0x2F},//BaseExample FF (2F=my output pins)
	{0x30B2, 0x24},//BaseExample 04  ??
		
	{0x3601, 0x00},//DVP Pins
	{0x3308, 0x00},//3801,01 = COLOR BAR TEST PATTERN

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
// 	{0x307E, 0xE5},//BaseExample E5
// 	{0x3079, 0x00},//BaseExample 00
// 	{0x30AA, 0x42},//BaseExample 42
// 	{0x3017, 0x40},//BaseExample 40
// 	{0x30F3, 0x83},//BaseExample 83
// 	{0x306A, 0x0C},//BaseExample 0C
// 	{0x306D, 0x00},//BaseExample 00
// 	{0x306A, 0x3C},//BaseExample 3C
// 	{0x3076, 0x6A},//BaseExample 6A
// 	{0x30D9, 0x95},//BaseExample 95
// 	{0x3016, 0x82},//BaseExample 82
// 		
// 	{0x304E, 0x88},//BaseExample 88 Repeated further down
// 	{0x30F1, 0x82},//BaseExample 82
// 	{0x306F, 0x14},//BaseExample 14
// 	{0x3391, 0x06},//BaseExample 06  NEW
// 	{0x3394, 0x38},//BaseExample 38  NEW
// 	{0x3395, 0x38},//BaseExample 38  NEW
		
	//AEC
	{0x3013, 0xF7},//DE7 BaseExample F7  17,37,77,C7,D7=black
	{0x3014, 0x8C},//D04 BaseExample 84 08 had the least streaking in low light (inside birdhouse with flashlight). Night Mode on.
	{0x3018, 0x88},//D78 BaseExample 80  AEC gap (stability)
	{0x3019, 0x70},//D68 BaseExample 70  AEC gap (stability)
	{0x301A, 0xD4},//DD4 BaseExample D4  AEC step
	// Considered, but decided not needed:  3047,3038,3039,303A,303B,303C,303D,303E,303F,3030,3031,3032,3033,3002,3003,3008,3009,3040,3041,3042,3043,3044,303A,303B,303E,303F
		
// 	//D5060
// 	{0x30AF, 0x00},//BaseExample 00  Was 10
// 	{0x3048, 0x1F},//BaseExample 1F  NEW
// 	{0x3049, 0x4E},//BaseExample 4E  NEW
// 	{0x304A, 0x20},//BaseExample 20  NEW
// 	{0x304F, 0x20},//BaseExample 20  NEW
// 	{0x30A3, 0x10},//BaseExample 10  80 NEW  ALSO IN OTHER FUNCTIONS
// 	{0x3013, 0xF7},//BaseExample 47  ALSO IN AEC
// 	{0x3014, 0x84},//BaseExample 84  BIT[3] 1=NIGHT MODE ON, 0=NIGHT MODE OFF
// 	{0x3071, 0x00},//BaseExample 00  NEW
// 	{0x3070, 0x5D},//BaseExample 5D  NEW
// 	{0x3073, 0x00},//BaseExample 00  NEW
// 	{0x3072, 0x4D},//BaseExample 4D
// 	{0x301C, 0x07},//BaseExample 07
// 	{0x301D, 0x08},//BaseExample 08
// 	{0x304D, 0x42},//BaseExample 42  NEW
// 	{0x304A, 0x40},//BaseExample 40  NEW OCCURS TWICE IN D5060
// 	{0x304F, 0x40},//BaseExample 40  NEW OCCURS TWICE IN D5060
// 	{0x3095, 0x07},//BaseExample 07
// 	{0x3096, 0x16},//BaseExample 16  NEW
// 	{0x3097, 0x1D},//BaseExample 1D  NEW
		
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


	//LENS CORRECTION
// 	{0x3350, 0x33},//BaseExample 33   SMOOTHER WITHOUT LENS CORRECTION
// 	{0x3351, 0x28},//BaseExample 28
// 	{0x3352, 0x00},//BaseExample 00
// 	{0x3353, 0x14},//BaseExample 14
// 	{0x3354, 0x00},//BaseExample 00
// 	{0x3355, 0x85},//BaseExample 85
// 	{0x3356, 0x35},//BaseExample 35
// 	{0x3357, 0x28},//BaseExample 28
// 	{0x3358, 0x00},//BaseExample 00
// 	{0x3359, 0x13},//BaseExample 13
// 	{0x335A, 0x00},//BaseExample 00
// 	{0x335B, 0x85},//BaseExample 85
// 	{0x335C, 0x34},//BaseExample 34
// 	{0x335D, 0x28},//BaseExample 28
// 	{0x335E, 0x00},//BaseExample 00
// 	{0x335F, 0x13},//BaseExample 13
// 	{0x3360, 0x00},//BaseExample 00
// 	{0x3361, 0x85},//BaseExample 85
// 	{0x3363, 0x70},//BaseExample 70
// 	{0x3364, 0x7F},//BaseExample 7F
// 	{0x3365, 0x00},//BaseExample 00
// 	{0x3366, 0x00},//BaseExample 00
// 	{0x3362, 0x90},//BaseExample 90

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

const ov_reg MY_OV2655_BASIC_CONFIG_OLD[] = {
		 
	{0xFFFF, 0xFF}};//STOPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
 
const ov_reg OV7692_QVGA_RGB888[] = {
	{0xFF, 0xFF},
};
const ov_reg OV7692_RESET[] = {
	{0xFF, 0xFF},
};
/**
 * \brief Addresses and values of the OV7740 registers for the
 * OV7740_QVGA_YUV422_10FPS configuration:
 *  - 320*240 pixel by picture (QVGA)
 *  - pixel data YUV422 in format (Y1, U, Y2, V)
 *  - 10 frames per second
 */
const ov_reg OV7740_QVGA_YUV422_10FPS[] = {
	{0x0e, 0x00},

	{0x12, 0x80},
	/* flag for soft reset delay */
	{0xFE, 0x05},
	{0x13, 0x00},

	/**************************************************************/
	/*  30fps  11 01 ;clock_divider ;sysclk=24MHz at XCLK=24MHz   */
	/*  20fps  11 02 ;clock_divider ;sysclk=16MHz at XCLK=24MHz   */
	/*  15fps  11 03 ;clock_divider ;sysclk=12MHz at XCLK=24MHz   */
	/*  10fps  11 05 ;sysclk=8MHz at XCLK=24MHz                   */
	/*  7p5fps 11 07 ;sysclk=6MHz at XCLK=24MHz                   */
	/**************************************************************/
	{0x11, 0x05},
	/**************************************************************/

	{0xFF, 0xFF}
};

/**
 * \brief Addresses and values of the OV7740 registers for the
 * OV7740_QVGA_YUV422_15FPS configuration:
 *  - 320*240 pixel by picture (QVGA)
 *  - pixel data in YUV422 format (Y1, U, Y2, V)
 *  - 15 frames per second
 */
const ov_reg OV7740_QVGA_YUV422_15FPS[] = {
	{0x0e, 0x00},

	{0x12, 0x80},
	/* flag for soft reset delay */
	{0xFE, 0x05},
	{0x13, 0x00},

	/**************************************************************/
	/*  30fps  11 01 ;clock_divider ;sysclk=24MHz at XCLK=24MHz   */
	/*  20fps  11 02 ;clock_divider ;sysclk=16MHz at XCLK=24MHz   */
	/*  15fps  11 03 ;clock_divider ;sysclk=12MHz at XCLK=24MHz   */
	/*  10fps  11 05 ;sysclk=8MHz at XCLK=24MHz                   */
	/*  7p5fps 11 07 ;sysclk=6MHz at XCLK=24MHz                   */
	/**************************************************************/
	{0x11, 0x03},

	{0xFF, 0xFF}
};

/**
 * \brief Addresses and values of the OV7740 registers for the
 * OV7740_QVGA_YUV422_20FPS configuration:
 *  - 320*240 pixel by picture (QVGA)
 *  - pixel data in YUV422 format (Y1, U, Y2, V)
 *  - 20 frames per second
 */
const ov_reg OV7740_QVGA_YUV422_20FPS[] = {

	{0xFF, 0xFF}
};

/**
 * \brief Addresses and values of the OV7740 registers for the
 * OV7740_QVGA_YUV422_30FPS configuration:
 *  - 320*240 pixel by picture (QVGA)
 *  - pixel data in YUV422 format (Y1, U, Y2, V)
 *  - 30 frames per second
 */
const ov_reg OV7740_QVGA_YUV422_30FPS[] = {

	{0xFF, 0xFF}
};

/**
 * \brief Addresses and values of the OV7740 registers for the
 * OV7740_QVGA_RGB888 configuration:
 *  - 320*240 pixel by picture (QVGA)
 *  - pixel data in RGB format (8-8-8)
 */
const ov_reg OV7740_QVGA_RGB888[] = {

	/*  */
	{0xFF, 0xFF},
};

/**
 * \brief Addresses and values of the OV7740 registers for the
 * OV7740_QQVGA_YUV422 configuration:
 *  - 160*120 pixel by picture (QQVGA)
 *  - pixel data in YUV422 format (Y1, U, Y2, V)
 */
const ov_reg OV7740_QQVGA_YUV422[] = {
	{ OV7740_REG0E, OV7740_REG0E_OUTPUT_1X},
	{0xFF, 0xFF}
};

/**
 * \brief Addresses and values of the OV7740 registers for the
 * OV7740_QVGA_RGB888 configuration:
 *  - 160*120 pixel by picture (QQVGA)
 *  - pixel data in RGB format (8-8-8)
 */
const ov_reg OV7740_QQVGA_RGB888[] = {
	{0xFF, 0xFF},
};

/**
 * \brief Addresses and values of the OV7740 registers for the
 * OV7740_TEST_PATTERN configuration:
 *  - 320*240 pixel by picture (QVGA)
 *  - pixel data in YUV422 format (Y1, U, Y2, V)
 *  - 20 frames per second
 *  - test pattern enable
 */
const ov_reg OV7740_TEST_PATTERN[] = {
	{0x0e, 0x00},
	{0xFF, 0xFF}
};

/**
 * \brief Addresses and values of the OV7740 registers for the
 * OV7740_VGA_YUV422_20FPS configuration:
 *  - 640*480 pixel by picture (VGA)
 *  - pixel data in YUV422 format (Y1, U, Y2, V)
 *  - 20 frames per second
 */
const ov_reg OV7740_VGA_YUV422_20FPS[] = {
	{0xFF, 0xFF}
};

/* @} */

#ifdef __cplusplus
}
#endif
