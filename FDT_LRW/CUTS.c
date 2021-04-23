short getALSfromVEML(char slaveAddress){
	unsigned char data[4];
	unsigned char readRet,writeRet = 0;
	char iters,loop;
	unsigned short lightMeasurement;
	char errCtr;
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

//for (loop=3;loop>0;loop--){	//use this loop to test multiple readings	
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;//Feeds the WatchDog
		data[0]=0x04;//
		write_data_L(data,1,0x20,1);
		SENS_SCL_FLOAT;
		SENS_SDA_FLOAT;
		
		for(iters=0;iters<3;iters++)data[iters]=0;//ZERO the buffer
		read_bytes_L(data,2,0x20);
		SENS_SCL_FLOAT;
		SENS_SDA_FLOAT;
		
		lightMeasurement = (data[1] << 4) + (data[0]>>4);
// 		delay_ms(800);
// 		printf("ALS:   %x\r\n",lightMeasurement);
//}
	return(lightMeasurement);
}
/***********************************************************************/
 /*  Get Altimeter
 /***********************************************************************/
unsigned int getAltimeterTemperature(void){
		unsigned int temperature,pressure;
		char data[5];
		pmc_enable_periph_clk(ID_PIOD);
		SENSPWR_OUTPUT;
		SENSPWR_ON;
		ioport_set_pin_dir(SENS_CLK, IOPORT_DIR_OUTPUT);
		ioport_set_pin_dir(SENS_SDA, IOPORT_DIR_OUTPUT);
		ioport_set_pin_level(SENS_CLK, 1);
		ioport_set_pin_level(SENS_SDA, 1);
#ifndef OLD_PVC_BOARD		
		delay_ms(5);//

		data[0]=0xAC;//Altimeter Command initiates measurement
		write_data_A(data,1);//!< write sequentially to the slave "1" sends addr plus data[0].
		char altTimeout = 30;
		while((altTimeout>0)&&(data[0]!=0x44)){  //typicall does 9 iterations before data ready
			altTimeout--;
			delay_ms(1);
  			read_bytes_A(data,5);//!< write sequentially to the slave "1" sends addr plus data[0].
		}
		ioport_set_pin_dir(SENS_SDA, IOPORT_DIR_INPUT);//SENS_SDA
		ioport_set_pin_dir(SENS_CLK, IOPORT_DIR_INPUT);//ATL_CLK
		ioport_set_pin_mode(SENS_SDA, IOPORT_MODE_PULLUP);//SENS_SDA
		ioport_set_pin_mode(SENS_CLK, IOPORT_MODE_PULLUP);//ATL_CLK
		SENSPWR_OUTPUT;
		SENSPWR_OFF;
		
		temperature = ((data[3]<<8)+(data[4]));
		pressure = ((data[1]<<8)+(data[2]));
		
		temperature = (temperature>>8)*125;
		temperature = (temperature>>7)*5;
		temperature -= 400;

		pressure = (pressure>>6)*25;
		pressure = (pressure>>6)*25;
		pressure += 2600;
#endif
#ifdef DO_DIAGS
//  	printf("%d  %d\r\n",temperature, pressure);//tshoooooooooooooooooot
#endif
		return ((temperature<<16)+pressure);
 }
 /***********************************************************************/
 /*  Get Altimeter
 /***********************************************************************/
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
