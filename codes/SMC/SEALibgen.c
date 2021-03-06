/***************************************************************************************
*    Title: Series Elastic Actuator General Library (SEALibgen)
*    Dependencies: BCM2835 from airspayce.com/mikem/bcm2835/
*    Author: Talha Kansizoglu (AD-DA Module program is obtained from Waveshare)
*    Date: 23.07.2018
*    Library version: v6.0
***************************************************************************************/

#include "SEALibgen.h"

/* WaveShare Digital-Analog and Analog-Digital Converter Functions Begins */

void Write_DAC8532(uint8_t channel, uint16_t Data){
		uint8_t i;
		CS_1x() ;
		CS_0x() ;
		bcm2835_spi_transfer(channel);
		bcm2835_spi_transfer((Data>>8));
		bcm2835_spi_transfer((Data&0xff));
		CS_1x() ;
}

void bsp_DelayUS(uint64_t micros){
	bcm2835_delayMicroseconds(micros);
}


void bsp_InitADS1256(void){
	#ifdef SOFT_SPI
		CS_1();
		SCK_0();
		DI_0();
	#endif
}

void ADS1256_StartScan(uint8_t _ucScanMode){
	g_tADS1256.ScanMode = _ucScanMode;
	{
		uint8_t i;

		g_tADS1256.Channel = 0;

		for (i = 0; i < 8; i++)
		{
			g_tADS1256.AdcNow[i] = 0;
		}
	}
}

static void ADS1256_Send8Bit(uint8_t _data){
	bsp_DelayUS(2);
	bcm2835_spi_transfer(_data);
}

void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate)
{
	g_tADS1256.Gain = _gain;
	g_tADS1256.DataRate = _drate;
	ADS1256_WaitDRDY();
	{
		uint8_t buf[4];
		buf[0] = (0 << 3) | (1 << 2) | (0 << 1);
		buf[1] = 0x08;
		buf[2] = (0 << 5) | (0 << 3) | (_gain << 0);
		buf[3] = s_tabDataRate[_drate];	// DRATE_10SPS;
		CS_0();
		ADS1256_Send8Bit(CMD_WREG | 0);
		ADS1256_Send8Bit(0x03);
		ADS1256_Send8Bit(buf[0]);
		ADS1256_Send8Bit(buf[1]);
		ADS1256_Send8Bit(buf[2]);
		ADS1256_Send8Bit(buf[3]);
		CS_1();
	}
	bsp_DelayUS(50);
}


static void ADS1256_DelayDATA(void){
	bsp_DelayUS(6.5);
}

static uint8_t ADS1256_Recive8Bit(void){
	uint8_t read = 0;
	read = bcm2835_spi_transfer(0xff);
	return read;
}

static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue){
	CS_0();
	ADS1256_Send8Bit(CMD_WREG | _RegID);
	ADS1256_Send8Bit(0x00);
	ADS1256_Send8Bit(_RegValue);
	CS_1();
}

static uint8_t ADS1256_ReadReg(uint8_t _RegID){
	uint8_t read;
	CS_0();
	ADS1256_Send8Bit(CMD_RREG | _RegID);
	ADS1256_Send8Bit(0x00);
	ADS1256_DelayDATA();
	read = ADS1256_Recive8Bit();
	CS_1();
	return read;
}

static void ADS1256_WriteCmd(uint8_t _cmd){
	CS_0();
	ADS1256_Send8Bit(_cmd);
	CS_1();
}

uint8_t ADS1256_ReadChipID(void){
	uint8_t id;
	ADS1256_WaitDRDY();
	id = ADS1256_ReadReg(REG_STATUS);
	return (id >> 4);
}

static void ADS1256_SetChannal(uint8_t _ch){
	if (_ch > 7)
	{
		return;
	}
	ADS1256_WriteReg(REG_MUX, (_ch << 4) | (1 << 3));
}

static void ADS1256_SetDiffChannal(uint8_t _ch){
	if (_ch == 0)
	{
		ADS1256_WriteReg(REG_MUX, (0 << 4) | 1);	/* DiffChannal  AIN0�� AIN1 */
	}
	else if (_ch == 1)
	{
		ADS1256_WriteReg(REG_MUX, (2 << 4) | 3);	/*DiffChannal   AIN2�� AIN3 */
	}
	else if (_ch == 2)
	{
		ADS1256_WriteReg(REG_MUX, (4 << 4) | 5);	/*DiffChannal    AIN4�� AIN5 */
	}
	else if (_ch == 3)
	{
		ADS1256_WriteReg(REG_MUX, (6 << 4) | 7);	/*DiffChannal   AIN6�� AIN7 */
	}
}

static void ADS1256_WaitDRDY(void){
	uint32_t i;

	for (i = 0; i < 400000; i++)
	{
		if (DRDY_IS_LOW())
		{
			break;
		}
	}
	if (i >= 400000)
	{
		printf("ADS1256_WaitDRDY() Time Out\r\n");
	}
}

static int32_t ADS1256_ReadData(void){
	uint32_t read = 0;
	static uint8_t buf[3];

	CS_0();
	ADS1256_Send8Bit(CMD_RDATA);
	ADS1256_DelayDATA();
	buf[0] = ADS1256_Recive8Bit();
	buf[1] = ADS1256_Recive8Bit();
	buf[2] = ADS1256_Recive8Bit();

	read = ((uint32_t)buf[0] << 16) & 0x00FF0000;
	read |= ((uint32_t)buf[1] << 8);
	read |= buf[2];

	CS_1();

	if (read & 0x800000)
	{
		read |= 0xFF000000;
	}

	return (int32_t)read;
}

int32_t ADS1256_GetAdc(uint8_t _ch){
	int32_t iTemp;

	if (_ch > 7)
	{
		return 0;
	}

	iTemp = g_tADS1256.AdcNow[_ch];

	return iTemp;
}

void ADS1256_ISR(void){
	if (g_tADS1256.ScanMode == 0){

		ADS1256_SetChannal(g_tADS1256.Channel);
		bsp_DelayUS(5);
		ADS1256_WriteCmd(CMD_SYNC);
		bsp_DelayUS(5);
		ADS1256_WriteCmd(CMD_WAKEUP);
		bsp_DelayUS(25);

		if (g_tADS1256.Channel == 0){
			g_tADS1256.AdcNow[7] = ADS1256_ReadData();
		}
		else{
			g_tADS1256.AdcNow[g_tADS1256.Channel - 1] = ADS1256_ReadData();
		}

		if (++g_tADS1256.Channel >= 8){
			g_tADS1256.Channel = 0;
		}
	}
	else{

		ADS1256_SetDiffChannal(g_tADS1256.Channel);
		bsp_DelayUS(5);
		ADS1256_WriteCmd(CMD_SYNC);
		bsp_DelayUS(5);
		ADS1256_WriteCmd(CMD_WAKEUP);
		bsp_DelayUS(25);

		if (g_tADS1256.Channel == 0){
			g_tADS1256.AdcNow[3] = ADS1256_ReadData();
		}
		else{
			g_tADS1256.AdcNow[g_tADS1256.Channel - 1] = ADS1256_ReadData();
		}

		if (++g_tADS1256.Channel >= 4){
			g_tADS1256.Channel = 0;
		}
	}
}

uint8_t ADS1256_Scan(void){
	if (DRDY_IS_LOW()){
		ADS1256_ISR();
		return 1;
	}
	return 0;
}


void Write_DAC8552(uint8_t channel, uint16_t Data){
	uint8_t i;
	CS_1x();
	CS_0x();
	bcm2835_spi_transfer(channel);
	bcm2835_spi_transfer((Data >> 8));
	bcm2835_spi_transfer((Data & 0xff));
	CS_1x();
}

uint16_t Voltage_Convert(float Vref, float voltage)
{
	uint16_t _D_;
	_D_ = (uint16_t)(65536 * voltage / Vref);

	return _D_;
}

/* WaveShare Digital-Analog and Analog-Digital Converter Functions ENS */



void saturate(void){

  /* This function prevents inputSignal values than 10 Amps. ( 58 Nm from 10/Kt) */
  if(SEA.inputSignal/SEAnominal.Kt>9.9999){
    SEA.inputSignal = 9.9999*SEAnominal.Kt;
  }

  if(SEA.inputSignal/SEAnominal.Kt<-9.9){
    SEA.inputSignal = -9.9*SEAnominal.Kt;
  }

}

void saturateMID(void){

  /* This function prevents inputSignal values than 10 Amps. ( 58 Nm from 10/Kt) Orig 1.4 */
  if(SEA.inputSignal> 1.4){
    SEA.inputSignal =  1.4;
  }

  if(SEA.inputSignal< - 1.4){
    SEA.inputSignal =  -1.4;
  }

}

void saturateLOW(void){

  /* This function prevents inputSignal values than 10 Amps. ( 58 Nm from 10/Kt) Orig 1.4 */
  if(SEA.inputSignal> 0.9){
    SEA.inputSignal =  0.9;
  }

  if(SEA.inputSignal<- 0.9){
    SEA.inputSignal = - 0.9;
  }

}


int setConfig(void){

	if (!bcm2835_init())
		return 1;
	bcm2835_spi_begin();
	DISABLEMultiplexer();
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_LSBFIRST);
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024);
	bcm2835_gpio_fsel(SPICS, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(SPICS, HIGH);
	bcm2835_gpio_fsel(SPICSx, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(SPICSx, LOW);
	bcm2835_gpio_fsel(DRDY, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_set_pud(DRDY, BCM2835_GPIO_PUD_UP);
	bcm2835_gpio_fsel(PIN_CS, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(PIN_EN, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);

}


void DACWrite(void) {

  /* This function sends commands to the motor driver via 16-bit DAC */
	Write_DAC8532(0x30, Voltage_Convert(5.0,(10 + (SEA.inputSignal)/SEAnominal.Kt)/4));

}


void EmergencyAutoSwitch(int value, int value2){

  /* This function is an soft emergency switch, if input signal is higher than first function parameter,
   it increases motor emergency condition by one and if motor emergency condition is higher than limit
   function will reset motor input until the end of the program.

   Defaukt value2 is 58
  */

  if(SEA.inputSignal>value){
    experiment.MotorEmergency = experiment.MotorEmergency + 1; /* Increase motor emergency condition */
  }
  if(SEA.inputSignal<-value){
    experiment.MotorEmergency = experiment.MotorEmergency + 1; /* Increase motor emergency condition */
  }

  if(experiment.MotorEmergency>value2){
    /* Uncomment this part for only troubleshoothing, because it will print error to the screen which may cause latencies */
    //printf("Emergency mode on with %d Ams! This happens when the motor is at limit power for least %d cycles.\n",value,value2);
    SEA.inputSignal = 0;
    DACWrite();
    printf("%s\n", "Program is terminated due to a possible unstability!");
    int status;
    exit(status);
  }

}

double unwrap_(double encoderPosition, double encoderPositionOLD){

    /* This function is an unwrap function for phase unwrapping. This piece of code does the same thing with the unwrap function of the MATLAB. */
    double encoderPositionNEW;
    double phase_PI = PI - 2.2204e-16;
    double encoderPosition0 = 0;

    encoderPositionNEW = encoderPosition*enc_constant + encoderPosition0;
    double diff_phase = encoderPositionNEW - encoderPositionOLD;

    if (diff_phase>phase_PI){
        while(diff_phase>phase_PI){
            encoderPosition0 = encoderPosition0 - PI2;
            diff_phase = diff_phase - PI2;
        }
    }
    if (diff_phase< -phase_PI){
        while(diff_phase< -phase_PI){
            encoderPosition0 = encoderPosition0 + PI2;
            diff_phase = diff_phase + PI2;
        }
    }

    encoderPositionNEW = encoderPosition*enc_constant + encoderPosition0;
    return(encoderPositionNEW);
}

void getGeneralParameters(void){

  SEA.torsion = 0;
  SEA.torsionVel = 0;
  SEA.torsionPrev = 0;
  SEA.motorPos = 0;
  SEA.motorVel = 0;
	SEA.motorVelFiltered = 0;
  SEA.motorPosPrev = 0;
  SEA.inputSignal = 0;
  SEA.previnputSignal = 0;
  SEA.torsionInit = 0;

	SEAnominal.Jmn = 0.000278;
	SEAnominal.Jln = 0.07;
	SEAnominal.Bmn = 0.0015;
	SEAnominal.Bln = 0.005;
	SEAnominal.Kn = 5000;
	SEAnominal.Nm = 100;
	SEAnominal.Kt = 0.172;
	SEAnominal.linkLength = 0.42;
	SEAnominal.linkMass = 1.23;

	TorsionLPF.LPFfreq = 1200;
	TorsionLPF.num[0] = 1/(1 + 2/(TorsionLPF.LPFfreq*experiment.samplingTime));
	TorsionLPF.num[1] = TorsionLPF.num[0];
	TorsionLPF.denum[0] = (1 - 2/(TorsionLPF.LPFfreq*experiment.samplingTime)) * TorsionLPF.num[0];
	TorsionLPF.output[0] = 0;
	TorsionLPF.input[0] = 0;
	TorsionLPF.output[1] = 0;
	TorsionLPF.input[1] = 0;


	GENLPF1.LPFfreq = 1800;
	GENLPF1.num[0] = 1/(1 + 2/(GENLPF1.LPFfreq*experiment.samplingTime));
	GENLPF1.num[1] = GENLPF1.num[0];
	GENLPF1.denum[0] = (1 - 2/(GENLPF1.LPFfreq*experiment.samplingTime)) * GENLPF1.num[0];
	GENLPF1.output[0] = 0;
	GENLPF1.input[0] = 0;
	GENLPF1.output[1] = 0;
	GENLPF1.input[1] = 0;

  GENLPF2.LPFfreq = 1800;
  GENLPF2.num[0] = 1/(1 + 2/(GENLPF2.LPFfreq*experiment.samplingTime));
  GENLPF2.num[1] = GENLPF2.num[0];
  GENLPF2.denum[0] = (1 - 2/(GENLPF2.LPFfreq*experiment.samplingTime)) * GENLPF2.num[0];
  GENLPF2.output[0] = 0;
  GENLPF2.input[0] = 0;
  GENLPF2.output[1] = 0;
  GENLPF2.input[1] = 0;


	MotorPosDerivativeLPF.LPFfreq = 1000;
	MotorPosDerivativeLPF.num[0] = 1/(1 + 2/(MotorPosDerivativeLPF.LPFfreq*experiment.samplingTime));
	MotorPosDerivativeLPF.num[1] = MotorPosDerivativeLPF.num[0];
	MotorPosDerivativeLPF.denum[0] = (1 - 2/(MotorPosDerivativeLPF.LPFfreq*experiment.samplingTime)) * MotorPosDerivativeLPF.num[0];
	MotorPosDerivativeLPF.output[0] = 0;
	MotorPosDerivativeLPF.input[0] = 0;
	MotorPosDerivativeLPF.output[1] = 0;
	MotorPosDerivativeLPF.input[1] = 0;

	TorsionDerivativeLPF.LPFfreq = 1000;
	TorsionDerivativeLPF.num[0] = 1/(1 + 2/(TorsionDerivativeLPF.LPFfreq*experiment.samplingTime));
	TorsionDerivativeLPF.num[1] = TorsionDerivativeLPF.num[0];
	TorsionDerivativeLPF.denum[0] = (1 - 2/(TorsionDerivativeLPF.LPFfreq*experiment.samplingTime)) * TorsionDerivativeLPF.num[0];
	TorsionDerivativeLPF.output[0] = 0;
	TorsionDerivativeLPF.input[0] = 0;
	TorsionDerivativeLPF.output[1] = 0;
	TorsionDerivativeLPF.input[1] = 0;

}

void readEncoders(void) {

	/* This function reads encoders in every cycle */

  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_512);
  ENABLEMultiplexer();
  ENABLEmotorangle();                              /* Select motor encoder */
  union FiveByte data_read; data_read.bit64 = 0;   /* Define 5 Byte Data (40Bit) */
  bcm2835_spi_transfern(data_read.bit8, 5U);       /* Read encoder */
  ENABLEtorsion();                                 /* Select torsion encoder */
  union FiveByte data_read2; data_read2.bit64 = 0; /* Define 5 Byte Data (40Bit) */
  bcm2835_spi_transfern(data_read2.bit8, 5U);      /* Read encoder */
  DISABLEMultiplexer();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_LSBFIRST);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024);

  /* Combine Data */
  unsigned long value1 = data_read.bit8[4] | (data_read.bit8[3]<<8) | (data_read.bit8[2]<<16) | (data_read.bit8[1]<<24) | ((unsigned long long )data_read.bit8[0] <<32);
  unsigned long value2 = data_read2.bit8[4] | (data_read2.bit8[3]<<8) | (data_read2.bit8[2]<<16) | (data_read2.bit8[1]<<24) | ((unsigned long long )data_read2.bit8[0] <<32);

  SEA.encoderPositionUPD = unwrap_((value1-SEA.motorPosInit), SEA.encoderPosition2_old); // Faz kaymaları için
  SEA.motorPos = (SEA.encoderPositionUPD); // Okunan motor açısı bilgisi
  SEA.encoderPosition2_old = SEA.encoderPositionUPD;
  SEA.torsion = (value2-torsionZero)*enc_constant-SEA.torsionInit;

	SEA.torsionFiltered = kalmanFilter(SEA.torsion);
  SEA.motorVel = MotorPosDerivativeBlock(SEA.motorPos);
	//SEA.torsionVel = TorsionDerivativeBlock(SEA.torsionFiltered);

  SEA.torsionVel = ((SEA.torsionFiltered-SEA.torsionFilteredPrev)*2000);
  SEA.torsionFilteredPrev = SEA.torsionFiltered;

	//SEA.torsionVel = GENLPF1Block((SEA.torsion-SEA.torsionPrev)*2000);

  SEA.torsionPrev = SEA.torsion;
	//SEA.torsion = 0*SEA.torsionFiltered;
	//SEA.motorVel = 0;
	//SEA.motorPos = 0;
}


void readEncodersInit(void) {

	/* This function reads encoders for the first time */

	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_512);
	ENABLEMultiplexer();
	ENABLEmotorangle();                              /* Select motor encoder */
	union FiveByte data_read; data_read.bit64 = 0;   /* Define 5 Byte Data (40Bit) */
	bcm2835_spi_transfern(data_read.bit8, 5U);       /* Read encoder */
	ENABLEtorsion();                                 /* Select torsion encoder */
	union FiveByte data_read2; data_read2.bit64 = 0; /* Define 5 Byte Data (40Bit) */
	bcm2835_spi_transfern(data_read2.bit8, 5U);      /* Read encoder */
	DISABLEMultiplexer();
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_LSBFIRST);
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024);

	/* Combine Data */
	unsigned long value1 = data_read.bit8[4] | (data_read.bit8[3]<<8) | (data_read.bit8[2]<<16) | (data_read.bit8[1]<<24) | ((unsigned long long )data_read.bit8[0] <<32);
	unsigned long value2 = data_read2.bit8[4] | (data_read2.bit8[3]<<8) | (data_read2.bit8[2]<<16) | (data_read2.bit8[1]<<24) | ((unsigned long long )data_read2.bit8[0] <<32);
	SEA.torsionInit = (value2-torsionZero)*enc_constant;
	SEA.motorPosInit = (value1);


}


double TorsionLPFBlock(double inPart){

  /* LPF */
  double TorsionLPFOut;

  /* Diff equation */
  TorsionLPF.input[0] = inPart;
  TorsionLPFOut = TorsionLPF.num[0] * TorsionLPF.input[0] + TorsionLPF.num[1] *  TorsionLPF.input[1] - TorsionLPF.denum[0]*TorsionLPF.output[0];

  /* Update states */
  TorsionLPF.input[1] =  TorsionLPF.input[0];
  TorsionLPF.output[0] = TorsionLPFOut;

  return TorsionLPFOut;
}

double GENLPF1Block(double inPart){

  /* LPF */
  double GENLPF1Out;

  /* Diff equation */
  GENLPF1.input[0] = inPart;
  GENLPF1Out = GENLPF1.num[0] * GENLPF1.input[0] + GENLPF1.num[1] *  GENLPF1.input[1] - GENLPF1.denum[0]*GENLPF1.output[0];

  /* Update states */
  GENLPF1.input[1] =  GENLPF1.input[0];
  GENLPF1.output[0] = GENLPF1Out;

  return GENLPF1Out;
}

double GENLPF2Block(double inPart){

  /* LPF */
  double GENLPF2Out;

  /* Diff equation */
  GENLPF2.input[0] = inPart;
  GENLPF2Out = GENLPF2.num[0] * GENLPF2.input[0] + GENLPF2.num[1] *  GENLPF2.input[1] - GENLPF2.denum[0]*GENLPF2.output[0];

  /* Update states */
  GENLPF2.input[1] =  GENLPF2.input[0];
  GENLPF2.output[0] = GENLPF2Out;

  return GENLPF2Out;
}


double MotorPosDerivativeBlock(double inPart){

	// Input is motor angle (rad) output is motor velocity (rad/s)
  /* LPF */
  double MotorPosDerivativeLPFOut;

  /* Diff equation */
  MotorPosDerivativeLPF.input[0] = inPart*MotorPosDerivativeLPF.LPFfreq;
  MotorPosDerivativeLPFOut = MotorPosDerivativeLPF.num[0] * MotorPosDerivativeLPF.input[0] + MotorPosDerivativeLPF.num[1] *  MotorPosDerivativeLPF.input[1] - MotorPosDerivativeLPF.denum[0]*MotorPosDerivativeLPF.output[0];

  /* Update states */
  MotorPosDerivativeLPF.input[1] =  MotorPosDerivativeLPF.input[0];
  MotorPosDerivativeLPF.output[0] = MotorPosDerivativeLPFOut;

  return inPart*MotorPosDerivativeLPF.LPFfreq-MotorPosDerivativeLPFOut;
}

double TorsionDerivativeBlock(double inPart){

	// Input is torsion (rad) output is torsion velocity (rad/s)
  /* LPF */
  double TorsionDerivativeLPFOut;

  /* Diff equation */
  TorsionDerivativeLPF.input[0] = inPart*TorsionDerivativeLPF.LPFfreq;
  TorsionDerivativeLPFOut = TorsionDerivativeLPF.num[0] * TorsionDerivativeLPF.input[0] + TorsionDerivativeLPF.num[1] *  TorsionDerivativeLPF.input[1] - TorsionDerivativeLPF.denum[0]*TorsionDerivativeLPF.output[0];

  /* Update states */
  TorsionDerivativeLPF.input[1] =  TorsionDerivativeLPF.input[0];
  TorsionDerivativeLPF.output[0] = TorsionDerivativeLPFOut;

  return inPart*TorsionDerivativeLPF.LPFfreq-TorsionDerivativeLPFOut;
}
