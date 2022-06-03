#define DAC_CS		_LATB4
#define LED1		_LATC8
#define AMP_POW		_LATB13
#define BTN1		_RC2


#define PI	 		3.14159265358979323846
#define COL	 		102

void    Task_START(void);
void    Task_AUDIO_PLAY(void);
void    Task_TEST(void);
void    Task_GPS_1(void);
void    Task_BTN(void);

#include "p24Fxxxx.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <osa.h>
#include "FSIO.h"
#include "DEE Emulation 16-bit.h"

_CONFIG1( JTAGEN_OFF & GCP_ON & GWRP_OFF & BKBUG_ON & ICS_PGx1 & FWDTEN_OFF);
_CONFIG2( IESO_OFF & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_ON & IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_HS)

unsigned int  PosM[COL], DIR[COL];
unsigned long LatM[COL], LonM[COL];
unsigned char ColM, ColM_cnt, TM = COL, TMt, TM_old = COL, ColM_tmp;

FSFILE * pointer;
char tmp_buf[256], audio_file[30];
unsigned char audio_buf1[512], audio_buf2[512];
char ib2[128];
volatile unsigned int audio_pos = 0, PFI;
unsigned char in_rx2;
unsigned int GPS_state = 0, msgL;
char *tmp_pos, *tmp_pos1;
char * ibp2;
volatile unsigned int Speed;
unsigned int deg;
volatile unsigned long Latitude, Longitude;
volatile unsigned char pps1 = 0, SatUs = 0, Year = 0, Month = 0, Day = 0, Hour = 0, Minute = 0, Second = 0;
volatile unsigned int Course = 0;
unsigned char gps_state = 0;
char Sts;
int err;
char in_byte2;
signed int curs1, curs2;
unsigned int azt, ztst = 0, delay_cnt, Menu = 0, ost_mode = 1;
unsigned int tst1, tst2;
unsigned char SPI_ret, move_d, Move;
long double lat1a, lat2a, lon1a, lon2a, dlat, dlon, tmpdst, dst;

OST_MSG_CB    msg_play;

unsigned int
write_byte(unsigned int data)
{
	DAC_CS = 0;
	SPI2BUF = data;
	while(!SPI2STATbits.SPIRBF);
	SPI_ret = SPI2BUF;
	DAC_CS = 1;
	return SPI_ret;
}

unsigned int 
dist(unsigned long lat1, unsigned long lon1, unsigned long lat2, unsigned long lon2)
{
	lat1a = (double)lat1 / 572957795.13082320876798154814105;
	lat2a = (double)lat2 / 572957795.13082320876798154814105;
	lon1a = (double)lon1 / 572957795.13082320876798154814105;
	lon2a = (double)lon2 / 572957795.13082320876798154814105;
	dlat = lat2a - lat1a;
	dlon = lon2a - lon1a;
	tmpdst = pow(sin(dlat / 2), 2) + cos(lat1a) * cos(lat2a) * pow(sin(dlon / 2), 2);
	dst = 6378800 * 2 * atan2(sqrt(tmpdst), sqrt(1 - tmpdst));
	return (unsigned int)dst;
}

unsigned int
dir_r(unsigned int dir1)
{
	unsigned int r1, r2, a1, a2;
	if(dir1 > Course)
	{
		r1 = dir1;
		r2 = Course;
	}
	else
	{
		r1 = Course;
		r2 = dir1;
	}
	a1 = r1 - r2;
	a2 = (360 - r1) + r2;
	if(a1 < a2)
		return a1;
	else
		return a2;
}

unsigned char
read_str(void)
{
	unsigned int cnt1 = 0;
	char rb, *inp_str = tmp_buf;

	do
	{
		if(FSfread(&rb, 1, 1, pointer) != 1)
			return cnt1;
		if((rb != 0x0D) && (rb != 0x0A))
		{
			if(cnt1 < 253)
			{
				*inp_str++ = rb;
				cnt1++;
			}
		}
		else
		{
			if((cnt1 != 0) && (rb == 0x0A))
			{
				*inp_str++ = 0;
				cnt1++;
				return cnt1;
			}
		}
	}while(!FSfeof(pointer));
	return cnt1;
}

unsigned char
read_m(void)
{
	unsigned int dir_t;
	char * inp_str;

	pointer = FSfopen("m3.txt", "r");
	if (pointer == NULL)
		return 1;
	ColM = 0;
	do{
		PosM[ColM] = FSftell(pointer);
		if(read_str() == 0)
		{
			FSfclose(pointer);
			return 2;
		}
		inp_str = tmp_buf;
		if(inp_str == NULL)
		{
			FSfclose(pointer);
			return 3;
		}
		inp_str = strchr(inp_str, ',');
		if(inp_str == NULL)
		{
			FSfclose(pointer);
			return 3;
		}
		inp_str++;
		LatM[ColM] = atol(inp_str);
		if(LatM[ColM] == 0)
		{
			FSfclose(pointer);
			return 3;
		}
		inp_str = strchr(inp_str, ',');
		if(inp_str == NULL)
		{
			FSfclose(pointer);
			return 3;
		}
		inp_str++;
		LonM[ColM] = atol(inp_str);
		if(LonM[ColM] == 0)
		{
			FSfclose(pointer);
			return 3;
		}
		inp_str = strchr(inp_str, ',');
		if(inp_str == NULL)
		{
			FSfclose(pointer);
			return 3;
		}
		inp_str++;
		if(*inp_str != ',')
		{
			dir_t = atoi(inp_str);
			DIR[ColM] = dir_t;
		}
		else
			DIR[ColM] = 380;
		ColM++;
		if(ColM == COL)
			break;
	}while(!FSfeof(pointer));
	ColM--;
	if(FSfclose(pointer))
		return 4;
	return 0;
}

void
Task_AUDIO_PLAY(void)
{
	OST_MSG msg;

	for(;;)
	{
		static FSFILE * audio_pntr;

		OS_Msg_Wait(msg_play, msg);
		OS_Bsem_Set(BS_PLAY);

		audio_pntr = FSfopen(msg, "r");
		if(audio_pntr != NULL)
		{
			if(!FSfseek(audio_pntr, 0x2C, SEEK_SET))
			{
				AMP_POW = 0;
				OS_Delay(100);
				FSfread(audio_buf1, 512, 1, audio_pntr);
				OS_Yield();
				FSfread(audio_buf2, 512, 1, audio_pntr);
				OS_Yield();
				audio_pos = 0;
				IFS0bits.T1IF = 0;
				IEC0bits.T1IE = 1;
				do{
					OS_Bsem_Wait(BS_BUF1_DONE);
					FSfread(audio_buf1, 512, 1, audio_pntr);
					OS_Bsem_Wait(BS_BUF2_DONE);
					FSfread(audio_buf2, 512, 1, audio_pntr);
				}while(!FSfeof(audio_pntr));
				AMP_POW = 1;
			}
		}
		FSfclose(audio_pntr);
		IEC0bits.T1IE = 0;
		OS_Bsem_Reset(BS_PLAY);
	}
}

void
Task_TEST(void)
{
	for(;;)
	{
		OS_Msg_Send(msg_play, (OST_MSG)"1.wav");
		OS_Bsem_Set(BS_PLAY);
		while(OS_Bsem_Check(BS_PLAY))
			OS_Yield();
		OS_Delay(1000);
		OS_Msg_Send(msg_play, (OST_MSG)"2.wav");
		OS_Bsem_Set(BS_PLAY);
		while(OS_Bsem_Check(BS_PLAY))
			OS_Yield();
		OS_Delay(1000);
		OS_Msg_Send(msg_play, (OST_MSG)"3.wav");
		OS_Bsem_Set(BS_PLAY);
		while(OS_Bsem_Check(BS_PLAY))
			OS_Yield();
		OS_Delay(1000);
		OS_Msg_Send(msg_play, (OST_MSG)"4.wav");
		OS_Bsem_Set(BS_PLAY);
		while(OS_Bsem_Check(BS_PLAY))
			OS_Yield();
		OS_Delay(1000);
		OS_Msg_Send(msg_play, (OST_MSG)"5.wav");
		OS_Bsem_Set(BS_PLAY);
		while(OS_Bsem_Check(BS_PLAY))
			OS_Yield();
		OS_Delay(1000);
		OS_Msg_Send(msg_play, (OST_MSG)"6.wav");
		OS_Bsem_Set(BS_PLAY);
		while(OS_Bsem_Check(BS_PLAY))
			OS_Yield();
		OS_Delay(1000);
		OS_Msg_Send(msg_play, (OST_MSG)"7.wav");
		OS_Bsem_Set(BS_PLAY);
		while(OS_Bsem_Check(BS_PLAY))
			OS_Yield();
		OS_Delay(1000);
		OS_Msg_Send(msg_play, (OST_MSG)"8.wav");
		OS_Bsem_Set(BS_PLAY);
		while(OS_Bsem_Check(BS_PLAY))
			OS_Yield();
		OS_Delay(1000);
		OS_Msg_Send(msg_play, (OST_MSG)"9.wav");
		OS_Bsem_Set(BS_PLAY);
		while(OS_Bsem_Check(BS_PLAY))
			OS_Yield();
		OS_Delay(1000);
	}
}

void Task_GPS_1(void)
{
	while(1)
	{
		if(pps1)
		{
			if((SatUs >> 6) == 2)
			{
				Speed = (unsigned int)((float)Speed * 1.852);
				if(Speed > 1)
				{
					if(move_d < 5)
						move_d++;
				}
				else
				{
					if(move_d != 0)
					move_d--;
				}
				if(move_d == 5)
				{
					Move = 1;
				}
				else
				{
					Move = 0;
				}
				if(Move)
				{
					ztst = 0;
					for(ColM_cnt = 0; ColM_cnt <= ColM; ColM_cnt++)
					{
						dst = dist(Latitude, Longitude, LatM[ColM_cnt], LonM[ColM_cnt]);
						OS_Yield();
						if(dst < 70)
						{
							if(DIR[ColM_cnt] == 380)
							{
								ztst = 1;
								TMt = ColM_cnt;
								break;
							}
							else
							{
								if(dir_r(DIR[ColM_cnt]) < 15)
								{
									ztst = 1;
									TMt = ColM_cnt;
									break;
								}
							}
						}
					}
					if(ztst == 1)
					{
						if(TMt != TM)
						{
							TM = TMt;
							if(TM != TM_old)
							{
								while(OS_Bsem_Check(BS_PLAY))
									OS_Yield();
								TM_old = TM;
								pointer = FSfopen("m3.txt", "r");
								FSfseek(pointer, PosM[TM], SEEK_SET);
								read_str();
								FSfclose(pointer);
								tmp_pos1 = strchr(tmp_buf, ',');				
								*tmp_pos1 = 0;
								strcpy(audio_file, tmp_buf);
								OS_Msg_Send(msg_play, (OST_MSG)audio_file);
								OS_Bsem_Set(BS_PLAY);
								while(OS_Bsem_Check(BS_PLAY))
									OS_Yield();
							}
						}
					}
					else
						TM = COL;
				}
			}
			pps1 = 0;
		}	
		OS_Yield();
	}
}

void
Task_BTN(void)
{
	for(;;)
	{
		if(!BTN1)
		{
			OS_Delay(10);
			while(!BTN1)
				OS_Yield();
			OS_Msg_Send(msg_play, (OST_MSG)"ost.wav");
			OS_Bsem_Set(BS_PLAY);
			while(OS_Bsem_Check(BS_PLAY))
				OS_Yield();
		}
	}
}

unsigned char asd;
void
Task_START(void)
{
	if(FSInit())
		LED1 = 0;
	else
		LED1 = 1;
	asd = read_m();
	if((asd == 2) || (asd == 0))
		LED1 = 0;
	else
		LED1 = 1;

	if(LED1 == 1)
	{
		OS_Delay(500);
		asm("RESET");
	}

	OS_Task_Create(3, Task_AUDIO_PLAY);
//	OS_Task_Create(3, Task_TEST);
	OS_Task_Create(3, Task_GPS_1);
	OS_Task_Create(3, Task_BTN);

	for(;;)
	{
		OS_Delay(500);
	}
}

void
T1set(void)
{
	T1CON = 0x00; //Stops the Timer1 and reset control reg.
	TMR1 = 0x00; //Clear contents of the timer register
//	PR1 = 0x05AB; //11025Hz
	PR1 = 0x02D5; //22050Hz
	IPC0bits.T1IP = 0x01; //Setup Timer1 interrupt for desired priority level
	//(This example assigns level 1 priority)
	IFS0bits.T1IF = 0; //Clear the Timer1 interrupt status flag
	IEC0bits.T1IE = 0; //Enable Timer1 interrupts
	T1CONbits.TCKPS1 = 0;
	T1CONbits.TCKPS0 = 0;
	T1CONbits.TON = 1; //Start Timer1 with prescaler settings at 1:1 and
	//clock source set to the internal instruction cycle
}

void
T3set(void)
{
	T3CON = 0x00; //Stops the Timer3 and reset control reg.
	TMR3 = 0x00; //Clear contents of the timer register
//	PR3 = 0x04E2; //5mSec
	PR3 = 0x09C4; //10mSec
	IPC2bits.T3IP = 0x01; //Setup Timer3 interrupt for desired priority level
	IFS0bits.T3IF = 0; //Clear the Timer3 interrupt status flag
	IEC0bits.T3IE = 1; //Enable Timer3 interrupts
	T3CONbits.TCKPS1 = 1;
	T3CONbits.TCKPS0 = 0;
	T3CONbits.TON = 1; //Start Timer3 with prescaler settings at 1:256 and
	//clock source set to the internal instruction cycle
}

void
U2set(void)
{
	RPINR19 = 0x1F11;			//GPS_TX - RP17
	U2MODE = 0b1100000000000000;//low
	U2STA = 0b0010010100010000;
	U2BRG = 103;				//скорость по GPS 9600
	IFS1bits.U2TXIF = 0;
	IEC1bits.U2TXIE = 0;
	IFS1bits.U2RXIF = 0;
	IEC1bits.U2RXIE = 1;
}

void
SPI1set(void)
{
	RPINR20 = 0x1F0A;				//SD_DO to RP10
	RPOR4 = 0x0008;					//SD_CLK to RP8
	RPOR4 = 0x0708;					//SD_DI to RP9
}

void
SPI2set(void)
{
	RPOR2 = 0x0B00;				//DAC_CLK - RP5
	RPOR3 = 0x000A;				//DAC_DI - RP6
	SPI2STAT = 0x0000;
	SPI2CON1 = 0b0000010001111011;
	SPI2STAT = 0x8000;
}

unsigned int er1;

int main(void)
{
	OS_Init();
	TRISA = 0b0000011110011111;
	TRISB = 0b1101110000001111;
	TRISC = 0b0000001011111111;
	AD1PCFG = 0b0001111111111111;
	CNPU1 = 0b0000000000000000;
	CNPU2 = 0b0000000000000000;

	T1set();
	T3set();
	U2set();
	SPI1set();
	SPI2set();
	DAC_CS = 1;
	AMP_POW = 1;

	OS_Task_Create(3, Task_START);

	for(;;)
		OS_Run();
}

void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void)
{
	if(audio_pos > 511)
	{
		DAC_CS = 0;
		SPI2BUF = 0b0001000000000000 + audio_buf2[audio_pos - 512];
		while(!SPI2STATbits.SPIRBF);
		SPI_ret = SPI2BUF;
		DAC_CS = 1;
	}
	else
	{
		DAC_CS = 0;
		SPI2BUF = 0b0001000000000000 + audio_buf1[audio_pos];
		while(!SPI2STATbits.SPIRBF);
		SPI_ret = SPI2BUF;
		DAC_CS = 1;
	}
	audio_pos++;
	if(audio_pos == 512)
		OS_Bsem_Set_I(BS_BUF1_DONE);
	if(audio_pos == 1024)
	{
		OS_Bsem_Set_I(BS_BUF2_DONE);
		audio_pos = 0;
	}
	IFS0bits.T1IF = 0;
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T3Interrupt(void)
{
	OS_Timer();
	IFS0bits.T3IF = 0;
}

void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(void)
{
	IFS1bits.U2RXIF = 0;
	while(U2STA & 1)
	{
		in_byte2 = U2RXREG;
		switch(gps_state)
		{
			case 0:
				if(in_byte2 == '$')
				{
					gps_state = 1;
					ibp2 = ib2;
				}
			break;
			case 1:
				*ibp2++ = in_byte2;
				if(in_byte2 == '*')
				{
					ibp2 = ib2;
					if(!memcmp(ibp2, "GPRMC", 5))
					{
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						if(isdigit(*ibp2))
						{
							Hour = ((*ibp2 - 0x30) * 10) + (*(ibp2 + 1) - 0x30);
							Minute = ((*(ibp2 + 2) - 0x30) * 10) + (*(ibp2 + 3) - 0x30);
							Second = ((*(ibp2 + 4) - 0x30) * 10) + (*(ibp2 + 5) - 0x30);
						}
						else
						{
							Hour = 0;
							Minute = 0;
							Second = 0;
						}
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						if(isdigit(*ibp2) & ((SatUs >> 6) > 0))
						{
							deg = ((*ibp2 - 0x30) * 10) + (*(ibp2 + 1) - 0x30);
							ibp2 = ibp2 + 2;
							Latitude = atoi(ibp2);
							Latitude = Latitude * 10000;
							ibp2 = ibp2 + 3;
							*(ibp2 + 4) = 0x20;
							Latitude = atoi(ibp2) + Latitude;
							Latitude = Latitude * 16.666666;
							Latitude = (deg * 10000000) + Latitude;
						}
						else
							Latitude = 0;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						if(isdigit(*ibp2) & ((SatUs >> 6) > 0))
						{
							deg = ((*ibp2 - 0x30) * 100) + ((*(ibp2 + 1) - 0x30) * 10) + (*(ibp2 + 2) - 0x30);
							ibp2 = ibp2 + 3;
							Longitude = atoi(ibp2);
							Longitude = Longitude * 10000;
							ibp2 = ibp2 + 3;
							*(ibp2 + 4) = 0x20;
							Longitude = atoi(ibp2) + Longitude;
							Longitude = Longitude * 16.666666;
							Longitude = (deg * 10000000) + Longitude;
						}
						else
							Longitude = 0;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						if(isdigit(*ibp2) & ((SatUs >> 6) > 0))
							Speed = atoi(ibp2);
						else
							Speed = 0;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						if(isdigit(*ibp2) & ((SatUs >> 6) > 0))
							Course = atoi(ibp2);
						else
							Course = 0;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						if(isdigit(*ibp2))
						{
							Day = ((*ibp2 - 0x30) * 10) + (*(ibp2 + 1) - 0x30);
							Month = ((*(ibp2 + 2) - 0x30) * 10) + (*(ibp2 + 3) - 0x30);
							Year = ((*(ibp2 + 4) - 0x30) * 10) + (*(ibp2 + 5) - 0x30);
						}
						else
						{
							Day = 0;
							Month = 0;
							Year = 0;
						}
						pps1 = 1;
					}
					if(!memcmp(ibp2, "GPGGA", 5))
					{
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						if(isdigit(*ibp2))
						{
							SatUs = SatUs & 0b11000000;
							SatUs = SatUs + atoi(ibp2);
						}
					}
					if(!memcmp(ibp2, "GPGSA", 5))
					{
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						if(isdigit(*ibp2))
						{
							SatUs = SatUs & 0b00111111;
							SatUs = SatUs + ((atoi(ibp2) - 1) << 6);
						}
					}
					gps_state = 0;
				}
			break;
		}
	}
}
