#define BTN1 		_RB4
#define BTN2 		_RA4
#define BTN3 		_RB6
#define BTN4 		_RB9
#define BTN5 		_RB10
#define PI	 		3.14159265358979323846
#define COL	 		102

#include "p24Fxxxx.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <osa.h>
#include "FSIO.h"
#include "time.c"
#include "lcd.c"
#include "DEE Emulation 16-bit.h"

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & BKBUG_ON & ICS_PGx1 & FWDTEN_OFF);
_CONFIG2( IESO_OFF & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_ON & IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_HS)

unsigned long  PosM[COL];
unsigned char TypeM[COL];
unsigned int FNameM[COL];
long double LatM[COL], LonM[COL];
unsigned char RadMin[COL], RadMax[COL];
unsigned int A1[COL], A2[COL], ColM, ColM_cnt, TM = COL, TMt, TM_old = COL, ColM_tmp;

FSFILE * pointer;
char tmp_buf[256], gps_str[110], audio_file[15];
unsigned char audio_buf1[512], audio_buf2[512];
volatile unsigned int audio_pos = 0, PFI, SatUs;
unsigned char in_rx2;
unsigned int GPS_state = 0, msgL;
char *gps_pos;
char *gps_str_pos, *tmp_pos;
unsigned int Speed;
long double Latitude, Longitude;
char Sts;
int err;
volatile unsigned int GPRMC_f = 0;
unsigned long dst, Counter;
unsigned int Counter1, Counter2;
signed int curs1, curs2;
unsigned int azt, ztst = 0, Course, delay_cnt, Menu = 0, ost_mode = 1;
unsigned int tst1, tst2;

OST_MSG_CB    msg_play;

unsigned long
dist(long double lat1, long double lon1, long double lat2, long double lon2)
{
	long double dlat, dlon, tmp;
	unsigned long dst1;

	lat1 = lat1 * (PI / 180);
	lat2 = lat2 * (PI / 180);
	lon1 = lon1 * (PI / 180);
	lon2 = lon2 * (PI / 180);
	dlat = lat2 - lat1;
	dlon = lon2 - lon1;
	tmp = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
	dst1 = 6378800 * 2 * atan2(sqrt(tmp), sqrt(1 - tmp));
	return dst1;
}

unsigned int
azimut(long double lat1, long double lon1, long double lat2, long double lon2)
{
	long double tmp, az;

	lat1 = lat1 * (PI / 180);
	lat2 = lat2 * (PI / 180);
	lon1 = lon1 * (PI / 180);
	lon2 = lon2 * (PI / 180);
	tmp = acos(cos((PI/2) - lat2) * cos((PI/2) - lat1) + sin((PI/2) - lat2) * sin((PI/2) - lat1) * cos(lon2 - lon1));
	az = asin(sin((PI/2) - lat2) * sin (lon2 - lon1) / sin(tmp));
	az = az * (180 / PI);
	if((az - floor(az)) > 0.5)
		az = ceil(az);
	else
		az = floor(az);
	if(lat2 < lat1)
		az = 180 - az;
	if((lat2 >= lat1) && (lon2 < lon1))
		az = 360 + az;
	return (unsigned int)az;
}

unsigned int
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

int
read_m(void)
{
	char * inp_str;

	pointer = FSfopen("m2.txt", "r");
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
		TypeM[ColM] = atoi(inp_str);
		if((TypeM[ColM] != 0) && (TypeM[ColM] != 1) && (TypeM[ColM] != 2))
		{
			FSfclose(pointer);
			return 3;
		}
		inp_str = strchr(inp_str, ' ');
		if(inp_str == NULL)
		{
			FSfclose(pointer);
			return 3;
		}
		inp_str++;
		FNameM[ColM] = atoi(inp_str);
		if(FNameM[ColM] == 0)
		{
			FSfclose(pointer);
			return 3;
		}
		inp_str = strchr(inp_str, ' ');
		if(inp_str == NULL)
		{
			FSfclose(pointer);
			return 3;
		}
		inp_str++;
		LatM[ColM] = atof(inp_str);
		if(LatM[ColM] == 0)
		{
			FSfclose(pointer);
			return 3;
		}
		inp_str = strchr(inp_str, ' ');
		if(inp_str == NULL)
		{
			FSfclose(pointer);
			return 3;
		}
		inp_str++;
		LonM[ColM] = atof(inp_str);
		if(LonM[ColM] == 0)
		{
			FSfclose(pointer);
			return 3;
		}
		inp_str = strchr(inp_str, ' ');
		if(inp_str == NULL)
		{
			FSfclose(pointer);
			return 3;
		}
		inp_str++;
		RadMin[ColM] = atoi(inp_str);
		if(RadMin[ColM] == 0)
		{
			FSfclose(pointer);
			return 3;
		}
		if((TypeM[ColM] == 0) || (TypeM[ColM] == 1))
		{
			inp_str = strchr(inp_str, ' ');
			if(inp_str == NULL)
			{
				FSfclose(pointer);
				return 3;
			}
			inp_str++;
			RadMax[ColM] = atoi(inp_str);
			if(RadMax[ColM] == 0)
			{
				FSfclose(pointer);
				return 3;
			}
			inp_str = strchr(inp_str, ' ');
			if(inp_str == NULL)
			{
				FSfclose(pointer);
				return 3;
			}
			inp_str++;
			A1[ColM] = atoi(inp_str);
			if(A1[ColM] > 359)
			{
				FSfclose(pointer);
				return 3;
			}
			inp_str = strchr(inp_str, ' ');
			if(inp_str == NULL)
			{
				FSfclose(pointer);
				return 3;
			}
			inp_str++;
			A2[ColM] = atoi(inp_str);
			if(A2[ColM] > 359)
			{
				FSfclose(pointer);
				return 3;
			}
		}
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
			}
		}
		FSfclose(audio_pntr);
		IEC0bits.T1IE = 0;
		OC1RS = 0;
		OS_Bsem_Reset(BS_PLAY);
	}
}

void
Task_GPRMC_PARSE(void)
{
	for(;;)
	{
		while(GPRMC_f == 0)
			OS_Yield();
		Counter++;
		gps_pos = strchr(gps_str, ',');
		gps_pos++;
		if(isdigit(*gps_pos))
		{
			Hour = ((*gps_pos - 0x30) * 10) + (*(gps_pos + 1) - 0x30);
			Minute = ((*(gps_pos + 2) - 0x30) * 10) + (*(gps_pos + 3) - 0x30);
			Second = ((*(gps_pos + 4) - 0x30) * 10) + (*(gps_pos + 5) - 0x30);
		}
		else
		{
			Hour = 0;
			Minute = 0;
			Second = 0;
		}
		gps_pos = strchr(gps_pos, ',');
		gps_pos++;
		if((*gps_pos == 'A') || (*gps_pos == 'V'))
		{
			Sts = *gps_pos;
		}
		gps_pos = strchr(gps_pos, ',');
		gps_pos++;
		if(isdigit(*gps_pos))
		{
			Latitude = ((*gps_pos - 0x30) * 10) + (*(gps_pos + 1) - 0x30);
			gps_pos = gps_pos + 2;
			Latitude = Latitude + (atof(gps_pos) / 60);
		}
		else
			Latitude = 0;
		gps_pos = strchr(gps_pos, ',');
		gps_pos++;
		gps_pos = strchr(gps_pos, ',');
		gps_pos++;
		if(isdigit(*gps_pos))
		{
			Longitude = ((*gps_pos - 0x30) * 100) + ((*(gps_pos + 1) - 0x30) * 10) + (*(gps_pos + 2) - 0x30);
			gps_pos = gps_pos + 3;
			Longitude = Longitude + (atof(gps_pos) / 60);
		}
		else
			Longitude = 0;
		gps_pos = strchr(gps_pos, ',');
		gps_pos++;
		gps_pos = strchr(gps_pos, ',');
		gps_pos++;
		if(isdigit(*gps_pos))
			Speed = atof(gps_pos) * 1.852;
		else
			Speed = 0;
		gps_pos = strchr(gps_pos, ',');
		gps_pos++;
		if(isdigit(*gps_pos))
			Course = atoi(gps_pos);
		else
			Course = 0;
		gps_pos = strchr(gps_pos, ',');
		gps_pos++;
		if(isdigit(*gps_pos))
		{
			Day = ((*gps_pos - 0x30) * 10) + (*(gps_pos + 1) - 0x30);
			Month = ((*(gps_pos + 2) - 0x30) * 10) + (*(gps_pos + 3) - 0x30);
			Year = ((*(gps_pos + 4) - 0x30) * 10) + (*(gps_pos + 5) - 0x30);
		}
		else
		{
			Day = 0;
			Month = 0;
			Year = 0;
		}
		GPRMC_f = 0;
		OS_Yield();
		if(Latitude != 0)
		{
			ztst = 0;
			for(ColM_cnt = 0; ColM_cnt < ColM; ColM_cnt++)
			{
				dst = dist(Latitude, Longitude, LatM[ColM_cnt], LonM[ColM_cnt]);
				OS_Yield();
				azt = azimut(LatM[ColM_cnt], LonM[ColM_cnt], Latitude, Longitude);
				OS_Yield();


				if(TypeM[ColM] == 0)
				{
					if((dst < RadMax[ColM_cnt]) && (dst > RadMin[ColM_cnt]))
					{
						curs1 = A1[ColM_cnt] - 75;
						if(curs1 < 0)
							curs1 = curs1 + 360;
						curs2 = A1[ColM_cnt] + 75;
						if(curs2 > 359)
								curs2 = curs2 - 360;
						if(A1[ColM_cnt] < A2[ColM_cnt])
						{
							if((azt <= A2[ColM_cnt]) && (azt >= A1[ColM_cnt]))
							{
								if(curs1 < curs2)
								{
									if((Course <= curs2) && (Course >= curs1))
									{
										ztst = 1;
										TMt = ColM_cnt;
										break;
									}
								}
								else
								{
									if((Course <= curs2) || (Course >= curs1))
									{
										ztst = 1;
										TMt = ColM_cnt;
										break;
									}
								}
							}
						}
						else
						{
							if((azt <= A2[ColM_cnt]) || (azt >= A1[ColM_cnt]))
							{
								if(curs1 < curs2)
								{
									if((Course <= curs2) && (Course >= curs1))
									{
										ztst = 1;
										TMt = ColM_cnt;
										break;
									}
								}
								else
								{
									if((Course <= curs2) || (Course >= curs1))
									{
										ztst = 1;
										TMt = ColM_cnt;
										break;
									}
								}
							}
						}
					}
				}
				if(TypeM[ColM] == 1)
				{
					if((dst < RadMax[ColM_cnt]) && (dst > RadMin[ColM_cnt]))
					{
						if(A1[ColM_cnt] < A2[ColM_cnt])
						{
							if((azt <= A2[ColM_cnt]) && (azt >= A1[ColM_cnt]))
							{
								ztst = 1;
								TMt = ColM_cnt;
								break;
							}
						}
						else
						{
							if((azt <= A2[ColM_cnt]) || (azt >= A1[ColM_cnt]))
							{
								ztst = 1;
								TMt = ColM_cnt;
								break;
							}
						}
					}
				}
				if(TypeM[ColM] == 2)
				{
					if(dst < RadMin[ColM_cnt])
					{
						ztst = 1;
						TMt = ColM_cnt;
						break;
					}
				}


			}
			if(ztst == 1)
			{
				if(TMt != TM)
				{
					TM = TMt;
					if(ost_mode == 1)
					{
						if(TM != TM_old)
						{
//							while(OS_Bsem_Check(BS_PLAY))
//								OS_Yield();
							TM_old = TM;
							sprintf(audio_file, "%u.wav", FNameM[TM]);
							OS_Msg_Send(msg_play, (OST_MSG)audio_file);
//							OS_Bsem_Set(BS_PLAY);
//							while(OS_Bsem_Check(BS_PLAY))
//								OS_Yield();
						}
					}
				}
			}
			else
				TM = COL;
		}

		TTime();
		CorrTime();
		if((Menu == 0) || (Menu == 1) || (Menu == 2))
			OS_Bsem_Set(BS_INDIK_REFRESH);
	}
}

void
Task_INDIK(void)
{
	for(;;)
	{
		OS_Bsem_Wait(BS_INDIK_REFRESH);
		switch(Menu)
		{
			case 0:
				lcd_clear();
				sprintf(tmp_buf, "%2.2u:%2.2u %2.2u/%2.2u/%2.2u %c", Hour, Minute, Day, Month, Year, Sts);
				lcd_puts(tmp_buf);
				lcd_goto(0x40);
				Counter1 = Counter / 3600;
				Counter2 = Counter % 3600;
				Counter2 = Counter2 / 60;
				sprintf(tmp_buf, "%2.2u:%2.2u:%2.2u", Counter1, Counter2, (unsigned int)(Counter % 60));
				lcd_puts(tmp_buf);
			break;
			case 1:
				lcd_clear();
				sprintf(tmp_buf, "%u км/час", Speed);
				lcd_puts(tmp_buf);
				lcd_goto(0x40);
				sprintf(tmp_buf, "%u", Course);
				lcd_puts(tmp_buf);
				lcd_putch(0xEF);
			break;
			case 2:
				lcd_clear();
				sprintf(tmp_buf, "Lat:%2.6f", (double)Latitude);
				lcd_puts(tmp_buf);
				lcd_goto(0x40);
				sprintf(tmp_buf, "Lon:%3.6f", (double)Longitude);
				lcd_puts(tmp_buf);
			break;
			case 3:
				pointer = FSfopen("m2.txt", "r");
				FSfseek(pointer, PosM[tst2], SEEK_SET);
				read_str();
				FSfclose(pointer);
				tmp_pos = strchr(tmp_buf, ' ');
				tmp_pos++;
				tmp_pos = strchr(tmp_pos, ' ');				
				tmp_pos++;
				tmp_pos = strchr(tmp_pos, ' ');				
				tmp_pos++;
				tmp_pos = strchr(tmp_pos, ' ');				
				tmp_pos++;
				tmp_pos = strchr(tmp_pos, ' ');				
				tmp_pos++;
				if((TypeM[tst2] == 0) || (TypeM[tst2] == 1))
				{
					tmp_pos = strchr(tmp_pos, ' ');				
					tmp_pos++;			
					tmp_pos = strchr(tmp_pos, ' ');				
					tmp_pos++;			
					tmp_pos = strchr(tmp_pos, ' ');				
					tmp_pos++;			
				}
				lcd_clear();
				lcd_goto(0x40);
				lcd_puts(tmp_pos);
				lcd_goto(13);
				sprintf(tmp_buf, "%u", tst2);
				lcd_puts(tmp_buf);
			break;
			case 4:
				lcd_clear();
				lcd_puts("Авт. режим:");
				lcd_goto(0x40);
				if(ost_mode == 1)
					lcd_puts("Включен");
				else
					lcd_puts("Выключен");
			break;
		}
	}
}

void
Task_BUTTONS(void)
{
	for(;;)
	{
		OS_Yield();
		switch(Menu)
		{
			case 0:
				if(!BTN2)
				{
					Counter = 0;
					OS_Bsem_Set(BS_INDIK_REFRESH);
					OS_Delay(60);
				}
			break;
			case 1:
			break;
			case 2:
			break;
			case 3:
				if(!BTN3)
				{
					if(tst2 == (ColM - 1))
						tst2 = 0;
					else
						tst2++;
					OS_Bsem_Set(BS_INDIK_REFRESH);
					OS_Delay(60);
				}
				if(!BTN5)
				{
					if(tst2 == 0)
						tst2 = ColM - 1;
					else
						tst2--;
					OS_Bsem_Set(BS_INDIK_REFRESH);
					OS_Delay(60);
				}
				if(!BTN2)
				{
					sprintf(audio_file, "%u.wav", FNameM[tst2]);
					OS_Msg_Send(msg_play, (OST_MSG)audio_file);
					OS_Delay(60);
				}
			break;
			case 4:
				if(!BTN3)
				{
					if(ost_mode == 0)
						ost_mode = 1;
					else
						ost_mode = 0;
					OS_Bsem_Set(BS_INDIK_REFRESH);
					OS_Delay(60);
				}
				if(!BTN5)
				{
					if(ost_mode == 0)
						ost_mode = 1;
					else
						ost_mode = 0;
					OS_Bsem_Set(BS_INDIK_REFRESH);
					OS_Delay(60);
				}
			break;
		}
		if(!BTN1)
		{
			if(Menu == 4)
				Menu = 0;
			else
				Menu++;
			OS_Bsem_Set(BS_INDIK_REFRESH);
			OS_Delay(60);
		}
		if(!BTN4)
		{
			if(Menu == 0)
				Menu = 4;
			else
				Menu--;
			OS_Bsem_Set(BS_INDIK_REFRESH);
			OS_Delay(60);
		}
	}
}

void
Task_TEST(void)
{
	for(;;)
	{
//		OS_Msg_Send(msg_play, (OST_MSG)"ding.wav");
//		OS_Bsem_Set(BS_PLAY);
//		while(OS_Bsem_Check(BS_PLAY))
//			OS_Yield();


/*		while(OS_Bsem_Check(BS_PLAY))
			OS_Yield();
		sprintf(audio_file, "%u.wav", FNameM[ColM_tmp]);
		OS_Msg_Send(msg_play, (OST_MSG)audio_file);
		OS_Bsem_Set(BS_PLAY);
		while(OS_Bsem_Check(BS_PLAY))
			OS_Yield();
		ColM_tmp++;
		if(ColM_tmp == ColM)
			ColM_tmp = 0;
*/
/*		TIC32_clear();
		sprintf(tmp_buf, "%2.6f %3.6f", (double)LatM[ColM_tmp], (double)LonM[ColM_tmp]);
		TIC32_puts(tmp_buf);
		TIC32_Cur_x = 0; TIC32_Cur_y = 1;
		sprintf(tmp_buf, "%u %u %u %u", RadMin[ColM_tmp], RadMax[ColM_tmp], A1[ColM_tmp], A2[ColM_tmp]);
		TIC32_puts(tmp_buf);
		TIC32_Cur_x = 0; TIC32_Cur_y = 2;
		sprintf(tmp_buf, "%u(%u) %u", ColM_tmp, ColM, FNameM[ColM_tmp]);
		TIC32_puts(tmp_buf);
		TIC32_refresh();
		ColM_tmp++;
		if(ColM_tmp == ColM)
			ColM_tmp = 0;
*/		OS_Delay(500);
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
PWM1set(void)
{
	OC1CONbits.OCM		= 0; 		// Output compare channel is disabled
	OC1R				= 0x0000;	// Initialize Compare Register1 with 50% duty cycle
	OC1RS				= 0x0000;	// Initialize Secondary Compare Register1 with 50% duty cycle
	OC1CONbits.OCSIDL	= 0;		// Output capture will continue to operate in CPU Idle mode
	OC1CONbits.OCFLT	= 0;		// No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
	OC1CONbits.OCTSEL	= 0;		// Timer2 is the clock source for output Compare
	OC1CONbits.OCM		= 0x6;		// PWM mode on OC, Fault pin disabled
	PR2					= 0x007F;	// Initialize PR2 with 0x0132 = 0d306 as PWM cycle
//	PR2					= 0x00FF;	// Initialize PR2 with 0x0132 = 0d306 as PWM cycle
	IFS0bits.T2IF		= 0;		// Clear Output Compare interrupt flag
	IEC0bits.T2IE		= 0;		// Enable Output Compare interrupts
	T2CONbits.TON		= 1;		// Start Timer2 with assumed settings
}

void
U2set(void)
{
	RPOR2 = 0x0005;
	U2MODE = 0b1100000000000000;
	U2STA = 0b0010010100010000;
	U2BRG = 207;                  //скорость по GPS 4800
	IFS1bits.U2TXIF = 0;
	IEC1bits.U2TXIE = 0;
	IFS1bits.U2RXIF = 0;
	IEC1bits.U2RXIE = 1;
}

int main(void)
{
	OS_Init();
	TRISA = 0b0000000000011100;
	TRISB = 0b0101011111110000;
	AD1PCFG = 0b0001111111111111;
	CNPU1 = 0b0000000000000011;
	CNPU2 = 0b0000000100100001;
	RPOR5 = 0x1200;					//OC1 to RP11
	RPINR19 = 0x1F05;				//U2RX to RP5
	RPINR20 = 0x1F0C;				//SCK1IN to RP12
	RPOR6 = 0x0800;					//SCK1OUT to RP13
	RPOR7 = 0x0007;					//SDO1 to RP14
	PWM1set();
	T1set();
	U2set();
	T3set();
/*
	DataEEInit();
    dataEEFlags.val = 0;

	if(DataEERead(0x00) == 0xA0B0)
	{
		
	}
*/
	lcd_init();

	if(!MDD_MediaDetect())
	{
		lcd_clear();
		lcd_puts("Insert SD card");
		while(!MDD_MediaDetect());
	}
	if(!FSInit())
	{
		lcd_clear();
		lcd_puts("SD card init error");
		while(MDD_MediaDetect());
		asm("RESET");
	}

	err = read_m();
	if(err == 1)
	{
		lcd_clear();
		lcd_puts("Error open mfile");
		while(MDD_MediaDetect());
		asm("RESET");
	}
	if(err == 2)
	{
		lcd_clear();
		lcd_puts("Error read mfile");
		while(MDD_MediaDetect());
		asm("RESET");
	}
	if(err == 3)
	{
		lcd_clear();
		sprintf(tmp_buf, "Wrong line %u", ColM + 1);
		lcd_puts(tmp_buf);
		while(MDD_MediaDetect());
		asm("RESET");
	}
	OS_Task_Create(3, Task_GPRMC_PARSE);
	OS_Task_Create(3, Task_AUDIO_PLAY);
	OS_Task_Create(3, Task_TEST);
	OS_Task_Create(3, Task_INDIK);
	OS_Task_Create(3, Task_BUTTONS);
	OS_Run();
	for(;;);
}

void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void)
{
	if(audio_pos > 511)
		OC1RS = audio_buf2[audio_pos - 512] >> 1;
	else
		OC1RS = audio_buf1[audio_pos] >> 1;
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
	in_rx2 = U2RXREG;
	if(GPRMC_f == 0)
	{
		switch(GPS_state)
		{
			case 0:
				if(in_rx2 == '$')
				{
					msgL = 0;
					GPS_state = 1;
				}
			break;
			case 1:
			{
				gps_str[msgL] = in_rx2;
				msgL++;
				if(msgL > 109)
					GPS_state = 0;
				if(in_rx2 == '*')
				{
					if(memcmp(gps_str, "GPGGA", 5) == 0)
					{
						gps_str_pos = strchr(gps_str, ',');
						gps_str_pos++;
						gps_str_pos = strchr(gps_str_pos, ',');
						gps_str_pos++;
						gps_str_pos = strchr(gps_str_pos, ',');
						gps_str_pos++;
						gps_str_pos = strchr(gps_str_pos, ',');
						gps_str_pos++;
						gps_str_pos = strchr(gps_str_pos, ',');
						gps_str_pos++;
						gps_str_pos = strchr(gps_str_pos, ',');
						gps_str_pos++;
						if(isdigit(*gps_str_pos))
							PFI = atoi(gps_str_pos);
						gps_str_pos = strchr(gps_str_pos, ',');
						gps_str_pos++;
						if(isdigit(*gps_str_pos))
							SatUs = atoi(gps_str_pos);
					}
					if((memcmp(gps_str, "GPRMC", 5) == 0))
						GPRMC_f = 1;
					GPS_state = 0;
				}
			}
			break;
		}
	}
	IFS1bits.U2RXIF = 0;
}
