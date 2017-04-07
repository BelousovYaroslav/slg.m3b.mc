#include <ADuC7026.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "serial.h"
#include "flashEE.h"
#include "errors.h"
#include "version.h"
#include "deadloops.h"
#include "settings.h"
#include "Main.h"

#define LONG_OUTPUT_PACK_LEN 24       //����� ����������� �� ������ 25 ����� (�� ����� ���. ����������� �� �������)
#define SHORT_OUTPUT_PACK_LEN 7       //� ����� �� ������ ���������� �� 7 ����� � ������. �����������

//********************
// Decrement coefficient calculation
#define DEC_COEFF_FIRST_CALCULATION_N 100
#define DEC_COEFF_CONTINUOUS_CALCULATION_N 1000
unsigned int gl_un_DecCoeffStatPoints = 0;
double gl_dbl_Nsumm = 0.;
double gl_dbl_Usumm = 0.;
double gl_dbl_Omega;

//********************

signed short gl_ssh_angle_inc = 0;      //���������� ����
signed short gl_ssh_angle_inc_prev = 0; //���������� ����

signed short gl_ssh_current_1 = 0;      //��������� ��� 1
signed short gl_ssh_current_2 = 0;      //��������� ��� 2
signed short gl_ssh_Perim_Voltage = 5;  //���������� �������� ����������������
signed short gl_ssh_ampl_angle = 0;     //���������� ���������������� ��������� ��������� ������� ��������� ������� ���. ��������

signed short gl_ssh_Utd1 = 0;           //���������� ���������� ������������
signed short gl_ssh_Utd2 = 0;           //���������� ��������� ������������
signed short gl_ssh_Utd3 = 0;           //���������� ? ������������
signed short gl_ssh_Utd1_cal = 0;       //���������� ���������� ������������ (������.)
signed short gl_ssh_Utd2_cal = 0;       //���������� ��������� ������������ (������.)
signed short gl_ssh_Utd3_cal = 0;       //���������� ? ������������ (������.)

signed short gl_ssh_SA_time = 0;        //������ SA
signed short gl_ssh_prT1VAL = 0x1000;   //������ SA

signed short gl_ssh_angle_hanger = 0;
signed short gl_ssh_angle_hanger_prev = 0;

unsigned short gl_ush_MeanImpulses = 1;

#define RULA_MAX 4095
#define RULA_MIN 25

//unsigned int gl_un_RULAControl = 64;      //64   = 0.039 V  
//unsigned int gl_un_RULAControl = 1638;    //1638 = 1.000 V  
//unsigned int gl_un_RULAControl = 2457;    //2457 = 1.500 V  
unsigned int gl_un_RULAControl = 4095;      //4095 = 2.500 V  



int nT2RepeatBang;


char gl_c_EmergencyCode = 0;


char bAsyncDu = 0;                    //���� �������� ������� SA ��� ������. ���� � ������. ������: 0-���������� SA 1-���������� dU


//�����
char gl_b_SyncMode = 0;               //���� ������ ������ ���������:   0=�����. 1=������.
//char gl_chAngleOutput = 0;            //���� ������ ���������� ����: 0 = dW (4b)         1 = dN (2b), dU(2b)
//char gl_chLockBit = 0;                //���� ������������ ����������: 0 - ����� "������������"   1 - ����� "������������"
//char gl_bOutData = 0;                 //���� ������ ������ ������ (���������� ����� 2.5 ��� ����� ��������� �������) 0 - ��� ������; 1 - ������ ������;
char gl_n_PerimeterReset = 0;         //���� ������� ������ ��������� (0 = ������ ����������, 1 = ������ ������������, ������ ���������� �����������, 2 = ������ ������������, ������ ��������� �����������)
char gl_b_SA_Processed = 0;           //���� ��������� ��������� ������� SA
char gl_bManualLaserOff = 0;          //���� ��� ���� ��������� ��� ������ (��������). ���� ����� ����� ��������� ��� ������� � ������������� �������� ����.


short nSentPacksCounter = 0;                    //������� �������
int nSentPacksRound = SHORT_OUTPUT_PACK_LEN;    //���� �������� �������
char gl_c_OutPackCounter = 0;                   //���������� ������ ������� �������

int ADCChannel = 0; //�������� ����� ���
                    //0 = ADC0 = UTD3
                    //1 = ADC1 = UTD1
                    //2 = ADC2 = UTD2
                    //3 = ADC3 = I1
                    //4 = ADC4 = I2
                    //5 = ADC5 = CntrPC
                    //6 = ADC6 = AmplAng


#define BIT_0 1
#define BIT_1 2
#define BIT_2 4
#define BIT_3 8
#define BIT_4 16
#define BIT_5 32
#define BIT_6 64
#define BIT_7 128


#define IN_COMMAND_BUF_LEN 3                    //����� ������ �������� ������
char input_buffer[6] = { 0, 0, 0, 0, 0, 0};     //����� �������� ������
char pos_in_in_buf = 0;                         //������� ������ � ������ �������� ������

//��������� �������� �� ����-������
unsigned short flashParamAmplitudeCode = 9000;    //��������� ��������� ������������
unsigned short flashParamTactCode = 0;            //��� ����� ���������
unsigned short flashParamMCoeff = 4;              //����������� ���������
unsigned short flashParamStartMode = 5;           //��������� ���� ������� ����������� ���������
unsigned short flashParamDecCoeff = 0;            //����������� ������
//unsigned short flashLockDev = 0;                //���� ���������� ����������

unsigned short flashParamI1min = 0;               //����������� �������� ���� ������� I1
unsigned short flashParamI2min = 0;               //����������� �������� ���� ������� I2
unsigned short flashParamAmplAngMin1 = 0;         //����������� �������� ������� �������� � ����


unsigned short flashParamSignCoeff = 2;           //�������� �����������
unsigned int flashParamDeviceId = 0;              //ID ����������
unsigned short flashParamDateYear = 0;            //���� ? �������.���
unsigned short flashParamDateMonth = 0;           //���� ? �������.�����
unsigned short flashParamDateDay = 0;             //���� ? �������.����
char flashParamOrg[17] = { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0};    //�������� �����������
unsigned short flashParamPhaseShift = 0;          //������� ����� (obsolete)

unsigned short gl_ushFlashParamLastRULA = 0;      //��������� RULA (obsolete)
unsigned short gl_ushFlashParamLastRULM = 0;      //��������� RULM (obsolete)

double dStartAmplAngCheck = 0.5;

unsigned short nFiringTry = 0;

//���������� �������������
signed short flashParam_calibT1;
unsigned short flashParamT1_TD1_val, flashParamT1_TD2_val, flashParamT1_TD3_val;
signed short flashParam_calibT2;
unsigned short flashParamT2_TD1_val, flashParamT2_TD2_val, flashParamT2_TD3_val;

char gl_bCalibProcessState;
char gl_bCalibrated;
double TD1_K, TD1_B;
double TD2_K, TD2_B;

int gl_nAppliedMCoeff;
int gl_nAmplStabStep = 0; //������� �������� ���������

double gl_dblAmplMean;
int  gl_nAmplMeanCounter;
unsigned int gl_un_PrevAmplRegulationT2;

double gl_dblMeanAbsDn;
double gl_dblMeanAbsDu;
int    gl_nMeanDecCoeffCounter;
unsigned int gl_un_PrevT2DecCoeffCalc;


////////////////////////////////////////////////////////////////////////////////////////////////////////////
//���������� ����������
void FIQ_Handler (void)	__fiq
{
  if( ( FIQSTA & UART_BIT) != 0)	{
    if( pos_in_in_buf < IN_COMMAND_BUF_LEN)
      input_buffer[ pos_in_in_buf++] = COMRX;
    //GP0DAT = ( 1 << 16);
  }
}

void pause_T0 ( int n) {
  unsigned int prval, chk;
  prval = T0VAL;
  chk = (( T0LD + prval - T0VAL) % T0LD);
  while( chk < n)
    chk = (( T0LD + prval - T0VAL) % T0LD);
}

void pause( int n) {
  unsigned int prval, chk;
  prval = T1VAL;
  chk = (( T1LD + prval - T1VAL) % T1LD);
  while( chk < n)
    chk = (( T1LD + prval - T1VAL) % T1LD);
}

double round( double val) {
  double lstd = val - floor( val);
  if( lstd < .5) return floor( val);
  else return ceil( val);
}

/*void PrintString( char*ptr, int n) {
}*/

void send_pack( signed short angle_inc1, short param_indicator, short analog_param) {
  char cCheckSumm = 0;
  char *pntr;
  char b1, b2, b3, b4;

#ifndef DEBUG
  //float angle_inc_corr;
  signed short angle_inc_corr;
  float f_dN;
  double dbl_dN;

  float Coeff = (( float) flashParamDecCoeff) / 65535.;

  signed short ssh_dN, ssh_dU;
  signed int n_dN, n_dU;
  double result;
  char bt;

  /*
  char b1 = 0, b2 = 0;

  b1 = angle_inc1 & 0xff;
  b2 = ( angle_inc1 & 0xff00) >> 8;

  PrintHexShortNumber( angle_inc1);
  putchar('=');
  PrintHexCharNumber( b2);
  putchar(' ');
  PrintHexCharNumber( b1);
  putchar(' ');
  putchar(' ');
  PrintHexShortNumber( param_indicator);
  putchar(' ');
  PrintHexShortNumber( analog_param);
  putchar('\n');
  */


  /*
  //**********************************************************************
  // ������ (������� ����� � ��� �������� �����)
  //**********************************************************************
  angle_inc1 = ( short) ( ( double) rand() / 32767. * 4.) - 2;
  switch( param_indicator) {
    case 0: analog_param = ( short) ( ( double) rand() / 32767. * 2.); break;             //0-1
    case 1: analog_param = ( short) ( ( double) rand() / 32767. * 2.); break;             //0-1
    case 2: analog_param = 2700 + ( short) ( ( double) rand() / 32767. * 100.); break;    //2700-2800
    case 3: analog_param = 2700 + ( short) ( ( double) rand() / 32767. * 2.); break;      //2700-2800
    case 4: analog_param = ( short) ( ( double) rand() / 32767. * 1.); break;             //0-1
    case 5: analog_param = 480 + ( short) ( ( double) rand() / 32767. * 20.); break;      //480-500
  }                                                             
  gl_ssh_SA_time = 78 + ( short) ( ( double) rand() / 32767. * 4.);
  */

  //***************************************************************************
  //START_MARKER
  //***************************************************************************
  putchar_nocheck( 0x55);
  putchar_nocheck( 0xAA);

  //***************************************************************************
  //    dN
  //***************************************************************************
  if( gl_b_SyncMode) {
    if( bAsyncDu) {
      //������: ������ dN-dU
      signed int siAngleInc1 = ( signed short) angle_inc1;
      //angle_inc_corr = (( float) (  siAngleInc1)) * 10.;
      angle_inc_corr = angle_inc1;
      f_dN = ( float) ( ( signed int) angle_inc1);
      dbl_dN = ( double) ( ( signed int) angle_inc1);
    }
    else {
      //������: ���������� �����
      /*double db_dN1 = ( double) gl_ssh_angle_inc_prev;
      double db_dN2 = ( double) gl_ssh_angle_inc;
      double dbU1 = ( double) gl_ssh_angle_hanger_prev;
      double dbU2 = ( double) gl_ssh_angle_hanger;*/

      ssh_dN = gl_ssh_angle_inc - gl_ssh_angle_inc_prev;
      ssh_dU = gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev;

      n_dN = ( signed int) ssh_dN;
      n_dU = ( signed int) ssh_dU;

      result =  ( double) n_dN - ( ( double) n_dU) * Coeff * (( signed short) flashParamSignCoeff - 1);
      //printf("\n%.3f %.3f %.3f %.3f %.3f\n", db_dN1, db_dN2, dbU1, dbU2, result);
      angle_inc_corr = ( signed short) ( result * 100.);

      f_dN = ( float) ( ( signed short) result);
      dbl_dN = ( double) ( ( signed short) result);
    }
  }
  else {
    //���������� �����
    double dAngleInc1 = ( double) angle_inc1;
    //angle_inc_corr = (( float) (  siAngleInc1)) * 10.;
    angle_inc_corr = ( signed short) ( angle_inc1 * 100.);

    f_dN = ( float) ( ( signed int) angle_inc1);
    dbl_dN = ( double) ( ( signed int) angle_inc1);
  }

  /*  
  putchar_nocheck( angle_inc_corr & 0xff);
  putchar_nocheck( ( angle_inc_corr & 0xff00) >> 8);
  */

  //����������� f_dN �� �������� [-99 310; + 99 310]
  dbl_dN = ( dbl_dN / 99310.) * 2147483647.;
  n_dN = ( int) dbl_dN;

  pntr = ( char *) &n_dN;
  b1 = pntr[0];
  b2 = pntr[1];
  b3 = pntr[2];
  b4 = pntr[3];

  putchar_nocheck( b1);   cCheckSumm += b1;
  putchar_nocheck( b2);   cCheckSumm += b2;
  putchar_nocheck( b3);   cCheckSumm += b3;
  putchar_nocheck( b4);   cCheckSumm += b4;


  //***************************************************************************
  //ANALOG PARAMETER INDICATOR
  //***************************************************************************
  putchar_nocheck( ( ( gl_n_PerimeterReset > 0) ? 0x80 : 0x00) + param_indicator & 0xff);
  cCheckSumm += (( ( gl_n_PerimeterReset > 0) ? 0x80 : 0x00) + param_indicator & 0xff);

  //***************************************************************************
  //ANALOG PARAMETER
  //***************************************************************************
  putchar_nocheck( analog_param & 0xff);
  cCheckSumm += (analog_param & 0xff);

  putchar_nocheck( ( analog_param & 0xff00) >> 8);
  cCheckSumm += ( ( analog_param & 0xff00) >> 8);

  //***************************************************************************
  //�����. �����: SA TIME
  //������. �����: SA TIME ��� ���������� ���� ��������
  if( gl_b_SyncMode) {
    //������. �����
    if( bAsyncDu) {
      //�������� dU
      putchar_nocheck( ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev) & 0xff);
      cCheckSumm += ( ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev) & 0xff);

      putchar_nocheck( ( ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev) & 0xff00) >> 8);
      cCheckSumm += ( ( ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev) & 0xff00) >> 8);
    }
    else {
      //�������� SA
      putchar_nocheck( gl_ssh_SA_time & 0xff);
      cCheckSumm += ( gl_ssh_SA_time & 0xff);

      putchar_nocheck( ( gl_ssh_SA_time & 0xff00) >> 8);
      cCheckSumm += ( ( gl_ssh_SA_time & 0xff00) >> 8);
    }
  }
  else {
    //���������� �����
    putchar_nocheck( gl_ssh_SA_time & 0xff);
    cCheckSumm += ( gl_ssh_SA_time & 0xff);

    putchar_nocheck( ( gl_ssh_SA_time & 0xff00) >> 8);
    cCheckSumm += ( ( gl_ssh_SA_time & 0xff00) >> 8);
  }

  //***************************************************************************
  //PACK COUNTER
  //***************************************************************************
  putchar_nocheck( gl_c_OutPackCounter);
  cCheckSumm += gl_c_OutPackCounter;

  gl_c_OutPackCounter++;

  //***************************************************************************
  //EMERGENCY CODE
  //***************************************************************************
  //8 bit - 0x80 - veracity
  bt = ( gl_n_PerimeterReset ? 0x80 : 0x00);

  //7 bit - 0x40 - lock bit
  //bt += ( gl_chLockBit ? 0x40 : 0x00);

  //6 bit - 0x20 - Sync (0)/Async(1) regime
  bt += ( gl_b_SyncMode ? 0x20 : 0x00);

  //5 bit - 0x10 - W (0) / dNdU (1) regime
  //bt += ( gl_chAngleOutput ? 0x10 : 0x00);

  //Error code (lower 4 byte)
  bt += gl_c_EmergencyCode;

  putchar_nocheck( bt);
  cCheckSumm += bt;

  //***************************************************************************
  // CHECKSUM
  //***************************************************************************
  putchar_nocheck( cCheckSumm);

#else
  //if( param_indicator > 10)

  angle_inc1 = ( short) ( ( double) rand() / 32767. * 10.) - 5;

  pntr = ( char *) &angle_inc1;
  b1 = pntr[0];
  b2 = pntr[1];
  b3 = pntr[2];
  b4 = pntr[3];

  /*
  printf("(0x55 0xAA)   (0x%02x 0x%02x 0x%02x 0x%02x)   (0x%02x)   (0x?? 0x??)   (0x?? 0x??)  (0x%02x)   (0x%02x)   (0x??)\n",
          b1, b2, b3, b4,
          ( ( gl_n_PerimeterReset > 0) ? 0x80 : 0x00) + param_indicator & 0xff,
          gl_c_OutPackCounter++,
          gl_c_EmergencyCode
          );
  */
#endif
}

void configure_hanger( void) {

  //1. ��� ����� ���������
  //�������� TactNoise0 (������� ��� ��������� "��� ����� ���������")
  if( ( flashParamTactCode & 0x01))  //Set TactNoise0 to TactCode parameter bit0
    GP3DAT |= ( 1 << (16 + 0));
  else
    GP3DAT &= ~( 1 << (16 + 0));

  //�������� TactNoise1 (������� ��� ��������� "��� ����� ���������")
  if( ( flashParamTactCode & 0x02))  //Set TactNoise1 to TactCode parameter bit1
    GP3DAT |= ( 1 << (16 + 1));
  else
    GP3DAT &= ~( 1 << (16 + 1));
}

void DACConfiguration( void) {
/*
#ifdef DEBUG
  printf("DEBUG: DACConfiguration(): enter\n");
#endif
*/
  //**********************************************************************
  // �������� �����
  //**********************************************************************
  // ��� 0
  DAC0DAT = (( int) ( 4095.0 * ( ( double) gl_un_RULAControl / ( double) RULA_MAX ))) << 16; //�������� �� ������ ���0 1,0 �


  // ��� 1 (����)
  /*gl_nAppliedMCoeff--;
  if( gl_nAppliedMCoeff <= flashParamMCoeff / 250. * 4096.)
    gl_nAppliedMCoeff = flashParamMCoeff / 250. * 4096.;
  */
  DAC1DAT = (( int) ( 4095.0 * ( ( double) gl_nAppliedMCoeff / 4096. * ( ( double) gl_un_RULAControl / ( double) RULA_MAX)))) << 16;  //(1.0) - ��� RULA � ������� ������� �� DAC0

  //DAC1DAT = (( int) ( 4096.0 * ( ( double) flashParamParam3 / 250. * 0.25) / 3.0)) << 16;  //(1.0) - ��� RULA � ������� ������� �� DAC0


  // ��� 2 (��������� ����)
  DAC2DAT = (( int) ( 4095.0 * ( ( double) flashParamStartMode / 250. ))) << 16;
  //DAC2DAT = (( int) ( 4095.0 * 1.25 / 2.5)) << 16;
}

void FirstDecrementCoeffCalculation( void) {
  char lb, hb;

  //****
  //������ ������ (��� ���� ������ �����)
  //****

  //�������� ����������� ����� ������� SA_TA (p0.4)
  //������ ��� ��� �������� ����������� ������ �� ���� ����
  while( !( GP0DAT & 0x10));

  // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
  // 1. ����������� � ������� ��� �������� �������������� ���������
  //��� �������� ������ ������� ���������� CntReady (p4.2)
  while( !( GP4DAT & 0x04)) {}

  //����������� ������� ���� ���� �������� �������������� ���������
  
  //GP1SET = 1 << (16 + 3);  //RDHBC (p1.3) = 1		WAY1
  GP1DAT |= 1 << (16 + 3);	//RDHBC (p1.3) = 1		WAY2
  
  pause( 1);                //�����

  //������
  hb = (( GP1DAT & BIT_5) >> 5) +
       ((( GP0DAT & BIT_7) >> 7) << 1) +
       ((( GP0DAT & BIT_1) >> 1) << 2) +
       ((( GP2DAT & BIT_3) >> 3) << 3) +
       ((( GP4DAT & BIT_6) >> 6) << 4) +
       ((( GP4DAT & BIT_7) >> 7) << 5) +
       ((( GP0DAT & BIT_6) >> 6) << 6) +
       ((( GP0DAT & BIT_2) >> 2) << 7);

  //GP1CLR = 1 << (16 + 3);  		//RDHBC (p1.3) = 0		WAY1
  GP1DAT &= ~( 1 << (16 + 3));		//RDHBC (p1.3) = 0		WAY2
  


  //����������� ������� ���� ���� �������� �������������� ���������
  //GP1SET = 1 << (16 + 4);  //RDLBC (p1.4) = 1		WAY1
  GP1DAT |= 1 << (16 + 4);  //RDLBC (p1.4) = 1			WAY2

  pause( 1);                //�����

  lb = (( GP1DAT & BIT_5) >> 5) +
       ((( GP0DAT & BIT_7) >> 7) << 1) +
       ((( GP0DAT & BIT_1) >> 1) << 2) +
       ((( GP2DAT & BIT_3) >> 3) << 3) +
       ((( GP4DAT & BIT_6) >> 6) << 4) +
       ((( GP4DAT & BIT_7) >> 7) << 5) +
       ((( GP0DAT & BIT_6) >> 6) << 6) +
       ((( GP0DAT & BIT_2) >> 2) << 7);

  //GP1CLR = 1 << (16 + 4);  	//RDLBC (p1.4) = 0		WAY1
  GP1DAT &= ~( 1 << (16 + 4));  //RDLBC (p1.4) = 0		WAY2

  //���������� ��� �����
  gl_ssh_angle_inc = lb + (hb << 8);




  // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
  // 2. � ������ ������ ������, ����������� � ������� ��� �������� �������������� ���������
  if( gl_b_SyncMode) {
    //��� �������� ������ ������� ���������� ANGLE_READY (p2.4)
    while( !( GP2DAT & 0x10)) {}

    //����������� ������� ���� ���� �������� ���������
    GP0SET = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 1
    pause( 1);                //�����

    //������
    hb =  (( GP1DAT & BIT_5) >> 5) +
          ((( GP0DAT & BIT_7) >> 7) << 1) +
          ((( GP0DAT & BIT_1) >> 1) << 2) +
          ((( GP2DAT & BIT_3) >> 3) << 3) +
          ((( GP4DAT & BIT_6) >> 6) << 4) +
          ((( GP4DAT & BIT_7) >> 7) << 5) +
          ((( GP0DAT & BIT_6) >> 6) << 6) +
          ((( GP0DAT & BIT_2) >> 2) << 7);

    GP0CLR = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 0


    //����������� ������� ���� ���� �������� �������������� ���������
    GP2SET = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 1
    pause( 1);                //�����

    lb = (( GP1DAT & BIT_5) >> 5) +
         ((( GP0DAT & BIT_7) >> 7) << 1) +
         ((( GP0DAT & BIT_1) >> 1) << 2) +
         ((( GP2DAT & BIT_3) >> 3) << 3) +
         ((( GP4DAT & BIT_6) >> 6) << 4) +
         ((( GP4DAT & BIT_7) >> 7) << 5) +
         ((( GP0DAT & BIT_6) >> 6) << 6) +
         ((( GP0DAT & BIT_2) >> 2) << 7);

    GP2CLR = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 0

    //���������� ��� �����
    gl_ssh_angle_hanger = lb + (hb << 8);

  }



  //�������� ������ ������� ������� SA_TA (p0.4)
  while( GP0DAT & 0x10);

  //****
  //� �����������
  //****
  do {
    //�������� ����������� ����� ������� SA_TA (p0.4)
    while( !(GP0DAT & 0x10));

    // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
    // 1. ����������� � ������� ��� �������� �������������� ���������
    //��� �������� ������ ������� ���������� CntReady (p4.2)
    while( !( GP4DAT & 0x04)) {}

    //����������� ������� ���� ���� �������� �������������� ���������
    //GP1SET = 1 << (16 + 3);  //RDHBC (p1.3) = 1   	WAY1
	  GP1DAT |= 1 << (16 + 3);  //RDHBC (p1.3) = 1   	WAY2

    pause( 1);                //�����

    //������
    hb = (( GP1DAT & BIT_5) >> 5) +
         ((( GP0DAT & BIT_7) >> 7) << 1) +
         ((( GP0DAT & BIT_1) >> 1) << 2) +
         ((( GP2DAT & BIT_3) >> 3) << 3) +
         ((( GP4DAT & BIT_6) >> 6) << 4) +
         ((( GP4DAT & BIT_7) >> 7) << 5) +
         ((( GP0DAT & BIT_6) >> 6) << 6) +
         ((( GP0DAT & BIT_2) >> 2) << 7);

    //GP1CLR = 1 << (16 + 3);  		//RDHBC (p1.3) = 0		WAY1
	  GP1DAT &= ~( 1 << (16 + 3));  	//RDHBC (p1.3) = 0		WAY2


    //����������� ������� ���� ���� �������� �������������� ���������
    //GP1SET = 1 << (16 + 4);  //RDLBC (p1.4) = 1		WAY1
	  GP1DAT |= 1 << (16 + 4);  //RDLBC (p1.4) = 1		WAY2

    pause( 1);                //�����

    lb = (( GP1DAT & BIT_5) >> 5) +
         ((( GP0DAT & BIT_7) >> 7) << 1) +
         ((( GP0DAT & BIT_1) >> 1) << 2) +
         ((( GP2DAT & BIT_3) >> 3) << 3) +
         ((( GP4DAT & BIT_6) >> 6) << 4) +
         ((( GP4DAT & BIT_7) >> 7) << 5) +
         ((( GP0DAT & BIT_6) >> 6) << 6) +
         ((( GP0DAT & BIT_2) >> 2) << 7);

    //GP1CLR = 1 << (16 + 4);  	//RDLBC (p1.4) = 0		WAY1
	  GP1DAT &= ~( 1 << (16 + 4));  //RDLBC (p1.4) = 0		WAY2

    //���������� ��� �����
    gl_ssh_angle_inc_prev = gl_ssh_angle_inc;
    gl_ssh_angle_inc = lb + (hb << 8);





    // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
    // 2. � ������ ������ ������, ����������� � ������� ��� �������� �������������� ���������

    //��� �������� ������ ������� ���������� ANGLE_READY (p2.4)
    while( !( GP2DAT & 0x10)) {}

    //����������� ������� ���� ���� �������� ���������
    GP0SET = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 1
    pause( 1);                //�����

    //������
    hb =  (( GP1DAT & BIT_5) >> 5) +
          ((( GP0DAT & BIT_7) >> 7) << 1) +
          ((( GP0DAT & BIT_1) >> 1) << 2) +
          ((( GP2DAT & BIT_3) >> 3) << 3) +
          ((( GP4DAT & BIT_6) >> 6) << 4) +
          ((( GP4DAT & BIT_7) >> 7) << 5) +
          ((( GP0DAT & BIT_6) >> 6) << 6) +
          ((( GP0DAT & BIT_2) >> 2) << 7);

    GP0CLR = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 0


    //����������� ������� ���� ���� �������� �������������� ���������
    GP2SET = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 1
    pause( 1);                //�����

    lb = (( GP1DAT & BIT_5) >> 5) +
         ((( GP0DAT & BIT_7) >> 7) << 1) +
         ((( GP0DAT & BIT_1) >> 1) << 2) +
         ((( GP2DAT & BIT_3) >> 3) << 3) +
         ((( GP4DAT & BIT_6) >> 6) << 4) +
         ((( GP4DAT & BIT_7) >> 7) << 5) +
         ((( GP0DAT & BIT_6) >> 6) << 6) +
         ((( GP0DAT & BIT_2) >> 2) << 7);

    GP2CLR = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 0

    gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;
    gl_ssh_angle_hanger = lb + (hb << 8);



    //�������� ��������� ����� ������� SA_TA (p0.4)
    while( (GP0DAT & 0x10));


    //������������ �����
    gl_dbl_Nsumm += fabs( ( double) gl_ssh_angle_inc - ( double) gl_ssh_angle_inc_prev);
    //TODO: �������� ����� dU?
    gl_dbl_Usumm += fabs( ( double) gl_ssh_angle_hanger - ( double) gl_ssh_angle_hanger_prev);

    gl_ssh_angle_inc_prev = gl_ssh_angle_inc;
    gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;

    gl_un_DecCoeffStatPoints++;

  } while( gl_un_DecCoeffStatPoints < DEC_COEFF_FIRST_CALCULATION_N);

  //������� ������ �������� ����. ������
  flashParamDecCoeff = ( short) ( gl_dbl_Nsumm / gl_dbl_Usumm * 65535.);

  gl_un_DecCoeffStatPoints = 0;
  gl_dbl_Nsumm = gl_dbl_Usumm = 0.;
}

void ThermoCalibrationCalculation( void)
{
  double x1, y1, x2, y2;
  //������� ���������� �������������
  if( flashParam_calibT1 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) && 
      flashParam_calibT1 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION) &&
      flashParam_calibT2 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) &&
      flashParam_calibT2 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {


    x1 = flashParam_calibT1 - THERMO_CALIB_PARAMS_BASE;
    x2 = flashParam_calibT2 - THERMO_CALIB_PARAMS_BASE;


    y1 = flashParamT1_TD1_val;
    y2 = flashParamT2_TD1_val;

    TD1_B = ( x2 * y1 - x1 * y2) / ( x1 - x2);
    TD1_K = ( y2 - y1) / ( x2 - x1);


    y1 = flashParamT1_TD2_val;
    y2 = flashParamT2_TD2_val;

    TD2_B = ( x2 * y1 - x1 * y2) / ( x1 - x2);
    TD2_K = ( y2 - y1) / ( x2 - x1);

    gl_bCalibrated = 1;
  }
  else
    gl_bCalibrated = 0;
}

void InitBaudRate115200() {
  /*
  //WORK PARAMETERS FOR PRECISE 115200
  COMDIV0 = 0x0B;
  COMDIV1 = 0x00;
  COMDIV2 = 0x0029;
  */
}

void InitBaudRate256000() {
  //DL = 41.78 * 10^6 / 2 ^ (CD=0) * 1/16 * 1/ (2*Baudrate)
  //RESULT = (M+N/2048) = 41.78 * 10^6 / ( 2 ^ (CD=0) * 16 *  2 * Baudrate)
  // M = Floor ( RESULT)
  // N = (RESULT - M) * 2048

  
  //WORK PARAMETERS FOR PRECISE 256000
  /*COMDIV0 = 0x05;
  COMDIV1 = 0x00;
  COMDIV2 = 0x8829;
  */
}

void InitBaudRate512000() {
  //DL = 41.78e6 / 16/2/512000 = 2.xxxxxx
  //M+N/2048 = 41.78e6 / 16/2/2/512000 = 1.275xxxxxx
  //M=1
  //N=563=0x233
  //      8             A
  //  1 0 0   0     1   0 1 0
  //  0 0 1   1     0   0 1 1
  //     3             3
  //COMDIV2 = 0x8A33
  
  //COMDIV0 = 2;
  //COMDIV1 = 0x00;
  //COMDIV2 = 0x8A33;
}

void InitBaudRate921600() {
  /****************************************************************************************** */
  /*   921 600 ********/
  /****************************************************************************************** */
  //DL = 41.78e6 / /921600 / (2^(CD=0)=1) / 16 / 2 = 1,416693793
  //DL=1
  //M+N/2048 = 41.78e6 / /921600 / (2^(CD=0)=1) / 16/ (DL=1) / 2 = 1,416693793
  //M=1
  //N=853=0x355
  //      8             B
  //  1 [0 0] [0     1]  0 1 1
  //  0  1 0   1     0   1 0 1
  //     5             5
  //COMDIV2 = 0x8B55
  
  COMDIV0 = 1;      //������� ���� �������� (DL)
  COMDIV1 = 0x00;   //������� ���� �������� (DL)
  COMDIV2 = 0x8B55; //16-��������� ������� �������� ��������
                    //15 - FBEN - ��� ���������� ������ ���������� � ������� ���������
                    //14-13 reserved
                    //12-11 M
                    //10-0  N
  
  /****************************************************************************************** */
}

void InitBaudRate930000() {
  /****************************************************************************************** */
  /*   930 000 ********/
  /****************************************************************************************** */
  //DL = 41.78e6 / 930 000 / (2^(CD=0)=1) / 16 / 2 = 1,404
  //DL=1
  //M+N/2048 = 41.78e6 / 930 000 / (2^(CD=0)=1) / 16 / (DL=1) / 2 = 1,404
  //M=1
  //N=827=0x33B
  //      8             B
  //  1 [0 0] [0     1]  0 1 1
  //  0  0 1   1     1   0 1 1
  //     3             B
  //COMDIV2 = 0x8B3B

  //COMDIV0 = 1;      //������� ���� �������� (DL)
  //COMDIV1 = 0x00;   //������� ���� �������� (DL)
  //COMDIV2 = 0x8B3B;
}

void InitBaudRate940000() {
  /****************************************************************************************** */
  /*   940 000 ********/
  /****************************************************************************************** */
  //DL = 41.78e6 / 940 000 / (2^(CD=0)=1) / 16 / 2 = 1,389
  //DL=1
  //M+N/2048 = 41.78e6 / 940 000 / (2^(CD=0)=1) / 16 / (DL=1) / 2 = 1,389
  //M=1
  //N=797=0x31D
  //      8             B
  //  1 [0 0] [0     1]  0 1 1
  //  0  0 0   1     1   1 0 1
  //     1             D
  //COMDIV2 = 0x8B1D

  //COMDIV0 = 1;      //������� ���� �������� (DL)
  //COMDIV1 = 0x00;   //������� ���� �������� (DL)
  //COMDIV2 = 0x8B1D;
}

void InitBaudRate950000() {
  /****************************************************************************************** */
  /*   950 000 ********/
  /****************************************************************************************** */
  //DL = 41.78e6 / 950 000 / (2^(CD=0)=1) / 16 / 2 = 1,374
  //DL=1
  //M+N/2048 = 41.78e6 / 950 000 / (2^(CD=0)=1) / 16 / (DL=1) / 2 = 1,374
  //M=1
  //N=766=0x2FE
  //      8             A
  //  1 [0 0] [0     1]  0 1 0
  //  1  1 1   1     1   1 1 0
  //     F             E
  //COMDIV2 = 0x8AFE

  //COMDIV0 = 1;      //������� ���� �������� (DL)
  //COMDIV1 = 0x00;   //������� ���� �������� (DL)
  //COMDIV2 = 0x8AFE;
}

void InitBaudRate960000() {
  /****************************************************************************************** */
  /*   960 000 ********/
  /****************************************************************************************** */
  //DL = 41.78e6 / 960 000 / (2^(CD=0)=1) / 16 / 2 = 1,36
  //DL=1
  //M+N/2048 = 41.78e6 / 960 000 / (2^(CD=0)=1) / 16 / (DL=1) / 2 = 1,36
  //M=1
  //N=737=0x2E1
  //      8             A
  //  1 [0 0] [0     1]  0 1 0
  //  1  1 1   0     0   0 0 1
  //     E             1
  //COMDIV2 = 0x8AE1

  //COMDIV0 = 1;      //������� ���� �������� (DL)
  //COMDIV1 = 0x00;   //������� ���� �������� (DL)
  //COMDIV2 = 0x8AE1;
}

void InitBaudRate990000() {
  //DL = 41.78e6 / 990 000 / (2^(CD=0)=1) / 16 / 2 = 1,319
  //DL=1
  //M+N/2048 = 41.78e6 / 990 000 / (2^(CD=0)=1) / 16 / (DL=1) / 2 = 1,319
  //M=1
  //N=653=0x28D
  //      8             A
  //  1 [0 0] [0     1]  0 1 0
  //  1  0 0   0     1   1 0 1
  //     8             D
  //COMDIV2 = 0x8A8D

  //COMDIV0 = 1;      //������� ���� �������� (DL)
  //COMDIV1 = 0x00;   //������� ���� �������� (DL)
  //COMDIV2 = 0x8A8D; //16-��������� ������� �������� ��������
                    //15 - FBEN - ��� ���������� ������ ���������� � ������� ���������
                    //14-13 reserved
                    //12-11 M
                    //10-0  N
}

void InitBaudRate1000000() {
  /****************************************************************************************** */
  /*   1 000 000 ********/
  /****************************************************************************************** */
  //DL = 41.78e6 / 1 000 000 / (2^(CD=0)=1) / 16 / 2 = 1,305625
  //DL=1
  //M+N/2048 = 41.78e6 / 1 000 000 / (2^(CD=0)=1) / 16 / (DL=1) / 2 = 1,305625
  //M=1
  //N=626=0x272
  //      8             A
  //  1 [0 0] [0     1]  0 1 0
  //  0  1 1   1     0   0 1 0
  //     7             2
  //COMDIV2 = 0x8A72

  //COMDIV0 = 1;      //������� ���� �������� (DL)
  //COMDIV1 = 0x00;   //������� ���� �������� (DL)
  //COMDIV2 = 0x8A72; //16-��������� ������� �������� ��������
                    //15 - FBEN - ��� ���������� ������ ���������� � ������� ���������
                    //14-13 reserved
                    //12-11 M
                    //10-0  N
  
  /****************************************************************************************** */
}

void testPike() {
  //� �������� ����� ������ ������ �� ����� P0.0
  GP0DAT |= 1 << ( 16);	//�������� ����� p0.0 set
  //for( i=0; i<100; i++);
  GP0DAT &= ~( 1 << ( 16));	//�������� ����� p0.0 clear
}

void main() {
  unsigned short ush_SA_check_time;

  short in_param_temp;
  int nRppTimer = 0;
  //unsigned char jchar = 0x30; 
  int time = 20000;
  int i;
  char lb, hb;
  short sSrc = -10;
  unsigned short unSrc = 10;

  int nSrc = 10;
  int nDst = 0;
  float fSrc = 10.2345;
  float fDst = 0.;
  /*char *pntr;
  char b1, b2, b3, b4;*/

  /*short q;
  unsigned short q1;
  unsigned int i1;*/

  char bSAFake = 0;
  int prt2val;

  double V_piezo;
  double temp_t;

  double dblDelta;

  char bSimpleDnDu = 0;
  signed short ssh_dN;
  signed short ssh_dU;
  char cChkSm;

  double dbl_N1, dbl_N2, dbl_U1, dbl_U2;

  float Coeff;

  double dbl_dN, dbl_dU;

  gl_bCalibProcessState = 0;    //0 - no calibration

                             //1 - processing min_t_point 1st thermosensor
                             //2 - processing min_t_point 2nd thermosensor
                             //3 - processing min_t_point 3rd thermosensor

                             //4 - processing max_t_point 1st thermosensor
                             //5 - processing max_t_point 2nd thermosensor
                             //6 - processing max_t_point 3rd thermosensor


  // Setup tx & rx pins on P1.0 and P1.1
  GP0CON = 0x00;
  GP1CON = 0x011;       //*** 0x011 = 0001 0001 = 00 01 00 01
                        //*** 01 - ����� ������� ������ ����� P1.0
                        //*** 00 - Reserved
                        //*** 01 - ����� ������� ������ ����� P1.1
                        //*** 00 - Reserved
                        //*** �������:
                        //***       00    01    10    11
                        //*** P1.0  GPIO  SIN   SCL0  PLAI[0]
                        //*** P1.1  GPIO  SOUT  SDA0  PLAI[1] 

  // Start setting up UART
  COMCON0 = 0x080;      // Setting DLAB

  // Setting DIV0 and DIV1 to DL calculated
  //InitBaudRate115200();
  //InitBaudRate256000();
  //InitBaudRate512000();
  InitBaudRate921600();
  //InitBaudRate930000();
  //InitBaudRate940000();
  //InitBaudRate950000();
  //InitBaudRate960000();
  //InitBaudRate990000();
  //InitBaudRate1000000();

  /****************************************************************************************** */
  COMCON0 = 0x007;  // Clearing DLAB

  /*
  //Test Baudrate
  while(1) {
    printf("921 600 Baudrate is great!\n");
  }
  */

#ifdef DEBUG
  printf("T7-SLG. Software version: %d.%d.%d\n", VERSION_MAJOR, VERSION_MIDDLE, VERSION_MINOR);
  printf("DEBUG MODE\n");
  i = LONG_OUTPUT_PACK_LEN;
  printf("LONG_PACK: %d\n", i);
  i = SHORT_OUTPUT_PACK_LEN;
  printf("SHORT_PACK: %d\n", i);
#endif

  //**********************************************************************
  // ������������ ��������
  //**********************************************************************	
  GP0DAT = 0x01000000;
  GP0DAT ^= (1 << 16);
  GP0CLR = (1 << 16);

#ifdef DEBUG
  printf("DEBUG: GPIO lines direction configuration...");
#endif

  //**********************************************************************
  // ������������ GPIO (��������������� ������/������� ������ ����������)
  //**********************************************************************
  GP0DAT |= 1 << (24 + 0);  //������������ �������� ����� (��������) (p0.0) � �������� ������
  GP0DAT |= 1 << (24 + 3);  //������������ ����� RDHBANGLE           (p0.3) � �������� ������
  GP0DAT |= 1 << (24 + 5);  //������������ ����� RP_P                (p0.5) � �������� ������

  GP1DAT |= 1 << (24 + 3);  //������������ ����� RdHbc               (p1.3) � �������� ������
  GP1DAT |= 1 << (24 + 4);  //������������ ����� RdLbc               (p1.4) � �������� ������
  GP1DAT |= 1 << (24 + 7);  //������������ ����� EN_RP               (p1.7) � �������� ������

  GP2DAT |= 1 << (24 + 2);  //������������ ����� EN_VB               (p2.2) � �������� ������
  GP2DAT |= 1 << (24 + 5);  //������������ ����� RDLBANGLE           (p2.5) � �������� ������

  GP3DAT |= 1 << (24 + 0);  //������������ ����� TactNoise0          (p3.0) � �������� ������
  GP3DAT |= 1 << (24 + 1);  //������������ ����� TactNoise1          (p3.1) � �������� ������
  GP3DAT |= 1 << (24 + 3);  //������������ ����� RD_AMPL_T_CODE      (p3.3) � �������� ������
  GP3DAT |= 1 << (24 + 5);  //������������ ����� OutLnfType          (p3.5) � �������� ������

  GP4DAT |= 1 << (24 + 0);  //������������ ����� ONHV                (p4.0) � �������� ������
  GP4DAT |= 1 << (24 + 1);  //������������ ����� OFF3KV              (p4.1) � �������� ������
  GP4DAT |= 1 << (24 + 3);  //������������ ����� Reset               (p4.3) � �������� ������


#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: GPIO lines values configuration...");
#endif
  GP4DAT |=    1 << (16 + 0);  //ONHV        (p4.0) = 1 (���������)
  GP4DAT &= ~( 1 << (16 + 1)); //OFF3KV      (p4.1) = 0 (��������)

  GP2DAT &= ~( 1 << (16 + 2));  //EN_VB       (p2.2) = 0

  GP1DAT &= ~( 1 << (16 + 7));  //EN_RP       (p1.7) = 0

  GP0DAT &= ~( 1 << (16 + 5));  //RP_P        (p0.5) = 0


#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: Pulsing Reset signal...");
#endif

  //**********************************************************************
  // ������� ������� Reset
  //**********************************************************************
  GP4DAT |= 1 << (16 + 3);      //Reset set
  for( i=0; i<100; i++);
  GP4DAT &= ~( 1 << (16 + 3));  //Reset clear

#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: Enabling UART0 FIQ...");
#endif

  //**********************************************************************
  // ��������� ���������� �� UART0
  //**********************************************************************	
  FIQEN |= UART_BIT;
  COMIEN0 |= 1;

#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: Internal ADC configuration...");
#endif

  //**********************************************************************
  // ������������ ���
  //**********************************************************************	
  ADCCON = 0x20;            // �������� ���
  while (time >=0)          // ���� ��������� � datasheet ����� (5�����) ��� ������� ��������� ���
    time--;

  ADCCON = 0x624;           // ������������� ���:
                            // ����������� �������������� � ����������� �� ���������
                            // ������������ ����
                            // (���������� ������� ���)
                            // ����������� ADCBusy
                            // ��� ������ ��������������
                            // �������������� 8 ������
                            // ������������ [fADC / 4]
  ADCCP = 0x01;             // ������ 1�� ����� ���
  REFCON = 0x01;            // ���������� �������� �������� ����������: (����: 0x02 ����������� ����������� 3-���������� ��� � ������ VREF.   ����� 0x01 ���������� 2.5V)

#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: Timers configuration...");
#endif

  //**********************************************************************
  // ������������ Timer0
  //**********************************************************************
  T0CON = 0x80;           //core clocking

  //**********************************************************************
  // ������������ Timer1
  //**********************************************************************
  T1CON = 0x2C0;          //32kHz clocking
  T1LD = 0x100000;

  //**********************************************************************
  // ������������ Timer2
  //**********************************************************************
  T2CON = 0x2C0;          //32kHz clocking
  T2LD = 0x100000;

#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: FlashEE configuration...");
#endif

  //**********************************************************************
  // ������������ ����-������ FlashEE
  //**********************************************************************
  flashEE_configure();

#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: loading flash params.\n");
#endif

  //**********************************************************************
  // �������� ���������� �� ����-������
  //**********************************************************************
  load_params();
  ThermoCalibrationCalculation();

#ifdef DEBUG
  printf("DEBUG: DAC Configuration...");
#endif




  //**********************************************************************
  // ������������ � �������� ���
  //**********************************************************************
  // ��� 0
  DAC0CON = 0x12;       // ������������ ��� 0:
                        // �������� 0-DAC(REF)
                        // �������� ���������� ���0 ����������� �� ������� ������ ����� ����

  // ��� 1
  DAC1CON = 0x12;       // ������������ ��� 1:
                        // �������� 0-DAC(REF)
                        // �������� ���������� ���1 ����������� �� ������� ������ ����� ����

  // ��� 2
  DAC2CON = 0x12;       // ������������ ��� 2:
                        // �������� 0-DAC(REF)
                        // �������� ���������� ���2 ����������� �� ������� ������ ����� ����


  //�������� ������������ RULA � RULM
  DAC0DAT = (( int) ( 4095.0 * 2.5 / 2.5)) << 16; //�������� �� ������ ���0 2.5 �
  DAC1DAT = (( int) ( 4095.0 * 2.5 / 2.5)) << 16; //�������� �� ������ ���1 2.5 �
  DAC2DAT = (( int) ( 4095.0 * 1.25 / 2.5)) << 16; //( ( double) flashParamStartMode / 250. * 2.5) / 3.0)) << 16;

#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: Hangerup configure...");
#endif

  //**********************************************************************
  // ������������ ��������� ������� (TactNoise0 � TactNoise1)
  //**********************************************************************
  configure_hanger();

  //**********************************************************************
  // ��������� ������������
  //**********************************************************************
  GP2DAT |= ( 1 << (16 + 2));  //EN_VB   (p2.2) = 1

  //**********************************************************************
  // �������� �������� ������������
  //**********************************************************************
#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: Hangerup vibration control...\n");
#endif

#ifdef SKIP_START_CHECKS
  #ifdef DEBUG
    pause( 16384*1);      //0.5 sec pause
    printf("DEBUG: Hangerup vibration control: SKIPPED\n");
  #endif
#else

  dStartAmplAngCheck = ( double) flashParamAmplAngMin1 / 65535. * 6.;
  dStartAmplAngCheck = 0.25;
  prt2val = T2VAL;
  ADCCP = 0x06;   //AmplAng channel
  while( 1) {
    ADCCON |= 0x80;
    while (!( ADCSTA & 0x01)){}
    gl_ssh_ampl_angle = (ADCDAT >> 16);

  #ifdef DEBUG
    printf("DEBUG: Hangerup vibration control: Measured: %.02f    CheckValue: %.02f\n", ( double) gl_ssh_ampl_angle / 4095. * 2.5 / 0.5, dStartAmplAngCheck );
  #endif

    if( ( ( double) gl_ssh_ampl_angle / 4095. * 2.5 / 0.5) > dStartAmplAngCheck) {
      //SUCCESS

      #ifdef DEBUG
        printf("DEBUG: Hangerup vibration control: successfully passed\n");
      #endif

      break;
    }

    if( ( double) (( prt2val + T2LD - T2VAL) % T2LD) / 32768. > 5.0) {
      #ifdef DEBUG
        printf("DEBUG: Hangerup vibration control: FAILED\n");
      #endif
     deadloop_no_hangerup();
    }
  }
#endif

  if( gl_ushFlashParamLastRULA != 0)
    gl_un_RULAControl = gl_ushFlashParamLastRULA;    //��������������� ��������� �������� RULA
  else
    gl_un_RULAControl = 1400;

  //if( gl_ushFlashParamLastRULM != 0)
  //  gl_nAppliedMCoeff = gl_ushFlashParamLastRULM;    //��������������� ��������� �������� RULM
  //else
    gl_nAppliedMCoeff = 4096;                    //���� ���������� �������� �� ���� ������ 4096 � ����� RULM ������ ���� ����� -- �� ������� ��������

  DACConfiguration();

  //**********************************************************************
  // ������ ������
  //**********************************************************************
#ifdef DEBUG
  printf("DEBUG: Laser fireup...\n");
#endif

#ifdef SKIP_START_CHECKS
  #ifdef DEBUG
    pause( 16384*1);      //0.5 sec pause
    printf("DEBUG: Laser fireup: SKIPPED\n");
  #endif
#else



  while( 1) {
    //��������� �������� (������) (�������� by default, �� �� ��� ������ ��������� (�� ������� 5 ������ �������)
    GP4DAT &= ~( 1 << (16 + 1));  //OFF3KV (p4.1) = 0

    //����� �������
    GP4DAT &= ~( 1 << (16 + 0));  //ONHV   (p4.0) = 0

    //�������� 0,5 �������
    pause( 16384);

    //�������� ��� I1
    ADCCP = 0x03;
    pause( 10);
    ADCCON |= 0x80;
    while (!( ADCSTA & 0x01)){}     // ������� ����� �������������� ���
    gl_ssh_current_1 = (ADCDAT >> 16);

    //�������� ��� I2
    ADCCP = 0x04;
    pause( 10);
    ADCCON |= 0x80;
    while (!( ADCSTA & 0x01)){}     // ������� ����� �������������� ���
    gl_ssh_current_2 = (ADCDAT >> 16);

    /*if( ( ( double) gl_ssh_current_1 / 4096. * 3. / 3.973 < ( double) flashParamI1min / 65535. * 0.75)  ||
        ( ( double) gl_ssh_current_2 / 4096. * 3. / 3.973 < ( double) flashParamI2min / 65535. * 0.75)) {*/

#ifdef DEBUG
	printf("DEBUG: Laser fireup: Measured I1=%.02f   Measured I2=%.02f\n", ( 2.5 - ( double) gl_ssh_current_1 / 4096. * 2.5) / 2.5, 
													( 2.5 - ( double) gl_ssh_current_2 / 4096. * 2.5) / 2.5);
#endif

    if( ( ( 2.5 - ( double) gl_ssh_current_1 / 4096. * 2.5) / 2.5  < ( double) flashParamI1min / 65535. * 0.2)  ||
        ( ( 2.5 - ( double) gl_ssh_current_2 / 4096. * 2.5) / 2.5 < ( double) flashParamI2min / 65535. * 0.2)) {
        //�� ��������

        //������� 3kV ����� (������� 800V)
        GP4DAT |= ( 1 << (16 + 1));   //OFF3KV (p4.1) = 1

        //�������������� � ��������� ����� ������� �������
        if( ++nFiringTry >= 25) {

          //5 ����� �� 5 ������� ������� �� ��������� - ������������ � ������� ����

          //��������� ������� 800V
          GP4DAT |= ( 1 << (16 + 0));   //ONHV   (p4.0) = 1

          //��������� ������ 3kV
          GP4DAT |= ( 1 << (16 + 1));   //OFF3KV (p4.1) = 1

          #ifdef DEBUG
            printf("DEBUG: Laser fireup: FAILED\n");
          #endif

          deadloop_no_firing();
        }

        if( ( nFiringTry % 5)) {
          //��� � ���� ������� ��������� 800V �����

          //��������� ������� 800V
          GP4DAT |= ( 1 << (16 + 0));   //ONHV   (p4.0) = 1

          //�������� 0,5 ������� (� ����� � ����������� ��� ���� 1 ���, ����� ������� �� 5 ���������)
          pause( 16384);
        }

        //�������� 0,5 �������
        pause( 16384);
    }
    else {
      //SUCCESS! ��������
      GP4DAT |= 1 << (16 + 1);	    //OFF3KV (p4.1) = 1     (��������� ������)

      #ifdef DEBUG
        printf("DEBUG: Laser fireup: successfully passed\n");
      #endif

      break;
    }
  }
#endif

  //����� ������������ � ������� ����������� ���������
  GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1
  pause( 3277);                  //pause 100msec

  //��������� ������������� ���������
  GP1DAT |= ( 1 << (16 + 7));   //EN_RP   (p1.7) = 1
  

  GP0DAT &= ~( 1 << (16 + 5));  //RP_P   (p0.5) = 0


  //**********************************************************************
  // �������� ��������� �������
  //**********************************************************************

#ifdef DEBUG
  printf("DEBUG: Tacting signal check...\n");
#endif

#ifdef SKIP_START_CHECKS
  gl_b_SyncMode = 1;

  #ifdef DEBUG
    pause( 16384*1);      //0.5 sec pause
    printf("SKIPPED\n");
    printf("DEBUG: Working in asynchronous mode\n");
  #endif

#else

  //**********************************************************************
  //������������
  //**********************************************************************
  


  //**********************************************************************
  // �������� ������� ����� TA. ���� ��� �� ����� � ������� 0.5 ��� - ���������� � ���������� �����
  //**********************************************************************
  prt2val = T2VAL;

  gl_b_SyncMode = 0;

  while(1) {

  if( gl_b_SyncMode == 0) {
    //������� ������� ������� �� ���� TA(p2.7)
    if( GP2DAT & 0x80)
      gl_b_SyncMode = 1;
    } else if( gl_b_SyncMode == 1) {
      //������� ������� ������� �� ���� TA(p2.7)
      if( !(GP2DAT & 0x80))
        gl_b_SyncMode = 2;
    } else if( gl_b_SyncMode == 2) {
      //������� ������� ������� �� ���� TA(p2.7)
      if( GP2DAT & 0x80) {
        gl_b_SyncMode = 1;
        #ifdef DEBUG
          printf("DEBUG: Got 1-0-1 on P2.7 so working in asynchronous mode\n");
        #endif

        GP3DAT |= ( 1 << (16 + 5));   //OutLnfType (p3.5) = 1  ���������� ����������� ������������

        break;
      }
    }

    //��� ��������� 0.5 ���
    if( ( double) (( prt2val + T2LD - T2VAL) % T2LD) / 32768. > 0.5) {

      #ifdef DEBUG
        printf("DEBUG: 0.5 sec passed. Zeroing I/O pin 39 (p3.5)...");
      #endif

      GP3DAT &= ~(1 << (16 + 5));     //OutInfType (p3.5) = 0    ���������� ���������� ������������
      GP3CLR = (1 << (16 + 5));       //�����

      #ifdef DEBUG
        printf("done\n");
      #endif

      gl_b_SyncMode = 0;
      break;
    }
  }

  if( !gl_b_SyncMode) {
    //�� ������������� � ���������� ����� - ���� �� ��������� ������ ������������ (���� SA)
    #ifdef DEBUG
      printf("DEBUG: Passed 0.5 sec with TA (P2.7) with no changes. So working in synchronous mode\n");
      printf("DEBUG: Waiting for SA signal on p0.4\n");
    #endif

    //�������� � ���������� ������ - �������� ������� SA
    prt2val = T2VAL;
    while( 1) {
      //������� ������� ������� ������� �� ���� SA_TA (p0.4)
      if( GP0DAT & 0x10) {
        //SA ������ - ��� ��
        #ifdef DEBUG
          printf("DEBUG: Got SA signal! SA TEST PASSED\n");
        #endif
        break;
      }

      //��� ��������� 0.5 ���
      if( ( double) (( prt2val + T2LD - T2VAL) % T2LD) / 32768. > 0.5) {
        //SA �� ������ � ������� 0.5 ��� - � ��� ��� ������������ - ����� � deadloop

        #ifdef DEBUG
          printf("FAILED\n");
        #endif

        deadloop_no_tact( ERROR_NO_TACT_SIGNAL);
        break;
      }
    }
  }
  
  /*
  //�������������� ���������� �����
  gl_b_SyncMode = 0;
  GP3DAT &= ~(1 << (16 + 5));	//OutLnfType (p3.5) = 0
  */

  #ifdef DEBUG
    printf("passed\n");
    printf("DEBUG: Internal ADC start...");
  #endif

#endif

  //**********************************************************************
  // ������ �������������� ���
  //**********************************************************************
  ADCChannel = 0;
  pause( 10);
  ADCCP = 0x00;
  ADCCON |= 0x80;

#ifdef DEBUG
  printf("passed\n");
  printf("DEBUG: Skipping first SA Tact...");
#endif

  //**********************************************************************  
  //������� ����� SA_TA (p0.4)
  //**********************************************************************
#ifndef SKIP_START_CHECKS
  while( (GP0DAT & 0x10));
  while( !(GP0DAT & 0x10));
  while( (GP0DAT & 0x10));
#endif

  //**********************************************************************
  

#ifdef DEBUG
  printf("done\n");


  //���������� ��������������� ������������ ������
  if( gl_b_SyncMode)
    printf("DEBUG: Calculation of first decrement coefficient\n");  
#endif

/*
  if( gl_b_SyncMode)
    FirstDecrementCoeffCalculation();
*/

#ifdef DEBUG
  printf("VALUE=%.2f  ", flashParamDecCoeff / 65535.);
#endif

#ifdef DEBUG
  if( gl_b_SyncMode)
    printf("passed\n");
  
  printf("DEBUG: Configuration passed. Main loop starts!\n");

#endif

  //������������ ���������� �������� ���������� ������� ���������
  gl_dblAmplMean = 0.;
  gl_nAmplMeanCounter = 0;

  //������������� ���������� �������� ���������� ������� ������������ ������
  gl_dblMeanAbsDn = 0.;
  gl_dblMeanAbsDu = 0.;
  gl_nMeanDecCoeffCounter = 0;

  nT2RepeatBang = ( T2VAL - 32768) % T2LD;
  gl_un_PrevAmplRegulationT2 = T2VAL;
  gl_un_PrevT2DecCoeffCalc = T2VAL;


  //**********************************************************************
  //**********************************************************************
  //******************* �������� ���� ������ ��������� *******************
  //**********************************************************************
  //**********************************************************************
  //**********************************************************************

  //gl_un_RULAControl = 2457;    //2457 = 1.500 V
  if( gl_ushFlashParamLastRULA != 0)
    gl_un_RULAControl = gl_ushFlashParamLastRULA;    //��������������� ��������� �������� RULA
  else
    gl_un_RULAControl = 1400;                    //LIE!!   ���� ���������� �������� �� ����, �� ������ 2457 = 1.500 V

  gl_nAppliedMCoeff = 4096;


  DACConfiguration();


  while(1) {
    if( bSimpleDnDu == 1) {

      while( !(GP0DAT & 0x10)) {}

      if( gl_b_SA_Processed == 0) {
        // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
        // 1. ����������� � ������� ��� �������� �������������� ���������
        //��� �������� ������ ������� ���������� CntReady (p4.2)
        //printf( "CNT_READY: %c", ( GP4DAT & 0x04) ? '1' : '0');
        while( !( GP4DAT & 0x04)) {}
        //putchar( ( GP4DAT & 0x04) ? '1' : '0');



        //����������� ������� ���� ���� �������� �������������� ���������
        //GP1SET = 1 << (16 + 3);  //RDHBC (p1.3) = 1			WAY1
        GP1DAT |= 1 << (16 + 3);  //RDHBC (p1.3) = 1			WAY2

        //pause( 1);                //�����

        //������
        hb = (( GP1DAT & BIT_5) >> 5) +
          ((( GP0DAT & BIT_7) >> 7) << 1) +
          ((( GP0DAT & BIT_1) >> 1) << 2) +
          ((( GP2DAT & BIT_3) >> 3) << 3) +
          ((( GP4DAT & BIT_6) >> 6) << 4) +
          ((( GP4DAT & BIT_7) >> 7) << 5) +
          ((( GP0DAT & BIT_6) >> 6) << 6) +
          ((( GP0DAT & BIT_2) >> 2) << 7);

        //GP1CLR = 1 << (16 + 3);  //RDHBC (p1.3) = 0				WAY1
        GP1DAT &= ~( 1 << (16 + 3));  //RDHBC (p1.3) = 0			WAY2


        //����������� ������� ���� ���� �������� �������������� ���������
        //GP1SET = 1 << (16 + 4);  //RDLBC (p1.4) = 1		WAY1
        GP1DAT |= 1 << (16 + 4);  //RDLBC (p1.4) = 1		WAY2

        //pause( 1);                //�����

        lb = (( GP1DAT & BIT_5) >> 5) +
            ((( GP0DAT & BIT_7) >> 7) << 1) +
            ((( GP0DAT & BIT_1) >> 1) << 2) +
            ((( GP2DAT & BIT_3) >> 3) << 3) +
            ((( GP4DAT & BIT_6) >> 6) << 4) +
            ((( GP4DAT & BIT_7) >> 7) << 5) +
            ((( GP0DAT & BIT_6) >> 6) << 6) +
            ((( GP0DAT & BIT_2) >> 2) << 7);

        //GP1CLR = 1 << (16 + 4);  //RDLBC (p1.4) = 0		WAY1
        GP1DAT &= ~( 1 << (16 + 4));  //RDLBC (p1.4) = 0		WAY2

        //���������� ��� �����
        gl_ssh_angle_inc_prev = gl_ssh_angle_inc;
        gl_ssh_angle_inc = lb + (hb << 8);

        // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
        // 2. � ������ ������ ������, ����������� � ������� ��� �������� �������������� ���������
        if( gl_b_SyncMode) {
          //��� �������� ������ ������� ���������� ANGLE_READY (p2.4)

          //printf( "\tANGLE_READY: %c", ( GP2DAT & 0x10) ? '1' : '0');
          while( !( GP2DAT & 0x10)) {}
          //putchar( ( GP2DAT & 0x10) ? '1' : '0');

          //����������� ������� ���� ���� �������� ���������
          GP0SET = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 1
          //pause( 1);                //�����

          //������
          hb =  (( GP1DAT & BIT_5) >> 5) +
                ((( GP0DAT & BIT_7) >> 7) << 1) +
                 ((( GP0DAT & BIT_1) >> 1) << 2) +
                 ((( GP2DAT & BIT_3) >> 3) << 3) +
                 ((( GP4DAT & BIT_6) >> 6) << 4) +
                 ((( GP4DAT & BIT_7) >> 7) << 5) +
                 ((( GP0DAT & BIT_6) >> 6) << 6) +
                 ((( GP0DAT & BIT_2) >> 2) << 7);

          GP0CLR = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 0


          //����������� ������� ���� ���� �������� �������������� ���������
          GP2SET = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 1
          //pause( 1);                //�����

          lb = (( GP1DAT & BIT_5) >> 5) +
                ((( GP0DAT & BIT_7) >> 7) << 1) +
               ((( GP0DAT & BIT_1) >> 1) << 2) +
               ((( GP2DAT & BIT_3) >> 3) << 3) +
               ((( GP4DAT & BIT_6) >> 6) << 4) +
               ((( GP4DAT & BIT_7) >> 7) << 5) +
               ((( GP0DAT & BIT_6) >> 6) << 6) +
               ((( GP0DAT & BIT_2) >> 2) << 7);

          GP2CLR = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 0

          //���������� ��� �����
          gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;
          gl_ssh_angle_hanger = lb + (hb << 8);

          if( gl_ssh_angle_hanger & 0x2000) {
            gl_ssh_angle_hanger = ( gl_ssh_angle_hanger & 0x3FFF) | 0xC000;
          }
          else
            gl_ssh_angle_hanger = ( gl_ssh_angle_hanger & 0x3FFF);

        }

        cChkSm = 0;

        putchar_nocheck( 0xCC);
        //putchar_nocheck( 0xDD);

        //N
        //putchar_nocheck( gl_ssh_angle_inc & 0xff);
        //putchar_nocheck( (gl_ssh_angle_inc & 0xff00) >> 8);

        

        ssh_dN = gl_ssh_angle_inc - gl_ssh_angle_inc_prev;
        ssh_dU = gl_ssh_angle_hanger- gl_ssh_angle_hanger_prev;

        //dN
        putchar_nocheck( ssh_dN & 0xff);              cChkSm += ( ssh_dN & 0xff);
        putchar_nocheck( ( ssh_dN & 0xff00) >> 8);    cChkSm += (( ssh_dN & 0xff00) >> 8);

        //U
        //putchar_nocheck( gl_ssh_angle_hanger & 0xff);
        //putchar_nocheck( ( gl_ssh_angle_hanger & 0xff00) >> 8);

        //dU
        putchar_nocheck( ssh_dU & 0xff);              cChkSm += ( ssh_dU & 0xff);
        putchar_nocheck( ( ssh_dU & 0xff00) >> 8);    cChkSm += (( ssh_dU & 0xff00) >> 8);

        //Counter
        nSentPacksCounter = (++nSentPacksCounter) % 0xFF;
        putchar_nocheck( nSentPacksCounter & 0xff);   cChkSm += ( nSentPacksCounter & 0xff);

        putchar_nocheck( cChkSm & 0xFF);

        //��������� ���� � ��� ��� ������� ������� ������� SA �� ����������
        gl_b_SA_Processed = 1;
      }
      else {
        while( (GP0DAT & 0x10)) {}
        gl_b_SA_Processed = 0;
      }
    }

    //**********************************************************************
    // ��������� ������ �������� ������
    //**********************************************************************
    if( pos_in_in_buf == IN_COMMAND_BUF_LEN) {
      switch( input_buffer[0]) {
        case 0: //���������� ��� ���������
          flashParamAmplitudeCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
          
          /*cRULAControl = ( RULA_MAX - RULA_MIN) / 2;
          delta = ( RULA_MAX - RULA_MIN) / 4;*/

          /*gl_un_RULAControl = 64;*/

          //��� 55��������� ��������� (��� ������������� 55*2.9 = 160") � ��� �������� (4096)
          //gl_un_RULAControl = ( unsigned int) ( 4095. / 55. * ( double) flashParamAmplitudeCode);

          //gl_un_RULAControl = ( RULA_MAX - RULA_MIN) / 2;
          //gl_un_RULAControl = 1400;
          //gl_un_RULAControl = 1000;

          //nDelta = ( RULA_MAX - RULA_MIN) / 2;
          //gl_nDelta = 10;

          //��������� ��������� (����� ��� ������� ������)
          gl_nAppliedMCoeff = 4096;

          //���������� ���������� ������� ���������
          gl_dblAmplMean = 0;
          gl_nAmplMeanCounter = 0;

          nT2RepeatBang = T2VAL;
          gl_nAmplStabStep = 0;
          //gl_nDelta = 4;
        break;

        case 1: //���������� ��� ����� ���������
          flashParamTactCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          configure_hanger(); nSentPacksRound = LONG_OUTPUT_PACK_LEN;

          gl_dblAmplMean = 0;
          gl_nAmplMeanCounter = 0;

          nT2RepeatBang = T2VAL;
          gl_nAmplStabStep = 0;
          //gl_nDelta = 4;
        break;

        case 2: //���������� ����������� M
          flashParamMCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          gl_nAppliedMCoeff = 4096;
          DACConfiguration();
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;

          gl_dblAmplMean = 0;
          gl_nAmplMeanCounter = 0;

          nT2RepeatBang = T2VAL;
          gl_nAmplStabStep = 0;
          //gl_nDelta = 4;
        break;

        case 3: //���������� ��������� ����
          flashParamStartMode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          DACConfiguration();
          GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1
          gl_n_PerimeterReset = 1;
          nRppTimer = T1VAL;
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;
        
        case 4: //���������� ����������� ��� I1
          flashParamI1min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
#ifdef DEBUG
          printf("DEBUG: Input command (0x04 - SetControlI1) accepted. Param: 0x%04x\n", flashParamI1min);
#endif
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 5: //���������� ����������� ��� I2
          flashParamI2min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 6: //���������� 1�� ������� ������� AmplAng
          flashParamAmplAngMin1 = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 7: //���������� ����������� ������
          flashParamDecCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;

		  //������� ���������� ���������� ����������� ��� ��������� �� "�� ����"
		  gl_nMeanDecCoeffCounter = 0;
		  gl_dblMeanAbsDn = 0.;
		  gl_dblMeanAbsDu = 0.;
        break;

        case 8: //���������� SA ����
          flashParamSignCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);

          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 9: //� ������. ������ �������� ����� SA (�������� ����� dU)
          bAsyncDu = 1;
        break;

        case 10: //� ������. ������ ��������� ����� dU (�������� ����� SA)
          bAsyncDu = 0;
        break;

        case 11: //���������� ������������� (�������� - �������� �����������)
          gl_bCalibrated = 0;
          in_param_temp  = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          if( flashParam_calibT1 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) && 
              flashParam_calibT1 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
              //� ��� ���� ���������� ����������� ����� ����������

              //������� �� ��, ����� �� ������ ��� ����� ������ ����
              if( in_param_temp == flashParam_calibT1) {
                nSentPacksRound = LONG_OUTPUT_PACK_LEN;
                break;
              }

              if( flashParam_calibT2 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION)  &&
                  flashParam_calibT2 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
                //� ��� ���� ���������� ����������� � ������������ ����� ����������
                //��������� ����� ���� ��������
                if( in_param_temp < flashParam_calibT1) {
                  //���� �������� �����������
                  gl_bCalibProcessState = 1;
                  flashParam_calibT1 = in_param_temp;
                }
                else {
                  //���� �������� ������������
                  gl_bCalibProcessState = 3;
                  flashParam_calibT2 = in_param_temp;
                }
              }
              else {
                //� ��� ���� ���������� ����������� ����� ����������, �� ��� ���������� ������������
                gl_bCalibProcessState = 3;
                flashParam_calibT2 = in_param_temp;
              }

          }
          else {
            //� ��� ��� ���� ����������� �����!! ������ ��� ����� ����������� :)
            flashParam_calibT1 = in_param_temp;
            gl_bCalibProcessState = 1;
          }
        break;

        case 12:    //������� ������ ������ ���������� �������������
          gl_bCalibrated = 0;
          flashParam_calibT1 = 0;
          flashParamT1_TD1_val = 0;
          flashParamT1_TD2_val = 0;

          flashParam_calibT2 = 0;
          flashParamT2_TD1_val = 1;
          flashParamT2_TD2_val = 1;

          SaveThermoCalibPoint();
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 14:    //������� ������ �������� ������
          in_param_temp  = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          flashParamPhaseShift = in_param_temp;
          //SendPhaseShift();
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 15:    //������� ���������� ������
          GP4DAT |= 1 << (16 + 0);      //ONHV       (p4.0) = 1
          GP4DAT |= 1 << (16 + 1);      //OFF3KV     (p4.1) = 1
        break;

        case 16:    //������� ���������� �����������
          GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1
          nRppTimer = 0;
          gl_n_PerimeterReset = 1;
        break;

        case 17:    //������� ��������� �����������
          GP0DAT &= ~( 1 << (16 + 5));  //RP_P   (p0.5) = 0
          nRppTimer = T1VAL;
          gl_n_PerimeterReset = 2;
        break;

        case 18:    //������� ������ �����������
          GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1
          nRppTimer = T1VAL;
          gl_n_PerimeterReset = 1;
        break;

        case 49: //������ ����������
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 50: //��������� ��������� �� ���� ������
#ifdef DEBUG
          printf("DEBUG: Input command (0x50 - SaveParams).\n");
#endif
          gl_ushFlashParamLastRULA = gl_un_RULAControl;
          save_params();
        break;

        case 51: //������������� ��������� �� ����-������ � �������� ��
          load_params(); nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 100: //FAKE! ���������� ��� ������ (���� ������ �������)
          gl_c_EmergencyCode = input_buffer[1];
        break;

        case 'S':
          bSimpleDnDu ^= 1;
        break;
/*
        case 'S':
#ifdef DEBUG
          printf("DEBUG: Input SECRET_SET command. Setting control I1 to 0x100\n");
          flashParamI1min = 0x100;
          printf("DEBUG: Here it is: 0x%04x\n", flashParamI1min);
#endif
          break;

        case 'T':
#ifdef DEBUG
          printf("DEBUG: Input SECRET_SAVE command. Calling save_params().\n");
          save_params();
#endif
          break;
*/

      }


      pos_in_in_buf = 0;
    }

    if( bSimpleDnDu == 1) {
      continue;
    }



    /*
    //�������� ����� - �������� ����� 10(5?) ��� � �������
    prt2val = T2VAL;
    while( (( 0x1000 + T2VAL - prt2val) % 0x1000) < 3276);
    bSAFake ^= 1;
    */

    //if( bSAFake) {	// �� ���� SA_TA (P0.4) ���� FAKE ������
    if( GP0DAT & 0x10) {	//�� ���� SA_TA (P0.4) ���� ������
      /*
      #ifdef DEBUG
        printf("DEBUG: got tact synchro signal! %d\n", gl_b_SA_Processed);
      #endif
      */
      if( gl_b_SA_Processed == 0) { //���� � ���� SA ����� �� ��� ��� �� ������������

        testPike();

        //������� ��������� TA_SA (��� ���������� ������������)
        gl_ssh_SA_time = ( T1LD + gl_ssh_prT1VAL - T1VAL) % T1LD;
        gl_ssh_prT1VAL = T1VAL;

        // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
        // 1. ����������� � ������� ��� �������� �������������� ���������
        //��� �������� ������ ������� ���������� CntReady (p4.2)
        while( !( GP4DAT & 0x04)) {}

        //����������� ������� ���� ���� �������� �������������� ���������
        //GP1SET = 1 << (16 + 3);  //RDHBC (p1.3) = 1			WAY1
        GP1DAT |= 1 << (16 + 3);  //RDHBC (p1.3) = 1			WAY2

        //pause( 1);                //�����

        //������
        hb = (( GP1DAT & BIT_5) >> 5) +
             ((( GP0DAT & BIT_7) >> 7) << 1) +
             ((( GP0DAT & BIT_1) >> 1) << 2) +
             ((( GP2DAT & BIT_3) >> 3) << 3) +
             ((( GP4DAT & BIT_6) >> 6) << 4) +
             ((( GP4DAT & BIT_7) >> 7) << 5) +
             ((( GP0DAT & BIT_6) >> 6) << 6) +
             ((( GP0DAT & BIT_2) >> 2) << 7);

        //GP1CLR = 1 << (16 + 3);  //RDHBC (p1.3) = 0				WAY1
        GP1DAT &= ~( 1 << (16 + 3));  //RDHBC (p1.3) = 0			WAY2


        //����������� ������� ���� ���� �������� �������������� ���������
        //GP1SET = 1 << (16 + 4);  //RDLBC (p1.4) = 1		WAY1
        GP1DAT |= 1 << (16 + 4);  //RDLBC (p1.4) = 1		WAY2

        //pause( 1);                //�����

        lb = (( GP1DAT & BIT_5) >> 5) +
             ((( GP0DAT & BIT_7) >> 7) << 1) +
             ((( GP0DAT & BIT_1) >> 1) << 2) +
             ((( GP2DAT & BIT_3) >> 3) << 3) +
             ((( GP4DAT & BIT_6) >> 6) << 4) +
             ((( GP4DAT & BIT_7) >> 7) << 5) +
             ((( GP0DAT & BIT_6) >> 6) << 6) +
             ((( GP0DAT & BIT_2) >> 2) << 7);

        //GP1CLR = 1 << (16 + 4);  //RDLBC (p1.4) = 0		WAY1
		    GP1DAT &= ~( 1 << (16 + 4));  //RDLBC (p1.4) = 0		WAY2

        //���������� ��� �����
        gl_ssh_angle_inc = lb + (hb << 8);
        #ifdef DEBUG
          PrintBinShortNumber(hb);
          printf(" ");
          PrintBinShortNumber(lb);
          printf(" ");
          PrintBinShortNumber( gl_ssh_angle_inc);
          printf(" ");
        #endif



        testPike();

        // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
        // 2. � ������ ������ ������, ����������� � ������� ��� �������� �������������� ���������
        if( gl_b_SyncMode) {
          //��� �������� ������ ������� ���������� ANGLE_READY (p2.4)
          while( !( GP2DAT & 0x10)) {}

            //����������� ������� ���� ���� �������� ���������
            GP0SET = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 1
            //pause( 1);                //�����

            //������
            hb =  (( GP1DAT & BIT_5) >> 5) +
                  ((( GP0DAT & BIT_7) >> 7) << 1) +
                  ((( GP0DAT & BIT_1) >> 1) << 2) +
                  ((( GP2DAT & BIT_3) >> 3) << 3) +
                  ((( GP4DAT & BIT_6) >> 6) << 4) +
                  ((( GP4DAT & BIT_7) >> 7) << 5) +
                  ((( GP0DAT & BIT_6) >> 6) << 6) +
                  ((( GP0DAT & BIT_2) >> 2) << 7);

            GP0CLR = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 0


            //����������� ������� ���� ���� �������� �������������� ���������
            GP2SET = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 1
            //pause( 1);                //�����

            lb = (( GP1DAT & BIT_5) >> 5) +
                  ((( GP0DAT & BIT_7) >> 7) << 1) +
                  ((( GP0DAT & BIT_1) >> 1) << 2) +
                  ((( GP2DAT & BIT_3) >> 3) << 3) +
                  ((( GP4DAT & BIT_6) >> 6) << 4) +
                  ((( GP4DAT & BIT_7) >> 7) << 5) +
                  ((( GP0DAT & BIT_6) >> 6) << 6) +
                  ((( GP0DAT & BIT_2) >> 2) << 7);

            GP2CLR = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 0

            //���������� ��� �����
            gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;
            gl_ssh_angle_hanger = lb + (hb << 8);

            if( gl_ssh_angle_hanger & 0x2000) {
              gl_ssh_angle_hanger = ( gl_ssh_angle_hanger & 0x3FFF) | 0xC000;
            }
            else
              gl_ssh_angle_hanger = ( gl_ssh_angle_hanger & 0x3FFF);

            /*
            #ifdef DEBUG
              printf("DEBUG: gl_ssh_angle_hanger = %.2f V\n", ( double) ( gl_ssh_angle_hanger) * 0.61 / 1000.);
            #endif
            */
        }

        testPike();

        //**********************************************************************
        // ��������� ����������� ���������
        //**********************************************************************
        while( !( ADCSTA & 0x01)){}     // ������� ����� �������������� ��� (������������ ����� �� �������� ���� �� ��� ������ ���� �����)

        switch( ADCChannel) { //����������� ��� �� ������������ � ��������� � ��������������� ����������
          case 0: //UTD3
            gl_ssh_Utd3 = (ADCDAT >> 16);
            gl_ssh_Utd3_cal = gl_ssh_Utd3;
            if( gl_bCalibrated)
              temp_t = ( double) gl_ssh_Utd3 * TD1_K + TD1_B;
            else
              temp_t = -1481.96 + sqrt( 2.1962e6 + ( ( 1.8639 - ( double) gl_ssh_Utd3 / 4096. * 2.5) / 3.88e-6));
            gl_ssh_Utd3 = ( short) ( ( temp_t + 100.) / 200. * 65535.);
          break;  //UTD3

          case 1: //UTD1
            gl_ssh_Utd1 = (ADCDAT >> 16);
            gl_ssh_Utd1_cal = gl_ssh_Utd1;
            if( gl_bCalibrated)
              temp_t = ( double) gl_ssh_Utd1 * TD1_K + TD1_B;
            else
              temp_t = -1481.96 + sqrt( 2.1962e6 + ( ( 1.8639 - ( double) gl_ssh_Utd1 / 4096. * 2.5) / 3.88e-6));
            gl_ssh_Utd1 = ( short) ( ( temp_t + 100.) / 200. * 65535.);
          break;  //UTD1

          case 2: //UTD2
            gl_ssh_Utd2 = (ADCDAT >> 16);
            gl_ssh_Utd2_cal = gl_ssh_Utd2;
            if( gl_bCalibrated)
              temp_t = ( double) gl_ssh_Utd2 * TD1_K + TD1_B;
            else
              temp_t = -1481.96 + sqrt( 2.1962e6 + ( ( 1.8639 - ( double) gl_ssh_Utd2 / 4096. * 2.5) / 3.88e-6));
            gl_ssh_Utd2 = ( short) ( ( temp_t + 100.) / 200. * 65535.);
          break;  //UTD2

          case 3: gl_ssh_current_1 = (ADCDAT >> 16); break;     //I1
          case 4: gl_ssh_current_2 = (ADCDAT >> 16); break;     //I2
          case 5: gl_ssh_Perim_Voltage = (ADCDAT >> 16); break; //CntrPc
          case 6: gl_ssh_ampl_angle = (ADCDAT >> 16); break;    //AmplAng
          case 8: gl_ssh_current_1 = (ADCDAT >> 16); break;     //I1   TEMP!!!!!!!!!!!!!!!!!!!!
        }

        if( gl_bCalibProcessState) {
          switch( gl_bCalibProcessState) {
            case 1:
              //��������� ������ ������ �� ����������� �����������
              if( ADCChannel == 1) {
                flashParamT1_TD1_val = gl_ssh_Utd1_cal;
                gl_bCalibProcessState = 2;
              }
            break;

            case 2:
              //��������� ������ ������ �� ����������� �����������
              if( ADCChannel == 2) {
                flashParamT1_TD2_val = gl_ssh_Utd2_cal;
                gl_bCalibProcessState = 3;
              }
            break;

            case 3:
              //��������� ������ ������ �� ����������� �����������
              if( ADCChannel == 0) {
                flashParamT1_TD3_val = gl_ssh_Utd3_cal;
                gl_bCalibProcessState = 0;
                SaveThermoCalibPoint();
                nSentPacksRound = LONG_OUTPUT_PACK_LEN;
              }
            break;

            case 4:
              //��������� ������ ������ �� ������������ �����������
              if( ADCChannel == 1) {
                flashParamT2_TD1_val = gl_ssh_Utd1_cal;
                gl_bCalibProcessState = 5;
              }
            break;

            case 5:
              //��������� ������ ������ �� ������������ �����������
              if( ADCChannel == 2) {
                flashParamT2_TD2_val = gl_ssh_Utd2_cal;
                gl_bCalibProcessState = 6;
              }

            case 6:
              //��������� ������ ������ �� ������������ �����������
              if( ADCChannel == 0) {
                flashParamT2_TD3_val = gl_ssh_Utd3_cal;
                gl_bCalibProcessState = 0;
                SaveThermoCalibPoint();
                nSentPacksRound = LONG_OUTPUT_PACK_LEN;
              }

            break;
          }

          if( !gl_bCalibProcessState)
            //���� ��� ����������� ���������� ����� ���� ����� - ������������� ������������� ���������
            ThermoCalibrationCalculation();
        }

        /*if( ADCChannel == 2) ADCChannel=8;
        else if( ADCChannel == 8) ADCChannel=4;
        else*/
          ADCChannel = (++ADCChannel) % 7;        //����������� �������-��������� ���������� ������. ����������


        ADCCP = ADCChannel;              //���������� ����� ����� ���
        //pause( 10);
        ADCCON |= 0x80;                  //������ ������ �������������� (���� ����� � ��������� ����� SA)




        testPike();

        //**********************************************************************
        // ��������� 1/2 ��������� ��������� ������������ � ���� ����� ��������� �� �������
        //**********************************************************************

        //��� �������� ������ ������� ���������� AMPL_FOR_T_READY (p3.2)
        //while( !( GP3DAT & 0x04)) {}
        if( GP3DAT & 0x04) {
          //����������� ������� �� ������ ��������� ���������
          GP3DAT |= 1 << (16 + 3);  //RD_AMPL_T_CODE (p3.3) -> 1
          //pause( 1);                //�����

          //������
          hb = (( GP1DAT & BIT_5) >> 5) +
             ((( GP0DAT & BIT_7) >> 7) << 1) +
             ((( GP0DAT & BIT_1) >> 1) << 2) +
             ((( GP2DAT & BIT_3) >> 3) << 3) +
             ((( GP4DAT & BIT_6) >> 6) << 4) +
             ((( GP4DAT & BIT_7) >> 7) << 5) +
             ((( GP0DAT & BIT_6) >> 6) << 6) +
             ((( GP0DAT & BIT_2) >> 2) << 7);

          GP3CLR = 1 << (16 + 3);  //RD_AMPL_T_CODE (p3.3) -> 0

          //���������� ��� ����� (���� �� ��� ����)
          gl_ush_MeanImpulses = hb;


          //��������� ���������� ������� �� ��������� 100 (�� ��� ������� � ����� ������� ���������) ������
          if( gl_nAmplMeanCounter == 0.) {
            gl_dblAmplMean = ( double) gl_ush_MeanImpulses;
          }
          else {
            gl_dblAmplMean = ( gl_dblAmplMean * gl_nAmplMeanCounter + ( double) gl_ush_MeanImpulses) / ( gl_nAmplMeanCounter + 1);
          }

          gl_nAmplMeanCounter++;
          if( gl_nAmplMeanCounter > 100.)
            gl_nAmplMeanCounter = 100.;

        }

        testPike();

        //**********************************************************************
        // ������ ������ �������� ���������
        //**********************************************************************
        switch( nSentPacksCounter) {
          case 0: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 0, gl_ssh_Utd3);      break; //UTD3
          case 1: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 1, gl_ssh_Utd1);      break; //UTD1
          case 2: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 2, gl_ssh_Utd2);      break; //UTD2
          case 3: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 3, gl_ssh_current_1);      break; //I1
          case 4: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 4, gl_ssh_current_2);      break; //I2
          case 5: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 5, gl_ssh_Perim_Voltage);  break; //CntrPc
          case 6: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 6, gl_ush_MeanImpulses);   break; //AmplAng

          case 7: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 7, flashParamAmplitudeCode);  break;  //��� ���������
          case 8: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 8, flashParamTactCode);       break;  //��� ����� ���������
          case 9: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 9, flashParamMCoeff);         break;  //����������� �
          case 10: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 10, flashParamStartMode);     break;  //��������� ����
          case 11: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 11, flashParamI1min);        break;  //flashParamI1min
          case 12: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 12, flashParamI2min);        break;  //flashParamI2min
          case 13: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 13, flashParamAmplAngMin1);  break;  //flashParamAmplAngMin1
          case 14: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 14, flashParamDecCoeff);     break;  //flashParamDecCoeff
          case 15: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 15, flashParamSignCoeff);    break;  //flashParamSignCoeff
          case 16: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 16, ( ( VERSION_MINOR * 16) << 8) + ( VERSION_MAJOR * 16 + VERSION_MIDDLE));    break;               //SOFTWARE VERSION
          case 17: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 17, flashParam_calibT1);     break;  //min thermo-calib point T
          case 18: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 18, flashParamT1_TD1_val);   break;  //min thermo-calib point thermo1 data
          case 19: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 19, flashParamT1_TD2_val);   break;  //min thermo-calib point thermo2 data
          case 20: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 20, flashParam_calibT2);     break;  //max thermo-calib point T
          case 21: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 21, flashParamT2_TD1_val);   break;  //max thermo-calib point thermo1 data
          case 22: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 22, flashParamT2_TD2_val);   break;  //max thermo-calib point thermo2 data
          case 23: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 23, gl_ushFlashParamLastRULA); break;//flashParamPhaseShift);   break;  //phase shift
        }

        //������� �������������� ������������ ������
        if( gl_b_SyncMode && nSentPacksCounter > 0) {

          dbl_dN = fabs( ( double) gl_ssh_angle_inc - ( double) gl_ssh_angle_inc_prev);
          dbl_dU = fabs( ( double) gl_ssh_angle_hanger - ( double) gl_ssh_angle_hanger_prev);

          if( gl_nMeanDecCoeffCounter > 0) {

            dbl_N1 = ( double) gl_ssh_angle_inc_prev;
            dbl_N2 = ( double) gl_ssh_angle_inc;
            dbl_U1  = ( double) gl_ssh_angle_hanger_prev;
            dbl_U2  = ( double) gl_ssh_angle_hanger;
            Coeff = (( float) flashParamDecCoeff) / 65535.;

            gl_dbl_Omega =  ( dbl_N2 - dbl_N1) - ( dbl_U2 - dbl_U1) * Coeff * ( ( signed short) flashParamSignCoeff - 1);

            if( fabs( gl_dbl_Omega) < 5) {
              gl_dblMeanAbsDn = ( ( ( double) gl_nMeanDecCoeffCounter) * gl_dblMeanAbsDn + dbl_dN) / ( ( double) ( gl_nMeanDecCoeffCounter + 1));
              gl_dblMeanAbsDu = ( ( ( double) gl_nMeanDecCoeffCounter) * gl_dblMeanAbsDu + dbl_dU) / ( ( double) ( gl_nMeanDecCoeffCounter + 1));

              if( ++gl_nMeanDecCoeffCounter > 10000) {
                gl_nMeanDecCoeffCounter = 10000;

                if( (( gl_un_PrevT2DecCoeffCalc + T2LD - T2VAL)) % T2LD >= 32768) {
                  gl_un_PrevT2DecCoeffCalc = T2VAL;

                  flashParamDecCoeff = ( short) ( ( int) ( gl_dblMeanAbsDn / gl_dblMeanAbsDu * 65535.));
                  nSentPacksRound = LONG_OUTPUT_PACK_LEN;
                }
              }
            }
          }
          else {
            gl_dblMeanAbsDn = dbl_dN;
            gl_dblMeanAbsDu = dbl_dU;
            gl_un_PrevT2DecCoeffCalc = T2VAL;
            gl_nMeanDecCoeffCounter++;
          }

          /*
          db_dN1 = ( double) gl_ssh_angle_inc_prev;
          db_dN2 = ( double) gl_ssh_angle_inc;
          dbU1 = ( double) gl_ssh_angle_hanger_prev;
          dbU2 = ( double) gl_ssh_angle_hanger;

          Coeff = (( float) flashParamDecCoeff) / 65535.;
          gl_dbl_Omega =  ( db_dN2 - db_dN1) - ( dbU2 - dbU1) * Coeff * ( ( signed short) flashParamSignCoeff - 1);
          if( fabs( gl_dbl_Omega) < 5) {
            gl_dbl_Nsumm += fabs( ( double) gl_ssh_angle_inc - ( double) gl_ssh_angle_inc_prev);
            gl_dbl_Usumm += fabs( ( double) gl_ssh_angle_hanger - ( double) gl_ssh_angle_hanger_prev);
            gl_un_DecCoeffStatPoints++;
            if( !( gl_un_DecCoeffStatPoints % DEC_COEFF_CONTINUOUS_CALCULATION_N)) {
              //flashParamDecCoeff = ( short) ( ( gl_dbl_Nsumm / gl_dbl_Usumm * 65535.));
              flashParamDecCoeff = ( short) ( ( int) ( gl_dbl_Nsumm / gl_dbl_Usumm * 65535.));


              gl_dbl_Nsumm = gl_dbl_Usumm = 0.;
              gl_un_DecCoeffStatPoints = 0;
              nSentPacksRound = LONG_OUTPUT_PACK_LEN;
            }
          }
          */
        }


        #ifdef DEBUG
          printf( "%d     %d     %d      %.2fV\n", gl_ssh_angle_inc, gl_ssh_angle_inc - gl_ssh_angle_inc_prev, gl_ssh_angle_hanger- gl_ssh_angle_hanger_prev, ( double) gl_ssh_angle_hanger * 0.61 / 1000.);
        #endif

        testPike();

        gl_ssh_angle_inc_prev = gl_ssh_angle_inc;

        // ************************************************************************************
        // 2010-04-22
        //�������������� ����������� ���������
        // ************************************************************************************
        if( nSentPacksCounter == 5) {
          V_piezo = (( gl_ssh_Perim_Voltage / 4095. * 2.5) - 1.23) * 101.;
          if( fabs( V_piezo) > 100.) {
            //flashParamStartMode = 125;
            //DACConfiguration();
            GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1
            nRppTimer = T1VAL;
            gl_n_PerimeterReset = 1;
            nSentPacksRound = LONG_OUTPUT_PACK_LEN;
          }
        }


        //����������� ������� ������������ ������� ������
        if( ++nSentPacksCounter == LONG_OUTPUT_PACK_LEN) {
          nSentPacksCounter = 0;
          nSentPacksRound = SHORT_OUTPUT_PACK_LEN;

          ADCChannel = 0;     //��������� ��������� � UTD3
          ADCCP = ADCChannel;
          //pause( 10);
          ADCCON |= 0x80;
        }
        else
          nSentPacksCounter = ( nSentPacksCounter) % nSentPacksRound;

        testPike();

        //**********************************************************************
        //������������ ��������� ��������� ������������
        //**********************************************************************
        if( gl_nAmplStabStep < 10) {
          if( T2VAL <= nT2RepeatBang) {
            gl_nAmplStabStep++;
            nT2RepeatBang = ( T2VAL +T2LD - 6553) % T2LD;

            gl_nAppliedMCoeff = 4096. * ( 1. - ( 1. - ( double) flashParamMCoeff / 250.) / 10. * ( double) gl_nAmplStabStep);

            /*switch( gl_nAmplStabStep) {
              case  0:  gl_nAmplStabApplyRulaTacts = 1;  gl_nDelta = 4; break;
              case  1:  gl_nAmplStabApplyRulaTacts = 2;  gl_nDelta = 4; break;
              case  2:  gl_nAmplStabApplyRulaTacts = 3;  gl_nDelta = 2; break;
              case  3:  gl_nAmplStabApplyRulaTacts = 4;  gl_nDelta = 2; break;
              case  4:  gl_nAmplStabApplyRulaTacts = 5;  gl_nDelta = 2; break;
              case  5:  gl_nAmplStabApplyRulaTacts = 6;  gl_nDelta = 2; break;
              case  6:  gl_nAmplStabApplyRulaTacts = 7;  gl_nDelta = 1; break;
              case  7:  gl_nAmplStabApplyRulaTacts = 8;  gl_nDelta = 1; break;
              case  8 : gl_nAmplStabApplyRulaTacts = 9;  gl_nDelta = 1; break;
              case  9:  gl_nAmplStabApplyRulaTacts = 10; gl_nDelta = 1; break;
              case 10:  gl_nAmplStabApplyRulaTacts = 10; gl_nDelta = 1; break;
            }
            */
            //gl_nDelta = 1;
          }
        }

        if( (( gl_un_PrevAmplRegulationT2 + T2LD - T2VAL)) % T2LD >= 3276) {
        //if( ( gl_nAmplMeanCounter % ) == 0) {

          gl_un_PrevAmplRegulationT2 = T2VAL;

          dblDelta = ( ( double) flashParamAmplitudeCode / 100.) - gl_dblAmplMean;

          //���� ����������� ���������� ������� � �������� �������� ������� - ��������� RULA
          /*
          if(      fabs( dblDelta) > 50)  { if( dblDelta > 0) gl_un_RULAControl += 50;  else  gl_un_RULAControl -= 50; }
          else if( fabs( dblDelta) > 10)  { if( dblDelta > 0) gl_un_RULAControl += 10;   else  gl_un_RULAControl -= 10;  }
          else if( fabs( dblDelta) > 5)   { if( dblDelta > 0) gl_un_RULAControl += 5;    else  gl_un_RULAControl -= 5;   }
          else if( fabs( dblDelta) > 1) { if( dblDelta > 0)   gl_un_RULAControl += 1;    else  gl_un_RULAControl -= 1;   }
          */
          if( dblDelta > 0.5) {
            gl_un_RULAControl +=  ( int) ( dblDelta * 5.);
          }
          else if( dblDelta < 0.5) {
            if( ( int) fabs( dblDelta * 5.) > gl_un_RULAControl) {
              gl_un_RULAControl = RULA_MIN;
            }
            else {
              gl_un_RULAControl +=  ( int) ( dblDelta * 5.);
            }
          }
        }

        if( gl_un_RULAControl > RULA_MAX) gl_un_RULAControl = RULA_MAX;
        if( gl_un_RULAControl < RULA_MIN) gl_un_RULAControl = RULA_MIN;

        //��������� RULA-RULM (��� �� RULM ����������� ������-��������)
        DACConfiguration();


        //**********************************************************************
        //��������� ����� ������ RP_P
        //**********************************************************************
        if( nRppTimer != 0) {

          if( gl_n_PerimeterReset == 1) {

            //32768 = 1 sec
            //3276  = 0.1 sec    = 100 msec
            //327   = 0.01 sec   = 10 msec
            //32    = 0.001 sec  = 1 msec
            if( (( T1LD + nRppTimer - T1VAL) % T1LD) > 32) {

              //����� (���������) ������������ � ������� ����������� ���������
              GP0DAT &= ~( 1 << (16 + 5));  //RP_P   (p0.5) = 0
              nRppTimer = T1VAL;
              gl_n_PerimeterReset = 2;
            }
          }
          else if( gl_n_PerimeterReset == 2) {

            //32768 = 1 sec
            //3276  = 0.1 sec = 100 msec
            //327   = 0.01 sec = 10 msec
            if( (( T1LD + nRppTimer - T1VAL) % T1LD) > 3276) {
              gl_n_PerimeterReset = 0;
              nRppTimer = 0;
            }
          }

        }

        //��������� ���� � ��� ��� ������� ������� ������� SA �� ����������
        gl_b_SA_Processed = 1;

        testPike();
      }
/*
#ifdef DEBUG
  printf("DEBUG: Main loop ends... %d %d \n", nSentPacksCounter, gl_b_SA_Processed);
#endif
*/
    }
    else {
      //���� ����� ������� SA � ������ ������ - �� ��� ������ ��� ���������� �������� ����� �������������� ����
      gl_b_SA_Processed = 0;

      //�������� ������������
      ush_SA_check_time = ( T1LD + gl_ssh_prT1VAL - T1VAL) % T1LD;

      //2 sec = 32768 * 2.0 = 65536
      if( ush_SA_check_time > 65536) {
        //������� ������������

        #ifdef DEBUG
          printf("DEBUG: Tact signal lost!");
        #endif

        //��������� �������
        GP4DAT |= ( 1 << ( 16 + 0));  //ONHV (p4.0) = 1

        deadloop_no_tact( ERROR_TACT_SIGNAL_LOST);
      }
    }
  }
}
