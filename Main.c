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
#include "SimpleMaxRateRegime.h"
#include "ProcessingIncomingCommand.h"
#include "AnalogueParamsConstList.h"
#include "debug.h"

signed short gl_ssh_angle_inc = 0;          //������� �������� � ������������ �������� (N2)
signed short gl_ssh_angle_inc_prev = 0;     //���������� �������� � ������������ �������� (N1)

signed short gl_ssh_angle_hanger = 0;       //������� �������� ���� ���������� �����-������� (U2)
signed short gl_ssh_angle_hanger_prev = 0;  //���������� �������� ���� ���������� �����-������� (U1)


signed short gl_ssh_current_1 = 0;      //��������� ��� 1
signed short gl_ssh_current_2 = 0;      //��������� ��� 2
signed short gl_ssh_Perim_Voltage = 5;  //���������� �������� ����������������
signed short gl_ssh_ampl_angle = 0;     //���������� ���������������� ��������� ��������� ������� ��������� ������� ���. ��������

signed short gl_ssh_Utd1 = 0;           //���������� ��������� ������������1
signed short gl_ssh_Utd2 = 0;           //���������� ��������� ������������2
signed short gl_ssh_Utd3 = 0;           //���������� ���������� ������������
signed short gl_ssh_Utd1_cal = 0;       //���������� ���������� ������������ (������.)
signed short gl_ssh_Utd2_cal = 0;       //���������� ��������� ������������ (������.)
signed short gl_ssh_Utd3_cal = 0;       //���������� ? ������������ (������.)
double gl_dbl_Tutd1 = 0;                //����������� ������������ TD1 (��1)
double gl_dbl_Tutd2 = 0;                //����������� ������������ TD2 (��2)
double gl_dbl_Tutd3 = 0;                //����������� ������������ TD3 (������)

signed short gl_ssh_SA_time = 0;        //������ SA
int gl_n_prT1VAL = 0x1000;              //������� ������� ��� ����������� ������� ����� SA

unsigned short gl_ush_MeanImpulses = 1;

#define RULA_MAX 4095
#define RULA_MIN 25
//unsigned int gl_un_RULAControl = 64;      //64   = 0.039 V
//unsigned int gl_un_RULAControl = 1638;    //1638 = 1.000 V
//unsigned int gl_un_RULAControl = 2457;    //2457 = 1.500 V
unsigned int gl_un_RULAControl = 4095;      //4095 = 2.500 V
unsigned int gl_nDelta = ( RULA_MAX - RULA_MIN) / 4;


char gl_c_EmergencyCode = 0;            //��� ������

//����� �������� ������
char gl_acInputBuffer[6] = { 0, 0, 0, 0, 0, 0};     //����� �������� ������
char gl_cPos_in_in_buf = 0;                         //������� ������ � ������ �������� ������

int gl_n3kVapplies = 0;                 //����� 3kV ��������� ��� �������


//�����
char gl_b_SyncMode = 0;                 //���� ������ ������ ���������:   0=�����. 1=������.
char gl_chAngleOutput =   0;            //���� ������ ���������� ����: 0 = dW (4b)         1 = dN (2b), dU(2b)
char gl_chLockBit = 0;                  //���� ������������ ����������: 0 - ����� "������������"   1 - ����� "������������"
//char gl_bOutData = 0;                 //���� ������ ������ ������ (���������� ����� 2.5 ��� ����� ��������� �������) 0 - ��� ������; 1 - ������ ������;
char gl_n_PerimeterReset = 0;           //���� ������� ������ ��������� (0 = ������ ����������, 1 = ������ ������������, ������ ���������� �����������, 2 = ������ ������������, ������ ��������� �����������)
char gl_b_SA_Processed = 0;             //���� ��������� ��������� ������� SA
char gl_bManualLaserOff = 0;            //���� ��� ���� ��������� ��� ������ (��������). ���� ����� ����� ��������� ��� ������� � ������������� �������� ����.
char gl_bSimpleDnDuRegime = 0;          //���� ������ "��������� �������" (������ dN,dU � ����������� ��������� ��������, ��� ������ ���������� ����������, ��� ���, ��� �������� ���������)

//������� ��������
int gl_nRppTimerT2 = 0;                 //������� ������� ��� ���������� ������ ����������� ������� ����������� ���������

short gl_nSentAddParamIndex;            //������ ����������� (���.) ��������� ����������� � �������
short gl_nSentAddParamSubIndex;         //���-������ ����������� (���.) ��������� ����������� � �������
char  gl_c_OutPackCounter = 0;          //���������� ������ ������� �������

int gl_nADCChannel = 0; //�������� ����� ���
                        //0 = ADC0 = UTD3
                        //1 = ADC1 = UTD1
                        //2 = ADC2 = UTD2
                        //3 = ADC3 = I1
                        //4 = ADC4 = I2
                        //5 = ADC5 = CntrPC
                        //6 = ADC6 = AmplAng

//���������� ����������� � �������� ������ ������������ ������ �������
long gl_lSecondsFromStart;
long gl_l5SecPrevValue;
long gl_lCalibratedPhaseShiftApplySecs;
long gl_lCalibratedDcApplySecs;
int  gl_nSecondsT2Lock;


//********************
#define ADC_CHANNEL_UTD1    0
#define ADC_CHANNEL_UTD2    1
#define ADC_CHANNEL_UTD3    2
#define ADC_CHANNEL_I1      3
//#define ADC_CHANNEL_I1      8
#define ADC_CHANNEL_I2      4
#define ADC_CHANNEL_CNTRPC  5
#define ADC_CHANNEL_AA      6


//��������� �������� �� ����-������
unsigned short gl_ush_flashParamAmplitudeCode = 90;       //��������� ��������� ������������
unsigned short gl_ush_flashParamTactCode = 0;             //��� ����� ���������
unsigned short gl_ush_flashParamMCoeff = 4;               //����������� ���������
unsigned short gl_ush_flashParamStartMode = 5;            //��������� ���� ������� ����������� ���������
unsigned short gl_ush_flashParamDecCoeff = 0;             //����������� ������
unsigned short gl_ush_flashLockDev = 0;                   //���� ���������� ����������

unsigned short gl_ush_flashParamI1min = 0;                //����������� �������� ���� ������� I1
unsigned short gl_ush_flashParamI2min = 0;                //����������� �������� ���� ������� I2
unsigned short gl_ush_flashParamAmplAngMin1 = 0;          //����������� �������� ������� �������� � ����

unsigned short gl_ush_flashParamSignCoeff = 2;            //�������� �����������
unsigned short gl_ush_flashParamDeviceId = 0;             //ID ����������
unsigned short gl_ush_flashParamDateYear = 0;             //���� ? �������.���
unsigned short gl_ush_flashParamDateMonth = 0;            //���� ? �������.�����
unsigned short gl_ush_flashParamDateDay = 0;              //���� ? �������.����
char gl_ac_flashParamOrg[17] = { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0};    //�������� �����������

unsigned short gl_ushFlashParamLastRULA = 0;      //��������� RULA (obsolete)
unsigned short gl_ushFlashParamLastRULM = 0;      //��������� RULM (obsolete)



//������ ���������� ���������� (��������������) ����������
unsigned short gl_aushListOutputAddParams[12];
int gl_nSentPackListIndex;

//���������� �������������
signed short   gl_ssh_flashParam_calibT1;
unsigned short gl_ush_flashParamT1_TD1_val, gl_ush_flashParamT1_TD2_val, gl_ush_flashParamT1_TD3_val;
signed short   gl_ssh_flashParam_calibT2;
unsigned short gl_ush_flashParamT2_TD1_val, gl_ush_flashParamT2_TD2_val, gl_ush_flashParamT2_TD3_val;
short gl_shFlashParamTCalibUsage;         //���� ������������� ���������� �������������: 0 - ������������, REST (����������� 0xFF) - �� ������������
char gl_cCalibProcessState;               //����-��������� ���������� ����������
                                          //      0-�� ����������
                                          //      1 - ��������� ������ ������ �� ����������� �����������
                                          //      2 - ��������� ������ ������ �� ����������� �����������
                                          //      3 - ��������� ������ ������ �� ����������� �����������
                                          //      4 - ��������� ������ ������ �� ������������ �����������
                                          //      5 - ��������� ������ ������ �� ������������ �����������
                                          //      6 - ��������� ������ ������ �� ������������ �����������
char gl_bTDCalibrated;                    //���� ��������� ������������� ���������� �������������: 0=�� ����������, 1=����������
double gl_dblTD1_K, gl_dblTD1_B;          //������ � ��������� ���������� ������� ������������
double gl_dblTD2_K, gl_dblTD2_B;          //������ � ��������� ���������� ������� ������������
double gl_dblTD3_K, gl_dblTD3_B;          //������ � ��������� ���������� �������� ������������

//���������� �������� ������
char gl_ac_calib_phsh_t[11];        //������ ����� ���������� �������� ������
char gl_ac_calib_phsh_phsh[11];     //������ ����� �������� �������� ������, ��������������� ������������, ��������� ����
char gl_cFlashParamPhaseShiftUsage; //���� ������������� �������� ������: 0 - ������������, REST (����������� 0xFF) - �� ������������
int  gl_nCurrentPhaseShift;         //������� (��������� ����������) ������� �����. 0xFF-�� ������������. default startup value = 0xFF        ����� 0x100 - manual set

//���������� ������������ ������
char gl_ac_calib_dc_t[11];              //������ ����� ���������� ���������� ������������ ������
unsigned short gl_ush_calib_dc_dc[11];  //������ ����� �������� ������������ ������, ��������������� ������������, ��������� ����
char gl_cFlashParamDcCalibUsage;        //���� ������������� ���������� ������������ ������: 0 - ������������ ����������, 1 - ������������ ��������������, REST (����������� 0xFF) - �� ������������

//���������� ����������� � ������ ������� ����������� ���������
int  gl_snMeaningCounter = 0;               //������� �������
int  gl_snMeaningCounterRound = 128;        //���������� ��������
int  gl_snMeaningShift = 7;                 //������� - ��������� �������� ����� (log2 �� gl_snMeaningCounterRound)
long gl_lnMeaningSumm = 0;                  //����� ��������
long gl_lnMeanImps = 0;                     //������� ��������� (� ��������� ������. ��������)
int  gl_nActiveRegulationT2 = 0;            //������� ������� ��� ������ ����� �������� ����������� ���������, �� �� ���� ����������� ���������

//���������� ����������� � �������� ������������ ������ "�� ����"
double gl_dblMeanAbsDn;
double gl_dblMeanAbsDu;
int    gl_nMeanDecCoeffCounter;
unsigned int gl_un_PrevT2DecCoeffCalc;

//********************
// Decrement coefficient calculation
#define DEC_COEFF_FIRST_CALCULATION_N 100
#define DEC_COEFF_CONTINUOUS_CALCULATION_N 1000
unsigned int gl_un_DecCoeffStatPoints = 0;
double gl_dbl_Nsumm = 0.;
double gl_dbl_Usumm = 0.;
double gl_dbl_Omega;


#define BIT_0 1
#define BIT_1 2
#define BIT_2 4
#define BIT_3 8
#define BIT_4 16
#define BIT_5 32
#define BIT_6 64
#define BIT_7 128

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//���������� ����������
void FIQ_Handler (void) __fiq
{
  char tmp;
  if( ( FIQSTA & UART_BIT) != 0) {
    if( gl_cPos_in_in_buf < IN_COMMAND_BUF_LEN)
      gl_acInputBuffer[ gl_cPos_in_in_buf++] = COMRX;
    else
      tmp = COMRX;
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
  prval = T2VAL;
  chk = (( T2LD + prval - T2VAL) % T2LD);
  while( chk < n)
    chk = (( T2LD + prval - T2VAL) % T2LD);
}

double round( double val) {
  double lstd = val - floor( val);
  if( lstd < .5) return floor( val);
  else return ceil( val);
}

/*void PrintString( char*ptr, int n) {
}*/

void send_pack( short shAnalogueParamValue) {
  short shAnalogueParamIndicator = gl_nSentAddParamIndex;
  signed short angle_inc1 = ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536;

  char cCheckSumm = 0;
  char *pntr;
  char b1, b2, b3, b4;

#ifndef DEBUG
  float f_dN;
  double dbl_dN;

  float Coeff = (( float) gl_ush_flashParamDecCoeff) / 65535.;

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
  //    ���� ��������
  //***************************************************************************
  if( gl_b_SyncMode) {

    ssh_dN = gl_ssh_angle_inc - gl_ssh_angle_inc_prev;
    ssh_dU = gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev;

    if( gl_chAngleOutput == 1) {
      //����������� �����: ������ dN, dU � ���������
      pntr = ( char *) &ssh_dN;
      b1 = pntr[0];
      b2 = pntr[1];

      pntr = ( char *) &ssh_dU;
      b3 = pntr[0];
      b4 = pntr[1];
    }
    else {
      //����������� ����� ������: ����� phi, �� ����� ������� � ����� "��������������� �������"

      //2017.04.24 �������� ��������������� ����� ���������
      //if( gl_bSimpleDnDuRegime == 1) {
      //  //������: "���������������� ������" dN-dU
      //  signed int siAngleInc1 = ( signed short) angle_inc1;
      //  f_dN = ( float) ( ( signed int) angle_inc1);
      //  dbl_dN = ( double) ( ( signed int) angle_inc1);

        //����� ���� ���������� ������ ����������
      //}
      //else {
        //������: ���������� �����
        /*double db_dN1 = ( double) gl_ssh_angle_inc_prev;
        double db_dN2 = ( double) gl_ssh_angle_inc;
        double dbU1 = ( double) gl_ssh_angle_hanger_prev;
        double dbU2 = ( double) gl_ssh_angle_hanger;*/

        n_dN = ( signed int) ssh_dN;
        n_dU = ( signed int) ssh_dU;

        result =  ( double) n_dN - ( ( double) n_dU) * Coeff * (( signed short) gl_ush_flashParamSignCoeff - 1);
        //printf("\n%.3f %.3f %.3f %.3f %.3f\n", db_dN1, db_dN2, dbU1, dbU2, result);

        f_dN = ( float) ( ( signed short) result);
        dbl_dN = ( double) ( ( signed short) result);

        //����������� f_dN �� �������� [-99 310; + 99 310]
        dbl_dN = ( dbl_dN / 99310.) * 2147483647.;
        n_dN = ( int) dbl_dN;

        pntr = ( char *) &n_dN;
        b1 = pntr[0];
        b2 = pntr[1];
        b3 = pntr[2];
        b4 = pntr[3];
      //}
    }

    putchar_nocheck( b1);   cCheckSumm += b1;
    putchar_nocheck( b2);   cCheckSumm += b2;
    putchar_nocheck( b3);   cCheckSumm += b3;
    putchar_nocheck( b4);   cCheckSumm += b4;
  }
  else {
    //���������� �����
    double dAngleInc1 = ( double) angle_inc1;

    f_dN = ( float) ( ( signed int) angle_inc1);
    dbl_dN = ( double) ( ( signed int) angle_inc1);

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
  }




  //***************************************************************************
  //ANALOG PARAMETER INDICATOR
  //***************************************************************************
  putchar_nocheck( ( ( gl_n_PerimeterReset > 0) ? 0x80 : 0x00) + shAnalogueParamIndicator & 0xff);
  cCheckSumm += (( ( gl_n_PerimeterReset > 0) ? 0x80 : 0x00) + shAnalogueParamIndicator & 0xff);

  //***************************************************************************
  //ANALOG PARAMETER
  //***************************************************************************
  putchar_nocheck(   shAnalogueParamValue & 0xFF);
  cCheckSumm += (    shAnalogueParamValue & 0xFF);

  putchar_nocheck( ( shAnalogueParamValue & 0xFF00) >> 8);
  cCheckSumm += ( (  shAnalogueParamValue & 0xFF00) >> 8);

  //***************************************************************************
  //SA TIME
  putchar_nocheck( gl_ssh_SA_time & 0xFF);
  cCheckSumm += ( gl_ssh_SA_time & 0xFF);

  putchar_nocheck( ( gl_ssh_SA_time & 0xFF00) >> 8);
  cCheckSumm += ( ( gl_ssh_SA_time & 0xFF00) >> 8);

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
  bt += ( gl_chLockBit ? 0x40 : 0x00);

  //6 bit - 0x20 - Sync (0)/Async(1) regime
  bt += ( gl_b_SyncMode ? 0x20 : 0x00);

  //5 bit - 0x10 - W (0) / dNdU (1) regime
  bt += ( gl_chAngleOutput ? 0x10 : 0x00);

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

  #if DEBUG == 2
    printf("(0x55 0xAA)   (0x%02x 0x%02x 0x%02x 0x%02x)   (0x%02x)   (0x?? 0x??)   (0x?? 0x??)  (0x%02x)   (0x%02x)   (0x??)\n",
            b1, b2, b3, b4,
            ( ( gl_n_PerimeterReset > 0) ? 0x80 : 0x00) + shAnalogueParamIndicator & 0xff,
            gl_c_OutPackCounter++,
            gl_c_EmergencyCode
            );
  #endif

#endif
}

void configure_hanger( void) {

  //1. ��� ����� ���������
  //�������� TactNoise0 (������� ��� ��������� "��� ����� ���������")
  if( ( gl_ush_flashParamTactCode & 0x01))  //Set TactNoise0 to TactCode parameter bit0
    GP3DAT |= ( 1 << (16 + 0));
  else
    GP3DAT &= ~( 1 << (16 + 0));

  //�������� TactNoise1 (������� ��� ��������� "��� ����� ���������")
  if( ( gl_ush_flashParamTactCode & 0x02))  //Set TactNoise1 to TactCode parameter bit1
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
  DAC0DAT = (( int) ( 4095.0 * ( ( double) gl_un_RULAControl / ( double) RULA_MAX ))) << 16;

  // ��� 1 (����)
  DAC1DAT = (( int) ( 4095.0 * ( ( double) gl_ush_flashParamMCoeff / 250. * ( ( double) gl_un_RULAControl / ( double) RULA_MAX * 2.5 )) / 3.0)) << 16;

  // ��� 2 (��������� ����)
  DAC2DAT = (( int) ( 4095.0 * ( ( double) gl_ush_flashParamStartMode / 250. ))) << 16;
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
  // 2. � ������ ������ ������, ����������� � ������� ���� ���������� ������������
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


    //����������� ������� ���� ���� �������� ���������
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
    // 2. � ������ ������ ������, ����������� � ������� ���� ���������� ������������

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


    //����������� ������� ���� ���� �������� ���������
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
  gl_ush_flashParamDecCoeff = ( short) ( gl_dbl_Nsumm / gl_dbl_Usumm * 65535.);

  gl_un_DecCoeffStatPoints = 0;
  gl_dbl_Nsumm = gl_dbl_Usumm = 0.;
}

void ThermoCalibrationCalculation( void)
{
  double x1, y1, x2, y2;
  //������� ���������� �������������
  if( gl_ssh_flashParam_calibT1 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) && 
      gl_ssh_flashParam_calibT1 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION) &&
      gl_ssh_flashParam_calibT2 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) &&
      gl_ssh_flashParam_calibT2 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {


    x1 = gl_ssh_flashParam_calibT1 - THERMO_CALIB_PARAMS_BASE;
    x2 = gl_ssh_flashParam_calibT2 - THERMO_CALIB_PARAMS_BASE;


    y1 = gl_ush_flashParamT1_TD1_val;
    y2 = gl_ush_flashParamT2_TD1_val;

    gl_dblTD1_B = ( x2 * y1 - x1 * y2) / ( x1 - x2);
    gl_dblTD1_K = ( y2 - y1) / ( x2 - x1);


    y1 = gl_ush_flashParamT1_TD2_val;
    y2 = gl_ush_flashParamT2_TD2_val;

    gl_dblTD2_B = ( x2 * y1 - x1 * y2) / ( x1 - x2);
    gl_dblTD2_K = ( y2 - y1) / ( x2 - x1);

    y1 = gl_ush_flashParamT1_TD3_val;
    y2 = gl_ush_flashParamT2_TD3_val;

    gl_dblTD3_B = ( x2 * y1 - x1 * y2) / ( x1 - x2);
    gl_dblTD3_K = ( y2 - y1) / ( x2 - x1);

    gl_bTDCalibrated = 1;
  }
  else
    gl_bTDCalibrated = 0;
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

char isAddParamDecriptorInList( int nAddParamDescriptor) {
  char cInList = 0;
  int i;
  for( i=0; i<12; i++) {
    if( gl_aushListOutputAddParams[ i] == nAddParamDescriptor) {
      cInList = 1;
      break;
    }
  }
  return cInList;
}

int getNextAddParamDescriptorToSendFromList() {
  int nResult;
  char cGoOn, cCnt;

  nResult = gl_aushListOutputAddParams[ gl_nSentPackListIndex];
  if ( nResult < 0 || nResult > 254){
    cGoOn = 1; cCnt = 0;
    do {
      cCnt++;
      gl_nSentPackListIndex = (++gl_nSentPackListIndex) % 12;
      nResult = gl_aushListOutputAddParams[ gl_nSentPackListIndex];
      if ( nResult < 0 || nResult > 254){
        if( cCnt > 12)
          return UTD1;
      }
      else
        cGoOn = 0;

    } while( cGoOn);
  }

  gl_nSentPackListIndex = (++gl_nSentPackListIndex) % 12;
  return nResult;
}

void main() {
  int time = 20000;
  int i;
  char lb, hb;
  char bSAFake = 0;
  int prt2val;

  //���������� ������������ � �������������� ������������ ������
  double dbl_N1 = 2., dbl_N2, dbl_U1, dbl_U2;
  float Coeff;

  //���������� ������������ � �������� �������� ������
  double dblTdCalib;
  char cNewPhaseShift;

  double dStartAmplAngCheck = 0.5;

  //�� ��������� �� �� ���������� ��������� (���������� ��� ��������������) ������������ ������ (�� ��� ���������� � �������� ��������)
  gl_cFlashParamDcCalibUsage = 0xFF;

  //�� ��������� �� �� ���������� ������� �����
  gl_cFlashParamPhaseShiftUsage = 0xFF;

  //�� ��������� ������������ ���� �������� ������ = 0xFF (��� ���������� ��� �� ����������)
  gl_nCurrentPhaseShift = 0xFF;

  //�� ��������� �� �� ���������� ���������� �������������
  gl_shFlashParamTCalibUsage = 0xFF;

  //���������� ���������� �������������
  gl_cCalibProcessState = 0;    //0 - no calibration process going on

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
  GP2DAT |= 1 << (24 + 6);  //������������ ����� SET_PS_SH_ACT       (p2.6) � �������� ������

  GP3DAT |= 1 << (24 + 0);  //������������ ����� TactNoise0          (p3.0) � �������� ������
  GP3DAT |= 1 << (24 + 1);  //������������ ����� TactNoise1          (p3.1) � �������� ������
  GP3DAT |= 1 << (24 + 3);  //������������ ����� RD_AMPL_T_CODE      (p3.3) � �������� ������
  GP3DAT |= 1 << (24 + 4);  //������������ ����� SET_PS_SH_LINE      (p3.4) � �������� ������
  GP3DAT |= 1 << (24 + 5);  //������������ ����� OutLnfType          (p3.5) � �������� ������

  GP4DAT |= 1 << (24 + 0);  //������������ ����� ONHV                (p4.0) � �������� ������
  GP4DAT |= 1 << (24 + 1);  //������������ ����� OFF3KV              (p4.1) � �������� ������
  GP4DAT |= 1 << (24 + 3);  //������������ ����� Reset               (p4.3) � �������� ������


#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: GPIO lines values configuration...");
#endif
  GP4DAT |=    1 << (16 + 0);   //ONHV            (p4.0) = 1 (���������)
  GP4DAT &= ~( 1 << (16 + 1));  //OFF3KV          (p4.1) = 0 (��������)

  GP3DAT |=    1 << (16 + 4);   //SET_PS_SH_LINE  (p3.4) = 1 (�������� ������ �� ���� ���������� � ����������������)

  GP2DAT &= ~( 1 << (16 + 2));  //EN_VB           (p2.2) = 0
  GP2DAT &= ~( 1 << (16 + 6));  //SET_PS_SH_ACT   (p2.6) = 0 (������� ������ �������� ������ - ����������� ��������� � 0)

  GP1DAT &= ~( 1 << (16 + 7));  //EN_RP           (p1.7) = 0

  GP0DAT &= ~( 1 << (16 + 5));  //RP_P            (p0.5) = 0


#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: Pulsing Reset signal...");
#endif

  //**********************************************************************
  // ������� ������� Reset
  //**********************************************************************
  GP4DAT |= 1 << (16 + 3);      //Reset set
  for( i=0; i<100; i++);
  //dbl_N1 = dbl_N1 / 2.;
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
  //T1CON = 0x2C0;          //32kHz clocking
  //T1LD = 0x100000;
  T1LD = 0x08FFFFFF;
  T1CON = 0x0C4;
  //0x0C4 = 0000 1100 0100
  // XXX X   X X XX   0100      SourceClock/16
  // XXX X   X X 00   XXXX      Binary
  // XXX X   X 1 XX   XXXX      Periodic
  // XXX X   1 X XX   XXXX      Enable
  // XXX 0   X X XX   XXXX      Count down
  // 000 X   X X XX   XXXX      CoreClock (41.78 Mhz, � ���� ������ 32768*1275=41779200, � �� �������� ��������!)



  //**********************************************************************
  // ������������ Timer2
  //**********************************************************************
  T2LD = 0x00FFFFFF;
  T2CON = 0x2C0;
  //0x2C0 = 0010 1100 0000
  // X XX X   X X XX   0000     SourceClock/1
  // X XX X   X X 00   XXXX     Binary
  // X XX X   X 1 XX   XXXX     Periodic
  // X XX X   1 X XX   XXXX     Enable
  // X XX 0   X X XX   XXXX     Count down
  // X 01 X   X X XX   XXXX     External Crystal

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

  //����� �������� ������ �� ������ ������ �������
  gl_lSecondsFromStart = 0;
  gl_l5SecPrevValue = 0;
  gl_nSecondsT2Lock = T2VAL;
  gl_lCalibratedPhaseShiftApplySecs = 0;
  gl_lCalibratedDcApplySecs = 0;

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
  DAC2DAT = (( int) ( 4095.0 * 1.25 / 2.5)) << 16; //( ( double) gl_ush_flashParamStartMode / 250. * 2.5) / 3.0)) << 16;

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
  //����������� ����������� TD1 (������ ����� �������������� � ��������� �������� ��� ���������� �� �����������)
  //**********************************************************************
  ADCCP = ADC_CHANNEL_UTD1;
  ADCCON |= 0x80;
  while (!( ADCSTA & 0x01)){}

  gl_ssh_Utd1 = (ADCDAT >> 16);
  gl_dbl_Tutd1 = -1481.96 + sqrt( 2.1962e6 + ( ( 1.8639 - ( double) gl_ssh_Utd1 / 4096. * 2.5) / 3.88e-6));
  /*
  #ifdef DEBUG
    printf("DEBUG: ADC0 = %.02f V\n", ( double) gl_ssh_ampl_angle / 4095. * 2.5);
  #endif
  */


  //**********************************************************************
  //��������� �������� �������� ������
  //**********************************************************************
  cNewPhaseShift = gl_ac_calib_phsh_phsh[0];

  if( cNewPhaseShift == 0xFF)
    cNewPhaseShift = 75;

  for( i=1; i<11; i++) {
    if( gl_ac_calib_phsh_t[i] == 0xFF) break;
    dblTdCalib = gl_ac_calib_phsh_t[i] - 128.;
    if( gl_dbl_Tutd1 > dblTdCalib ) cNewPhaseShift = gl_ac_calib_phsh_phsh[i];
  }

  //����������� ����������� �������� ������ �� ���� �� ���������������� � ������� (pin38 -> 0)
  GP3DAT &= ~( 1 << (16 + 4));  //SET_PS_SH_LINE  (p3.4) = 0

  GP1DAT |= 1 << (24 + 5);      //������������ ����� BIT_00 (p1.5) � �������� ������
  GP0DAT |= 1 << (24 + 7);      //������������ ����� BIT_01 (p0.7) � �������� ������
  GP0DAT |= 1 << (24 + 1);      //������������ ����� BIT_02 (p0.1) � �������� ������
  GP2DAT |= 1 << (24 + 3);      //������������ ����� BIT_03 (p2.3) � �������� ������
  GP4DAT |= 1 << (24 + 6);      //������������ ����� BIT_04 (p4.6) � �������� ������
  GP4DAT |= 1 << (24 + 7);      //������������ ����� BIT_05 (p4.7) � �������� ������
  GP0DAT |= 1 << (24 + 6);      //������������ ����� BIT_06 (p0.6) � �������� ������
  GP0DAT |= 1 << (24 + 2);      //������������ ����� BIT_07 (p0.2) � �������� ������

  //���������� �������� �������� ������
  ( cNewPhaseShift & 0x01) ? GP1DAT |= 1 << (16 + 5) : GP1DAT &= ~( 1 << (16 + 5));
  ( cNewPhaseShift & 0x02) ? GP0DAT |= 1 << (16 + 7) : GP0DAT &= ~( 1 << (16 + 7));
  ( cNewPhaseShift & 0x04) ? GP0DAT |= 1 << (16 + 1) : GP0DAT &= ~( 1 << (16 + 1));
  ( cNewPhaseShift & 0x08) ? GP2DAT |= 1 << (16 + 3) : GP2DAT &= ~( 1 << (16 + 3));
  ( cNewPhaseShift & 0x10) ? GP4DAT |= 1 << (16 + 6) : GP4DAT &= ~( 1 << (16 + 6));
  ( cNewPhaseShift & 0x20) ? GP4DAT |= 1 << (16 + 7) : GP4DAT &= ~( 1 << (16 + 7));
  ( cNewPhaseShift & 0x40) ? GP0DAT |= 1 << (16 + 6) : GP0DAT &= ~( 1 << (16 + 6));
  ( cNewPhaseShift & 0x80) ? GP0DAT |= 1 << (16 + 2) : GP0DAT &= ~( 1 << (16 + 2));

  GP2SET = 1 << (16 + 6);       //�������� ���� SET_PS_SH_ACT (pin36 = p2.6) - �������� ����� ������� �����
  dbl_N1 = dbl_N1 / 2.;
  GP2CLR = 1 << (16 + 6);       // ������� ���� SET_PS_SH_ACT (pin36 = p2.6) - �������� ����� ������� �����

  GP1DAT &= ~( 1 << (24 + 5));  //������������ ����� BIT_00 (p1.5) � �������� �����
  GP0DAT &= ~( 1 << (24 + 7));  //������������ ����� BIT_01 (p0.7) � �������� �����
  GP0DAT &= ~( 1 << (24 + 1));  //������������ ����� BIT_02 (p0.1) � �������� �����
  GP2DAT &= ~( 1 << (24 + 3));  //������������ ����� BIT_03 (p2.3) � �������� �����
  GP4DAT &= ~( 1 << (24 + 6));  //������������ ����� BIT_04 (p4.6) � �������� �����
  GP4DAT &= ~( 1 << (24 + 7));  //������������ ����� BIT_05 (p4.7) � �������� �����
  GP0DAT &= ~( 1 << (24 + 6));  //������������ ����� BIT_06 (p0.6) � �������� �����
  GP0DAT &= ~( 1 << (24 + 2));  //������������ ����� BIT_07 (p0.2) � �������� �����

  GP3DAT |= 1 << (16 + 4);    //SET_PS_SH_LINE  (p3.4) = 1



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

  dStartAmplAngCheck = ( double) gl_ush_flashParamAmplAngMin1 / 65535. * 3.;
  //dStartAmplAngCheck = 0.25;
  prt2val = T2VAL;
  ADCCP = 0x06;   //AmplAng channel
  while( 1) {
    ADCCON |= 0x80;
    while (!( ADCSTA & 0x01)){}
    gl_ssh_ampl_angle = (ADCDAT >> 16);
  /*
  #ifdef DEBUG
    printf("DEBUG: Hangerup vibration control: AA: %.02f    CONTROL: %.02f\n", ( double) gl_ssh_ampl_angle / 4095. * 2.5, dStartAmplAngCheck );
  #endif
  */
    if( ( ( double) gl_ssh_ampl_angle / 4095. * 2.5) > dStartAmplAngCheck) {
      //SUCCESS

      #ifdef DEBUG
        //printf("DEBUG: Hangerup vibration control: successfully passed\n");
        printf("DEBUG: Hangerup vibration control: successfully passed with AA: %.02f    CONTROL: %.02f\n", ( double) gl_ssh_ampl_angle / 4095. * 2.5, dStartAmplAngCheck );
      #endif

      break;
    }

    //printf( "%08X        %08X\n", T1VAL, T2VAL);
    //printf( "%f\n", ( double) (( prt2val + T2LD - T2VAL) % T2LD) / 32768.);

    if( ( double) (( prt2val + T2LD - T2VAL) % T2LD) / 32768. > 5.0) {
      #ifdef DEBUG
        printf("DEBUG: Hangerup vibration control: FAILED\n");
      #endif
     deadloop_no_hangerup();
    }
  }
#endif

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
    ADCCP = ADC_CHANNEL_I1;
    pause( 10);
    ADCCON |= 0x80;
    while (!( ADCSTA & 0x01)){}     // ������� ����� �������������� ���
    gl_ssh_current_1 = (ADCDAT >> 16);

    //�������� ��� I2
    ADCCP = ADC_CHANNEL_I2;
    pause( 10);
    ADCCON |= 0x80;
    while (!( ADCSTA & 0x01)){}     // ������� ����� �������������� ���
    gl_ssh_current_2 = (ADCDAT >> 16);

    /*if( ( ( double) gl_ssh_current_1 / 4096. * 3. / 3.973 < ( double) gl_ush_flashParamI1min / 65535. * 0.75)  ||
        ( ( double) gl_ssh_current_2 / 4096. * 3. / 3.973 < ( double) gl_ush_flashParamI2min / 65535. * 0.75)) {*/

#ifdef DEBUG
  printf("DEBUG: Laser fireup: code=0x%04x   I1=%.02f   CONTROL=%.02f\n",
            gl_ssh_current_1,
            ( 2.5 - ( double) gl_ssh_current_1 / 4096. * 2.5) / 2.5,
            ( double) gl_ush_flashParamI1min / 65535. * 0.75);
  printf("DEBUG: Laser fireup: code=0x%04x   I2=%.02f   CONTROL=%.02f\n",
            gl_ssh_current_2,
            ( 2.5 - ( double) gl_ssh_current_2 / 4096. * 2.5) / 2.5,
            ( double) gl_ush_flashParamI2min / 65535. * 0.75);
#endif

    if( ( ( 2.5 - ( double) gl_ssh_current_1 / 4096. * 2.5) / 2.5 < ( double) gl_ush_flashParamI1min / 65535. * 0.75)  ||
        ( ( 2.5 - ( double) gl_ssh_current_2 / 4096. * 2.5) / 2.5 < ( double) gl_ush_flashParamI2min / 65535. * 0.75)) {
        //�� ��������

        //������� 3kV ����� (������� 800V)
        GP4DAT |= ( 1 << (16 + 1));   //OFF3KV (p4.1) = 1

        //�������������� � ��������� ����� ������� �������
        if( ++gl_n3kVapplies >= 25) {

          //5 ����� �� 5 ������� ������� �� ��������� - ������������ � ������� ����

          //��������� ������� 800V
          GP4DAT |= ( 1 << (16 + 0));   //ONHV   (p4.0) = 1

          //��������� ������ 3kV
          GP4DAT |= ( 1 << (16 + 1));   //OFF3KV (p4.1) = 1

          #ifdef DEBUG
            printf("DEBUG: Laser fireup: FAILED\n");
          #endif

          deadloop_no_firing( ERROR_NO_LASER_FIRING);
        }

        if( ( gl_n3kVapplies % 5)) {
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
  gl_nADCChannel = 0;
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

#ifdef DEBUG
  printf("done\n");
#endif

  //**********************************************************************
  //�������������� ����������� ������ (������ ��� ������. ������)
  //**********************************************************************
  if( gl_b_SyncMode != 0) {
#ifdef DEBUG
    printf("DEBUG: Decrement coefficient initial value\n");
#endif
    //� ����� ������� � ��� ���� ���-�� ���������� �� �������� (������ ��� ��������� �������)
    switch( gl_cFlashParamDcCalibUsage) {
      case 0: //��� ���������� ������� �� ���������� ������� - �������������
        gl_ush_flashParamDecCoeff = gl_ush_calib_dc_dc[0];
        for( i=1; i<11; i++) {
          if( gl_ac_calib_dc_t[i] == 0xFF) break;
          if( gl_dbl_Tutd1 > gl_ac_calib_dc_t[i] - 128. ) gl_ush_flashParamDecCoeff = gl_ush_calib_dc_dc[i];
        }
      break;

      case 0xFE:  //���� ������ - ����� � �������� ��� �������
        FirstDecrementCoeffCalculation();
      break;
    }

#ifdef DEBUG
    printf("VALUE=%.2f  ", gl_ush_flashParamDecCoeff / 65535.);
    printf("passed\n");
#endif
  }


#ifdef DEBUG
  printf("DEBUG: Configuration passed. Main loop starts!\n");
#endif

  //������������� ���������� �������� ���������� ������� ������������ ������
  gl_dblMeanAbsDn = 0.;
  gl_dblMeanAbsDu = 0.;
  gl_nMeanDecCoeffCounter = 0;

  gl_un_PrevT2DecCoeffCalc = T2VAL;


  //FAKE ('VERACITY DATA' flag on)
  gl_n_PerimeterReset = 1;

  //Starting packs.VERSION
  gl_nSentAddParamIndex = VERSION;
  send_pack( ( ( VERSION_MINOR * 16) << 8) + ( VERSION_MAJOR * 16 + VERSION_MIDDLE));

  //Starting packs.Device_Num
  gl_nSentAddParamIndex = DEVNUM;
  send_pack( gl_ush_flashParamDeviceId);

  //FAKE ('VERACITY DATA' flag off)
  gl_n_PerimeterReset = 0;

  //����� ������ ���. �������� �� ������
  gl_nSentPackListIndex = 0;
  gl_nSentAddParamIndex = getNextAddParamDescriptorToSendFromList();


  //**********************************************************************
  //**********************************************************************
  //******************* �������� ���� ������ ��������� *******************
  //**********************************************************************
  //**********************************************************************
  //**********************************************************************

  //�������� ���� �������� ����������� ��������� (����� �������)
  gl_nActiveRegulationT2 = T2VAL;
  if( gl_nActiveRegulationT2 == 0) gl_nActiveRegulationT2 = 1;
  gl_un_RULAControl = 2457;    //2457 = 1.500 V
  DACConfiguration();


  while(1) {

    if( ( ( gl_nSecondsT2Lock + T2LD - T2VAL) % T2LD) > 32768) {
      gl_nSecondsT2Lock = ( gl_nSecondsT2Lock - 32768 + T2LD) % T2LD;
      gl_lSecondsFromStart++;
    }



    if( gl_bSimpleDnDuRegime == 1) {
      SimpleMaxRateDnDuRegime();
    }

    processIncomingCommand();

    if( gl_bSimpleDnDuRegime == 1) {
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

#ifdef DEBUG
  //printf( ".0x%08X 0x%08X\n", T1VAL, T2VAL);
#endif

        testPike();

        //������� ��������� TA_SA (��� ���������� ������������)
        gl_ssh_SA_time = ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
        gl_n_prT1VAL = T1VAL;

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
          #if DEBUG == 2
            PrintBinShortNumber(hb);
            printf(" ");
            PrintBinShortNumber(lb);
            printf(" ");
            PrintBinShortNumber( gl_ssh_angle_inc);
            printf("\n");
          #endif
        #endif



        testPike();

        // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
        // 2. � ������ ������ ������, ����������� � ������� ���� ���������� ������������
        if( gl_b_SyncMode) {

          #ifdef DEBUG
          #if DEBUG == 2
            printf( "DEBUG: AN_R...");
          #endif
          #endif

          //��� �������� ������ ������� ���������� ANGLE_READY (p2.4)
          while( !( GP2DAT & 0x10)) {
            putchar( ( GP2DAT & 0x10) ? '1':'0');
          }

          #ifdef DEBUG
          #if DEBUG == 2
            printf( "!\nDEBUG: RDHBA\n");
          #endif
          #endif

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


          #ifdef DEBUG
          #if DEBUG == 2
            printf( "DEBUG: RDLBA\n");
          #endif
          #endif

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
        #ifdef DEBUG
        #if DEBUG == 2
          printf( "DEBUG: AnPar...");
        #endif
        #endif

        while( !( ADCSTA & 0x01)){}     // ������� ����� �������������� ��� (������������ ����� �� �������� ���� �� ��� ������ ���� �����)

        #ifdef DEBUG
        #if DEBUG == 2
          printf( "got\n");
        #endif
        #endif

        switch( gl_nADCChannel) { //����������� ��� �� ������������ � ��������� � ��������������� ����������
          case 0: //UTD3
            gl_ssh_Utd3 = (ADCDAT >> 16);
            gl_ssh_Utd3_cal = gl_ssh_Utd3;
            if( gl_bTDCalibrated)
              gl_dbl_Tutd3 = ( double) gl_ssh_Utd3 * gl_dblTD3_K + gl_dblTD3_B;
            else
              gl_dbl_Tutd3 = -1481.96 + sqrt( 2.1962e6 + ( ( 1.8639 - ( double) gl_ssh_Utd3 / 4096. * 2.5) / 3.88e-6));
            gl_ssh_Utd3 = ( short) ( ( gl_dbl_Tutd3 + 100.) / 200. * 65535.);
          break;  //UTD3

          case 1: //UTD1
            gl_ssh_Utd1 = (ADCDAT >> 16);
            gl_ssh_Utd1_cal = gl_ssh_Utd1;
            if( gl_bTDCalibrated)
              gl_dbl_Tutd1 = ( double) gl_ssh_Utd1 * gl_dblTD1_K + gl_dblTD1_B;
            else
              gl_dbl_Tutd1 = -1481.96 + sqrt( 2.1962e6 + ( ( 1.8639 - ( double) gl_ssh_Utd1 / 4096. * 2.5) / 3.88e-6));
            gl_ssh_Utd1 = ( short) ( ( gl_dbl_Tutd2 + 100.) / 200. * 65535.);
          break;  //UTD1

          case 2: //UTD2
            gl_ssh_Utd2 = (ADCDAT >> 16);
            gl_ssh_Utd2_cal = gl_ssh_Utd2;
            if( gl_bTDCalibrated)
              gl_dbl_Tutd2 = ( double) gl_ssh_Utd2 * gl_dblTD2_K + gl_dblTD2_B;
            else
              gl_dbl_Tutd2 = -1481.96 + sqrt( 2.1962e6 + ( ( 1.8639 - ( double) gl_ssh_Utd2 / 4096. * 2.5) / 3.88e-6));
            gl_ssh_Utd2 = ( short) ( ( gl_dbl_Tutd2 + 100.) / 200. * 65535.);
          break;  //UTD2

          case 3: gl_ssh_current_1 = (ADCDAT >> 16); break;     //I1
          case 4: gl_ssh_current_2 = (ADCDAT >> 16); break;     //I2
          case 5: gl_ssh_Perim_Voltage = (ADCDAT >> 16); break; //CntrPc
          case 6: gl_ssh_ampl_angle = (ADCDAT >> 16); break;    //AmplAng
          case 8: gl_ssh_current_1 = (ADCDAT >> 16); break;     //I1   TEMP!!!!!!!!!!!!!!!!!!!!
        }

        if( gl_cCalibProcessState) {
          switch( gl_cCalibProcessState) {
            case 1:
              //��������� ������ ������ �� ����������� �����������
              if( gl_nADCChannel == 1) {
                gl_ush_flashParamT1_TD1_val = gl_ssh_Utd1_cal;
                gl_cCalibProcessState = 2;
              }
            break;

            case 2:
              //��������� ������ ������ �� ����������� �����������
              if( gl_nADCChannel == 2) {
                gl_ush_flashParamT1_TD2_val = gl_ssh_Utd2_cal;
                gl_cCalibProcessState = 3;
              }
            break;

            case 3:
              //��������� ������ ������ �� ����������� �����������
              if( gl_nADCChannel == 0) {
                gl_ush_flashParamT1_TD3_val = gl_ssh_Utd3_cal;
                gl_cCalibProcessState = 0;
                save_params_p4();
                gl_nSentAddParamIndex = CALIB_T1;
              }
            break;

            case 4:
              //��������� ������ ������ �� ������������ �����������
              if( gl_nADCChannel == 1) {
                gl_ush_flashParamT2_TD1_val = gl_ssh_Utd1_cal;
                gl_cCalibProcessState = 5;
              }
            break;

            case 5:
              //��������� ������ ������ �� ������������ �����������
              if( gl_nADCChannel == 2) {
                gl_ush_flashParamT2_TD2_val = gl_ssh_Utd2_cal;
                gl_cCalibProcessState = 6;
              }

            case 6:
              //��������� ������ ������ �� ������������ �����������
              if( gl_nADCChannel == 0) {
                gl_ush_flashParamT2_TD3_val = gl_ssh_Utd3_cal;
                gl_cCalibProcessState = 0;
                save_params_p4();
                gl_nSentAddParamIndex = CALIB_T1;
              }

            break;
          }

          if( !gl_cCalibProcessState)
            //���� ��� ����������� ���������� ����� ���� ����� - ������������� ������������� ���������
            ThermoCalibrationCalculation();
        }

        //������������ ������ ���
        switch( gl_nADCChannel) {
          case ADC_CHANNEL_UTD1:    gl_nADCChannel = ADC_CHANNEL_UTD2;    break;
          case ADC_CHANNEL_UTD2:    gl_nADCChannel = ADC_CHANNEL_UTD3;    break;
          case ADC_CHANNEL_UTD3:    gl_nADCChannel = ADC_CHANNEL_I1;      break;

          case ADC_CHANNEL_I1:      gl_nADCChannel = ADC_CHANNEL_I2;      break;
          case ADC_CHANNEL_I2:      gl_nADCChannel = ADC_CHANNEL_CNTRPC;  break;

          case ADC_CHANNEL_CNTRPC:  gl_nADCChannel = ADC_CHANNEL_AA;      break;
          case ADC_CHANNEL_AA:      gl_nADCChannel = ADC_CHANNEL_UTD1;    break;

          default:                  gl_nADCChannel = ADC_CHANNEL_UTD1;    break;
        }
        /*
        if( gl_nADCChannel == 2) gl_nADCChannel = ADC_CHANNEL_I1;
        else if( gl_nADCChannel == ADC_CHANNEL_I1) gl_nADCChannel=4;
        else
          gl_nADCChannel = (++gl_nADCChannel) % 7;        //����������� �������-��������� ���������� ������. ����������
        */

        ADCCP = gl_nADCChannel;          //���������� ����� ����� ���
        for( i=0; i<100; i++);
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
        }

        testPike();

        if( gl_lSecondsFromStart - gl_l5SecPrevValue > 5) {
          if( gl_nSentAddParamIndex == UTD1) {
            gl_nSentAddParamIndex = SECONDS_FROM_START;
            gl_l5SecPrevValue = gl_lSecondsFromStart;
          }
        }

        //**********************************************************************
        // ������ ������ �������� ���������
        //**********************************************************************
        switch( gl_nSentAddParamIndex) {
          //****************************************************************************************************************************************************************
          //REGULAR PACK
          //****************************************************************************************************************************************************************
          case UTD1:            send_pack( gl_ssh_Utd1);              break; //UTD1
          case UTD2:            send_pack( gl_ssh_Utd2);              break; //UTD2
          case UTD3:            send_pack( gl_ssh_Utd3);              break; //UTD3
          case I1:              send_pack( gl_ssh_current_1);         break; //I1
          case I2:              send_pack( gl_ssh_current_2);         break; //I2
          case CNTRPC:          send_pack( gl_ssh_Perim_Voltage);     break; //CntrPc
          case AMPLANG_ALTERA:  send_pack( gl_ush_MeanImpulses << 1); break; //AmplAng �� �������
          case AMPLANG_DUS:     send_pack( gl_ssh_ampl_angle);        break; //AmplAng � ����
          case RULA:            send_pack( gl_un_RULAControl);        break; //RULA

          //****************************************************************************************************************************************************************
          // PARAMETERS BY REQUEST
          //****************************************************************************************************************************************************************

          case AMPLITUDE:       send_pack( gl_ush_flashParamAmplitudeCode);   break; //������� ��������� ���������
          case TACT_CODE:       send_pack( gl_ush_flashParamTactCode);        break; //������� ���� ����� ���������
          case M_COEFF:         send_pack( gl_ush_flashParamMCoeff);          break; //������� ������������ ���������
          case STARTMODE:       send_pack( gl_ush_flashParamStartMode);       break; //������� ��������� ����
          case DECCOEFF:        send_pack( gl_ush_flashParamDecCoeff);        break; //����������� ������
          case CONTROL_I1:      send_pack( gl_ush_flashParamI1min);           break; //gl_ush_flashParamI1min
          case CONTROL_I2:      send_pack( gl_ush_flashParamI2min);           break; //gl_ush_flashParamI2min
          case CONTROL_AA:      send_pack( gl_ush_flashParamAmplAngMin1);     break; //gl_ush_flashParamAmplAngMin1

          /*
          case HV_APPLY_COUNT_SET: send_pack( flashParamHvApplyCount);  break; //HV apply cycles in pack
          */
          case HV_APPLY_COUNT_TR:  send_pack( gl_n3kVapplies);                break; //HV apply cycles applied in this run
          /*
          case HV_APPLY_DURAT_SET: send_pack( flashParamHvApplyDurat); break; //HV apply cycle duration
          case HV_APPLY_PACKS:     send_pack( flashParamHvApplyPacks); break; //HV apply packs
          */

          case SIGNCOEFF:       send_pack( gl_ush_flashParamSignCoeff);       break; //������� ��������� ������������
          case DEVNUM:          send_pack( gl_ush_flashParamDeviceId);        break; //Device_Num

          case DATE_Y:          send_pack( gl_ush_flashParamDateYear);        break; //Date.Year
          case DATE_M:          send_pack( gl_ush_flashParamDateMonth);       break; //Date.Month
          case DATE_D:          send_pack( gl_ush_flashParamDateDay);         break; //Date.Day

          case ORG_B1:          send_pack( gl_ac_flashParamOrg[ 0]);          break; //Organization.Byte1
          case ORG_B2:          send_pack( gl_ac_flashParamOrg[ 1]);          break; //Organization.Byte2
          case ORG_B3:          send_pack( gl_ac_flashParamOrg[ 2]);          break; //Organization.Byte3
          case ORG_B4:          send_pack( gl_ac_flashParamOrg[ 3]);          break; //Organization.Byte4
          case ORG_B5:          send_pack( gl_ac_flashParamOrg[ 4]);          break; //Organization.Byte5
          case ORG_B6:          send_pack( gl_ac_flashParamOrg[ 5]);          break; //Organization.Byte6
          case ORG_B7:          send_pack( gl_ac_flashParamOrg[ 6]);          break; //Organization.Byte7
          case ORG_B8:          send_pack( gl_ac_flashParamOrg[ 7]);          break; //Organization.Byte8
          case ORG_B9:          send_pack( gl_ac_flashParamOrg[ 8]);          break; //Organization.Byte9
          case ORG_B10:         send_pack( gl_ac_flashParamOrg[ 9]);          break; //Organization.Byte10
          case ORG_B11:         send_pack( gl_ac_flashParamOrg[10]);          break; //Organization.Byte11
          case ORG_B12:         send_pack( gl_ac_flashParamOrg[11]);          break; //Organization.Byte12
          case ORG_B13:         send_pack( gl_ac_flashParamOrg[12]);          break; //Organization.Byte13
          case ORG_B14:         send_pack( gl_ac_flashParamOrg[13]);          break; //Organization.Byte14
          case ORG_B15:         send_pack( gl_ac_flashParamOrg[14]);          break; //Organization.Byte15
          case ORG_B16:         send_pack( gl_ac_flashParamOrg[15]);          break; //Organization.Byte16    ��� ������������ 0 �� �����!!!!!

          case VERSION:         send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE)); break; //SOFTWARE VERSION

          case CALIB_T1:        send_pack( gl_ssh_flashParam_calibT1);    break; //min thermo-calib point T
          case T1_TD1:          send_pack( gl_ush_flashParamT1_TD1_val);  break; //min thermo-calib point thermo1 data
          case T1_TD2:          send_pack( gl_ush_flashParamT1_TD2_val);  break; //min thermo-calib point thermo2 data
          case T1_TD3:          send_pack( gl_ush_flashParamT1_TD3_val);  break; //min thermo-calib point thermo3 data
          case CALIB_T2:        send_pack( gl_ssh_flashParam_calibT2);    break; //max thermo-calib point T
          case T2_TD1:          send_pack( gl_ush_flashParamT2_TD1_val);  break; //max thermo-calib point thermo1 data
          case T2_TD2:          send_pack( gl_ush_flashParamT2_TD2_val);  break; //max thermo-calib point thermo2 data
          case T2_TD3:          send_pack( gl_ush_flashParamT2_TD3_val);  break; //max thermo-calib point thermo3 data

          case AMPL_HOLD_MEAN:  send_pack( gl_lnMeanImps);              break; //amplitude hold algoryhtm: mean
          case AMPL_HOLD_ROUND: send_pack( gl_snMeaningCounterRound);   break; //amplitude hold algoryhtm: round
          case AMPL_HOLD_ACTIVE:send_pack( gl_nActiveRegulationT2?1:0); break; //amplitude hold algoryhtm: falg of active regulation

          case SECONDS_FROM_START: send_pack( gl_lSecondsFromStart & 0xFFFF);    break; //From start seconds timer

          case SLG_ADDITIONAL_PARAM_LIST_ELEMENT:
                                send_pack( ( ( gl_aushListOutputAddParams[ gl_nSentAddParamSubIndex] & 0xFF) << 8) + gl_nSentAddParamSubIndex);
                                                                                 break; //element of analogue (additional) output list

          case PH_SH_CALIB_T:
                                send_pack( ( gl_ac_calib_phsh_t[ gl_nSentAddParamSubIndex] << 8) + gl_nSentAddParamSubIndex);
                                                                                 break; //���������� �������� ������. �����������

          case PH_SH_CALIB_PH_SH:
                                send_pack( ( gl_ac_calib_phsh_phsh[ gl_nSentAddParamSubIndex] << 8) + gl_nSentAddParamSubIndex);
                                                                                 break; //���������� �������� ������. ������� �����


          case PH_SH_USAGE:     send_pack( gl_cFlashParamPhaseShiftUsage);       break; //���������� �������� ������. �������������

          case PH_SH_CURRENT_VAL: 
                                send_pack( gl_nCurrentPhaseShift);               break; //���������� �������� ������. ������� ������������ �������� �������� ������.

          case DC_CALIB_T:
                                send_pack( ( gl_ac_calib_dc_t[ gl_nSentAddParamSubIndex] << 8) + gl_nSentAddParamSubIndex);
                                                                                 break; //���������� ������������ ������. �����������

          case DC_CALIB_DC_L:
                                send_pack( ( ( gl_ush_calib_dc_dc[ gl_nSentAddParamSubIndex] & 0xFF) << 8) + gl_nSentAddParamSubIndex);
                                                                                 break; //���������� ������������ ������. ����������� ������. ������� ����

          case DC_CALIB_DC_H:
                                send_pack( ( gl_ush_calib_dc_dc[ gl_nSentAddParamSubIndex] & 0xFF00) + gl_nSentAddParamSubIndex);
                                                                                 break; //���������� ������������ ������. ����������� ������. ������� ����

          case DC_CALIB_USAGE:  send_pack( gl_cFlashParamDcCalibUsage);          break; //���������� ������������ ������. �������������

          default:              send_pack( gl_ssh_Utd1);                         break; //DEFAULT = UTD1
        }

        //������� ���������, ����� ���. �������� �� ������� � ��������� ���
        if( isAddParamDecriptorInList( gl_nSentAddParamIndex)) {
          //���� ������� �������� ���. �������� �� ������ - �� ����� ��������� �� ������
          gl_nSentAddParamIndex = getNextAddParamDescriptorToSendFromList();
        }
        else {
          //���� ������� �������� ���. �������� �� �� ������
          switch( gl_nSentAddParamIndex) {
            case CONTROL_I1:      gl_nSentAddParamIndex = CONTROL_I2;     break; //gl_ush_flashParamI1min
            case CONTROL_I2:      gl_nSentAddParamIndex = CONTROL_AA;     break; //gl_ush_flashParamI2min

            case ORG_B1:          gl_nSentAddParamIndex = ORG_B2;        break; //Organization.Byte1
            case ORG_B2:          gl_nSentAddParamIndex = ORG_B3;        break; //Organization.Byte2
            case ORG_B3:          gl_nSentAddParamIndex = ORG_B4;        break; //Organization.Byte3
            case ORG_B4:          gl_nSentAddParamIndex = ORG_B5;        break; //Organization.Byte4
            case ORG_B5:          gl_nSentAddParamIndex = ORG_B6;        break; //Organization.Byte5
            case ORG_B6:          gl_nSentAddParamIndex = ORG_B7;        break; //Organization.Byte6
            case ORG_B7:          gl_nSentAddParamIndex = ORG_B8;        break; //Organization.Byte7
            case ORG_B8:          gl_nSentAddParamIndex = ORG_B9;        break; //Organization.Byte8
            case ORG_B9:          gl_nSentAddParamIndex = ORG_B10;       break; //Organization.Byte9
            case ORG_B10:         gl_nSentAddParamIndex = ORG_B11;       break; //Organization.Byte10
            case ORG_B11:         gl_nSentAddParamIndex = ORG_B12;       break; //Organization.Byte11
            case ORG_B12:         gl_nSentAddParamIndex = ORG_B13;       break; //Organization.Byte12
            case ORG_B13:         gl_nSentAddParamIndex = ORG_B14;       break; //Organization.Byte13
            case ORG_B14:         gl_nSentAddParamIndex = ORG_B15;       break; //Organization.Byte14
            case ORG_B15:         gl_nSentAddParamIndex = ORG_B16;       break; //Organization.Byte15


            case CALIB_T1:        gl_nSentAddParamIndex = T1_TD1;        break; //min thermo-calib point T
            case T1_TD1:          gl_nSentAddParamIndex = T1_TD2;        break; //min thermo-calib point thermo1 data
            case T1_TD2:          gl_nSentAddParamIndex = T1_TD3;        break; //min thermo-calib point thermo2 data
            case T1_TD3:          gl_nSentAddParamIndex = CALIB_T2;      break; //min thermo-calib point thermo3 data
            case CALIB_T2:        gl_nSentAddParamIndex = T2_TD1;        break; //max thermo-calib point T
            case T2_TD1:          gl_nSentAddParamIndex = T2_TD2;        break; //max thermo-calib point thermo1 data
            case T2_TD2:          gl_nSentAddParamIndex = T2_TD3;        break; //max thermo-calib point thermo2 data


            case AMPL_HOLD_MEAN:  gl_nSentAddParamIndex = AMPL_HOLD_ROUND;   break; //amplitude hold algoryhtm: mean
            case AMPL_HOLD_ROUND: gl_nSentAddParamIndex = AMPL_HOLD_ACTIVE;  break; //amplitude hold algoryhtm: round

            case DC_CALIB_DC_L:   gl_nSentAddParamIndex = DC_CALIB_DC_H;     break; //���������� ������������ ������. ��������� ����������� ������. ������� ���� (�� ��� ����-������� �������)

            default:
              gl_nSentAddParamIndex = getNextAddParamDescriptorToSendFromList();
          }
        }


        #ifdef DEBUG
        #if DEBUG == 2
          printf( "DEBUG: DC\n");
        #endif
        #endif

        //�������������� ������������ ������
        switch( gl_cFlashParamDcCalibUsage) {

          case 0: //����� ������������� ����������
            if( gl_lSecondsFromStart >= gl_lCalibratedDcApplySecs) {
              gl_lCalibratedDcApplySecs = gl_lSecondsFromStart + 60;

              gl_ush_flashParamDecCoeff = gl_ush_calib_dc_dc[0];
              for( i=1; i<11; i++) {
                if( gl_ac_calib_dc_t[i] == 0xFF) break;
                if( gl_dbl_Tutd1 > gl_ac_calib_dc_t[i] - 128. ) gl_ush_flashParamDecCoeff = gl_ush_calib_dc_dc[i];
              }

            }
          break;

          case 2: //�������������� �� ����

            if( gl_b_SyncMode && gl_lSecondsFromStart) {

              dbl_N1 = ( double) gl_ssh_angle_inc_prev;
              dbl_N2 = ( double) gl_ssh_angle_inc;
              dbl_U1 = ( double) gl_ssh_angle_hanger_prev;
              dbl_U2 = ( double) gl_ssh_angle_hanger;

              Coeff = (( float) gl_ush_flashParamDecCoeff) / 65535.;
              gl_dbl_Omega =  ( dbl_N2 - dbl_N1) - ( dbl_U2 - dbl_U1) * Coeff * ( ( signed short) gl_ush_flashParamSignCoeff - 1);
              if( fabs( gl_dbl_Omega) < 5) {
                gl_dbl_Nsumm += fabs( ( double) gl_ssh_angle_inc - ( double) gl_ssh_angle_inc_prev);
                gl_dbl_Usumm += fabs( ( double) gl_ssh_angle_hanger - ( double) gl_ssh_angle_hanger_prev);
                gl_un_DecCoeffStatPoints++;
                if( ( gl_un_DecCoeffStatPoints % DEC_COEFF_CONTINUOUS_CALCULATION_N) == 0 &&
                    ( ( ( gl_un_PrevT2DecCoeffCalc + T2LD - T2VAL) % T2LD) >= 32768 ) ) {
                  //gl_ush_flashParamDecCoeff = ( short) ( ( gl_dbl_Nsumm / gl_dbl_Usumm * 65535.));
                  gl_ush_flashParamDecCoeff = ( short) ( ( int) ( gl_dbl_Nsumm / gl_dbl_Usumm * 65535.));
                  gl_dbl_Nsumm = gl_dbl_Usumm = 0.;
                  gl_un_DecCoeffStatPoints = 0;
                  gl_nSentAddParamIndex = DECCOEFF;
                }
              }
            }
          break;

        }


        #ifdef DEBUG
          #if DEBUG == 2
            printf( "%d     %d     %d      %.2fV\n", gl_ssh_angle_inc, gl_ssh_angle_inc - gl_ssh_angle_inc_prev, gl_ssh_angle_hanger- gl_ssh_angle_hanger_prev, ( double) gl_ssh_angle_hanger * 0.61 / 1000.);
          #endif
        #endif

        testPike();

        gl_ssh_angle_inc_prev = gl_ssh_angle_inc;

        // ************************************************************************************
        // 2010-04-22
        //�������������� ����������� ���������
        // ************************************************************************************
        //���� ����� =6, �� ���� �� ������ ��� ���������� � ��������� ���������� ����������������
        if( gl_nADCChannel == 6) {
          if( gl_ssh_Perim_Voltage > 3637 ||      //V_piezo > 100
              gl_ssh_Perim_Voltage < 393) {       //V_piezo < -100
            //gl_ush_flashParamStartMode = 125;
            //DACConfiguration();
            GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1
            gl_nRppTimerT2 = T2VAL;
            gl_n_PerimeterReset = 1;
          }
        }

        testPike();

        //**********************************************************************
        //������������ ��������� ��������� ������������
        //**********************************************************************
        gl_snMeaningCounter = ( ++gl_snMeaningCounter) % gl_snMeaningCounterRound;

        //���������� ���� ���������� ���������� RULA
        if( gl_snMeaningCounter == 0) {

          //�� �������... ����� �������� ��������� � ���� ����� ���������
          gl_lnMeanImps = gl_lnMeaningSumm >> ( gl_snMeaningShift - 4);
          //gl_lnMeanImps = gl_lnMeanImps >> 1;

          if( gl_lnMeanImps > 90) {  //90 = 16 * 5
            //�� ����.... ��������� � ������ �� �������� 4095=2,5�, (2016.02.05 14:15, ��������: 2.5 ��� 3?)
            //����� ������� 2,2�=120"
            //� �������� �� ���������� ����������� 2,9 �� �������� ����� ���������
            //gl_dMeanImps = gl_dMeaningSumm / ( double) gl_snMeaningCounterRound / 4095. * 2.5 / 2.2 * 120. / 2.9;

            if( abs( gl_lnMeanImps - ( gl_ush_flashParamAmplitudeCode << 4)) > 1) {    //�� ���� ��������� �� ������� ���� ������� �� ������ 1/16 �� ��������
              if( gl_lnMeanImps > ( gl_ush_flashParamAmplitudeCode << 4)) {

                //gl_un_RULAControl -= gl_nDelta;

                if( gl_nDelta >= gl_un_RULAControl) {
                  //delta = cRULAControl;
                  gl_un_RULAControl = gl_un_RULAControl / 2;
                }
                else
                  gl_un_RULAControl -= gl_nDelta;

              }
              if( gl_lnMeanImps < ( gl_ush_flashParamAmplitudeCode << 4)) {

                //gl_un_RULAControl += gl_nDelta;

                if( gl_un_RULAControl + gl_nDelta > RULA_MAX) {
                  //delta = 255 - cRULAControl;
                  gl_un_RULAControl = ( RULA_MAX + gl_un_RULAControl) / 2;
                }
                else
                  gl_un_RULAControl += gl_nDelta;

              }
            }

            if( gl_un_RULAControl > RULA_MAX) gl_un_RULAControl = RULA_MAX;
            if( gl_un_RULAControl < RULA_MIN) gl_un_RULAControl = RULA_MIN;

            //���������� ��������� "��������" (���� ��� ��� > 1)
            gl_nDelta = gl_nDelta >> 1;
            if( gl_nDelta < 1) {
              gl_nDelta = 1;
            }

            /*
            //��������� "�����������" ���������� ����� 7 ������
            if( gl_nT2StartDataOut) {
              if( ( T2LD + gl_nT2StartDataOut - T2VAL) % T2LD >= 32768 * 7) {
                /*
                gl_nDelta = ( RULA_MAX - RULA_MIN) / 4;
                gl_snMeaningCounterRound = MEANING_IMP_PERIOD_100;
                */
                /*
                gl_nT2StartDataOut = 0;
              }
            }
            */


            //������� ����� (����� ���������� ����������) ��� �� �������������� ��������
            //2014.10.09 - ������� ��� ���� ������� ������� - ��������� � �����������
            
            if( gl_nActiveRegulationT2 != 0) {
              if( gl_nDelta < 10) {
                //�������� ����������� ���������
                if(      abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 160){ gl_nDelta = 200; gl_snMeaningCounterRound = 128;  gl_snMeaningShift = 7; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 90) { gl_nDelta = 100; gl_snMeaningCounterRound = 128;  gl_snMeaningShift = 7; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 16) { gl_nDelta = 50;  gl_snMeaningCounterRound = 128;  gl_snMeaningShift = 7; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 14) { gl_nDelta = 25;  gl_snMeaningCounterRound = 128;  gl_snMeaningShift = 7; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 13) { gl_nDelta = 12;  gl_snMeaningCounterRound = 256;  gl_snMeaningShift = 8; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 11) {                  gl_snMeaningCounterRound = 256;  gl_snMeaningShift = 8; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 10) {                  gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) >  8) {                  gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else                                                                        {                  gl_snMeaningCounterRound = 1024; gl_snMeaningShift = 10; }
              }
            }
            else {
              //����������� ��������� � �������� ������ �������
              if( gl_nDelta == 1) {

                if(      abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 160){ gl_nDelta = 64; gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 90) { gl_nDelta = 32; gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 16) { gl_nDelta = 16; gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 14) { gl_nDelta = 8;  gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 13) { gl_nDelta = 4;  gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 11) {                 gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 10) {                 gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) >  8) {                 gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else                                                                        {                 gl_snMeaningCounterRound = 1024; gl_snMeaningShift = 10; }
              }
            }
          }

          gl_lnMeaningSumm = 0;

        }
        else {
          gl_lnMeaningSumm += gl_ush_MeanImpulses;    //�� �������
          //gl_dMeaningSumm += gl_ssh_ampl_angle;    //�� ����
        }

        //**********************************************************************
        //��������� ������ ����� �������� ����������� ���������
        //**********************************************************************
        if( gl_nActiveRegulationT2 != 0) {
          if( (( T2LD + gl_nActiveRegulationT2 - T2VAL) % T2LD) > 32768. * 10.0) {    //������������ ���� �������� ����������� ��������� 10 ���
            gl_nActiveRegulationT2 = 0;
          }
        }

        DACConfiguration();

        //**********************************************************************
        //��������� ����� ������ RP_P
        //**********************************************************************
        if( gl_nRppTimerT2 != 0) {

          if( gl_n_PerimeterReset == 1) {

            //32768 = 1 sec
            //3276  = 0.1 sec    = 100 msec
            //327   = 0.01 sec   = 10 msec
            //32    = 0.001 sec  = 1 msec
            if( (( T2LD + gl_nRppTimerT2 - T2VAL) % T2LD) > 32) {

              //����� (���������) ������������ � ������� ����������� ���������
              GP0DAT &= ~( 1 << (16 + 5));  //RP_P   (p0.5) = 0
              gl_nRppTimerT2 = T2VAL;
              gl_n_PerimeterReset = 2;
            }
          }
          else if( gl_n_PerimeterReset == 2) {

            //32768 = 1 sec
            //3276  = 0.1 sec = 100 msec
            //327   = 0.01 sec = 10 msec
            if( (( T2LD + gl_nRppTimerT2 - T2VAL) % T2LD) > 3276) {
              gl_n_PerimeterReset = 0;
              gl_nRppTimerT2 = 0;
            }
          }

        }


        #ifdef DEBUG
        #if DEBUG == 2
          printf( "DEBUG: PS\n");
        #endif
        #endif

        //��������� �������� ������
        cNewPhaseShift = gl_nCurrentPhaseShift;

        switch( gl_cFlashParamPhaseShiftUsage) {
          case 0:
            //���������� �� �������

            if( gl_nCurrentPhaseShift & 0x100) {
              //������ ������� - ������� ��... �������... ������� ����� 5 ���, � �����, ��� ������ ������, ����� �������� �� ������������� �������
              cNewPhaseShift = gl_nCurrentPhaseShift - 0x100;
              gl_lCalibratedPhaseShiftApplySecs = gl_lSecondsFromStart + 300;
            }
            else {
              if( gl_lSecondsFromStart >= gl_lCalibratedPhaseShiftApplySecs) {
                //������ ����� ����������� �� �������
                gl_lCalibratedPhaseShiftApplySecs = gl_lSecondsFromStart + 300;

                cNewPhaseShift = gl_ac_calib_phsh_phsh[0];
                for( i=1; i<11; i++) {
                  if( gl_ac_calib_phsh_t[i] == 0xFF) break;
                  dblTdCalib = gl_ac_calib_phsh_t[i] - 128.;
                  if( gl_dbl_Tutd1 > dblTdCalib ) cNewPhaseShift = gl_ac_calib_phsh_phsh[i];
                }
              }
            }
          break;

          case 1:
            //������ ����� ��������� (��������� � ������)
            if( gl_nCurrentPhaseShift & 0x100) {
              //������ ������� - �������� ��
              cNewPhaseShift = gl_nCurrentPhaseShift - 0x100;
            }
          break;

          //� ��������� ������� ��������� �������� ������ �� ����������
        }

        if( cNewPhaseShift != gl_nCurrentPhaseShift) {
          gl_nCurrentPhaseShift = cNewPhaseShift;

          //����������� ����������� �������� ������ �� ���� �� ���������������� � ������� (pin38 -> 0)
          GP3DAT &= ~( 1 << (16 + 4));  //SET_PS_SH_LINE  (p3.4) = 0

          GP1DAT |= 1 << (24 + 5);      //������������ ����� BIT_00 (p1.5) � �������� ������
          GP0DAT |= 1 << (24 + 7);      //������������ ����� BIT_01 (p0.7) � �������� ������
          GP0DAT |= 1 << (24 + 1);      //������������ ����� BIT_02 (p0.1) � �������� ������
          GP2DAT |= 1 << (24 + 3);      //������������ ����� BIT_03 (p2.3) � �������� ������
          GP4DAT |= 1 << (24 + 6);      //������������ ����� BIT_04 (p4.6) � �������� ������
          GP4DAT |= 1 << (24 + 7);      //������������ ����� BIT_05 (p4.7) � �������� ������
          GP0DAT |= 1 << (24 + 6);      //������������ ����� BIT_06 (p0.6) � �������� ������
          GP0DAT |= 1 << (24 + 2);      //������������ ����� BIT_07 (p0.2) � �������� ������

          //���������� �������� �������� ������
          ( gl_nCurrentPhaseShift & 0x01) ? GP1DAT |= 1 << (16 + 5) : GP1DAT &= ~( 1 << (16 + 5));
          ( gl_nCurrentPhaseShift & 0x02) ? GP0DAT |= 1 << (16 + 7) : GP0DAT &= ~( 1 << (16 + 7));
          ( gl_nCurrentPhaseShift & 0x04) ? GP0DAT |= 1 << (16 + 1) : GP0DAT &= ~( 1 << (16 + 1));
          ( gl_nCurrentPhaseShift & 0x08) ? GP2DAT |= 1 << (16 + 3) : GP2DAT &= ~( 1 << (16 + 3));
          ( gl_nCurrentPhaseShift & 0x10) ? GP4DAT |= 1 << (16 + 6) : GP4DAT &= ~( 1 << (16 + 6));
          ( gl_nCurrentPhaseShift & 0x20) ? GP4DAT |= 1 << (16 + 7) : GP4DAT &= ~( 1 << (16 + 7));
          ( gl_nCurrentPhaseShift & 0x40) ? GP0DAT |= 1 << (16 + 6) : GP0DAT &= ~( 1 << (16 + 6));
          ( gl_nCurrentPhaseShift & 0x80) ? GP0DAT |= 1 << (16 + 2) : GP0DAT &= ~( 1 << (16 + 2));

          GP2SET = 1 << (16 + 6);       //�������� ���� SET_PS_SH_ACT (pin36 = p2.6) - �������� ����� ������� �����
          for( i=0; i<100; i++);        // �����
          GP2CLR = 1 << (16 + 6);       // ������� ���� SET_PS_SH_ACT (pin36 = p2.6) - �������� ����� ������� �����

          GP1DAT &= ~( 1 << (24 + 5));  //������������ ����� BIT_00 (p1.5) � �������� �����
          GP0DAT &= ~( 1 << (24 + 7));  //������������ ����� BIT_01 (p0.7) � �������� �����
          GP0DAT &= ~( 1 << (24 + 1));  //������������ ����� BIT_02 (p0.1) � �������� �����
          GP2DAT &= ~( 1 << (24 + 3));  //������������ ����� BIT_03 (p2.3) � �������� �����
          GP4DAT &= ~( 1 << (24 + 6));  //������������ ����� BIT_04 (p4.6) � �������� �����
          GP4DAT &= ~( 1 << (24 + 7));  //������������ ����� BIT_05 (p4.7) � �������� �����
          GP0DAT &= ~( 1 << (24 + 6));  //������������ ����� BIT_06 (p0.6) � �������� �����
          GP0DAT &= ~( 1 << (24 + 2));  //������������ ����� BIT_07 (p0.2) � �������� �����

          GP3DAT |= 1 << (16 + 4);    //SET_PS_SH_LINE  (p3.4) = 1
        }




        //��������� ���� � ��� ��� ������� ������� ������� SA �� ����������
        gl_b_SA_Processed = 1;

        testPike();
      }
/*
#ifdef DEBUG
  printf("DEBUG: Main loop ends... %d %d \n", gl_nSentPacksCounter, gl_b_SA_Processed);
#endif
*/
    }
    else {
      //���� ����� ������� SA � ������ ������ - �� ��� ������ ��� ���������� �������� ����� �������������� ����
      gl_b_SA_Processed = 0;

      /*
      //�������� ������������
      ush_SA_check_time = ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;

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
      */
    }
  }
}
