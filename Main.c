#include <ADuC7026.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "serial.h"
#include "flashEE.h"
#include "errors.h"
#include "version.h"

#define LONG_OUTPUT_PACK_LEN 24       //когда запрашивают мы выдаем 25 пачки (со всеми доп. параметрами по очереди)
#define SHORT_OUTPUT_PACK_LEN 7       //в норме мы выдаем циклически по 7 пачек с аналог. параметрами

//#define HIRO_COEFF   1.      //большой гироскоп
//#define HIRO_COEFF  -1.      //маленький гироскоп

//#define DEBUG
//#define SKIP_START_CHECKS

char gl_c_numbers[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

//********************
// Decrement coefficient calculation
#define DEC_COEFF_FIRST_CALCULATION_N 100
#define DEC_COEFF_CONTINUOUS_CALCULATION_N 1000
unsigned int gl_un_DecCoeffStatPoints = 0;
double gl_dbl_Nsumm = 0.;
double gl_dbl_Usumm = 0.;
double gl_dbl_Omega;

//********************

signed short gl_ssh_angle_inc = 0;      //приращение угла
signed short gl_ssh_angle_inc_prev = 0; //приращение угла

signed short gl_ssh_current_1 = 0;      //разрядный ток 1
signed short gl_ssh_current_2 = 0;      //разрядный ток 2
signed short gl_ssh_Perim_Voltage = 5;  //напряжение контроля пьезокорректоров
signed short gl_ssh_ampl_angle = 0;     //напряжение пропорциональное амплитуде выходного сигнала усилителя датчика угл. скорости

signed short gl_ssh_Utd1 = 0;           //напряжение корпусного термодатчика
signed short gl_ssh_Utd2 = 0;           //напряжение лазерного термодатчика
signed short gl_ssh_Utd3 = 0;           //напряжение ? термодатчика
signed short gl_ssh_Utd1_cal = 0;       //напряжение корпусного термодатчика (калибр.)
signed short gl_ssh_Utd2_cal = 0;       //напряжение лазерного термодатчика (калибр.)
signed short gl_ssh_Utd3_cal = 0;       //напряжение ? термодатчика (калибр.)

signed short gl_ssh_SA_time = 0;        //период SA
signed short gl_ssh_prT1VAL = 0x1000;   //период SA

signed short gl_ssh_angle_hanger = 0;
signed short gl_ssh_angle_hanger_prev = 0;

unsigned short gl_ush_MeanImpulses = 1;

#define RULA_MAX 4096
#define RULA_MIN 25

//unsigned char cRulaH = RULA_MAX, cRulaL = RULA_MIN;
//unsigned char cRULAControl = 40; //( RULA_MAX - RULA_MIN) / 2;      127 = 1.25V (Dac0)

//unsigned int gl_un_RULAControl = 0;       //0    = 0.000 V
//unsigned int gl_un_RULAControl = 64;      //64   = 0.039 V

//unsigned int gl_un_RULAControl = 1638;    //1638 = 1.000 V 
//unsigned int gl_un_RULAControl = 2457;    //2457 = 1.500 V
unsigned int gl_un_RULAControl = 4095;    //4095 = 2.500 V

//unsigned char delta = ( RULA_MAX - RULA_MIN) / 2;
unsigned int nDelta = ( RULA_MAX - RULA_MIN) / 2;

#define MEANING_IMP_PERIOD_100 100
#define MEANING_IMP_PERIOD_200 200
#define MEANING_IMP_PERIOD_300 300
#define MEANING_IMP_PERIOD_400 400
#define MEANING_IMP_PERIOD_500 500
#define MEANING_IMP_PERIOD_STABLE 1000

int gl_sn_MeaningCounter = 0;
int gl_sn_MeaningCounterRound = 500;
double dMeaningSumm = 0.;
double dMeanImps = 0.;
int nT2RepeatBang;

char gl_b_PerimeterReset = 0;
char gl_c_EmergencyCode = 0;
char gl_b_SA_Processed = 0;	          //флаг окончания обработки сигнала SA
char gl_b_SyncMode = 0;               //флаг режима работы гироскопа:   0=синхр. 1=асинхр.
char bAsyncDu = 0;                    //флаг передачи времени SA или приращ. угла в асинхр. режиме: 0-передается SA 1-передается dU

short nSentPacksCounter = 0;                    //счетчик посылок
int nSentPacksRound = SHORT_OUTPUT_PACK_LEN;    //круг счетчика посылок
char gl_c_OutPackCounter = 0;                   //выдаваемый наружу счётчик посылок

int ADCChannel = 0; //читаемый канал АЦП
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

#define IN_COMMAND_BUF_LEN 3
char input_buffer[6] = { 0, 0, 0, 0, 0, 0};
char pos_in_in_buf = 0;

unsigned short flashParamAmplitudeCode = 90, flashParamTactCode = 0, flashParamMCoeff = 4, flashParamStartMode = 5;
unsigned int flashParamDeviceId = 0;
char flashParamOrg[17] = { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0};
unsigned short flashParamDateYear = 0, flashParamDateMonth = 0, flashParamDateDay = 0;
unsigned short flashParamI1min = 0;
unsigned short flashParamI2min = 0;
unsigned short flashParamAmplAngMin1 = 0;
unsigned short flashParamDecCoeff = 0;
unsigned short flashParamSignCoeff = 2;
unsigned short flashParamPhaseShift = 0;

unsigned short gl_ushFlashParamLastRULA = 0;
unsigned short gl_ushFlashParamLastRULM = 0;

int gl_nSaveRulaRulm = 0;

double dStartAmplAngCheck = 0.5;

unsigned short nFiringTry = 0;

//калибровка термодатчиков
signed short flashParam_calibT1;
unsigned short flashParamT1_TD1_val, flashParamT1_TD2_val, flashParamT1_TD3_val;
signed short flashParam_calibT2;
unsigned short flashParamT2_TD1_val, flashParamT2_TD2_val, flashParamT2_TD3_val;

char bCalibProcessState;
#define MIN_T_THERMO_CALIBRATION -60
#define MAX_T_THERMO_CALIBRATION 60
#define THERMO_CALIB_PARAMS_BASE 10000
char bCalibrated;
double TD1_K, TD1_B;
double TD2_K, TD2_B;

int gl_nAppliedMCoeff;

#define AMPLFORT_ROUND_BUFLEN 3000
unsigned short gl_ushCircleBufferAmplForT[ AMPLFORT_ROUND_BUFLEN];
int gl_nCircleBufferAmplForTPositon = 0;
char gl_bCircleBufferAmplForTOverRound = 0;
double gl_dblCircleBufferSumm = 0.;
double gl_dblCircleBufferMean;

int gl_nAmplStabStep = 0;
int gl_nAmplStabApplyRulaTacts = 1;
int gl_nAmplStabMovAvWidth = 100;

void CircleBufferAmplForT_add( unsigned short newVal) {
  gl_dblCircleBufferSumm -= gl_ushCircleBufferAmplForT[ gl_nCircleBufferAmplForTPositon];
  gl_dblCircleBufferSumm += newVal;

  gl_ushCircleBufferAmplForT[ gl_nCircleBufferAmplForTPositon] = newVal;
  if( ++gl_nCircleBufferAmplForTPositon == gl_nAmplStabMovAvWidth) {
    gl_nCircleBufferAmplForTPositon = 0;
    gl_bCircleBufferAmplForTOverRound = 1;
  }

  if( gl_bCircleBufferAmplForTOverRound == 1)
    gl_dblCircleBufferMean = gl_dblCircleBufferSumm / ( double) AMPLFORT_ROUND_BUFLEN;
  else
    gl_dblCircleBufferMean = gl_dblCircleBufferSumm / ( double) gl_nCircleBufferAmplForTPositon;

}



/*
double CircleBufferAmplForT_mean( int nLen) {
  double dblSumm = 0.;
  double dblMean = 0.;
  int i;

  if( nLen > AMPLFORT_ROUND_BUFLEN)
    nLen = AMPLFORT_ROUND_BUFLEN;

  if( gl_bCircleBufferAmplForTOverRound == 0) {

    if( gl_nCircleBufferAmplForTPositon == 0) return 0.;

    if( nLen >= gl_nCircleBufferAmplForTPositon)
      nLen = gl_nCircleBufferAmplForTPositon;

    for( i=0; i<nLen; i++) {
      int nIndx = gl_nCircleBufferAmplForTPositon - 1 - i;
      dblSumm += ( double) gl_ushCircleBufferAmplForT[ nIndx];
    }

    dblMean = dblSumm / ( double) nLen;
  }
  else {
    for( i=0; i < nLen; i++) {
      int nIndx = ( gl_nCircleBufferAmplForTPositon - 1 - i + AMPLFORT_ROUND_BUFLEN) % AMPLFORT_ROUND_BUFLEN;
      dblSumm += ( double) gl_ushCircleBufferAmplForT[ nIndx];
    }

    dblMean = dblSumm / ( double) nLen;
  }

  return dblMean;
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//обработчик прерываний
void FIQ_Handler (void)	__fiq
{
  if( ( FIQSTA & UART_BIT) != 0)	{
    if( pos_in_in_buf < IN_COMMAND_BUF_LEN)
      input_buffer[ pos_in_in_buf++] = COMRX;
    //GP0DAT = ( 1 << 16);
  }
}

void pause( int n) {
  unsigned int prval, chk;
  prval = T1VAL;
  chk = (( T1LD + prval - T1VAL) % T1LD);
  while( chk < n)
    chk = (( T1LD + prval - T1VAL) % T1LD);
}

void PrintHexIntNumber( int n) {
  putchar( gl_c_numbers[((n & 0xf0000000) >> 28)]);
  putchar( gl_c_numbers[((n & 0xf000000) >> 24)]);

  putchar( gl_c_numbers[((n & 0xf00000) >> 20)]);
  putchar( gl_c_numbers[((n & 0xf0000) >> 16)]);

  putchar( gl_c_numbers[((n & 0xf000) >> 12)]);
  putchar( gl_c_numbers[((n & 0xf00) >> 8)]);

  putchar( gl_c_numbers[((n & 0xf0) >> 4)]);
  putchar( gl_c_numbers[(n & 0xf)]);
}

void PrintHexShortNumber( short n) {
  putchar( gl_c_numbers[((n & 0xf000) >> 12)]);
  putchar( gl_c_numbers[((n & 0xf00) >> 8)]);

  putchar( gl_c_numbers[((n & 0xf0) >> 4)]);
  putchar( gl_c_numbers[(n & 0xf)]);
}

void PrintHexCharNumber( char n) {
  putchar( gl_c_numbers[((n & 0xf0) >> 4)]);
  putchar( gl_c_numbers[(n & 0xf)]);
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
  // Наебка (сделано когда у нас ибанулся лазер)
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
      //АСИНХР: выдача dN-dU
      signed int siAngleInc1 = ( signed short) angle_inc1;
      //angle_inc_corr = (( float) (  siAngleInc1)) * 10.;
      angle_inc_corr = angle_inc1;
      f_dN = ( float) ( ( signed int) angle_inc1);
      dbl_dN = ( double) ( ( signed int) angle_inc1);
    }
    else {
      //АСИНХР: нормальный режим
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
    //СИНХРОННЫЙ РЕЖИМ
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

  //размазываем f_dN на диапазон [-99 310; + 99 310]
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
  putchar_nocheck( ( gl_b_PerimeterReset ? 0x80 : 0x00) + param_indicator & 0xff);
  gl_b_PerimeterReset = 0;
  cCheckSumm += (( gl_b_PerimeterReset ? 0x80 : 0x00) + param_indicator & 0xff);

  //***************************************************************************
  //ANALOG PARAMETER
  //***************************************************************************
  if( param_indicator == 15) {
    putchar_nocheck( VERSION_MAJOR * 16 + VERSION_MIDDLE);
    cCheckSumm += ( VERSION_MAJOR * 16 + VERSION_MIDDLE);

    putchar_nocheck( VERSION_MINOR * 16);// + 1 + HIRO_COEFF);
    cCheckSumm += ( VERSION_MINOR * 16);
  }
  else {
    putchar_nocheck( analog_param & 0xff);
    cCheckSumm += (analog_param & 0xff);

    putchar_nocheck( ( analog_param & 0xff00) >> 8);
    cCheckSumm += ( ( analog_param & 0xff00) >> 8);
  }

  //***************************************************************************
  //синхр. режим: SA TIME
  //асинхр. режим: SA TIME или приращение угла поворота
  if( gl_b_SyncMode) {
    //асинхр. режим
    if( bAsyncDu) {
      //передаем dU
      putchar_nocheck( ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev) & 0xff);
      cCheckSumm += ( ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev) & 0xff);

      putchar_nocheck( ( ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev) & 0xff00) >> 8);
      cCheckSumm += ( ( ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev) & 0xff00) >> 8);
    }
    else {
      //передаем SA
      putchar_nocheck( gl_ssh_SA_time & 0xff);
      cCheckSumm += ( gl_ssh_SA_time & 0xff);

      putchar_nocheck( ( gl_ssh_SA_time & 0xff00) >> 8);
      cCheckSumm += ( ( gl_ssh_SA_time & 0xff00) >> 8);
    }
  }
  else {
    //синхронный режим
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
  putchar_nocheck( gl_c_EmergencyCode);
  cCheckSumm += gl_c_EmergencyCode;

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

  printf("(0x55 0xAA)   (0x%02x 0x%02x 0x%02x 0x%02x)   (0x%02x)   (0x?? 0x??)   (0x?? 0x??)  (0x%02x)   (0x%02x)   (0x??)\n",
          b1, b2, b3, b4,
          ( gl_b_PerimeterReset ? 0x80 : 0x00) + param_indicator & 0xff,
          gl_c_OutPackCounter++,
          gl_c_EmergencyCode
          );
#endif
}

void load_params( void) {
  //код амплитуды
  if( flashEE_load_short( 0xf000, &flashParamAmplitudeCode)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //код такта подставки
  if( flashEE_load_short( 0xf002, &flashParamTactCode)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //коэффициент М
  if( flashEE_load_short( 0xf004, &flashParamMCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Начальная мода
  if( flashEE_load_short( 0xf006, &flashParamStartMode)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //серийный номер
  if( flashEE_load_int( 0xf008, &flashParamDeviceId)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //организация
  if( flashEE_load_text( 0xf00C, flashParamOrg, 16)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  } 
  //год
  if( flashEE_load_short( 0xf02C, &flashParamDateYear)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //месяц
  if( flashEE_load_short( 0xf02E, &flashParamDateMonth)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //день
  if( flashEE_load_short( 0xf030, &flashParamDateDay)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //минимальный ток I1
  if( flashEE_load_short( 0xf032, &flashParamI1min)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //минимальный ток I2
  if( flashEE_load_short( 0xf034, &flashParamI2min)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //минимальный AmplAng
  if( flashEE_load_short( 0xf036, &flashParamAmplAngMin1)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //коэффициент вычета
  if( flashEE_load_short( 0xf038, &flashParamDecCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //знаковый коэффициент
  if( flashEE_load_short( 0xf03A, &flashParamSignCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  //**************************************************************************************
  // КАЛИБРОВКА ТЕРМОДАТЧИКОВ
  //**************************************************************************************
  //Температура минимальной точки калибровки
  if( flashEE_load_short( 0xf03C, ( unsigned short *) &flashParam_calibT1)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты первого термодатчика при минимальной температуре калибровки
  if( flashEE_load_short( 0xf03E, &flashParamT1_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты второго термодатчика при минимальной температуре калибровки
  if( flashEE_load_short( 0xf040, &flashParamT1_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Температура максимальной точки калибровки
  if( flashEE_load_short( 0xf042, ( unsigned short *) &flashParam_calibT2)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты первого термодатчика при максимальной температуре калибровки
  if( flashEE_load_short( 0xf044, &flashParamT2_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты второго термодатчика при максимальной температуре калибровки
  if( flashEE_load_short( 0xf046, &flashParamT2_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  //********************************************************************
  // параметр сдвига
  if( flashEE_load_short( 0xf048, &flashParamPhaseShift)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  //последнее RULA
  if( flashEE_load_short( 0xf04A, &gl_ushFlashParamLastRULA)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  //последнее RULM
  if( flashEE_load_short( 0xf04C, &gl_ushFlashParamLastRULM)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

#ifdef DEBUG
  printf("DEBUG: load_params(): params loaded from flash memory. Here they are:\n");
  printf("DEBUG:   Amplitude Code: 0x%04x (%04d)\n", flashParamAmplitudeCode, flashParamAmplitudeCode); //код амплитуды
  printf("DEBUG:   Base Tact Code: 0x%04x (%04d)\n", flashParamTactCode, flashParamTactCode);   //код такта подставки
  printf("DEBUG:   M Coefficient:  0x%04x (%04d)\n", flashParamMCoeff, flashParamMCoeff);       //коэффициент М
  printf("DEBUG:   Start Mode:     0x%04x (%04d)\n", flashParamStartMode, flashParamStartMode); //Начальная мода
  printf("DEBUG:   Serial number:  0x%04x (%04d)\n", flashParamDeviceId, flashParamDeviceId);   //серийный номер
  printf("DEBUG:   Organization:   '%s'\n", flashParamOrg);                                     //организация
  printf("DEBUG:   Year:           0x%04x (%04d)\n", flashParamDateYear, flashParamDateYear);   //год
  printf("DEBUG:   Month:          0x%04x (%04d)\n", flashParamDateMonth, flashParamDateMonth); //месяц
  printf("DEBUG:   Day:            0x%04x (%04d)\n", flashParamDateDay, flashParamDateDay);     //день
  printf("DEBUG:   Control I1:     0x%04x (%04d)\n", flashParamI1min, flashParamI1min);         //минимальный ток I1
  printf("DEBUG:   Control I2:     0x%04x (%04d)\n", flashParamI2min, flashParamI2min);         //минимальный ток I2
  printf("DEBUG:   Control AA:     0x%04x (%04d)\n", flashParamAmplAngMin1, flashParamAmplAngMin1); //минимальный AmplAng
  printf("DEBUG:   Dec. Coeff:     0x%04x (%04d)\n", flashParamDecCoeff, flashParamDecCoeff);   //коэффициент вычета
  printf("DEBUG:   Sign coeff:     0x%04x (%04d)\n", flashParamSignCoeff, flashParamSignCoeff); //знаковый коэффициент
  printf("DEBUG:   Phase shift:    0x%04x (%04d)\n", flashParamPhaseShift, flashParamPhaseShift); //фазовый сдвиг
  printf("DEBUG:   Last RULA:      0x%04x (%04d)\n", gl_ushFlashParamLastRULA, gl_ushFlashParamLastRULA);   //последнее RULA
  printf("DEBUG:   Last RULM:      0x%04x (%04d)\n", gl_ushFlashParamLastRULM, gl_ushFlashParamLastRULM);   //последнее RULM
#endif

  //PARAMS CHECKING
  if( flashParamAmplitudeCode > 255)     //Код амплитуды [0-255]. дефолтное значение 90
    flashParamAmplitudeCode = 35;       //90 для большого 35 для маленького

  if( flashParamTactCode > 3)       //Код такта амплитуды [0-3]. дефолтное значение 0
    flashParamTactCode = 0;

  if( flashParamMCoeff > 250)     //Коэффициент М[0-1] = значения параметра [0-250].
    flashParamMCoeff = 125;       //дефолтное значение 125 (что означает M=0.5 и DAC1 = 0.5 * DAC0)  

  if( flashParamStartMode > 250)     //Начальная мода [0-250]. дефолтное значение 125 (что означает 1,25В на DAC2)
    flashParamStartMode = 125;

  //device_id = 0;
  //organization[17] = { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0};

  if( flashParamDateYear < 2000 && flashParamDateYear > 2200)
    flashParamDateYear = 2009;

  if( flashParamDateMonth > 12)
    flashParamDateMonth = 9;

  if( flashParamDateDay > 31)
    flashParamDateDay = 3;

/*
  if( flashParamI1min > 255)
    flashParamI1min = 0;

  if( flashParamI2min > 255)
    flashParamI2min = 0;

  if( flashParamAmplAngMin1 > 255)
    flashParamAmplAngMin1 = 0;
*/

  if( flashParamDecCoeff == 0xffff)
    flashParamDecCoeff = 0;

  if( flashParamSignCoeff > 2)
    flashParamSignCoeff = 2;

  if( flashParam_calibT1 < ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION)  ||
      flashParam_calibT1 > ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
    flashParam_calibT1 = 0;
    flashParamT1_TD1_val = 0;
    flashParamT1_TD2_val = 1;
  }

  if( flashParam_calibT2 < ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) ||
      flashParam_calibT2 > ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
    flashParam_calibT2 = 0;
    flashParamT2_TD1_val = 0;
    flashParamT2_TD2_val = 1;
  }

  if( flashParamPhaseShift > 63) {
    flashParamPhaseShift = 0;
  }

  if( gl_ushFlashParamLastRULA > 4096) {
    gl_ushFlashParamLastRULA = 0;
  }

  if( gl_ushFlashParamLastRULM > 4096) {
    gl_ushFlashParamLastRULM = 0;
  }
#ifdef DEBUG
  printf("DEBUG: load_params(): params checked for the range. Here they are:\n");
  printf("DEBUG:   Amplitude Code: 0x%04x (%04d)\n", flashParamAmplitudeCode, flashParamAmplitudeCode); //код амплитуды
  printf("DEBUG:   Base Tact Code: 0x%04x (%04d)\n", flashParamTactCode, flashParamTactCode);   //код такта подставки
  printf("DEBUG:   M Coefficient:  0x%04x (%04d)\n", flashParamMCoeff, flashParamMCoeff);       //коэффициент М
  printf("DEBUG:   Start Mode:     0x%04x (%04d)\n", flashParamStartMode, flashParamStartMode); //Начальная мода
  printf("DEBUG:   Serial number:  0x%04x (%04d)\n", flashParamDeviceId, flashParamDeviceId);   //серийный номер
  printf("DEBUG:   Organization:   '%s'\n", flashParamOrg);                                     //организация
  printf("DEBUG:   Year:           0x%04x (%04d)\n", flashParamDateYear, flashParamDateYear);   //год
  printf("DEBUG:   Month:          0x%04x (%04d)\n", flashParamDateMonth, flashParamDateMonth); //месяц
  printf("DEBUG:   Day:            0x%04x (%04d)\n", flashParamDateDay, flashParamDateDay);     //день
  printf("DEBUG:   Control I1:     0x%04x (%04d)\n", flashParamI1min, flashParamI1min);         //минимальный ток I1
  printf("DEBUG:   Control I2:     0x%04x (%04d)\n", flashParamI2min, flashParamI2min);         //минимальный ток I2
  printf("DEBUG:   Control AA:     0x%04x (%04d)\n", flashParamAmplAngMin1, flashParamAmplAngMin1); //минимальный AmplAng
  printf("DEBUG:   Dec. Coeff:     0x%04x (%04d)\n", flashParamDecCoeff, flashParamDecCoeff);   //коэффициент вычета
  printf("DEBUG:   Sign coeff:     0x%04x (%04d)\n", flashParamSignCoeff, flashParamSignCoeff); //знаковый коэффициент
  printf("DEBUG:   Phase shift:    0x%04x (%04d)\n", flashParamPhaseShift, flashParamPhaseShift); //фазовый сдвиг
  printf("DEBUG:   Last RULA:      0x%04x (%04d)\n", gl_ushFlashParamLastRULA, gl_ushFlashParamLastRULA);   //последнее RULA
  printf("DEBUG:   Last RULM:      0x%04x (%04d)\n", gl_ushFlashParamLastRULM, gl_ushFlashParamLastRULM);   //последнее RULM
#endif
}

void SaveRulaRulm( void) {
  if( flashEE_save_short( 0xF04A, gl_ushFlashParamLastRULA)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xF04C, gl_ushFlashParamLastRULM)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
}

void SaveThermoCalibPoint( void) {
  if( flashEE_save_short( 0xf03C, flashParam_calibT1)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf03E, flashParamT1_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf040, flashParamT1_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf042, flashParam_calibT2)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf044, flashParamT2_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf046, flashParamT2_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
}

void save_params( void) {
#ifdef DEBUG
  printf("DEBUG: save_params(): params to be saved are:\n");
  printf("DEBUG:   Amplitude Code: 0x%04x (%04d)\n", flashParamAmplitudeCode, flashParamAmplitudeCode); //код амплитуды
  printf("DEBUG:   Base Tact Code: 0x%04x (%04d)\n", flashParamTactCode, flashParamTactCode);   //код такта подставки
  printf("DEBUG:   M Coefficient:  0x%04x (%04d)\n", flashParamMCoeff, flashParamMCoeff);       //коэффициент М
  printf("DEBUG:   Start Mode:     0x%04x (%04d)\n", flashParamStartMode, flashParamStartMode); //Начальная мода
  printf("DEBUG:   Serial number:  0x%04x (%04d)\n", flashParamDeviceId, flashParamDeviceId);   //серийный номер
  printf("DEBUG:   Organization:   '%s'\n", flashParamOrg);                                     //организация
  printf("DEBUG:   Year:           0x%04x (%04d)\n", flashParamDateYear, flashParamDateYear);   //год
  printf("DEBUG:   Month:          0x%04x (%04d)\n", flashParamDateMonth, flashParamDateMonth); //месяц
  printf("DEBUG:   Day:            0x%04x (%04d)\n", flashParamDateDay, flashParamDateDay);     //день
  printf("DEBUG:   Control I1:     0x%04x (%04d)\n", flashParamI1min, flashParamI1min);         //минимальный ток I1
  printf("DEBUG:   Control I2:     0x%04x (%04d)\n", flashParamI2min, flashParamI2min);         //минимальный ток I2
  printf("DEBUG:   Control AA:     0x%04x (%04d)\n", flashParamAmplAngMin1, flashParamAmplAngMin1); //минимальный AmplAng
  printf("DEBUG:   Dec. Coeff:     0x%04x (%04d)\n", flashParamDecCoeff, flashParamDecCoeff);   //коэффициент вычета
  printf("DEBUG:   Sign coeff:     0x%04x (%04d)\n", flashParamSignCoeff, flashParamSignCoeff); //Знаковый коэффициент
  printf("DEBUG:   Phase shift:    0x%04x (%04d)\n", flashParamPhaseShift, flashParamPhaseShift); //фазовый сдвиг
#endif

  if( flashEE_erase_page( 0xf000)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf000, flashParamAmplitudeCode)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf002, flashParamTactCode)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf004, flashParamMCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf006, flashParamStartMode)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_int( 0xf008, flashParamDeviceId)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_text( 0xf00C, flashParamOrg, 16)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  } 
  if( flashEE_save_short( 0xf02C, flashParamDateYear)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf02E, flashParamDateMonth)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf030, flashParamDateDay)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf032, flashParamI1min)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf034, flashParamI2min)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf036, flashParamAmplAngMin1)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf038, flashParamDecCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf03A, flashParamSignCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  //0xf03C - 0xf046 включительно - калибровочные данные термодатчиков
  SaveThermoCalibPoint();

  if( flashEE_save_short( 0xf048, flashParamPhaseShift)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }

  if( flashEE_save_short( 0xF04A, gl_ushFlashParamLastRULA)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xF04C, gl_ushFlashParamLastRULM)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
}

void configure_hanger( void) {

  //1. Код такта подставки
  //Выставка TactNoise0 (младший бит параметра "код такта подставки")
  if( ( flashParamTactCode & 0x01))  //Set TactNoise0 to TactCode parameter bit0
    GP3DAT |= ( 1 << (16 + 0));
  else
    GP3DAT &= ~( 1 << (16 + 0));

  //Выставка TactNoise1 (старший бит параметра "код такта подставки")
  if( ( flashParamTactCode & 0x02))  //Set TactNoise1 to TactCode parameter bit1
    GP3DAT |= ( 1 << (16 + 1));
  else
    GP3DAT &= ~( 1 << (16 + 1));
}

void DACConfiguration( void) {
#ifdef DEBUG
  printf("DEBUG: DACConfiguration(): enter\n");
#endif
  //**********************************************************************
  // выставка ЦАПов
  //**********************************************************************
  // ЦАП 0
  DAC0DAT = (( int) ( 4095.0 * ( ( double) gl_un_RULAControl / ( double) RULA_MAX ))) << 16; //выставка на выходе ЦАП0 1,0 В


  // ЦАП 1 (мода)
  /*gl_nAppliedMCoeff--;
  if( gl_nAppliedMCoeff <= flashParamMCoeff / 250. * 4096.)
    gl_nAppliedMCoeff = flashParamMCoeff / 250. * 4096.;
  */
  DAC1DAT = (( int) ( 4095.0 * ( ( double) gl_nAppliedMCoeff / 4096. * ( ( double) gl_un_RULAControl / ( double) RULA_MAX)))) << 16;  //(1.0) - это RULA в вольтах который на DAC0

  //DAC1DAT = (( int) ( 4096.0 * ( ( double) flashParamParam3 / 250. * 0.25) / 3.0)) << 16;  //(1.0) - это RULA в вольтах который на DAC0


  // ЦАП 2 (начальная мода)
  DAC2DAT = (( int) ( 4095.0 * ( ( double) flashParamStartMode / 250. ))) << 16;
  //DAC2DAT = (( int) ( 4095.0 * 1.25 / 2.5)) << 16;
}

void deadloop_no_firing( void) {
  //ОБРАБОТКА ОТКАЗА ПОДЖИГА
#ifdef DEBUG
  printf("DEBUG: NO LASER FIREUP! DEADLOOP.\n");
#endif

  //выставляем код ошибки
  gl_c_EmergencyCode = ERROR_NO_LASER_FIRING;

  //высылка настроечных параметров
  send_pack( 0, 6, flashParamAmplitudeCode);
  send_pack( 0, 7, flashParamTactCode);
  send_pack( 0, 8, flashParamMCoeff);
  send_pack( 0, 9, flashParamStartMode);
  send_pack( 0, 10, flashParamI1min);
  send_pack( 0, 11, flashParamI2min);
  send_pack( 0, 12, flashParamAmplAngMin1);
  send_pack( 0, 13, flashParamDecCoeff);
  send_pack( 0, 14, flashParamSignCoeff);

  gl_ssh_prT1VAL = T1VAL;
  while( 1) {
    //пауза 0,1 секунда
    pause( 327);

    gl_ssh_SA_time = ( T1LD + gl_ssh_prT1VAL - T1VAL) % T1LD;
    gl_ssh_prT1VAL = T1VAL;

    //**********************************************************************
    // Обработка буфера входящих команд
    //**********************************************************************
    if( pos_in_in_buf == IN_COMMAND_BUF_LEN) {
      switch( input_buffer[0]) {
        case 0: //установить код амплитуды
          flashParamAmplitudeCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 6, flashParamAmplitudeCode);
        break;

        case 1: //установить код такта подставки
          flashParamTactCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 7, flashParamTactCode);
        break;

        case 2: //установить коэффициент M
          flashParamMCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 8, flashParamMCoeff);
        break;

        case 3: //установить начальную моду
          flashParamStartMode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 9, flashParamStartMode);
        break;
        
        case 4: //установить минимальный ток I1
          flashParamI1min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 10, flashParamI1min);
        break;

        case 5: //установить минимальный ток I2
          flashParamI2min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 11, flashParamI2min);
        break;

        case 6: //установить 1ый минимум сигнала AmplAng
          flashParamAmplAngMin1 = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 12, flashParamAmplAngMin1);
        break;

        case 7: //установить коэффициент вычета
          flashParamDecCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 13, flashParamDecCoeff);
        break;

        case 8: //установить SA такт
          flashParamSignCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 14, flashParamSignCoeff);
        break;

        /*
        case 9: //в асинхр. режиме вылючить вывод SA (включить вывод dU)
        	bAsyncDu = 1;
        break;

        case 10: //в асинхр. режиме выключить вывод dU (включить вывод SA)
        	bAsyncDu = 0;
        break; */

        case 49: //запрос параметров
        	send_pack( 0, 6, flashParamAmplitudeCode);
          send_pack( 0, 7, flashParamTactCode);
          send_pack( 0, 8, flashParamMCoeff);
          send_pack( 0, 9, flashParamStartMode);
          send_pack( 0, 10, flashParamI1min);
          send_pack( 0, 11, flashParamI2min);
          send_pack( 0, 12, flashParamAmplAngMin1);
          send_pack( 0, 13, flashParamDecCoeff);
          send_pack( 0, 14, flashParamSignCoeff);
        break;

        case 50: //сохранить параметры во флэш память
        	save_params(); break;

        case 51: //перезагрузить параметры из флэш-памяти и показать их
        	load_params(); break;
      }
      
      pos_in_in_buf = 0;
    }
    else
      //Если входящих команд не было, то
      //посылка пустого сообщения с ошибкой
      send_pack( 0, 0, 0);

  } //"мертвый" while
}

void deadloop_no_hangerup( void) {
  //ОБРАБОТКА ОТКАЗА РАСКАЧКИ ВИБРОПОДВЕСА
#ifdef DEBUG
  printf("DEBUG: NO HANGER VIBRATION! DEADLOOP.\n");
#endif
  //выставляем код ошибки
  gl_c_EmergencyCode = ERROR_INITIAL_AMPL_ANG_TEST_FAIL;

  ADCCP = 0x06;     //мы будем посылать ТОЛЬКО AmplAng
  ADCCON |= 0x80;   //запуск преобразования

  //высылка настроечных параметров
  send_pack( 0, 6, flashParamAmplitudeCode);
  send_pack( 0, 7, flashParamTactCode);
  send_pack( 0, 8, flashParamMCoeff);
  send_pack( 0, 9, flashParamStartMode);
  send_pack( 0, 10, flashParamI1min);
  send_pack( 0, 11, flashParamI2min);
  send_pack( 0, 12, flashParamAmplAngMin1);
  send_pack( 0, 13, flashParamDecCoeff);
  send_pack( 0, 14, flashParamSignCoeff);

  gl_ssh_prT1VAL = T1VAL;
  while( 1) {
    //пауза 0,1 секунда
    pause( 327);

    gl_ssh_SA_time = ( T1LD + gl_ssh_prT1VAL - T1VAL) % T1LD;
    gl_ssh_prT1VAL = T1VAL;

    //измерение AmplAng (и ТОЛЬКО ЕГО)
    while (!( ADCSTA & 0x01)){}     // ожидаем конца преобразования АЦП (теоретически когда мы приходим сюда он уже должен быть готов)
    gl_ssh_ampl_angle = (ADCDAT >> 16);
    ADCCON |= 0x80;                 //запуск преобразования

    //**********************************************************************
    // Обработка буфера входящих команд
    //**********************************************************************
    if( pos_in_in_buf == IN_COMMAND_BUF_LEN) {
      switch( input_buffer[0]) {
        case 0: //установить код амплитуды
          flashParamAmplitudeCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 6, flashParamAmplitudeCode);
        break;

        case 1: //установить код такта подставки
          flashParamTactCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 7, flashParamTactCode);
        break;

        case 2: //установить коэффициент M
          flashParamMCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 8, flashParamMCoeff);
        break;

        case 3: //установить начальную моду
          flashParamStartMode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 9, flashParamStartMode);
        break;
        
        case 4: //установить минимальный ток I1
          flashParamI1min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 10, flashParamI1min);
        break;

        case 5: //установить минимальный ток I2
          flashParamI2min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 11, flashParamI2min);
        break;

        case 6: //установить 1ый минимум сигнала AmplAng
          flashParamAmplAngMin1 = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 12, flashParamAmplAngMin1);
        break;

        case 7: //установить коэффициент вычета
          flashParamDecCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 13, flashParamDecCoeff);
        break;

        case 8: //установить SA такт
          flashParamSignCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 14, flashParamSignCoeff);
        break;

        /*
        case 9: //в асинхр. режиме вылючить вывод SA (включить вывод dU)
          bAsyncDu = 1;
        break;

        case 10: //в асинхр. режиме выключить вывод dU (включить вывод SA)
          bAsyncDu = 0;
        break;
        */

        case 49: //запрос параметров
          send_pack( 0, 6, flashParamAmplitudeCode);
          send_pack( 0, 7, flashParamTactCode);
          send_pack( 0, 8, flashParamMCoeff);
          send_pack( 0, 9, flashParamStartMode);
          send_pack( 0, 10, flashParamI1min);
          send_pack( 0, 11, flashParamI2min);
          send_pack( 0, 12, flashParamAmplAngMin1);
          send_pack( 0, 13, flashParamDecCoeff);
          send_pack( 0, 14, flashParamSignCoeff);
        break;

        case 50: //сохранить параметры во флэш память
          save_params(); break;

        case 51: //перезагрузить параметры из флэш-памяти и показать их
          load_params(); break;
      }

      pos_in_in_buf = 0;
    }
    else
      //Если входящих команд не было, то
      //посылка пустого сообщения с ошибкой
      send_pack( 0, 5, gl_ssh_ampl_angle);

  } //"мертвый" захват отказа раскачки виброподвеса
}

void deadloop_no_tact( int nError) {
  //ОБРАБОТКА ОТСУТСТВИЯ ТАКТИРОВАНИЯ
#ifdef DEBUG
  printf("DEBUG: NO TACT SIGNAL! DEADLOOP.\n");
#endif
  //выставляем код ошибки
  gl_c_EmergencyCode = nError;

  //высылка настроечных параметров
  send_pack( 0, 6, flashParamAmplitudeCode);
  send_pack( 0, 7, flashParamTactCode);
  send_pack( 0, 8, flashParamMCoeff);
  send_pack( 0, 9, flashParamStartMode);
  send_pack( 0, 10, flashParamI1min);
  send_pack( 0, 11, flashParamI2min);
  send_pack( 0, 12, flashParamAmplAngMin1);
  send_pack( 0, 13, flashParamDecCoeff);
  send_pack( 0, 14, flashParamSignCoeff);

  gl_ssh_prT1VAL = T1VAL;
  while( 1) {
    //пауза 0,1 секунда
    pause( 327);

    gl_ssh_SA_time = ( T1LD + gl_ssh_prT1VAL - T1VAL) % T1LD;
    gl_ssh_prT1VAL = T1VAL;

    //**********************************************************************
    // Обработка буфера входящих команд
    //**********************************************************************
    if( pos_in_in_buf == IN_COMMAND_BUF_LEN) {
      switch( input_buffer[0]) {
        case 0: //установить код амплитуды
          flashParamAmplitudeCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 6, flashParamAmplitudeCode);
        break;

        case 1: //установить код такта подставки
          flashParamTactCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 7, flashParamTactCode);
        break;

        case 2: //установить коэффициент M
          flashParamMCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 8, flashParamMCoeff);
        break;

        case 3: //установить начальную моду
          flashParamStartMode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 9, flashParamStartMode);
        break;
        
        case 4: //установить минимальный ток I1
          flashParamI1min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 10, flashParamI1min);
        break;

        case 5: //установить минимальный ток I2
          flashParamI2min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 11, flashParamI2min);
        break;

        case 6: //установить 1ый минимум сигнала AmplAng
          flashParamAmplAngMin1 = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 12, flashParamAmplAngMin1);
        break;

        case 7: //установить коэффициент вычета
          flashParamDecCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 13, flashParamDecCoeff);
        break;

        case 8: //установить SA такт
          flashParamSignCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 14, flashParamSignCoeff);
        break;

        /*
        case 9: //в асинхр. режиме вылючить вывод SA (включить вывод dU)
          bAsyncDu = 1;
        break;

        case 10: //в асинхр. режиме выключить вывод dU (включить вывод SA)
          bAsyncDu = 0;
        break; */

        case 49: //запрос параметров
          send_pack( 0, 6, flashParamAmplitudeCode);
          send_pack( 0, 7, flashParamTactCode);
          send_pack( 0, 8, flashParamMCoeff);
          send_pack( 0, 9, flashParamStartMode);
          send_pack( 0, 10, flashParamI1min);
          send_pack( 0, 11, flashParamI2min);
          send_pack( 0, 12, flashParamAmplAngMin1);
          send_pack( 0, 13, flashParamDecCoeff);
          send_pack( 0, 14, flashParamSignCoeff);
        break;

        case 50: //сохранить параметры во флэш память
          save_params(); break;

        case 51: //перезагрузить параметры из флэш-памяти и показать их
          load_params(); break;
      }
      
      pos_in_in_buf = 0;
    }
    else
      //Если входящих команд не было, то
      //посылка пустого сообщения с ошибкой
      send_pack( 0, 0, 0);

  } //"мертвый" while
}

void FirstDecrementCoeffCalculation( void) {
  char lb, hb;

  //****
  //ПЕРВЫЙ ПРОХОД (нам ведь дельты нужны)
  //****

  //выжидаем нарастающий фронт сигнала SA_TA (p0.4)
  //точнее тут ждём верхнего логического уровня на этой ноге
  while( !( GP0DAT & 0x10));

  // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
  // 1. запрашиваем у альтеры код счётчика информационных импульсов
  //ждём высокого уровня сигнала готовности CntReady (p4.2)
  while( !( GP4DAT & 0x04)) {}

  //запрашиваем старший байт кода счётчика информационных импульсов
  
  //GP1SET = 1 << (16 + 3);  //RDHBC (p1.3) = 1		WAY1
  GP1DAT |= 1 << (16 + 3);	//RDHBC (p1.3) = 1		WAY2
  
  pause( 1);                //пауза

  //чтение
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
  


  //запрашиваем младший байт кода счётчика информационных импульсов
  //GP1SET = 1 << (16 + 4);  //RDLBC (p1.4) = 1		WAY1
  GP1DAT |= 1 << (16 + 4);  //RDLBC (p1.4) = 1			WAY2

  pause( 1);                //пауза

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

  //складываем два байта
  gl_ssh_angle_inc = lb + (hb << 8);




  // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
  // 2. в случае асинхр режима, запрашиваем у альтеры код счётчика информационных импульсов
  if( gl_b_SyncMode) {
    //ждём высокого уровня сигнала готовности ANGLE_READY (p2.4)
    while( !( GP2DAT & 0x10)) {}

    //запрашиваем старший байт угла поворота вибратора
    GP0SET = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 1
    pause( 1);                //пауза

    //чтение
    hb =  (( GP1DAT & BIT_5) >> 5) +
          ((( GP0DAT & BIT_7) >> 7) << 1) +
          ((( GP0DAT & BIT_1) >> 1) << 2) +
          ((( GP2DAT & BIT_3) >> 3) << 3) +
          ((( GP4DAT & BIT_6) >> 6) << 4) +
          ((( GP4DAT & BIT_7) >> 7) << 5) +
          ((( GP0DAT & BIT_6) >> 6) << 6) +
          ((( GP0DAT & BIT_2) >> 2) << 7);

    GP0CLR = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 0


    //запрашиваем младший байт кода счётчика информационных импульсов
    GP2SET = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 1
    pause( 1);                //пауза

    lb = (( GP1DAT & BIT_5) >> 5) +
         ((( GP0DAT & BIT_7) >> 7) << 1) +
         ((( GP0DAT & BIT_1) >> 1) << 2) +
         ((( GP2DAT & BIT_3) >> 3) << 3) +
         ((( GP4DAT & BIT_6) >> 6) << 4) +
         ((( GP4DAT & BIT_7) >> 7) << 5) +
         ((( GP0DAT & BIT_6) >> 6) << 6) +
         ((( GP0DAT & BIT_2) >> 2) << 7);

    GP2CLR = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 0

    //складываем два байта
    gl_ssh_angle_hanger = lb + (hb << 8);

  }



  //выжидаем низкий уровень сигнала SA_TA (p0.4)
  while( GP0DAT & 0x10);

  //****
  //И ПОСЛЕДУЮЩИЕ
  //****
  do {
    //выжидаем нарастающий фронт сигнала SA_TA (p0.4)
    while( !(GP0DAT & 0x10));

    // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
    // 1. запрашиваем у альтеры код счётчика информационных импульсов
    //ждём высокого уровня сигнала готовности CntReady (p4.2)
    while( !( GP4DAT & 0x04)) {}

    //запрашиваем старший байт кода счётчика информационных импульсов
    //GP1SET = 1 << (16 + 3);  //RDHBC (p1.3) = 1   	WAY1
	  GP1DAT |= 1 << (16 + 3);  //RDHBC (p1.3) = 1   	WAY2

    pause( 1);                //пауза

    //чтение
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


    //запрашиваем младший байт кода счётчика информационных импульсов
    //GP1SET = 1 << (16 + 4);  //RDLBC (p1.4) = 1		WAY1
	  GP1DAT |= 1 << (16 + 4);  //RDLBC (p1.4) = 1		WAY2

    pause( 1);                //пауза

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

    //складываем два байта
    gl_ssh_angle_inc_prev = gl_ssh_angle_inc;
    gl_ssh_angle_inc = lb + (hb << 8);





    // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
    // 2. в случае асинхр режима, запрашиваем у альтеры код счётчика информационных импульсов

    //ждём высокого уровня сигнала готовности ANGLE_READY (p2.4)
    while( !( GP2DAT & 0x10)) {}

    //запрашиваем старший байт угла поворота вибратора
    GP0SET = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 1
    pause( 1);                //пауза

    //чтение
    hb =  (( GP1DAT & BIT_5) >> 5) +
          ((( GP0DAT & BIT_7) >> 7) << 1) +
          ((( GP0DAT & BIT_1) >> 1) << 2) +
          ((( GP2DAT & BIT_3) >> 3) << 3) +
          ((( GP4DAT & BIT_6) >> 6) << 4) +
          ((( GP4DAT & BIT_7) >> 7) << 5) +
          ((( GP0DAT & BIT_6) >> 6) << 6) +
          ((( GP0DAT & BIT_2) >> 2) << 7);

    GP0CLR = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 0


    //запрашиваем младший байт кода счётчика информационных импульсов
    GP2SET = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 1
    pause( 1);                //пауза

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



    //выжидаем спадающий фронт сигнала SA_TA (p0.4)
    while( (GP0DAT & 0x10));


    //рассчитываем суммы
    gl_dbl_Nsumm += fabs( ( double) gl_ssh_angle_inc - ( double) gl_ssh_angle_inc_prev);
    //TODO: отсекать малые dU?
    gl_dbl_Usumm += fabs( ( double) gl_ssh_angle_hanger - ( double) gl_ssh_angle_hanger_prev);

    gl_ssh_angle_inc_prev = gl_ssh_angle_inc;
    gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;

    gl_un_DecCoeffStatPoints++;

  } while( gl_un_DecCoeffStatPoints < DEC_COEFF_FIRST_CALCULATION_N);

  //считаем собсно значение коэф. вычета
  flashParamDecCoeff = ( short) ( gl_dbl_Nsumm / gl_dbl_Usumm * 65535.);

  gl_un_DecCoeffStatPoints = 0;
  gl_dbl_Nsumm = gl_dbl_Usumm = 0.;
}

void ThermoCalibrationCalculation( void)
{
  double x1, y1, x2, y2;
  //рассчёт калибровки термодатчиков
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

    bCalibrated = 1;
  }
  else
    bCalibrated = 0;
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

  double db_dN1, db_dN2, dbU1, dbU2;
  float Coeff;

  double V_piezo;
  double temp_t;

  double dblDelta;

  bCalibProcessState = 0;    //0 - no calibration

                             //1 - processing min_t_point 1st thermosensor
                             //2 - processing min_t_point 2nd thermosensor
                             //3 - processing min_t_point 3rd thermosensor

                             //4 - processing max_t_point 1st thermosensor
                             //5 - processing max_t_point 2nd thermosensor
                             //6 - processing max_t_point 3rd thermosensor



  // Setup tx & rx pins on P1.0 and P1.1
  GP0CON = 0x00;
  GP1CON = 0x011;       //*** 0x011 = 0001 0001 = 00 01 00 01
                        //*** 01 - Выбор функции вывода порта P1.0
                        //*** 00 - Reserved
                        //*** 01 - Выбор функции вывода порта P1.1
                        //*** 00 - Reserved
                        //*** Функция:
                        //***       00    01    10    11
                        //*** P1.0  GPIO  SIN   SCL0  PLAI[0]
                        //*** P1.1  GPIO  SOUT  SDA0  PLAI[1] 

  // Start setting up UART
  COMCON0 = 0x080;      // Setting DLAB

  // Setting DIV0 and DIV1 to DL calculated

  /*
  //WORK PARAMETERS FOR PRECISE 115200
  COMDIV0 = 0x0B;
  COMDIV1 = 0x00;
  COMDIV2 = 0x0029;
  */


  //DL = 41.78 * 10^6 / 2 ^ (CD=0) * 1/16 * 1/ (2*Baudrate)
  //RESULT = (M+N/2048) = 41.78 * 10^6 / ( 2 ^ (CD=0) * 16 *  2 * Baudrate)
  // M = Floor ( RESULT)
  // N = (RESULT - M) * 2048

  /*
  //WORK PARAMETERS FOR PRECISE 256000
  COMDIV0 = 0x05;
  COMDIV1 = 0x00;
  COMDIV2 = 0x8829;
  */

  /****************************************************************************************** */
  /*   512 000 ********/
  /****************************************************************************************** */
  //DL = 41.78e6 / 16/2/512000 = 2.xxxxxx
  //M+N/2048 = 41.78e6 / 16/2/2/512000 = 1.275xxxxxx
  //M=1
  //N=563=0x233
  //      8             A
  //  1 0 0   0     1   0 1 0
  //  0 0 1   1     0   0 1 1
  //     3             3
  //COMDIV2 = 0x8A33

  COMDIV0 = 2;
  COMDIV1 = 0x00;
  COMDIV2 = 0x8A33;

  /****************************************************************************************** */

  COMCON0 = 0x007;  // Clearing DLAB

#ifdef DEBUG
  printf("T7-SLG. Software version: %d.%d.%d\n", VERSION_MAJOR, VERSION_MIDDLE, VERSION_MINOR);
  printf("DEBUG MODE\n");
  gl_sn_MeaningCounterRound = LONG_OUTPUT_PACK_LEN;
  printf("LONG_PACK: %d\n", gl_sn_MeaningCounterRound);
  gl_sn_MeaningCounterRound = SHORT_OUTPUT_PACK_LEN;
  printf("SHORT_PACK: %d\n", gl_sn_MeaningCounterRound);
#endif

  /*
  for( i=0; i<20; i++) {
    CircleBufferAmplForT_add( i);
    printf("%d  (%d %d %d %d %d %d %d %d %d %d)  M(2)=%.2f  M(5)=%.2f  M(3)=%.2f\n",
            i,
            gl_ushCircleBufferAmplForT[0], gl_ushCircleBufferAmplForT[1], gl_ushCircleBufferAmplForT[2], gl_ushCircleBufferAmplForT[3], gl_ushCircleBufferAmplForT[4],
            gl_ushCircleBufferAmplForT[5], gl_ushCircleBufferAmplForT[6], gl_ushCircleBufferAmplForT[7], gl_ushCircleBufferAmplForT[8], gl_ushCircleBufferAmplForT[9],
            CircleBufferAmplForT_mean( 2), CircleBufferAmplForT_mean( 5), CircleBufferAmplForT_mean( 3));
  }
  
  while(1);
  */

  //**********************************************************************
  // Конфигурация лампочки
  //**********************************************************************	
  GP0DAT = 0x01000000;
  GP0DAT ^= (1 << 16);
  GP0CLR = (1 << 16);

#ifdef DEBUG
  printf("DEBUG: GPIO lines direction configuration...");
#endif

  //**********************************************************************
  // Конфигурация GPIO (двунаправленных входов/выходов общего назначения)
  //**********************************************************************
  GP0DAT |= 1 << (24 + 0);  //Конфигурация тестовой линии (лампочка) (p0.0) в качестве выхода
  GP0DAT |= 1 << (24 + 3);  //Конфигурация линии RDHBANGLE           (p0.3) в качестве выхода
  GP0DAT |= 1 << (24 + 5);  //Конфигурация линии RP_P                (p0.5) в качестве выхода

  GP1DAT |= 1 << (24 + 3);  //Конфигурация линии RdHbc               (p1.3) в качестве выхода
  GP1DAT |= 1 << (24 + 4);  //Конфигурация линии RdLbc               (p1.4) в качестве выхода
  GP1DAT |= 1 << (24 + 7);  //Конфигурация линии EN_RP               (p1.7) в качестве выхода

  GP2DAT |= 1 << (24 + 2);  //Конфигурация линии EN_VB               (p2.2) в качестве выхода
  GP2DAT |= 1 << (24 + 5);  //Конфигурация линии RDLBANGLE           (p2.5) в качестве выхода

  GP3DAT |= 1 << (24 + 0);  //Конфигурация линии TactNoise0          (p3.0) в качестве выхода
  GP3DAT |= 1 << (24 + 1);  //Конфигурация линии TactNoise1          (p3.1) в качестве выхода
  GP3DAT |= 1 << (24 + 3);  //Конфигурация линии RD_AMPL_T_CODE      (p3.3) в качестве выхода
  GP3DAT |= 1 << (24 + 5);  //Конфигурация линии OutLnfType          (p3.5) в качестве выхода

  GP4DAT |= 1 << (24 + 0);  //Конфигурация линии ONHV                (p4.0) в качестве выхода
  GP4DAT |= 1 << (24 + 1);  //Конфигурация линии OFF3KV              (p4.1) в качестве выхода
  GP4DAT |= 1 << (24 + 3);  //Конфигурация линии Reset               (p4.3) в качестве выхода


#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: GPIO lines values configuration...");
#endif
  GP4DAT |=    1 << (16 + 0);  //ONHV        (p4.0) = 1 (выключено)
  GP4DAT &= ~( 1 << (16 + 1)); //OFF3KV      (p4.1) = 0 (включено)

  GP2DAT &= ~( 1 << (16 + 2));  //EN_VB       (p2.2) = 0

  GP1DAT &= ~( 1 << (16 + 7));  //EN_RP       (p1.7) = 0

  GP0DAT &= ~( 1 << (16 + 5));  //RP_P        (p0.5) = 0


#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: Pulsing Reset signal...");
#endif

  //**********************************************************************
  // Посылка сигнала Reset
  //**********************************************************************
  GP4DAT |= 1 << (16 + 3);      //Reset set
  for( i=0; i<100; i++);
  GP4DAT &= ~( 1 << (16 + 3));  //Reset clear

#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: Enabling UART0 FIQ...");
#endif

  //**********************************************************************
  // Включение прерывания по UART0
  //**********************************************************************	
  FIQEN |= UART_BIT;
  COMIEN0 |= 1;

#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: Internal ADC configuration...");
#endif

  //**********************************************************************
  // Конфигурация АЦП
  //**********************************************************************	
  ADCCON = 0x20;            // включаем АЦП
  while (time >=0)          // ждем указанное в datasheet время (5мксек) для полного включения АЦП
    time--;

  ADCCON = 0x624;           // Конфигурируем АЦП:
                            // непрерывное преобразование с управлением от программы
                            // однополярный вход
                            // (включенное питание АЦП)
                            // запрещенный ADCBusy
                            // без старта преобразования
                            // преобразование 8 тактов
                            // тактирование [fADC / 4]
  ADCCP = 0x01;             // ставим 1ый канал АЦП
  REFCON = 0x01;            // выставляем источник опорного напряжения: (было: 0x02 подключение внутреннего 3-вольтового ИОН к выходу VREF.   стало 0x01 внутренняя 2.5V)

#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: Timers configuration...");
#endif

  //**********************************************************************
  // Конфигурация Timer1
  //**********************************************************************
  T1CON = 0x2c0;
  T1LD = 0x100000;

  //**********************************************************************
  // Конфигурация Timer2
  //**********************************************************************
  T2CON = 0x0C0;
  T2LD = 0x100000;

#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: FlashEE configuration...");
#endif

  //**********************************************************************
  // Конфигурация флэш-памяти FlashEE
  //**********************************************************************
  flashEE_configure();

#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: loading flash params.\n");
#endif

  //**********************************************************************
  // Загрузка параметров из флэш-памяти
  //**********************************************************************
  load_params();
  ThermoCalibrationCalculation();

#ifdef DEBUG
  printf("DEBUG: DAC Configuration...");
#endif




  //**********************************************************************
  // Конфигурация и выставка ЦАП
  //**********************************************************************
  // ЦАП 0
  DAC0CON = 0x12;       // конфигурация ЦАП 0:
                        // диапазон 0-DAC(REF)
                        // значение выдаваемое ЦАП0 обновляется по заднему фронту такта ядра

  // ЦАП 1
  DAC1CON = 0x12;       // конфигурация ЦАП 1:
                        // диапазон 0-DAC(REF)
                        // значение выдаваемое ЦАП1 обновляется по заднему фронту такта ядра

  // ЦАП 2
  DAC2CON = 0x12;       // конфигурация ЦАП 2:
                        // диапазон 0-DAC(REF)
                        // значение выдаваемое ЦАП2 обновляется по заднему фронту такта ядра


  //Выставка максимальных RULA и RULM
  DAC0DAT = (( int) ( 4095.0 * 2.5 / 2.5)) << 16; //выставка на выходе ЦАП0 2.5 В
  DAC1DAT = (( int) ( 4095.0 * 2.5 / 2.5)) << 16; //выставка на выходе ЦАП1 2.5 В
  DAC2DAT = (( int) ( 4095.0 * 1.25 / 2.5)) << 16; //( ( double) flashParamStartMode / 250. * 2.5) / 3.0)) << 16;

#ifdef DEBUG
  printf("done\n");
  printf("DEBUG: Hangerup configure...");
#endif

  //**********************************************************************
  // Конфигурация ошумления подвеса (TactNoise0 и TactNoise1)
  //**********************************************************************
  configure_hanger();

  //**********************************************************************
  // ВКЛЮЧЕНИЕ ВИБРОПОДВЕСА
  //**********************************************************************
  GP2DAT |= ( 1 << (16 + 2));  //EN_VB   (p2.2) = 1

  //**********************************************************************
  // Ожидание раскачки виброподвеса
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
    gl_un_RULAControl = gl_ushFlashParamLastRULA;    //восстанавливаем последнее значение RULA
  else
    gl_un_RULAControl = 2457;                    //Если последнего значения не было 2457 = 1.500 V

  //if( gl_ushFlashParamLastRULM != 0)
  //  gl_nAppliedMCoeff = gl_ushFlashParamLastRULM;    //восстанавливаем последнее значение RULM
  //else
    gl_nAppliedMCoeff = 4096;                    //Если последнего значения не было ставим 4096 и далее RULM каждый такт будет -- до нужного значения

  DACConfiguration();

  //**********************************************************************
  // ПОДЖИГ ЛАЗЕРА
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
    //поджигаем усиленно (поджиг) (включено by default)
    //GP4DAT &= ~( 1 << (16 + 1));  //OFF3KV (p4.1) = 0

    //подаём высокое
    GP4DAT &= ~( 1 << (16 + 0));  //ONHV   (p4.0) = 0

    //выжидаем секунду
    pause( 32768);

    //измеряем ток I1
    ADCCP = 0x03;
    pause( 10);
	ADCCON |= 0x80;
    while (!( ADCSTA & 0x01)){}     // ожидаем конца преобразования АЦП
    gl_ssh_current_1 = (ADCDAT >> 16);

    //измеряем ток I2
    ADCCP = 0x04;
    pause( 10);
	ADCCON |= 0x80;
    while (!( ADCSTA & 0x01)){}     // ожидаем конца преобразования АЦП
    gl_ssh_current_2 = (ADCDAT >> 16);

    /*if( ( ( double) gl_ssh_current_1 / 4096. * 3. / 3.973 < ( double) flashParamI1min / 65535. * 0.75)  ||
        ( ( double) gl_ssh_current_2 / 4096. * 3. / 3.973 < ( double) flashParamI2min / 65535. * 0.75)) {*/

#ifdef DEBUG
	printf("DEBUG: Laser fireup: Measured I1=%.02f   Measured I2=%.02f\n", ( 2.5 - ( double) gl_ssh_current_1 / 4096. * 2.5) / 2.5, 
													( 2.5 - ( double) gl_ssh_current_2 / 4096. * 2.5) / 2.5);
#endif

    if( ( ( 2.5 - ( double) gl_ssh_current_1 / 4096. * 2.5) / 2.5  < ( double) flashParamI1min / 65535. * 0.2)  ||
        ( ( 2.5 - ( double) gl_ssh_current_2 / 4096. * 2.5) / 2.5 < ( double) flashParamI2min / 65535. * 0.2)) {
        //не зажглось

        //выключаем высокое
        GP4DAT |= ( 1 << (16 + 0));   //ONHV   (p4.0) = 1

        //инкрементируем и проверяем число попыток поджига
        if( ++nFiringTry == 5) {

          //5 попыток поджига не сработали - отваливаемся в мертвый цикл

          //отключаем поджиг
          GP4DAT |= ( 1 << (16 + 1));   //OFF3KV (p4.1) = 1

          #ifdef DEBUG
            printf("DEBUG: Laser fireup: FAILED\n");
          #endif

          deadloop_no_firing();
        }

        //выжидаем секунду
        pause( 32768);
    }
    else {
      //SUCCESS! зажглось
      GP4DAT |= 1 << (16 + 1);	    //OFF3KV (p4.1) = 1     (отключаем поджиг)

      #ifdef DEBUG
        printf("DEBUG: Laser fireup: successfully passed\n");
      #endif

      break;
    }
  }
#endif

  //сброс интеграторов в системе регулировки периметра
  GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1
  pause( 3277);                  //pause 100msec

  //включение пьезодрайвера периметра
  GP1DAT |= ( 1 << (16 + 7));   //EN_RP   (p1.7) = 1
  

  GP0DAT &= ~( 1 << (16 + 5));  //RP_P   (p0.5) = 0


  //**********************************************************************
  // ПРОВЕРКА ТАКТОВОГО СИГНАЛА
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
  //ТАКТИРОВАНИЕ
  //**********************************************************************
  


  //**********************************************************************
  // Ожидание первого такта TA. Если его не будет в течении 0.5 сек - включаемся в синхронный режим
  //**********************************************************************
  prt2val = T2VAL;

  gl_b_SyncMode = 0;

  while(1) {

  if( gl_b_SyncMode == 0) {
    //ожидаем высокий уровень на ноге TA(p2.7)
    if( GP2DAT & 0x80)
      gl_b_SyncMode = 1;
    } else if( gl_b_SyncMode == 1) {
      //ожидаем высокий уровень на ноге TA(p2.7)
      if( !(GP2DAT & 0x80))
        gl_b_SyncMode = 2;
    } else if( gl_b_SyncMode == 2) {
      //ожидаем высокий уровень на ноге TA(p2.7)
      if( GP2DAT & 0x80) {
        gl_b_SyncMode = 1;
        #ifdef DEBUG
          printf("DEBUG: Got 1-0-1 on P2.7 so working in asynchronous mode\n");
        #endif

        GP3DAT |= ( 1 << (16 + 5));   //OutLnfType (p3.5) = 1  получается асинхронное тактирование

        break;
      }
    }

    //или истечение 0.5 сек
    if( ( double) (( prt2val + T2LD - T2VAL) % T2LD) / 32768. > 0.5) {

      #ifdef DEBUG
        printf("DEBUG: 0.5 sec passed. Zeroing I/O pin 39 (p3.5)...");
      #endif

      GP3DAT &= ~(1 << (16 + 5));     //OutInfType (p3.5) = 0    получается синхронное тактирование
      GP3CLR = (1 << (16 + 5));       //дублёр

      #ifdef DEBUG
        printf("done\n");
      #endif

      gl_b_SyncMode = 0;
      break;
    }
  }

  if( !gl_b_SyncMode) {
    //мы переключились в синхронный режим - надо бы проверить вообще тактирование (даже SA)
    #ifdef DEBUG
      printf("DEBUG: Passed 0.5 sec with TA (P2.7) with no changes. So working in synchronous mode\n");
      printf("DEBUG: Waiting for SA signal on p0.4\n");
    #endif

    //работаем в синхронном режиме - проверка наличия SA
    prt2val = T2VAL;
    while( 1) {
      //ожидаем высокий уровень сигнала на ноге SA_TA (p0.4)
      if( GP0DAT & 0x10) {
        //SA пришел - все ОК
        #ifdef DEBUG
          printf("DEBUG: Got SA signal! SA TEST PASSED\n");
        #endif
        break;
      }

      //или истечения 0.5 сек
      if( ( double) (( prt2val + T2LD - T2VAL) % T2LD) / 32768. > 0.5) {
        //SA не пришел в течении 0.5 сек - у нас нет тактирования - отвал в deadloop

        #ifdef DEBUG
          printf("FAILED\n");
        #endif

        deadloop_no_tact( ERROR_NO_TACT_SIGNAL);
        break;
      }
    }
  }
  
  /*
  //принудительный синхронный режим
  gl_b_SyncMode = 0;
  GP3DAT &= ~(1 << (16 + 5));	//OutLnfType (p3.5) = 0
  */

  #ifdef DEBUG
    printf("passed\n");
    printf("DEBUG: Internal ADC start...");
  #endif

#endif

  //**********************************************************************
  // Запуск преобразования АЦП
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
  //пропуск такта SA_TA (p0.4)
  //**********************************************************************
#ifndef SKIP_START_CHECKS
  while( (GP0DAT & 0x10));
  while( !(GP0DAT & 0x10));
  while( (GP0DAT & 0x10));
#endif

  //**********************************************************************
  
#ifdef DEBUG
  printf("done\n");


  //вычисление первоначального коэффициента вычета
  if( gl_b_SyncMode)
    printf("DEBUG: Calculation of first decrement coefficient\n");  
#endif
  
  if( gl_b_SyncMode)
    FirstDecrementCoeffCalculation();

#ifdef DEBUG
  printf("VALUE=%.2f  ", flashParamDecCoeff / 65535.);
#endif

#ifdef DEBUG
  if( gl_b_SyncMode)
    printf("passed\n");
  
  printf("DEBUG: Configuration passed. Main loop starts!\n");

#endif

  nT2RepeatBang = ( T2VAL - 32768) % T2LD;

  memset( gl_ushCircleBufferAmplForT, '\0', sizeof( gl_ushCircleBufferAmplForT));
  //**********************************************************************
  //**********************************************************************
  //******************* ОСНОВНОЙ ЦИКЛ РАБОТЫ ПРОГРАММЫ *******************
  //**********************************************************************
  //**********************************************************************
  //**********************************************************************

  //gl_un_RULAControl = 2457;    //2457 = 1.500 V
  if( gl_ushFlashParamLastRULA != 0)
    gl_un_RULAControl = gl_ushFlashParamLastRULA;    //восстанавливаем последнее значение RULA
  else
    gl_un_RULAControl = 2457;                    //Если последнего значения не было, то ставим 2457 = 1.500 V

  gl_nAppliedMCoeff = 4096;
  /*
  if( gl_ushFlashParamLastRULM != 0)
    gl_nAppliedMCoeff = gl_ushFlashParamLastRULM;    //восстанавливаем последнее значение RULM
  else
    gl_nAppliedMCoeff = 4096;                    //Если последнего значения не было, то ставим 4096 и далее RULM каждый такт будет делаться -- до нужного значения
  */

  DACConfiguration();

  gl_nSaveRulaRulm = 1;   //запускаем счёт тактов с малым отклонением скользящей средней от заданной  чтобы сохранить RULA-RULM

  while(1) {
    //**********************************************************************
    // Обработка буфера входящих команд
    //**********************************************************************
    if( pos_in_in_buf == IN_COMMAND_BUF_LEN) {
      switch( input_buffer[0]) {
        case 0: //установить код амплитуды
          flashParamAmplitudeCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
          
          /*cRULAControl = ( RULA_MAX - RULA_MIN) / 2;
          delta = ( RULA_MAX - RULA_MIN) / 4;*/

          /*gl_un_RULAControl = 64;*/

          //для 55импульсов амплитуды (это соответствует 55*2.9 = 160") у нас максимум (4096)
          //gl_un_RULAControl = ( unsigned int) ( 4095. / 55. * ( double) flashParamAmplitudeCode);

          gl_un_RULAControl = ( RULA_MAX - RULA_MIN) / 2;

          //nDelta = ( RULA_MAX - RULA_MIN) / 2;
          nDelta = 10;

          //отключаем ошумление
          gl_nAppliedMCoeff = 4096;

          gl_sn_MeaningCounter = 0;
          gl_sn_MeaningCounterRound = 500;
          dMeaningSumm = 0.;
          dMeanImps = 0.;

          gl_dblCircleBufferSumm = 0.;
          gl_nCircleBufferAmplForTPositon = 0;
          gl_bCircleBufferAmplForTOverRound = 0;
          memset( gl_ushCircleBufferAmplForT, '\0', sizeof( gl_ushCircleBufferAmplForT));

          nT2RepeatBang = T2VAL;
          gl_nAmplStabStep = 0;
          gl_nAmplStabApplyRulaTacts = 1;
          gl_nAmplStabMovAvWidth =  100;

        break;

        case 1: //установить код такта подставки
          flashParamTactCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          configure_hanger(); nSentPacksRound = LONG_OUTPUT_PACK_LEN;

          gl_dblCircleBufferSumm = 0.;
          gl_nCircleBufferAmplForTPositon = 0;
          gl_bCircleBufferAmplForTOverRound = 0;
          memset( gl_ushCircleBufferAmplForT, '\0', sizeof( gl_ushCircleBufferAmplForT));

          nT2RepeatBang = T2VAL;
          gl_nAmplStabStep = 0;
          gl_nAmplStabApplyRulaTacts = 1;
          gl_nAmplStabMovAvWidth =  100;

        break;

        case 2: //установить коэффициент M
          flashParamMCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          gl_nAppliedMCoeff = 4096;
          DACConfiguration();
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
          gl_nSaveRulaRulm = 1;

          gl_dblCircleBufferSumm = 0.;
          gl_nCircleBufferAmplForTPositon = 0;
          gl_bCircleBufferAmplForTOverRound = 0;
          memset( gl_ushCircleBufferAmplForT, '\0', sizeof( gl_ushCircleBufferAmplForT));

          nT2RepeatBang = T2VAL;
          gl_nAmplStabStep = 0;
          gl_nAmplStabApplyRulaTacts = 1;
          gl_nAmplStabMovAvWidth =  100;
        break;

        case 3: //установить начальную моду
          flashParamStartMode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          DACConfiguration();
          GP0DAT |= ( 1 << (16 + 5));	  //RP_P   (p0.5) = 1
          nRppTimer = T1VAL;
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
          //gl_b_PerimeterReset = 1;
        break;
        
        case 4: //установить минимальный ток I1
          flashParamI1min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
#ifdef DEBUG
          printf("DEBUG: Input command (0x04 - SetControlI1) accepted. Param: 0x%04x\n", flashParamI1min);
#endif
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 5: //установить минимальный ток I2
          flashParamI2min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 6: //установить 1ый минимум сигнала AmplAng
          flashParamAmplAngMin1 = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 7: //установить коэффициент вычета
          flashParamDecCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 8: //установить SA такт
          flashParamSignCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);

          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 9: //в асинхр. режиме вылючить вывод SA (включить вывод dU)
          bAsyncDu = 1;
        break;

        case 10: //в асинхр. режиме выключить вывод dU (включить вывод SA)
          bAsyncDu = 0;
        break;

        case 11: //калибровка термодатчиков (параметр - реальная температура)
          bCalibrated = 0;
          in_param_temp  = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          if( flashParam_calibT1 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) && 
              flashParam_calibT1 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
              //у нас есть нормальная минимальная точка калибровки

              //затычка на то, чтобы не давали мин точку равной макс
              if( in_param_temp == flashParam_calibT1) {
                nSentPacksRound = LONG_OUTPUT_PACK_LEN;
                break;
              }

              if( flashParam_calibT2 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION)  &&
                  flashParam_calibT2 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
                //у нас есть нормальные минимальная и максимальная точка калибровки
                //определим какую надо заменить
                if( in_param_temp < flashParam_calibT1) {
                  //надо заменить минимальную
                  bCalibProcessState = 1;
                  flashParam_calibT1 = in_param_temp;
                }
                else {
                  //надо заменить максимальную
                  bCalibProcessState = 3;
                  flashParam_calibT2 = in_param_temp;
                }
              }
              else {
                //у нас есть нормальная минимальная точка калибровки, но нет нормальной максимальной
                bCalibProcessState = 3;
                flashParam_calibT2 = in_param_temp;
              }

          }
          else {
            //у нас нет даже минимальной точки!! значит это будет минимальная :)
            flashParam_calibT1 = in_param_temp;
            bCalibProcessState = 1;
          }
        break;

        case 12:    //команда сброса данных калибровки термодатчиков
          bCalibrated = 0;
          flashParam_calibT1 = 0;
          flashParamT1_TD1_val = 0;
          flashParamT1_TD2_val = 0;

          flashParam_calibT2 = 0;
          flashParamT2_TD1_val = 1;
          flashParamT2_TD2_val = 1;

          SaveThermoCalibPoint();
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 14:    //команда нового фазового сдвига
          in_param_temp  = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          flashParamPhaseShift = in_param_temp;
          //SendPhaseShift();
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 15:    //команда выключения лазера
          GP4DAT |= 1 << (16 + 0);      //ONHV       (p4.0) = 1
          GP4DAT |= 1 << (16 + 1);      //OFF3KV     (p4.1) = 1
        break;

        case 49: //запрос параметров
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 50: //сохранить параметры во флэш память

          gl_ushFlashParamLastRULA = gl_un_RULAControl;
          gl_ushFlashParamLastRULM = gl_nAppliedMCoeff;

#ifdef DEBUG
          printf("DEBUG: Input command (0x50 - SaveParams).\n");
#endif
          save_params();
        break;

        case 51: //перезагрузить параметры из флэш-памяти и показать их
          load_params(); nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 100: //FAKE! установить код ошибки (типа ошибка прибора)
          gl_c_EmergencyCode = input_buffer[1];
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

    //наёбочная часть - эмуляция такта 10(5?) раз в секунду
    prt2val = T2VAL;
    while( (( 0x1000 + T2VAL - prt2val) % 0x1000) < 3276);
    bSAFake ^= 1;

    //if( bSAFake) {	// на ноге SA_TA (P0.4) есть FAKE сигнал
    if( GP0DAT & 0x10) {	//на ноге SA_TA (P0.4) есть сигнал

      #ifdef DEBUG
        printf("DEBUG: got tact synchro signal! %d\n", gl_b_SA_Processed);
      #endif
      if( gl_b_SA_Processed == 0) { //если в этом SA цикле мы его еще не обрабатывали

        //отсечка получения TA_SA (для вычисления длительности)
        gl_ssh_SA_time = ( T1LD + gl_ssh_prT1VAL - T1VAL) % T1LD;
        gl_ssh_prT1VAL = T1VAL;

        // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
        // 1. запрашиваем у альтеры код счётчика информационных импульсов
        //ждём высокого уровня сигнала готовности CntReady (p4.2)
        while( !( GP4DAT & 0x04)) {}

        //запрашиваем старший байт кода счётчика информационных импульсов
        //GP1SET = 1 << (16 + 3);  //RDHBC (p1.3) = 1			WAY1
		    GP1DAT |= 1 << (16 + 3);  //RDHBC (p1.3) = 1			WAY2

        pause( 1);                //пауза

        //чтение
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


        //запрашиваем младший байт кода счётчика информационных импульсов
        //GP1SET = 1 << (16 + 4);  //RDLBC (p1.4) = 1		WAY1
		    GP1DAT |= 1 << (16 + 4);  //RDLBC (p1.4) = 1		WAY2

        pause( 1);                //пауза

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

        //складываем два байта
        gl_ssh_angle_inc = lb + (hb << 8);




        // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
        // 2. в случае асинхр режима, запрашиваем у альтеры код счётчика информационных импульсов
        if( gl_b_SyncMode) {
          //ждём высокого уровня сигнала готовности ANGLE_READY (p2.4)
          while( !( GP2DAT & 0x10)) {}

            //запрашиваем старший байт угла поворота вибратора
            GP0SET = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 1
            pause( 1);                //пауза

            //чтение
            hb =  (( GP1DAT & BIT_5) >> 5) +
                  ((( GP0DAT & BIT_7) >> 7) << 1) +
                  ((( GP0DAT & BIT_1) >> 1) << 2) +
                  ((( GP2DAT & BIT_3) >> 3) << 3) +
                  ((( GP4DAT & BIT_6) >> 6) << 4) +
                  ((( GP4DAT & BIT_7) >> 7) << 5) +
                  ((( GP0DAT & BIT_6) >> 6) << 6) +
                  ((( GP0DAT & BIT_2) >> 2) << 7);

            GP0CLR = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 0


            //запрашиваем младший байт кода счётчика информационных импульсов
            GP2SET = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 1
            pause( 1);                //пауза

            lb = (( GP1DAT & BIT_5) >> 5) +
                  ((( GP0DAT & BIT_7) >> 7) << 1) +
                  ((( GP0DAT & BIT_1) >> 1) << 2) +
                  ((( GP2DAT & BIT_3) >> 3) << 3) +
                  ((( GP4DAT & BIT_6) >> 6) << 4) +
                  ((( GP4DAT & BIT_7) >> 7) << 5) +
                  ((( GP0DAT & BIT_6) >> 6) << 6) +
                  ((( GP0DAT & BIT_2) >> 2) << 7);

            GP2CLR = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 0

            //складываем два байта
            gl_ssh_angle_hanger = lb + (hb << 8);

        }

        //**********************************************************************
        // Получение аналогового параметра
        //**********************************************************************
        while( !( ADCSTA & 0x01)){}     // ожидаем конца преобразования АЦП (теоретически когда мы приходим сюда он уже должен быть готов)

        switch( ADCChannel) { //анализируем что мы оцифровывали и сохраняем в соответствующую переменную
          case 0: //UTD3
            gl_ssh_Utd3 = (ADCDAT >> 16);
            gl_ssh_Utd3_cal = gl_ssh_Utd3;
            if( bCalibrated)
              temp_t = ( double) gl_ssh_Utd3 * TD1_K + TD1_B;
            else
              temp_t = -1481.96 + sqrt( 2.1962e6 + ( ( 1.8639 - ( double) gl_ssh_Utd3 / 4096. * 2.5) / 3.88e-6));
            gl_ssh_Utd3 = ( short) ( ( temp_t + 100.) / 200. * 65535.);
          break;  //UTD3

          case 1: //UTD1
            gl_ssh_Utd1 = (ADCDAT >> 16);
            gl_ssh_Utd1_cal = gl_ssh_Utd1;
            if( bCalibrated)
              temp_t = ( double) gl_ssh_Utd1 * TD1_K + TD1_B;
            else
              temp_t = -1481.96 + sqrt( 2.1962e6 + ( ( 1.8639 - ( double) gl_ssh_Utd1 / 4096. * 2.5) / 3.88e-6));
            gl_ssh_Utd1 = ( short) ( ( temp_t + 100.) / 200. * 65535.);
          break;  //UTD1

          case 2: //UTD2
            gl_ssh_Utd2 = (ADCDAT >> 16);
            gl_ssh_Utd2_cal = gl_ssh_Utd2;
            if( bCalibrated)
              temp_t = ( double) gl_ssh_Utd2 * TD1_K + TD1_B;
            else
              temp_t = -1481.96 + sqrt( 2.1962e6 + ( ( 1.8639 - ( double) gl_ssh_Utd2 / 4096. * 2.5) / 3.88e-6));
            gl_ssh_Utd2 = ( short) ( ( temp_t + 100.) / 200. * 65535.);
          break;  //UTD2

          case 3: gl_ssh_current_1 = (ADCDAT >> 16); break;     //I1
          case 4: gl_ssh_current_2 = (ADCDAT >> 16); break;     //I2
          case 5: gl_ssh_Perim_Voltage = (ADCDAT >> 16); break; //CntrPc
          case 6: gl_ssh_ampl_angle = (ADCDAT >> 16); break;    //AmplAng
        }

        if( bCalibProcessState) {
          switch( bCalibProcessState) {
            case 1:
              //калибруем первый датчик на минимальной температуре
              if( ADCChannel == 1) {
                flashParamT1_TD1_val = gl_ssh_Utd1_cal;
                bCalibProcessState = 2;
              }
            break;

            case 2:
              //калибруем второй датчик на минимальной температуре
              if( ADCChannel == 2) {
                flashParamT1_TD2_val = gl_ssh_Utd2_cal;
                bCalibProcessState = 3;
              }
            break;

            case 3:
              //калибруем третий датчик на минимальной температуре
              if( ADCChannel == 0) {
                flashParamT1_TD3_val = gl_ssh_Utd3_cal;
                bCalibProcessState = 0;
                SaveThermoCalibPoint();
                nSentPacksRound = LONG_OUTPUT_PACK_LEN;
              }
            break;

            case 4:
              //калибруем первый датчик на максимальной температуре
              if( ADCChannel == 1) {
                flashParamT2_TD1_val = gl_ssh_Utd1_cal;
                bCalibProcessState = 5;
              }
            break;

            case 5:
              //калибруем второй датчик на максимальной температуре
              if( ADCChannel == 2) {
                flashParamT2_TD2_val = gl_ssh_Utd2_cal;
                bCalibProcessState = 6;
              }

            case 6:
              //калибруем третий датчик на максимальной температуре
              if( ADCChannel == 0) {
                flashParamT2_TD3_val = gl_ssh_Utd3_cal;
                bCalibProcessState = 0;
                SaveThermoCalibPoint();
                nSentPacksRound = LONG_OUTPUT_PACK_LEN;
              }

            break;
          }

          if( !bCalibProcessState)
            //если это закончилась калбировка какой либо точки - перерасчитаем калибровочные параметры
            ThermoCalibrationCalculation();
        }

        ADCChannel = (++ADCChannel) % 7;        //увеличиваем счетчик-указатель измеряемых аналог. параметров

        ADCCP = ADCChannel;              //выставляем новый канал АЦП
        pause( 10);
        ADCCON |= 0x80;                  //запуск нового преобразования (съем будет в следующем такте SA)





        //**********************************************************************
        // Получение 1/2 амплитуды колебаний виброподвеса в виде числа импульсов от альтеры
        //**********************************************************************

        //ждём высокого уровня сигнала готовности AMPL_FOR_T_READY (p3.2)
        while( !( GP3DAT & 0x04)) {}

        //запрашиваем среднее за период частотной подставки
        GP3DAT |= 1 << (16 + 3);  //RD_AMPL_T_CODE (p3.3) -> 1
        pause( 1);                //пауза

        //чтение
        hb = (( GP1DAT & BIT_5) >> 5) +
             ((( GP0DAT & BIT_7) >> 7) << 1) +
             ((( GP0DAT & BIT_1) >> 1) << 2) +
             ((( GP2DAT & BIT_3) >> 3) << 3) +
             ((( GP4DAT & BIT_6) >> 6) << 4) +
             ((( GP4DAT & BIT_7) >> 7) << 5) +
             ((( GP0DAT & BIT_6) >> 6) << 6) +
             ((( GP0DAT & BIT_2) >> 2) << 7);

        GP3CLR = 1 << (16 + 3);  //RD_AMPL_T_CODE (p3.3) -> 0

        //складываем два байта (хотя он тут один)
        gl_ush_MeanImpulses = hb;
        CircleBufferAmplForT_add( gl_ush_MeanImpulses);

        #ifdef DEBUG
          printf("DEBUG: Ampl: %d Mean: %.2f\n", gl_ush_MeanImpulses, CircleBufferAmplForT_mean( gl_sn_MeaningCounterRound));
        #endif


        //**********************************************************************
        // Выдача данных согласно протоколу
        //**********************************************************************
        switch( nSentPacksCounter) {
          case 0: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 0, gl_dblCircleBufferMean);      break; //UTD3
          case 1: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 1, gl_ssh_Utd1);      break; //UTD1
          case 2: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 2, gl_un_RULAControl);      break; //UTD2
          case 3: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 3, gl_ssh_current_1);      break; //I1
          case 4: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 4, gl_ssh_current_2);      break; //I2
          case 5: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 5, gl_ssh_Perim_Voltage);  break; //CntrPc
          case 6: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 6, gl_ush_MeanImpulses);   break; //AmplAng

          case 7: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 7, flashParamAmplitudeCode);  break;  //код амплитуды
          case 8: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 8, flashParamTactCode);       break;  //код такта подставки
          case 9: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 9, flashParamMCoeff);         break;  //Коэффициент М
          case 10: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 10, flashParamStartMode);     break;  //Начальная мода
          case 11: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 11, flashParamI1min);        break;  //flashParamI1min
          case 12: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 12, flashParamI2min);        break;  //flashParamI2min
          case 13: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 13, flashParamAmplAngMin1);  break;  //flashParamAmplAngMin1
          case 14: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 14, flashParamDecCoeff);     break;  //flashParamAmplAngMin2
          case 15: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 15, gl_nAmplStabStep);    break;  //flashParamSignCoeff
          case 16: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 16, 0);    break;               //SOFTWARE VERSION
          case 17: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 17, flashParam_calibT1);     break;  //min thermo-calib point T
          case 18: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 18, flashParamT1_TD1_val);   break;  //min thermo-calib point thermo1 data
          case 19: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 19, flashParamT1_TD2_val);   break;  //min thermo-calib point thermo2 data
          case 20: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 20, flashParam_calibT2);     break;  //max thermo-calib point T
          case 21: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 21, flashParamT2_TD1_val);   break;  //max thermo-calib point thermo1 data
          case 22: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 22, flashParamT2_TD2_val);   break;  //max thermo-calib point thermo2 data
          case 23: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 23, gl_ushFlashParamLastRULA); break;//flashParamPhaseShift);   break;  //phase shift
        }

        //РАБОЧЕЕ ПЕРЕВЫЧИСЛЕНИЕ КОЭФФИЦИЕНТА ВЫЧЕТА
        if( gl_b_SyncMode) {

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
              flashParamDecCoeff = ( short) ( gl_dbl_Nsumm / gl_dbl_Usumm * 65535.);
              gl_dbl_Nsumm = gl_dbl_Usumm = 0.;
              gl_un_DecCoeffStatPoints = 0;
              nSentPacksRound = LONG_OUTPUT_PACK_LEN;
            }
          }
        }

        gl_ssh_angle_inc_prev = gl_ssh_angle_inc;

        // ************************************************************************************
        // 2010-04-22
        //автоматическая перестройка периметра
        // ************************************************************************************
        if( nSentPacksCounter == 5) {
          V_piezo = (( gl_ssh_Perim_Voltage / 4095. * 2.5) - 1.23) * 101.;
          if( fabs( V_piezo) > 80.) {
            flashParamStartMode = 125;
            DACConfiguration();
            GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1
            nRppTimer = T1VAL;
            nSentPacksRound = LONG_OUTPUT_PACK_LEN;
            gl_b_PerimeterReset = 1;
          }
        }


        //увеличиваем счетчик отправленных посылок данных
        if( ++nSentPacksCounter == LONG_OUTPUT_PACK_LEN) {
          nSentPacksCounter = 0;
          nSentPacksRound = SHORT_OUTPUT_PACK_LEN;

          ADCChannel = 0;     //запускаем измерения с UTD3
          ADCCP = ADCChannel;
		      pause( 10);
          ADCCON |= 0x80;
        }
        else
          nSentPacksCounter = ( nSentPacksCounter) % nSentPacksRound;


		    //**********************************************************************
        //Стабилизация амплитуды колебаний виброподвеса
        //**********************************************************************
        if( gl_nAmplStabStep < 10) {
          if( T2VAL <= nT2RepeatBang) {
            gl_nAmplStabStep++;
            nT2RepeatBang = ( T2VAL - 32768) % T2LD;

            gl_dblCircleBufferSumm = 0.;
            gl_nCircleBufferAmplForTPositon = 0;
            gl_bCircleBufferAmplForTOverRound = 0;
            memset( gl_ushCircleBufferAmplForT, '\0', sizeof( gl_ushCircleBufferAmplForT));

            gl_nAppliedMCoeff = 4096. * (1. - (1. - ( double) flashParamMCoeff / 250.) / 10. * ( double) gl_nAmplStabStep);

            switch( gl_nAmplStabStep) {
              case  0:  gl_nAmplStabApplyRulaTacts = 1;  gl_nAmplStabMovAvWidth =  100;  break;
              case  1:  gl_nAmplStabApplyRulaTacts = 1;  gl_nAmplStabMovAvWidth =  200;  break;
              case  2:  gl_nAmplStabApplyRulaTacts = 1;  gl_nAmplStabMovAvWidth =  400;  break;
              case  3:  gl_nAmplStabApplyRulaTacts = 1;  gl_nAmplStabMovAvWidth =  800;  break;
              case  4:  gl_nAmplStabApplyRulaTacts = 2;  gl_nAmplStabMovAvWidth = 1000;  break;
              case  5:  gl_nAmplStabApplyRulaTacts = 4;  gl_nAmplStabMovAvWidth = 1500;  break;
              case  6:  gl_nAmplStabApplyRulaTacts = 5;  gl_nAmplStabMovAvWidth = 2000;  break;
              case  7:  gl_nAmplStabApplyRulaTacts = 10; gl_nAmplStabMovAvWidth = 2500;  break;
              case  8 : gl_nAmplStabApplyRulaTacts = 20; gl_nAmplStabMovAvWidth = 3000;  break;
              case  9:  gl_nAmplStabApplyRulaTacts = 40; gl_nAmplStabMovAvWidth = 3000;  break;
              case 10:  gl_nAmplStabApplyRulaTacts = 45; gl_nAmplStabMovAvWidth = 3000;  break;
            }
          }
        }

        dblDelta = flashParamAmplitudeCode - gl_dblCircleBufferMean;
        if( gl_nCircleBufferAmplForTPositon % gl_nAmplStabApplyRulaTacts == 0) {

          //если расхождение скользящей средней и заданной амплитуд большое - подкрутим RULA
          if( fabs( dblDelta) > 0.5) {

            if( dblDelta > 0)
              gl_un_RULAControl++;
            else
              gl_un_RULAControl--;
          }
        }

        if( gl_un_RULAControl > RULA_MAX) gl_un_RULAControl = RULA_MAX;
        if( gl_un_RULAControl < RULA_MIN) gl_un_RULAControl = RULA_MIN;

        //применяем RULA-RULM (там же RULM применяется плавно-медленно)
        DACConfiguration();


        //**********************************************************************
        //обработка флага сброса RP_P
        //**********************************************************************
        if( nRppTimer != 0) {
          if( (( T1LD + nRppTimer - T1VAL) % T1LD) > 327) {
            //сброс интеграторов в системе регулировки периметра
            GP0DAT &= ~( 1 << (16 + 5));	//RP_P   (p0.5) = 0
            nRppTimer = 0;
          }
        }

        //поднимаем флаг о том что текущий высокий уровень SA мы обработали
        gl_b_SA_Processed = 1;

        //В тестовых целях делаем сигнал на линии P0.0
        GP0DAT |= 1 << ( 16);	//тестовая линия p0.0 set
        for( i=0; i<100; i++);
        GP0DAT &= ~( 1 << ( 16));	//тестовая линия p0.0 clear      
      }
#ifdef DEBUG
  printf("DEBUG: Main loop ends... %d %d \n", nSentPacksCounter, gl_b_SA_Processed);
#endif
    }
    else {
      //если линия сигнала SA в низком уровне - то как только она поднимется начнется новый необработанный такт
      gl_b_SA_Processed = 0;

      //проверка тактирования
      ush_SA_check_time = ( T1LD + gl_ssh_prT1VAL - T1VAL) % T1LD;

      //2 sec = 32768 * 2.0 = 65536
      if( ush_SA_check_time > 65536) {
        //пропало тактирование

        #ifdef DEBUG
          printf("DEBUG: Tact signal lost!");
        #endif

        //отключаем горение
        GP4DAT |= ( 1 << ( 16 + 0));  //ONHV (p4.0) = 1

        deadloop_no_tact( ERROR_TACT_SIGNAL_LOST);
      }
    }
  }
}
