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

#define LONG_OUTPUT_PACK_LEN 24       //когда запрашивают мы выдаем 25 пачки (со всеми доп. параметрами по очереди)
#define SHORT_OUTPUT_PACK_LEN 7       //в норме мы выдаем циклически по 7 пачек с аналог. параметрами

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
int gl_n_prT1VAL = 0x1000;   //период SA

signed short gl_ssh_angle_hanger = 0;       //угол отклонения вибро-подвеса
signed short gl_ssh_angle_hanger_prev = 0;  //предыдущее значение угла отклонения вибро-подвеса

unsigned short gl_ush_MeanImpulses = 1;

#define RULA_MAX 4095
#define RULA_MIN 25

//unsigned int gl_un_RULAControl = 64;      //64   = 0.039 V  
//unsigned int gl_un_RULAControl = 1638;    //1638 = 1.000 V  
//unsigned int gl_un_RULAControl = 2457;    //2457 = 1.500 V  
unsigned int gl_un_RULAControl = 4095;      //4095 = 2.500 V  

char gl_c_EmergencyCode = 0;            //код ошибки


char bAsyncDu = 0;                      //флаг передачи времени SA или приращ. угла в асинхр. режиме: 0-передается SA 1-передается dU


//ФЛАГИ
char gl_b_SyncMode = 0;                 //флаг режима работы гироскопа:   0=синхр. 1=асинхр.
char gl_chAngleOutput =   0;            //флаг вывода приращения угла: 0 = dW (4b)         1 = dN (2b), dU(2b)
char gl_chLockBit = 0;                  //флаг блокирования устройства: 0 - режим "разработчика"   1 - режим "пользователя"
//char gl_bOutData = 0;                 //флаг выдачи данных наружу (включается через 2.5 сек после включения прибора) 0 - нет выдачи; 1 - выдача данных;
char gl_n_PerimeterReset = 0;           //флаг сигнала сброса периметра (0 = данные достоверны, 1 = данные НЕдостоверны, прошло выключение интегратора, 2 = данные НЕдостоверны, прошло включение интегратора)
char gl_b_SA_Processed = 0;             //флаг окончания обработки сигнала SA
char gl_bManualLaserOff = 0;            //флаг что сами выключили ток лазера (командой). Флаг нужен чтобы исключить это событие в отслеживателе просадок тока.
char gl_bSimpleDnDuRegime = 0;          //флаг режима "частотной нарезки" (выдачи dN,dU с максимально возможной частотой, без снятий аналоговых параметров, без СРП, без контроля амплитуды)

//Засечки таймеров
int gl_nRppTimerT1 = 0;                 //засечка таймера для проведения сброса интегратора Системы Регулировки Периметра
int gl_nSmoothMCoeffApplyT2;            //засечка таймера для плавного применения коэффициента ошумления

short gl_nSentPackIndex;                //индекс выдаваемой посылки
char gl_c_OutPackCounter = 0;           //выдаваемый наружу счётчик посылок

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

//Буфер входящих команд
char input_buffer[6] = { 0, 0, 0, 0, 0, 0};     //буфер входящих команд
char pos_in_in_buf = 0;                         //позиция записи в буфере входящих команд

//ПАРАМЕТРЫ ХРАНИМЫЕ ВО ФЛЭШ-ПАМЯТИ
unsigned short flashParamAmplitudeCode = 9000;    //амплитуда колебаний виброподвеса
unsigned short flashParamTactCode = 0;            //код такта ошумления
unsigned short flashParamMCoeff = 4;              //коэффициент ошумления
unsigned short flashParamStartMode = 5;           //начальная мода Системы Регулировки Периметра
unsigned short flashParamDecCoeff = 0;            //коэффициент вычета
unsigned short flashLockDev = 0;                  //флаг блокировки устройства

unsigned short flashParamI1min = 0;               //контрольное значение тока поджига I1
unsigned short flashParamI2min = 0;               //контрольное значение тока поджига I2
unsigned short flashParamAmplAngMin1 = 0;         //контрольное значение сигнала раскачки с ДУСа

unsigned short flashParamSignCoeff = 2;           //знаковый коэффициент
unsigned short flashParamDeviceId = 0;            //ID устройства
unsigned short flashParamDateYear = 0;            //дата ? прибора.год
unsigned short flashParamDateMonth = 0;           //дата ? прибора.месяц
unsigned short flashParamDateDay = 0;             //дата ? прибора.день
char flashParamOrg[17] = { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0};    //название организации

unsigned short gl_ushFlashParamLastRULA = 0;      //последнее RULA (obsolete)
unsigned short gl_ushFlashParamLastRULM = 0;      //последнее RULM (obsolete)

double dStartAmplAngCheck = 0.5;

unsigned short nFiringTry = 0;

//калибровка термодатчиков
signed short flashParam_calibT1;
unsigned short flashParamT1_TD1_val, flashParamT1_TD2_val, flashParamT1_TD3_val;
signed short flashParam_calibT2;
unsigned short flashParamT2_TD1_val, flashParamT2_TD2_val, flashParamT2_TD3_val;

char gl_cCalibProcessState;
char gl_bCalibrated;
double TD1_K, TD1_B;
double TD2_K, TD2_B;



//Плавное введение коэффициента ошумления
int gl_nAppliedMCoeff;          //Применённый коэффициент ошумления (мы его вводим плавно)
int gl_nAmplStabStep = 0;       //плавное введение ошумления

//Переменные участвующие в работе системы регулировки амплитуды
double gl_dblAmplMean;
int  gl_nAmplMeanCounter;
unsigned int gl_un_PrevAmplRegulationT2;

//Переменные участвующие в рассчёте коэффициента вычета "на лету"
double gl_dblMeanAbsDn;
double gl_dblMeanAbsDu;
int    gl_nMeanDecCoeffCounter;
unsigned int gl_un_PrevT2DecCoeffCalc;


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

void send_pack( short shAnalogueParamValue) {
  short shAnalogueParamIndicator = gl_nSentPackIndex;
  signed short angle_inc1 = ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536;

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
  putchar_nocheck( ( ( gl_n_PerimeterReset > 0) ? 0x80 : 0x00) + shAnalogueParamIndicator & 0xff);
  cCheckSumm += (( ( gl_n_PerimeterReset > 0) ? 0x80 : 0x00) + shAnalogueParamIndicator & 0xff);

  //***************************************************************************
  //ANALOG PARAMETER
  //***************************************************************************
  putchar_nocheck(   shAnalogueParamValue & 0xff);
  cCheckSumm += (    shAnalogueParamValue & 0xff);

  putchar_nocheck( ( shAnalogueParamValue & 0xff00) >> 8);
  cCheckSumm += ( (  shAnalogueParamValue & 0xff00) >> 8);

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
/*
#ifdef DEBUG
  printf("DEBUG: DACConfiguration(): enter\n");
#endif
*/
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
  
  COMDIV0 = 1;      //младший байт делителя (DL)
  COMDIV1 = 0x00;   //старший байт делителя (DL)
  COMDIV2 = 0x8B55; //16-разрядный регистр дробного делителя
                    //15 - FBEN - бит разрешения работы генератора с дробным делителем
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

  //COMDIV0 = 1;      //младший байт делителя (DL)
  //COMDIV1 = 0x00;   //старший байт делителя (DL)
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

  //COMDIV0 = 1;      //младший байт делителя (DL)
  //COMDIV1 = 0x00;   //старший байт делителя (DL)
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

  //COMDIV0 = 1;      //младший байт делителя (DL)
  //COMDIV1 = 0x00;   //старший байт делителя (DL)
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

  //COMDIV0 = 1;      //младший байт делителя (DL)
  //COMDIV1 = 0x00;   //старший байт делителя (DL)
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

  //COMDIV0 = 1;      //младший байт делителя (DL)
  //COMDIV1 = 0x00;   //старший байт делителя (DL)
  //COMDIV2 = 0x8A8D; //16-разрядный регистр дробного делителя
                    //15 - FBEN - бит разрешения работы генератора с дробным делителем
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

  //COMDIV0 = 1;      //младший байт делителя (DL)
  //COMDIV1 = 0x00;   //старший байт делителя (DL)
  //COMDIV2 = 0x8A72; //16-разрядный регистр дробного делителя
                    //15 - FBEN - бит разрешения работы генератора с дробным делителем
                    //14-13 reserved
                    //12-11 M
                    //10-0  N
  
  /****************************************************************************************** */
}

void testPike() {
  //В тестовых целях делаем сигнал на линии P0.0
  GP0DAT |= 1 << ( 16);	//тестовая линия p0.0 set
  //for( i=0; i<100; i++);
  GP0DAT &= ~( 1 << ( 16));	//тестовая линия p0.0 clear
}

void main() {
  unsigned short ush_SA_check_time;

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

  double dbl_N1, dbl_N2, dbl_U1, dbl_U2;

  float Coeff;

  double dbl_dN, dbl_dU;

  gl_cCalibProcessState = 0;    //0 - no calibration

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
  // Конфигурация Timer0
  //**********************************************************************
  T0CON = 0x80;           //core clocking

  //**********************************************************************
  // Конфигурация Timer1
  //**********************************************************************
  T1CON = 0x2C0;          //32kHz clocking
  T1LD = 0x100000;

  //**********************************************************************
  // Конфигурация Timer2
  //**********************************************************************
  T2CON = 0x2C0;          //32kHz clocking
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
    gl_un_RULAControl = 1400;

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
    //поджигаем усиленно (поджиг) (включено by default, но мы его иногда выключаем (по кратным 5 числам попыток)
    GP4DAT &= ~( 1 << (16 + 1));  //OFF3KV (p4.1) = 0

    //подаём высокое
    GP4DAT &= ~( 1 << (16 + 0));  //ONHV   (p4.0) = 0

    //выжидаем 0,5 секунды
    pause( 16384);

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
  printf("DEBUG: Laser fireup: Measured I1=%.02f   Measured I2=%.02f\n",
            ( 2.5 - ( double) gl_ssh_current_1 / 4096. * 2.5) / 2.5, 
            ( 2.5 - ( double) gl_ssh_current_2 / 4096. * 2.5) / 2.5);
#endif

    if( ( ( 2.5 - ( double) gl_ssh_current_1 / 4096. * 2.5) / 2.5  < ( double) flashParamI1min / 65535. * 0.2)  ||
        ( ( 2.5 - ( double) gl_ssh_current_2 / 4096. * 2.5) / 2.5 < ( double) flashParamI2min / 65535. * 0.2)) {
        //не зажглось

        //снимаем 3kV линию (остаётся 800V)
        GP4DAT |= ( 1 << (16 + 1));   //OFF3KV (p4.1) = 1

        //инкрементируем и проверяем число попыток поджига
        if( ++nFiringTry >= 25) {

          //5 пачек по 5 попыток поджига не сработали - отваливаемся в мертвый цикл

          //выключаем высокое 800V
          GP4DAT |= ( 1 << (16 + 0));   //ONHV   (p4.0) = 1

          //отключаем поджиг 3kV
          GP4DAT |= ( 1 << (16 + 1));   //OFF3KV (p4.1) = 1

          #ifdef DEBUG
            printf("DEBUG: Laser fireup: FAILED\n");
          #endif

          deadloop_no_firing( ERROR_NO_LASER_FIRING);
        }

        if( ( nFiringTry % 5)) {
          //раз в пять попыток отключаем 800V линию

          //выключаем высокое 800V
          GP4DAT |= ( 1 << (16 + 0));   //ONHV   (p4.0) = 1

          //выжидаем 0,5 секунды (в сумме с последующей это даст 1 сек, между пачками по 5 импульсов)
          pause( 16384);
        }

        //выжидаем 0,5 секунды
        pause( 16384);
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

  //иницилизация переменных рассчёта скользящей средней амплитуды
  gl_dblAmplMean = 0.;
  gl_nAmplMeanCounter = 0;

  //инициализация переменных рассчёта скользящей средней коэффициента вычета
  gl_dblMeanAbsDn = 0.;
  gl_dblMeanAbsDu = 0.;
  gl_nMeanDecCoeffCounter = 0;

  gl_nSmoothMCoeffApplyT2 = ( T2VAL - 32768) % T2LD;
  gl_un_PrevAmplRegulationT2 = T2VAL;
  gl_un_PrevT2DecCoeffCalc = T2VAL;


  //FAKE ('VERACITY DATA' flag on)
  gl_n_PerimeterReset = 1;

  //Starting packs.VERSION
  gl_nSentPackIndex = VERSION;
  send_pack( ( ( VERSION_MINOR * 16) << 8) + ( VERSION_MAJOR * 16 + VERSION_MIDDLE));

  //Starting packs.Device_Num
  gl_nSentPackIndex = DEVNUM;
  send_pack( flashParamDeviceId);

  //FAKE ('VERACITY DATA' flag off)
  gl_n_PerimeterReset = 0;

  gl_nSentPackIndex = UTD1;

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
    gl_un_RULAControl = 1400;                    //LIE!!   Если последнего значения не было, то ставим 2457 = 1.500 V

  gl_nAppliedMCoeff = 4096;


  DACConfiguration();


  while(1) {
    if( gl_bSimpleDnDuRegime == 1) {
      SimpleMaxRateDnDuRegime();
    }

    processIncomingCommand();

    if( gl_bSimpleDnDuRegime == 1) {
      continue;
    }



    /*
    //наёбочная часть - эмуляция такта 10(5?) раз в секунду
    prt2val = T2VAL;
    while( (( 0x1000 + T2VAL - prt2val) % 0x1000) < 3276);
    bSAFake ^= 1;
    */

    //if( bSAFake) {	// на ноге SA_TA (P0.4) есть FAKE сигнал
    if( GP0DAT & 0x10) {	//на ноге SA_TA (P0.4) есть сигнал
      /*
      #ifdef DEBUG
        printf("DEBUG: got tact synchro signal! %d\n", gl_b_SA_Processed);
      #endif
      */
      if( gl_b_SA_Processed == 0) { //если в этом SA цикле мы его еще не обрабатывали

        testPike();

        //отсечка получения TA_SA (для вычисления длительности)
        gl_ssh_SA_time = ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
        gl_n_prT1VAL = T1VAL;

        // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
        // 1. запрашиваем у альтеры код счётчика информационных импульсов
        //ждём высокого уровня сигнала готовности CntReady (p4.2)
        while( !( GP4DAT & 0x04)) {}

        //запрашиваем старший байт кода счётчика информационных импульсов
        //GP1SET = 1 << (16 + 3);  //RDHBC (p1.3) = 1			WAY1
        GP1DAT |= 1 << (16 + 3);  //RDHBC (p1.3) = 1			WAY2

        //pause( 1);                //пауза

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

        //pause( 1);                //пауза

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
        // 2. в случае асинхр режима, запрашиваем у альтеры код счётчика информационных импульсов
        if( gl_b_SyncMode) {
          //ждём высокого уровня сигнала готовности ANGLE_READY (p2.4)
          while( !( GP2DAT & 0x10)) {}

            //запрашиваем старший байт угла поворота вибратора
            GP0SET = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 1
            //pause( 1);                //пауза

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
            //pause( 1);                //пауза

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
        // Получение аналогового параметра
        //**********************************************************************
        while( !( ADCSTA & 0x01)){}     // ожидаем конца преобразования АЦП (теоретически когда мы приходим сюда он уже должен быть готов)

        switch( ADCChannel) { //анализируем что мы оцифровывали и сохраняем в соответствующую переменную
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

        if( gl_cCalibProcessState) {
          switch( gl_cCalibProcessState) {
            case 1:
              //калибруем первый датчик на минимальной температуре
              if( ADCChannel == 1) {
                flashParamT1_TD1_val = gl_ssh_Utd1_cal;
                gl_cCalibProcessState = 2;
              }
            break;

            case 2:
              //калибруем второй датчик на минимальной температуре
              if( ADCChannel == 2) {
                flashParamT1_TD2_val = gl_ssh_Utd2_cal;
                gl_cCalibProcessState = 3;
              }
            break;

            case 3:
              //калибруем третий датчик на минимальной температуре
              if( ADCChannel == 0) {
                flashParamT1_TD3_val = gl_ssh_Utd3_cal;
                gl_cCalibProcessState = 0;
                save_params_p4();
                gl_nSentPackIndex = CALIB_T1;
              }
            break;

            case 4:
              //калибруем первый датчик на максимальной температуре
              if( ADCChannel == 1) {
                flashParamT2_TD1_val = gl_ssh_Utd1_cal;
                gl_cCalibProcessState = 5;
              }
            break;

            case 5:
              //калибруем второй датчик на максимальной температуре
              if( ADCChannel == 2) {
                flashParamT2_TD2_val = gl_ssh_Utd2_cal;
                gl_cCalibProcessState = 6;
              }

            case 6:
              //калибруем третий датчик на максимальной температуре
              if( ADCChannel == 0) {
                flashParamT2_TD3_val = gl_ssh_Utd3_cal;
                gl_cCalibProcessState = 0;
                save_params_p4();
                gl_nSentPackIndex = CALIB_T1;
              }

            break;
          }

          if( !gl_cCalibProcessState)
            //если это закончилась калбировка какой либо точки - перерасчитаем калибровочные параметры
            ThermoCalibrationCalculation();
        }

        /*if( ADCChannel == 2) ADCChannel=8;
        else if( ADCChannel == 8) ADCChannel=4;
        else*/
          ADCChannel = (++ADCChannel) % 7;        //увеличиваем счетчик-указатель измеряемых аналог. параметров


        ADCCP = ADCChannel;              //выставляем новый канал АЦП
        //pause( 10);
        ADCCON |= 0x80;                  //запуск нового преобразования (съем будет в следующем такте SA)




        testPike();

        //**********************************************************************
        // Получение 1/2 амплитуды колебаний виброподвеса в виде числа импульсов от альтеры
        //**********************************************************************

        //ждём высокого уровня сигнала готовности AMPL_FOR_T_READY (p3.2)
        //while( !( GP3DAT & 0x04)) {}
        if( GP3DAT & 0x04) {
          //запрашиваем среднее за период частотной подставки
          GP3DAT |= 1 << (16 + 3);  //RD_AMPL_T_CODE (p3.3) -> 1
          //pause( 1);                //пауза

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


          //вычисляем скользящую среднюю за последние 100 (ну или сколько к этому моменту прилетело) тактов
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
        // Выдача данных согласно протоколу
        //**********************************************************************
        switch( gl_nSentPackIndex) {
          //****************************************************************************************************************************************************************
          //REGULAR PACK
          //****************************************************************************************************************************************************************
          //case UTD1:            send_pack( ( short) gl_snMeaningCounterRound);           gl_nSentPackIndex = UTD2;           break; //UTD1
          case UTD1:            send_pack( gl_ssh_Utd1);           gl_nSentPackIndex = UTD2;           break; //UTD1

          //case UTD2:            send_pack( ( short) gl_lnMeanImps);           gl_nSentPackIndex = UTD3;           break; //UTD2
          case UTD2:            send_pack( gl_ssh_Utd2);           gl_nSentPackIndex = UTD3;           break; //UTD2

          //case UTD3:            send_pack( gl_un_RULAControl);           gl_nSentPackIndex = I1;             break; //UTD3
          case UTD3:            send_pack( gl_ssh_Utd3);           gl_nSentPackIndex = I1;             break; //UTD3


          case I1:              send_pack( gl_ssh_current_1);      gl_nSentPackIndex = I2;             break; //I1
          case I2:              send_pack( gl_ssh_current_2);      gl_nSentPackIndex = CNTRPC;         break; //I2
          case CNTRPC:          send_pack( gl_ssh_Perim_Voltage);  gl_nSentPackIndex = AMPLANG_ALTERA; break; //CntrPc
          case AMPLANG_ALTERA:  send_pack( gl_ush_MeanImpulses);   gl_nSentPackIndex = UTD1;           break; //AmplAng от альтеры
          //case AMPLANG_DUS:    send_pack( gl_ssh_ampl_angle);     gl_nSentPackIndex = UTD1;           break; //AmplAng с ДУСа
          //case RULA:          send_pack( gl_un_RULAControl);     gl_nSentPackIndex = UTD3;           break; //RULA

          //****************************************************************************************************************************************************************
          // PARAMETERS BY REQUEST
          //****************************************************************************************************************************************************************

          case AMPLITUDE:       send_pack( flashParamAmplitudeCode);  gl_nSentPackIndex = UTD1;        break; //Уставка амплитуды колебания
          case TACT_CODE:       send_pack( flashParamTactCode);    gl_nSentPackIndex = UTD1;           break; //Уставка кода такта подставки
          case M_COEFF:         send_pack( flashParamMCoeff);      gl_nSentPackIndex = UTD1;           break; //Уставка коэффициента ошумления
          case STARTMODE:       send_pack( flashParamStartMode);   gl_nSentPackIndex = UTD1;           break; //Уставка начальной моды
          case DECCOEFF:        send_pack( flashParamDecCoeff);    gl_nSentPackIndex = UTD1;           break; //Коэффициент вычета
          case CONTROL_I1:      send_pack( flashParamI1min);       gl_nSentPackIndex = CONTROL_I2;     break; //flashParamI1min
          case CONTROL_I2:      send_pack( flashParamI2min);       gl_nSentPackIndex = CONTROL_AA;     break; //flashParamI2min
          case CONTROL_AA:      send_pack( flashParamAmplAngMin1); gl_nSentPackIndex = UTD1;           break; //flashParamAmplAngMin1

          /*
          case HV_APPLY_COUNT_SET: send_pack( flashParamHvApplyCount); gl_nSentPackIndex = UTD1;        break; //HV apply cycles in pack
          case HV_APPLY_COUNT_TR:  send_pack( nFiringTry);             gl_nSentPackIndex = UTD1;        break; //HV apply cycles applied in this run
          case HV_APPLY_DURAT_SET: send_pack( flashParamHvApplyDurat); gl_nSentPackIndex = UTD1;        break; //HV apply cycle duration
          case HV_APPLY_PACKS:     send_pack( flashParamHvApplyPacks); gl_nSentPackIndex = UTD1;        break; //HV apply packs
          */

          case SIGNCOEFF:       send_pack( flashParamSignCoeff);        gl_nSentPackIndex = UTD1;       break; //Уставка знакового коэффициента
          case DEVNUM:          send_pack( flashParamDeviceId);         gl_nSentPackIndex = UTD1;       break; //Device_Num

          case DATE_Y:          send_pack( flashParamDateYear);    gl_nSentPackIndex = UTD1;          break; //Date.Year
          case DATE_M:          send_pack( flashParamDateMonth);   gl_nSentPackIndex = UTD1;          break; //Date.Month
          case DATE_D:          send_pack( flashParamDateDay);     gl_nSentPackIndex = UTD1;          break; //Date.Day

          case ORG_B1:          send_pack( flashParamOrg[ 0]);     gl_nSentPackIndex = ORG_B2;        break; //Organization.Byte1
          case ORG_B2:          send_pack( flashParamOrg[ 1]);     gl_nSentPackIndex = ORG_B3;        break; //Organization.Byte2
          case ORG_B3:          send_pack( flashParamOrg[ 2]);     gl_nSentPackIndex = ORG_B4;        break; //Organization.Byte3
          case ORG_B4:          send_pack( flashParamOrg[ 3]);     gl_nSentPackIndex = ORG_B5;        break; //Organization.Byte4
          case ORG_B5:          send_pack( flashParamOrg[ 4]);     gl_nSentPackIndex = ORG_B6;        break; //Organization.Byte5
          case ORG_B6:          send_pack( flashParamOrg[ 5]);     gl_nSentPackIndex = ORG_B7;        break; //Organization.Byte6
          case ORG_B7:          send_pack( flashParamOrg[ 6]);     gl_nSentPackIndex = ORG_B8;        break; //Organization.Byte7
          case ORG_B8:          send_pack( flashParamOrg[ 7]);     gl_nSentPackIndex = ORG_B9;        break; //Organization.Byte8
          case ORG_B9:          send_pack( flashParamOrg[ 8]);     gl_nSentPackIndex = ORG_B10;       break; //Organization.Byte9
          case ORG_B10:         send_pack( flashParamOrg[ 9]);     gl_nSentPackIndex = ORG_B11;       break; //Organization.Byte10
          case ORG_B11:         send_pack( flashParamOrg[10]);     gl_nSentPackIndex = ORG_B12;       break; //Organization.Byte11
          case ORG_B12:         send_pack( flashParamOrg[11]);     gl_nSentPackIndex = ORG_B13;       break; //Organization.Byte12
          case ORG_B13:         send_pack( flashParamOrg[12]);     gl_nSentPackIndex = ORG_B14;       break; //Organization.Byte13
          case ORG_B14:         send_pack( flashParamOrg[13]);     gl_nSentPackIndex = ORG_B15;       break; //Organization.Byte14
          case ORG_B15:         send_pack( flashParamOrg[14]);     gl_nSentPackIndex = ORG_B16;       break; //Organization.Byte15
          case ORG_B16:         send_pack( flashParamOrg[15]);     gl_nSentPackIndex = UTD1;          break; //Organization.Byte16    БЕЗ завершающего 0 на конце!!!!!

          case VERSION:         send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE)); gl_nSentPackIndex = UTD1; break; //SOFTWARE VERSION

          case CALIB_T1:        send_pack( flashParam_calibT1);    gl_nSentPackIndex = T1_TD1;        break; //min thermo-calib point T
          case T1_TD1:          send_pack( flashParamT1_TD1_val);  gl_nSentPackIndex = T1_TD2;        break; //min thermo-calib point thermo1 data
          case T1_TD2:          send_pack( flashParamT1_TD2_val);  gl_nSentPackIndex = T1_TD3;        break; //min thermo-calib point thermo2 data
          case T1_TD3:          send_pack( flashParamT1_TD3_val);  gl_nSentPackIndex = CALIB_T2;      break; //min thermo-calib point thermo3 data
          case CALIB_T2:        send_pack( flashParam_calibT2);    gl_nSentPackIndex = T2_TD1;        break; //max thermo-calib point T
          case T2_TD1:          send_pack( flashParamT2_TD1_val);  gl_nSentPackIndex = T2_TD2;        break; //max thermo-calib point thermo1 data
          case T2_TD2:          send_pack( flashParamT2_TD2_val);  gl_nSentPackIndex = T2_TD3;        break; //max thermo-calib point thermo2 data
          case T2_TD3:          send_pack( flashParamT2_TD3_val);  gl_nSentPackIndex = UTD1;          break; //max thermo-calib point thermo3 data

        }

        //РАБОЧЕЕ ПЕРЕВЫЧИСЛЕНИЕ КОЭФФИЦИЕНТА ВЫЧЕТА
        if( gl_b_SyncMode) {

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
                  gl_nSentPackIndex = DECCOEFF;
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
        //автоматическая перестройка периметра
        // ************************************************************************************
        //Если канал =6, то есть мы только что перевелись с измерения напряжения пьезокорректоров
        if( ADCChannel == 6) {
          V_piezo = (( gl_ssh_Perim_Voltage / 4095. * 2.5) - 1.23) * 101.;
          if( fabs( V_piezo) > 100.) {
            //flashParamStartMode = 125;
            //DACConfiguration();
            GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1
            gl_nRppTimerT1 = T1VAL;
            gl_n_PerimeterReset = 1;
          }
        }

        testPike();

        //**********************************************************************
        //Стабилизация амплитуды колебаний виброподвеса
        //**********************************************************************
        if( gl_nAmplStabStep < 10) {
          if( T2VAL <= gl_nSmoothMCoeffApplyT2) {
            gl_nAmplStabStep++;
            gl_nSmoothMCoeffApplyT2 = ( T2VAL +T2LD - 6553) % T2LD;

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

          //если расхождение скользящей средней и заданной амплитуд большое - подкрутим RULA
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

        //применяем RULA-RULM (там же RULM применяется плавно-медленно)
        DACConfiguration();


        //**********************************************************************
        //обработка флага сброса RP_P
        //**********************************************************************
        if( gl_nRppTimerT1 != 0) {

          if( gl_n_PerimeterReset == 1) {

            //32768 = 1 sec
            //3276  = 0.1 sec    = 100 msec
            //327   = 0.01 sec   = 10 msec
            //32    = 0.001 sec  = 1 msec
            if( (( T1LD + gl_nRppTimerT1 - T1VAL) % T1LD) > 32) {

              //сброс (включение) интеграторов в системе регулировки периметра
              GP0DAT &= ~( 1 << (16 + 5));  //RP_P   (p0.5) = 0
              gl_nRppTimerT1 = T1VAL;
              gl_n_PerimeterReset = 2;
            }
          }
          else if( gl_n_PerimeterReset == 2) {

            //32768 = 1 sec
            //3276  = 0.1 sec = 100 msec
            //327   = 0.01 sec = 10 msec
            if( (( T1LD + gl_nRppTimerT1 - T1VAL) % T1LD) > 3276) {
              gl_n_PerimeterReset = 0;
              gl_nRppTimerT1 = 0;
            }
          }

        }

        //поднимаем флаг о том что текущий высокий уровень SA мы обработали
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
      //если линия сигнала SA в низком уровне - то как только она поднимется начнется новый необработанный такт
      gl_b_SA_Processed = 0;

      //проверка тактирования
      ush_SA_check_time = ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;

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
