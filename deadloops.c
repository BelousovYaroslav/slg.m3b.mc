#include <ADuC7026.h>
#include "version.h"
#include "errors.h"
#include "Main.h"
#include "AnalogueParamsConstList.h"
#include "processingIncomingCommand.h"

extern unsigned short flashParamAmplitudeCode;
extern unsigned short flashParamTactCode;
extern unsigned short flashParamMCoeff;
extern unsigned short flashParamStartMode;
extern unsigned short flashParamI1min;
extern unsigned short flashParamI2min;
extern unsigned short flashParamAmplAngMin1;
extern unsigned short flashParamDecCoeff;
extern unsigned short flashParamSignCoeff;
extern unsigned short flashParamDeviceId;
extern unsigned short flashParamDateYear;
extern unsigned short flashParamDateMonth;
extern unsigned short flashParamDateDay;
extern char flashParamOrg[];

//declared in Main.c
extern char gl_c_EmergencyCode;                     //код ошибки
extern int gl_n_prT1VAL;
extern signed short gl_ssh_SA_time;
extern signed short gl_ssh_ampl_angle;
extern signed short gl_ssh_angle_inc;
extern signed short gl_ssh_angle_inc_prev;
extern short gl_nSentAddParamIndex;

extern void send_pack( short shAnalogParamValue);

extern void pause_T0 ( int n);
extern void pause( int n);

extern void save_params( void);
extern void load_params( void);

void deadloop_no_tact( int nError) {
  //ОБРАБОТКА ОТСУТСТВИЯ ТАКТИРОВАНИЯ
#ifdef DEBUG
  printf("DEBUG: NO TACT SIGNAL! DEADLOOP.\n");
#endif
  //выставляем код ошибки
  gl_c_EmergencyCode = nError;

  //показания приращения угла = 0
  gl_ssh_angle_inc = gl_ssh_angle_inc_prev = 0;

  gl_n_prT1VAL = T1VAL;
  while( 1) {
    //пауза 0,1 секунда
    pause( 327);

    gl_ssh_SA_time = ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
    gl_n_prT1VAL = T1VAL;

    //**********************************************************************
    // Обработка буфера входящих команд
    //**********************************************************************
    processIncomingCommand();

    //выдача настроечных параметров
    gl_nSentAddParamIndex = AMPLITUDE;    send_pack( flashParamAmplitudeCode);
    gl_nSentAddParamIndex = TACT_CODE;    send_pack( flashParamTactCode);
    gl_nSentAddParamIndex = M_COEFF;      send_pack( flashParamMCoeff);
    gl_nSentAddParamIndex = STARTMODE;    send_pack( flashParamStartMode);
    gl_nSentAddParamIndex = DECCOEFF;     send_pack( flashParamDecCoeff);
    gl_nSentAddParamIndex = CONTROL_I1;   send_pack( flashParamI1min);
    gl_nSentAddParamIndex = CONTROL_I2;   send_pack( flashParamI2min);
    gl_nSentAddParamIndex = CONTROL_AA;   send_pack( flashParamAmplAngMin1);
    gl_nSentAddParamIndex = SIGNCOEFF;    send_pack( flashParamSignCoeff);
    gl_nSentAddParamIndex = DEVNUM;       send_pack( flashParamDeviceId);
    gl_nSentAddParamIndex = DATE_Y;       send_pack( flashParamDateYear);
    gl_nSentAddParamIndex = DATE_M;       send_pack( flashParamDateMonth);
    gl_nSentAddParamIndex = DATE_D;       send_pack( flashParamDateDay);
    gl_nSentAddParamIndex = ORG_B1;       send_pack( flashParamOrg[ 0]);
    gl_nSentAddParamIndex = ORG_B2;       send_pack( flashParamOrg[ 1]);
    gl_nSentAddParamIndex = ORG_B3;       send_pack( flashParamOrg[ 2]);
    gl_nSentAddParamIndex = ORG_B4;       send_pack( flashParamOrg[ 3]);
    gl_nSentAddParamIndex = ORG_B5;       send_pack( flashParamOrg[ 4]);
    gl_nSentAddParamIndex = ORG_B6;       send_pack( flashParamOrg[ 5]);
    gl_nSentAddParamIndex = ORG_B7;       send_pack( flashParamOrg[ 6]);
    gl_nSentAddParamIndex = ORG_B8;       send_pack( flashParamOrg[ 7]);
    gl_nSentAddParamIndex = ORG_B9;       send_pack( flashParamOrg[ 8]);
    gl_nSentAddParamIndex = ORG_B10;      send_pack( flashParamOrg[ 9]);
    gl_nSentAddParamIndex = ORG_B11;      send_pack( flashParamOrg[ 10]);
    gl_nSentAddParamIndex = ORG_B12;      send_pack( flashParamOrg[ 11]);
    gl_nSentAddParamIndex = ORG_B13;      send_pack( flashParamOrg[ 12]);
    gl_nSentAddParamIndex = ORG_B14;      send_pack( flashParamOrg[ 13]);
    gl_nSentAddParamIndex = ORG_B15;      send_pack( flashParamOrg[ 14]);
    gl_nSentAddParamIndex = ORG_B16;      send_pack( flashParamOrg[ 15]);
    gl_nSentAddParamIndex = VERSION;      send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  }  //"мертвый" while отсутствия тактирования
}

void deadloop_no_hangerup( void) {
  //ОБРАБОТКА ОТКАЗА РАСКАЧКИ ВИБРОПОДВЕСА
  double dStartAmplAngCheck = ( double) flashParamAmplAngMin1 / 65535. * 3.;

#ifdef DEBUG
  printf("DBG: NO HANGER VIBRATION! DEADLOOP.\n");
#endif

  //показания приращения угла = 0
  gl_ssh_angle_inc = gl_ssh_angle_inc_prev = 0;

  //выставляем код ошибки
  gl_c_EmergencyCode = ERROR_INITIAL_AMPL_ANG_TEST_FAIL;

  ADCCP = 0x08;     //мы будем посылать ТОЛЬКО AmplAng
  pause(10);
  ADCCON |= 0x80;   //запуск преобразования

  gl_n_prT1VAL = T1VAL;
  while( 1) {
    //пауза 0,1 секунда
    pause( 327);

    gl_ssh_SA_time = ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
    gl_n_prT1VAL = T1VAL;

    //измерение AmplAng (и ТОЛЬКО ЕГО)
    while (!( ADCSTA & 0x01)){}     // ожидаем конца преобразования АЦП (теоретически когда мы приходим сюда он уже должен быть готов)
    gl_ssh_ampl_angle = (ADCDAT >> 16);
    ADCCON |= 0x80;                 //запуск преобразования

#ifdef DEBUG
    printf( "DBG: AA: %d = %.2fV    Control value: %.2f\n",
            gl_ssh_ampl_angle,
            3. * ( double) gl_ssh_ampl_angle / 4095.,
            dStartAmplAngCheck);
#endif
    //**********************************************************************
    // Обработка буфера входящих команд
    //**********************************************************************
    processIncomingCommand();

    //выдача данных
    gl_nSentAddParamIndex = AMPLANG_DUS;  send_pack( gl_ssh_ampl_angle);
    gl_nSentAddParamIndex = AMPLITUDE;    send_pack( flashParamAmplitudeCode);
    gl_nSentAddParamIndex = TACT_CODE;    send_pack( flashParamTactCode);
    gl_nSentAddParamIndex = M_COEFF;      send_pack( flashParamMCoeff);
    gl_nSentAddParamIndex = STARTMODE;    send_pack( flashParamStartMode);
    gl_nSentAddParamIndex = DECCOEFF;     send_pack( flashParamDecCoeff);
    gl_nSentAddParamIndex = CONTROL_I1;   send_pack( flashParamI1min);
    gl_nSentAddParamIndex = CONTROL_I2;   send_pack( flashParamI2min);
    gl_nSentAddParamIndex = CONTROL_AA;   send_pack( flashParamAmplAngMin1);
    gl_nSentAddParamIndex = SIGNCOEFF;    send_pack( flashParamSignCoeff);
    gl_nSentAddParamIndex = DEVNUM;       send_pack( flashParamDeviceId);
    gl_nSentAddParamIndex = DATE_Y;       send_pack( flashParamDateYear);
    gl_nSentAddParamIndex = DATE_M;       send_pack( flashParamDateMonth);
    gl_nSentAddParamIndex = DATE_D;       send_pack( flashParamDateDay);
    gl_nSentAddParamIndex = ORG_B1;       send_pack( flashParamOrg[ 0]);
    gl_nSentAddParamIndex = ORG_B2;       send_pack( flashParamOrg[ 1]);
    gl_nSentAddParamIndex = ORG_B3;       send_pack( flashParamOrg[ 2]);
    gl_nSentAddParamIndex = ORG_B4;       send_pack( flashParamOrg[ 3]);
    gl_nSentAddParamIndex = ORG_B5;       send_pack( flashParamOrg[ 4]);
    gl_nSentAddParamIndex = ORG_B6;       send_pack( flashParamOrg[ 5]);
    gl_nSentAddParamIndex = ORG_B7;       send_pack( flashParamOrg[ 6]);
    gl_nSentAddParamIndex = ORG_B8;       send_pack( flashParamOrg[ 7]);
    gl_nSentAddParamIndex = ORG_B9;       send_pack( flashParamOrg[ 8]);
    gl_nSentAddParamIndex = ORG_B10;      send_pack( flashParamOrg[ 9]);
    gl_nSentAddParamIndex = ORG_B11;      send_pack( flashParamOrg[ 10]);
    gl_nSentAddParamIndex = ORG_B12;      send_pack( flashParamOrg[ 11]);
    gl_nSentAddParamIndex = ORG_B13;      send_pack( flashParamOrg[ 12]);
    gl_nSentAddParamIndex = ORG_B14;      send_pack( flashParamOrg[ 13]);
    gl_nSentAddParamIndex = ORG_B15;      send_pack( flashParamOrg[ 14]);
    gl_nSentAddParamIndex = ORG_B16;      send_pack( flashParamOrg[ 15]);
    gl_nSentAddParamIndex = VERSION;      send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  } //"мертвый" захват отказа раскачки виброподвеса
}

void deadloop_no_firing( int nError) {
  //ОБРАБОТКА ОТКАЗА ПОДЖИГА
#ifdef DEBUG
  printf("DBG: NO LASER FIREUP! DEADLOOP.\n");
#endif

  //выставляем код ошибки
  gl_c_EmergencyCode = nError;

  //показания приращения угла = 0
  gl_ssh_angle_inc = gl_ssh_angle_inc_prev = 0;

  //выдача данных
    gl_nSentAddParamIndex = AMPLITUDE;  send_pack( flashParamAmplitudeCode);
    gl_nSentAddParamIndex = TACT_CODE;  send_pack( flashParamTactCode);
    gl_nSentAddParamIndex = M_COEFF;    send_pack( flashParamMCoeff);
    gl_nSentAddParamIndex = STARTMODE;  send_pack( flashParamStartMode);
    gl_nSentAddParamIndex = CONTROL_I1; send_pack( flashParamI1min);
    gl_nSentAddParamIndex = CONTROL_I2; send_pack( flashParamI2min);
    gl_nSentAddParamIndex = CONTROL_AA; send_pack( flashParamAmplAngMin1);
    gl_nSentAddParamIndex = DECCOEFF;   send_pack( flashParamDecCoeff);
    gl_nSentAddParamIndex = SIGNCOEFF;  send_pack( flashParamSignCoeff);
    gl_nSentAddParamIndex = VERSION;    send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  gl_n_prT1VAL = T1VAL;
  while( 1) {
    //пауза 0,1 секунда
    pause( 327);

    gl_ssh_SA_time = ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
    gl_n_prT1VAL = T1VAL;

    //**********************************************************************
    // Обработка буфера входящих команд
    //**********************************************************************
    processIncomingCommand();

    //выдача настроечных параметров
    gl_nSentAddParamIndex = AMPLITUDE;    send_pack( flashParamAmplitudeCode);
    gl_nSentAddParamIndex = TACT_CODE;    send_pack( flashParamTactCode);
    gl_nSentAddParamIndex = M_COEFF;      send_pack( flashParamMCoeff);
    gl_nSentAddParamIndex = STARTMODE;    send_pack( flashParamStartMode);
    gl_nSentAddParamIndex = DECCOEFF;     send_pack( flashParamDecCoeff);
    gl_nSentAddParamIndex = CONTROL_I1;   send_pack( flashParamI1min);
    gl_nSentAddParamIndex = CONTROL_I2;   send_pack( flashParamI2min);
    gl_nSentAddParamIndex = CONTROL_AA;   send_pack( flashParamAmplAngMin1);
    
    gl_nSentAddParamIndex = SIGNCOEFF;    send_pack( flashParamSignCoeff);
    gl_nSentAddParamIndex = DEVNUM;       send_pack( flashParamDeviceId);
    gl_nSentAddParamIndex = DATE_Y;       send_pack( flashParamDateYear);
    gl_nSentAddParamIndex = DATE_M;       send_pack( flashParamDateMonth);
    gl_nSentAddParamIndex = DATE_D;       send_pack( flashParamDateDay);
    gl_nSentAddParamIndex = ORG_B1;       send_pack( flashParamOrg[ 0]);
    gl_nSentAddParamIndex = ORG_B2;       send_pack( flashParamOrg[ 1]);
    gl_nSentAddParamIndex = ORG_B3;       send_pack( flashParamOrg[ 2]);
    gl_nSentAddParamIndex = ORG_B4;       send_pack( flashParamOrg[ 3]);
    gl_nSentAddParamIndex = ORG_B5;       send_pack( flashParamOrg[ 4]);
    gl_nSentAddParamIndex = ORG_B6;       send_pack( flashParamOrg[ 5]);
    gl_nSentAddParamIndex = ORG_B7;       send_pack( flashParamOrg[ 6]);
    gl_nSentAddParamIndex = ORG_B8;       send_pack( flashParamOrg[ 7]);
    gl_nSentAddParamIndex = ORG_B9;       send_pack( flashParamOrg[ 8]);
    gl_nSentAddParamIndex = ORG_B10;      send_pack( flashParamOrg[ 9]);
    gl_nSentAddParamIndex = ORG_B11;      send_pack( flashParamOrg[ 10]);
    gl_nSentAddParamIndex = ORG_B12;      send_pack( flashParamOrg[ 11]);
    gl_nSentAddParamIndex = ORG_B13;      send_pack( flashParamOrg[ 12]);
    gl_nSentAddParamIndex = ORG_B14;      send_pack( flashParamOrg[ 13]);
    gl_nSentAddParamIndex = ORG_B15;      send_pack( flashParamOrg[ 14]);
    gl_nSentAddParamIndex = ORG_B16;      send_pack( flashParamOrg[ 15]);
    gl_nSentAddParamIndex = VERSION;      send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  } //"мертвый" while
}