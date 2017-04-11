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
extern char gl_c_EmergencyCode;                     //��� ������
extern int gl_n_prT1VAL;
extern signed short gl_ssh_SA_time;
extern signed short gl_ssh_ampl_angle;
extern signed short gl_ssh_angle_inc;
extern signed short gl_ssh_angle_inc_prev;
extern short gl_nSentPackIndex;

extern void send_pack( short shAnalogParamValue);

extern void pause_T0 ( int n);
extern void pause( int n);

extern void save_params( void);
extern void load_params( void);

void deadloop_no_tact( int nError) {
  //��������� ���������� ������������
#ifdef DEBUG
  printf("DEBUG: NO TACT SIGNAL! DEADLOOP.\n");
#endif
  //���������� ��� ������
  gl_c_EmergencyCode = nError;

  //��������� ���������� ���� = 0
  gl_ssh_angle_inc = gl_ssh_angle_inc_prev = 0;

  gl_n_prT1VAL = T1VAL;
  while( 1) {
    //����� 0,1 �������
    pause( 327);

    gl_ssh_SA_time = ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
    gl_n_prT1VAL = T1VAL;

    //**********************************************************************
    // ��������� ������ �������� ������
    //**********************************************************************
    processIncomingCommand();

    //������ ����������� ����������
    gl_nSentPackIndex = AMPLITUDE;    send_pack( flashParamAmplitudeCode);
    gl_nSentPackIndex = TACT_CODE;    send_pack( flashParamTactCode);
    gl_nSentPackIndex = M_COEFF;      send_pack( flashParamMCoeff);
    gl_nSentPackIndex = STARTMODE;    send_pack( flashParamStartMode);
    gl_nSentPackIndex = DECCOEFF;     send_pack( flashParamDecCoeff);
    gl_nSentPackIndex = CONTROL_I1;   send_pack( flashParamI1min);
    gl_nSentPackIndex = CONTROL_I2;   send_pack( flashParamI2min);
    gl_nSentPackIndex = CONTROL_AA;   send_pack( flashParamAmplAngMin1);
    gl_nSentPackIndex = SIGNCOEFF;    send_pack( flashParamSignCoeff);
    gl_nSentPackIndex = DEVNUM;       send_pack( flashParamDeviceId);
    gl_nSentPackIndex = DATE_Y;       send_pack( flashParamDateYear);
    gl_nSentPackIndex = DATE_M;       send_pack( flashParamDateMonth);
    gl_nSentPackIndex = DATE_D;       send_pack( flashParamDateDay);
    gl_nSentPackIndex = ORG_B1;       send_pack( flashParamOrg[ 0]);
    gl_nSentPackIndex = ORG_B2;       send_pack( flashParamOrg[ 1]);
    gl_nSentPackIndex = ORG_B3;       send_pack( flashParamOrg[ 2]);
    gl_nSentPackIndex = ORG_B4;       send_pack( flashParamOrg[ 3]);
    gl_nSentPackIndex = ORG_B5;       send_pack( flashParamOrg[ 4]);
    gl_nSentPackIndex = ORG_B6;       send_pack( flashParamOrg[ 5]);
    gl_nSentPackIndex = ORG_B7;       send_pack( flashParamOrg[ 6]);
    gl_nSentPackIndex = ORG_B8;       send_pack( flashParamOrg[ 7]);
    gl_nSentPackIndex = ORG_B9;       send_pack( flashParamOrg[ 8]);
    gl_nSentPackIndex = ORG_B10;      send_pack( flashParamOrg[ 9]);
    gl_nSentPackIndex = ORG_B11;      send_pack( flashParamOrg[ 10]);
    gl_nSentPackIndex = ORG_B12;      send_pack( flashParamOrg[ 11]);
    gl_nSentPackIndex = ORG_B13;      send_pack( flashParamOrg[ 12]);
    gl_nSentPackIndex = ORG_B14;      send_pack( flashParamOrg[ 13]);
    gl_nSentPackIndex = ORG_B15;      send_pack( flashParamOrg[ 14]);
    gl_nSentPackIndex = ORG_B16;      send_pack( flashParamOrg[ 15]);
    gl_nSentPackIndex = VERSION;      send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  }  //"�������" while ���������� ������������
}

void deadloop_no_hangerup( void) {
  //��������� ������ �������� ������������
  double dStartAmplAngCheck = ( double) flashParamAmplAngMin1 / 65535. * 3.;

#ifdef DEBUG
  printf("DBG: NO HANGER VIBRATION! DEADLOOP.\n");
#endif

  //��������� ���������� ���� = 0
  gl_ssh_angle_inc = gl_ssh_angle_inc_prev = 0;

  //���������� ��� ������
  gl_c_EmergencyCode = ERROR_INITIAL_AMPL_ANG_TEST_FAIL;

  ADCCP = 0x08;     //�� ����� �������� ������ AmplAng
  pause(10);
  ADCCON |= 0x80;   //������ ��������������

  gl_n_prT1VAL = T1VAL;
  while( 1) {
    //����� 0,1 �������
    pause( 327);

    gl_ssh_SA_time = ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
    gl_n_prT1VAL = T1VAL;

    //��������� AmplAng (� ������ ���)
    while (!( ADCSTA & 0x01)){}     // ������� ����� �������������� ��� (������������ ����� �� �������� ���� �� ��� ������ ���� �����)
    gl_ssh_ampl_angle = (ADCDAT >> 16);
    ADCCON |= 0x80;                 //������ ��������������

#ifdef DEBUG
    printf( "DBG: AA: %d = %.2fV    Control value: %.2f\n",
            gl_ssh_ampl_angle,
            3. * ( double) gl_ssh_ampl_angle / 4095.,
            dStartAmplAngCheck);
#endif
    //**********************************************************************
    // ��������� ������ �������� ������
    //**********************************************************************
    processIncomingCommand();

    //������ ������
    gl_nSentPackIndex = AMPLANG_DUS;  send_pack( gl_ssh_ampl_angle);
    gl_nSentPackIndex = AMPLITUDE;    send_pack( flashParamAmplitudeCode);
    gl_nSentPackIndex = TACT_CODE;    send_pack( flashParamTactCode);
    gl_nSentPackIndex = M_COEFF;      send_pack( flashParamMCoeff);
    gl_nSentPackIndex = STARTMODE;    send_pack( flashParamStartMode);
    gl_nSentPackIndex = DECCOEFF;     send_pack( flashParamDecCoeff);
    gl_nSentPackIndex = CONTROL_I1;   send_pack( flashParamI1min);
    gl_nSentPackIndex = CONTROL_I2;   send_pack( flashParamI2min);
    gl_nSentPackIndex = CONTROL_AA;   send_pack( flashParamAmplAngMin1);
    gl_nSentPackIndex = SIGNCOEFF;    send_pack( flashParamSignCoeff);
    gl_nSentPackIndex = DEVNUM;       send_pack( flashParamDeviceId);
    gl_nSentPackIndex = DATE_Y;       send_pack( flashParamDateYear);
    gl_nSentPackIndex = DATE_M;       send_pack( flashParamDateMonth);
    gl_nSentPackIndex = DATE_D;       send_pack( flashParamDateDay);
    gl_nSentPackIndex = ORG_B1;       send_pack( flashParamOrg[ 0]);
    gl_nSentPackIndex = ORG_B2;       send_pack( flashParamOrg[ 1]);
    gl_nSentPackIndex = ORG_B3;       send_pack( flashParamOrg[ 2]);
    gl_nSentPackIndex = ORG_B4;       send_pack( flashParamOrg[ 3]);
    gl_nSentPackIndex = ORG_B5;       send_pack( flashParamOrg[ 4]);
    gl_nSentPackIndex = ORG_B6;       send_pack( flashParamOrg[ 5]);
    gl_nSentPackIndex = ORG_B7;       send_pack( flashParamOrg[ 6]);
    gl_nSentPackIndex = ORG_B8;       send_pack( flashParamOrg[ 7]);
    gl_nSentPackIndex = ORG_B9;       send_pack( flashParamOrg[ 8]);
    gl_nSentPackIndex = ORG_B10;      send_pack( flashParamOrg[ 9]);
    gl_nSentPackIndex = ORG_B11;      send_pack( flashParamOrg[ 10]);
    gl_nSentPackIndex = ORG_B12;      send_pack( flashParamOrg[ 11]);
    gl_nSentPackIndex = ORG_B13;      send_pack( flashParamOrg[ 12]);
    gl_nSentPackIndex = ORG_B14;      send_pack( flashParamOrg[ 13]);
    gl_nSentPackIndex = ORG_B15;      send_pack( flashParamOrg[ 14]);
    gl_nSentPackIndex = ORG_B16;      send_pack( flashParamOrg[ 15]);
    gl_nSentPackIndex = VERSION;      send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  } //"�������" ������ ������ �������� ������������
}

void deadloop_no_firing( int nError) {
  //��������� ������ �������
#ifdef DEBUG
  printf("DBG: NO LASER FIREUP! DEADLOOP.\n");
#endif

  //���������� ��� ������
  gl_c_EmergencyCode = nError;

  //��������� ���������� ���� = 0
  gl_ssh_angle_inc = gl_ssh_angle_inc_prev = 0;

  //������ ������
    gl_nSentPackIndex = AMPLITUDE;  send_pack( flashParamAmplitudeCode);
    gl_nSentPackIndex = TACT_CODE;  send_pack( flashParamTactCode);
    gl_nSentPackIndex = M_COEFF;    send_pack( flashParamMCoeff);
    gl_nSentPackIndex = STARTMODE;  send_pack( flashParamStartMode);
    gl_nSentPackIndex = CONTROL_I1; send_pack( flashParamI1min);
    gl_nSentPackIndex = CONTROL_I2; send_pack( flashParamI2min);
    gl_nSentPackIndex = CONTROL_AA; send_pack( flashParamAmplAngMin1);
    gl_nSentPackIndex = DECCOEFF;   send_pack( flashParamDecCoeff);
    gl_nSentPackIndex = SIGNCOEFF;  send_pack( flashParamSignCoeff);
    gl_nSentPackIndex = VERSION;    send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  gl_n_prT1VAL = T1VAL;
  while( 1) {
    //����� 0,1 �������
    pause( 327);

    gl_ssh_SA_time = ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
    gl_n_prT1VAL = T1VAL;

    //**********************************************************************
    // ��������� ������ �������� ������
    //**********************************************************************
    processIncomingCommand();

    //������ ����������� ����������
    gl_nSentPackIndex = AMPLITUDE;    send_pack( flashParamAmplitudeCode);
    gl_nSentPackIndex = TACT_CODE;    send_pack( flashParamTactCode);
    gl_nSentPackIndex = M_COEFF;      send_pack( flashParamMCoeff);
    gl_nSentPackIndex = STARTMODE;    send_pack( flashParamStartMode);
    gl_nSentPackIndex = DECCOEFF;     send_pack( flashParamDecCoeff);
    gl_nSentPackIndex = CONTROL_I1;   send_pack( flashParamI1min);
    gl_nSentPackIndex = CONTROL_I2;   send_pack( flashParamI2min);
    gl_nSentPackIndex = CONTROL_AA;   send_pack( flashParamAmplAngMin1);
    
    gl_nSentPackIndex = SIGNCOEFF;    send_pack( flashParamSignCoeff);
    gl_nSentPackIndex = DEVNUM;       send_pack( flashParamDeviceId);
    gl_nSentPackIndex = DATE_Y;       send_pack( flashParamDateYear);
    gl_nSentPackIndex = DATE_M;       send_pack( flashParamDateMonth);
    gl_nSentPackIndex = DATE_D;       send_pack( flashParamDateDay);
    gl_nSentPackIndex = ORG_B1;       send_pack( flashParamOrg[ 0]);
    gl_nSentPackIndex = ORG_B2;       send_pack( flashParamOrg[ 1]);
    gl_nSentPackIndex = ORG_B3;       send_pack( flashParamOrg[ 2]);
    gl_nSentPackIndex = ORG_B4;       send_pack( flashParamOrg[ 3]);
    gl_nSentPackIndex = ORG_B5;       send_pack( flashParamOrg[ 4]);
    gl_nSentPackIndex = ORG_B6;       send_pack( flashParamOrg[ 5]);
    gl_nSentPackIndex = ORG_B7;       send_pack( flashParamOrg[ 6]);
    gl_nSentPackIndex = ORG_B8;       send_pack( flashParamOrg[ 7]);
    gl_nSentPackIndex = ORG_B9;       send_pack( flashParamOrg[ 8]);
    gl_nSentPackIndex = ORG_B10;      send_pack( flashParamOrg[ 9]);
    gl_nSentPackIndex = ORG_B11;      send_pack( flashParamOrg[ 10]);
    gl_nSentPackIndex = ORG_B12;      send_pack( flashParamOrg[ 11]);
    gl_nSentPackIndex = ORG_B13;      send_pack( flashParamOrg[ 12]);
    gl_nSentPackIndex = ORG_B14;      send_pack( flashParamOrg[ 13]);
    gl_nSentPackIndex = ORG_B15;      send_pack( flashParamOrg[ 14]);
    gl_nSentPackIndex = ORG_B16;      send_pack( flashParamOrg[ 15]);
    gl_nSentPackIndex = VERSION;      send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  } //"�������" while
}