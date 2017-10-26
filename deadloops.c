#include <ADuC7026.h>
#include "version.h"
#include "errors.h"
#include "Main.h"
#include "AnalogueParamsConstList.h"
#include "processingIncomingCommand.h"

extern unsigned short gl_ush_flashParamAmplitudeCode;
extern unsigned short gl_ush_flashParamTactCode;
extern unsigned short gl_ush_flashParamMCoeff;
extern unsigned short gl_ush_flashParamStartMode;
extern unsigned short gl_ush_flashParamI1min;
extern unsigned short gl_ush_flashParamI2min;
extern unsigned short gl_ush_flashParamAmplAngMin1;
extern unsigned short gl_ush_flashParamDecCoeff;
extern unsigned short gl_ush_flashParamSignCoeff;
extern unsigned short gl_ush_flashParamDeviceId;
extern unsigned short gl_ush_flashParamDateYear;
extern unsigned short gl_ush_flashParamDateMonth;
extern unsigned short gl_ush_flashParamDateDay;
extern char           gl_ac_flashParamOrg[];

//declared in Main.c
extern char gl_c_EmergencyCode;                     //��� ������
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
  //��������� ���������� ������������
#ifdef DEBUG
  printf("DBG: NO TACT SIGNAL! DEADLOOP.\n");
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
    gl_nSentAddParamIndex = AMPLITUDE;    send_pack( gl_ush_flashParamAmplitudeCode);
    gl_nSentAddParamIndex = TACT_CODE;    send_pack( gl_ush_flashParamTactCode);
    gl_nSentAddParamIndex = M_COEFF;      send_pack( gl_ush_flashParamMCoeff);
    gl_nSentAddParamIndex = STARTMODE;    send_pack( gl_ush_flashParamStartMode);
    gl_nSentAddParamIndex = DECCOEFF;     send_pack( gl_ush_flashParamDecCoeff);
    gl_nSentAddParamIndex = CONTROL_I1;   send_pack( gl_ush_flashParamI1min);
    gl_nSentAddParamIndex = CONTROL_I2;   send_pack( gl_ush_flashParamI2min);
    gl_nSentAddParamIndex = CONTROL_AA;   send_pack( gl_ush_flashParamAmplAngMin1);
    gl_nSentAddParamIndex = SIGNCOEFF;    send_pack( gl_ush_flashParamSignCoeff);
    gl_nSentAddParamIndex = DEVNUM;       send_pack( gl_ush_flashParamDeviceId);
    gl_nSentAddParamIndex = DATE_Y;       send_pack( gl_ush_flashParamDateYear);
    gl_nSentAddParamIndex = DATE_M;       send_pack( gl_ush_flashParamDateMonth);
    gl_nSentAddParamIndex = DATE_D;       send_pack( gl_ush_flashParamDateDay);
    gl_nSentAddParamIndex = ORG_B1;       send_pack( gl_ac_flashParamOrg[ 0]);
    gl_nSentAddParamIndex = ORG_B2;       send_pack( gl_ac_flashParamOrg[ 1]);
    gl_nSentAddParamIndex = ORG_B3;       send_pack( gl_ac_flashParamOrg[ 2]);
    gl_nSentAddParamIndex = ORG_B4;       send_pack( gl_ac_flashParamOrg[ 3]);
    gl_nSentAddParamIndex = ORG_B5;       send_pack( gl_ac_flashParamOrg[ 4]);
    gl_nSentAddParamIndex = ORG_B6;       send_pack( gl_ac_flashParamOrg[ 5]);
    gl_nSentAddParamIndex = ORG_B7;       send_pack( gl_ac_flashParamOrg[ 6]);
    gl_nSentAddParamIndex = ORG_B8;       send_pack( gl_ac_flashParamOrg[ 7]);
    gl_nSentAddParamIndex = ORG_B9;       send_pack( gl_ac_flashParamOrg[ 8]);
    gl_nSentAddParamIndex = ORG_B10;      send_pack( gl_ac_flashParamOrg[ 9]);
    gl_nSentAddParamIndex = ORG_B11;      send_pack( gl_ac_flashParamOrg[ 10]);
    gl_nSentAddParamIndex = ORG_B12;      send_pack( gl_ac_flashParamOrg[ 11]);
    gl_nSentAddParamIndex = ORG_B13;      send_pack( gl_ac_flashParamOrg[ 12]);
    gl_nSentAddParamIndex = ORG_B14;      send_pack( gl_ac_flashParamOrg[ 13]);
    gl_nSentAddParamIndex = ORG_B15;      send_pack( gl_ac_flashParamOrg[ 14]);
    gl_nSentAddParamIndex = ORG_B16;      send_pack( gl_ac_flashParamOrg[ 15]);
    gl_nSentAddParamIndex = VERSION;      send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  }  //"�������" while ���������� ������������
}

void deadloop_no_hangerup( void) {
  //��������� ������ �������� ������������
  double dStartAmplAngCheck = ( double) gl_ush_flashParamAmplAngMin1 / 65535. * 3.;

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
    gl_nSentAddParamIndex = AMPLANG_DUS;  send_pack( gl_ssh_ampl_angle);
    gl_nSentAddParamIndex = AMPLITUDE;    send_pack( gl_ush_flashParamAmplitudeCode);
    gl_nSentAddParamIndex = TACT_CODE;    send_pack( gl_ush_flashParamTactCode);
    gl_nSentAddParamIndex = M_COEFF;      send_pack( gl_ush_flashParamMCoeff);
    gl_nSentAddParamIndex = STARTMODE;    send_pack( gl_ush_flashParamStartMode);
    gl_nSentAddParamIndex = DECCOEFF;     send_pack( gl_ush_flashParamDecCoeff);
    gl_nSentAddParamIndex = CONTROL_I1;   send_pack( gl_ush_flashParamI1min);
    gl_nSentAddParamIndex = CONTROL_I2;   send_pack( gl_ush_flashParamI2min);
    gl_nSentAddParamIndex = CONTROL_AA;   send_pack( gl_ush_flashParamAmplAngMin1);
    gl_nSentAddParamIndex = SIGNCOEFF;    send_pack( gl_ush_flashParamSignCoeff);
    gl_nSentAddParamIndex = DEVNUM;       send_pack( gl_ush_flashParamDeviceId);
    gl_nSentAddParamIndex = DATE_Y;       send_pack( gl_ush_flashParamDateYear);
    gl_nSentAddParamIndex = DATE_M;       send_pack( gl_ush_flashParamDateMonth);
    gl_nSentAddParamIndex = DATE_D;       send_pack( gl_ush_flashParamDateDay);
    gl_nSentAddParamIndex = ORG_B1;       send_pack( gl_ac_flashParamOrg[ 0]);
    gl_nSentAddParamIndex = ORG_B2;       send_pack( gl_ac_flashParamOrg[ 1]);
    gl_nSentAddParamIndex = ORG_B3;       send_pack( gl_ac_flashParamOrg[ 2]);
    gl_nSentAddParamIndex = ORG_B4;       send_pack( gl_ac_flashParamOrg[ 3]);
    gl_nSentAddParamIndex = ORG_B5;       send_pack( gl_ac_flashParamOrg[ 4]);
    gl_nSentAddParamIndex = ORG_B6;       send_pack( gl_ac_flashParamOrg[ 5]);
    gl_nSentAddParamIndex = ORG_B7;       send_pack( gl_ac_flashParamOrg[ 6]);
    gl_nSentAddParamIndex = ORG_B8;       send_pack( gl_ac_flashParamOrg[ 7]);
    gl_nSentAddParamIndex = ORG_B9;       send_pack( gl_ac_flashParamOrg[ 8]);
    gl_nSentAddParamIndex = ORG_B10;      send_pack( gl_ac_flashParamOrg[ 9]);
    gl_nSentAddParamIndex = ORG_B11;      send_pack( gl_ac_flashParamOrg[ 10]);
    gl_nSentAddParamIndex = ORG_B12;      send_pack( gl_ac_flashParamOrg[ 11]);
    gl_nSentAddParamIndex = ORG_B13;      send_pack( gl_ac_flashParamOrg[ 12]);
    gl_nSentAddParamIndex = ORG_B14;      send_pack( gl_ac_flashParamOrg[ 13]);
    gl_nSentAddParamIndex = ORG_B15;      send_pack( gl_ac_flashParamOrg[ 14]);
    gl_nSentAddParamIndex = ORG_B16;      send_pack( gl_ac_flashParamOrg[ 15]);
    gl_nSentAddParamIndex = VERSION;      send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

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
  gl_nSentAddParamIndex = AMPLITUDE;  send_pack( gl_ush_flashParamAmplitudeCode);
  gl_nSentAddParamIndex = TACT_CODE;  send_pack( gl_ush_flashParamTactCode);
  gl_nSentAddParamIndex = M_COEFF;    send_pack( gl_ush_flashParamMCoeff);
  gl_nSentAddParamIndex = STARTMODE;  send_pack( gl_ush_flashParamStartMode);
  gl_nSentAddParamIndex = CONTROL_I1; send_pack( gl_ush_flashParamI1min);
  gl_nSentAddParamIndex = CONTROL_I2; send_pack( gl_ush_flashParamI2min);
  gl_nSentAddParamIndex = CONTROL_AA; send_pack( gl_ush_flashParamAmplAngMin1);
  gl_nSentAddParamIndex = DECCOEFF;   send_pack( gl_ush_flashParamDecCoeff);
  gl_nSentAddParamIndex = SIGNCOEFF;  send_pack( gl_ush_flashParamSignCoeff);
  gl_nSentAddParamIndex = VERSION;    send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

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
    gl_nSentAddParamIndex = AMPLITUDE;    send_pack( gl_ush_flashParamAmplitudeCode);
    gl_nSentAddParamIndex = TACT_CODE;    send_pack( gl_ush_flashParamTactCode);
    gl_nSentAddParamIndex = M_COEFF;      send_pack( gl_ush_flashParamMCoeff);
    gl_nSentAddParamIndex = STARTMODE;    send_pack( gl_ush_flashParamStartMode);
    gl_nSentAddParamIndex = DECCOEFF;     send_pack( gl_ush_flashParamDecCoeff);
    gl_nSentAddParamIndex = CONTROL_I1;   send_pack( gl_ush_flashParamI1min);
    gl_nSentAddParamIndex = CONTROL_I2;   send_pack( gl_ush_flashParamI2min);
    gl_nSentAddParamIndex = CONTROL_AA;   send_pack( gl_ush_flashParamAmplAngMin1);

    gl_nSentAddParamIndex = SIGNCOEFF;    send_pack( gl_ush_flashParamSignCoeff);
    gl_nSentAddParamIndex = DEVNUM;       send_pack( gl_ush_flashParamDeviceId);
    gl_nSentAddParamIndex = DATE_Y;       send_pack( gl_ush_flashParamDateYear);
    gl_nSentAddParamIndex = DATE_M;       send_pack( gl_ush_flashParamDateMonth);
    gl_nSentAddParamIndex = DATE_D;       send_pack( gl_ush_flashParamDateDay);
    gl_nSentAddParamIndex = ORG_B1;       send_pack( gl_ac_flashParamOrg[ 0]);
    gl_nSentAddParamIndex = ORG_B2;       send_pack( gl_ac_flashParamOrg[ 1]);
    gl_nSentAddParamIndex = ORG_B3;       send_pack( gl_ac_flashParamOrg[ 2]);
    gl_nSentAddParamIndex = ORG_B4;       send_pack( gl_ac_flashParamOrg[ 3]);
    gl_nSentAddParamIndex = ORG_B5;       send_pack( gl_ac_flashParamOrg[ 4]);
    gl_nSentAddParamIndex = ORG_B6;       send_pack( gl_ac_flashParamOrg[ 5]);
    gl_nSentAddParamIndex = ORG_B7;       send_pack( gl_ac_flashParamOrg[ 6]);
    gl_nSentAddParamIndex = ORG_B8;       send_pack( gl_ac_flashParamOrg[ 7]);
    gl_nSentAddParamIndex = ORG_B9;       send_pack( gl_ac_flashParamOrg[ 8]);
    gl_nSentAddParamIndex = ORG_B10;      send_pack( gl_ac_flashParamOrg[ 9]);
    gl_nSentAddParamIndex = ORG_B11;      send_pack( gl_ac_flashParamOrg[ 10]);
    gl_nSentAddParamIndex = ORG_B12;      send_pack( gl_ac_flashParamOrg[ 11]);
    gl_nSentAddParamIndex = ORG_B13;      send_pack( gl_ac_flashParamOrg[ 12]);
    gl_nSentAddParamIndex = ORG_B14;      send_pack( gl_ac_flashParamOrg[ 13]);
    gl_nSentAddParamIndex = ORG_B15;      send_pack( gl_ac_flashParamOrg[ 14]);
    gl_nSentAddParamIndex = ORG_B16;      send_pack( gl_ac_flashParamOrg[ 15]);
    gl_nSentAddParamIndex = VERSION;      send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  } //"�������" while
}