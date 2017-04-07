#include <ADuC7026.h>
#include "version.h"
#include "errors.h"
#include "Main.h"

extern unsigned short flashParamAmplitudeCode;
extern unsigned short flashParamTactCode;
extern unsigned short flashParamMCoeff;
extern unsigned short flashParamStartMode;
extern unsigned int flashParamDeviceId;
//extern char *flashParamOrg;
//extern unsigned short flashParamDateYear = 0, flashParamDateMonth = 0, flashParamDateDay = 0;
extern unsigned short flashParamI1min;
extern unsigned short flashParamI2min;
extern unsigned short flashParamAmplAngMin1;
extern unsigned short flashParamDecCoeff;
extern unsigned short flashParamSignCoeff;
extern unsigned short flashParamPhaseShift;
extern char gl_c_EmergencyCode;

extern signed short gl_ssh_SA_time;
extern signed short gl_ssh_prT1VAL;

extern signed short gl_ssh_ampl_angle;

extern char input_buffer[];
extern char pos_in_in_buf;

extern void send_pack( signed short angle_inc1, short param_indicator, short analog_param);

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

  //������� ����������� ����������
  send_pack( 0, 7, flashParamAmplitudeCode);
  send_pack( 0, 8, flashParamTactCode);
  send_pack( 0, 9, flashParamMCoeff);
  send_pack( 0, 10, flashParamStartMode);
  send_pack( 0, 11, flashParamI1min);
  send_pack( 0, 12, flashParamI2min);
  send_pack( 0, 13, flashParamAmplAngMin1);
  send_pack( 0, 14, flashParamDecCoeff);
  send_pack( 0, 15, flashParamSignCoeff);
  send_pack( 0, 16, (VERSION_MINOR * 16 << 12) + VERSION_MAJOR * 16 + VERSION_MIDDLE);

  gl_ssh_prT1VAL = T1VAL;
  while( 1) {
    //����� 0,1 �������
    pause( 327);

    gl_ssh_SA_time = ( T1LD + gl_ssh_prT1VAL - T1VAL) % T1LD;
    gl_ssh_prT1VAL = T1VAL;

    //**********************************************************************
    // ��������� ������ �������� ������
    //**********************************************************************
    if( pos_in_in_buf == IN_COMMAND_BUF_LEN) {
      switch( input_buffer[0]) {
        case 0: //���������� ��� ���������
          flashParamAmplitudeCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 7, flashParamAmplitudeCode);
        break;

        case 1: //���������� ��� ����� ���������
          flashParamTactCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 8, flashParamTactCode);
        break;

        case 2: //���������� ����������� M
          flashParamMCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 9, flashParamMCoeff);
        break;

        case 3: //���������� ��������� ����
          flashParamStartMode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 10, flashParamStartMode);
        break;
        
        case 4: //���������� ����������� ��� I1
          flashParamI1min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 11, flashParamI1min);
        break;

        case 5: //���������� ����������� ��� I2
          flashParamI2min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 12, flashParamI2min);
        break;

        case 6: //���������� 1�� ������� ������� AmplAng
          flashParamAmplAngMin1 = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 13, flashParamAmplAngMin1);
        break;

        case 7: //���������� ����������� ������
          flashParamDecCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 14, flashParamDecCoeff);
        break;

        case 8: //���������� SA ����
          flashParamSignCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 15, flashParamSignCoeff);
        break;

        /*
        case 9: //� ������. ������ �������� ����� SA (�������� ����� dU)
          bAsyncDu = 1;
        break;

        case 10: //� ������. ������ ��������� ����� dU (�������� ����� SA)
          bAsyncDu = 0;
        break; */

        case 49: //������ ����������
          send_pack( 0, 7, flashParamAmplitudeCode);
          send_pack( 0, 8, flashParamTactCode);
          send_pack( 0, 9, flashParamMCoeff);
          send_pack( 0, 10, flashParamStartMode);
          send_pack( 0, 11, flashParamI1min);
          send_pack( 0, 12, flashParamI2min);
          send_pack( 0, 13, flashParamAmplAngMin1);
          send_pack( 0, 14, flashParamDecCoeff);
          send_pack( 0, 15, flashParamSignCoeff);
          send_pack( 0, 16, (VERSION_MINOR * 16 << 12) + VERSION_MAJOR * 16 + VERSION_MIDDLE);
        break;

        case 50: //��������� ��������� �� ���� ������
          save_params(); break;

        case 51: //������������� ��������� �� ����-������ � �������� ��
          load_params(); break;
      }
      
      pos_in_in_buf = 0;
    }
    else
      //���� �������� ������ �� ����, ��
      //������� ������� ��������� � �������
      send_pack( 0, 0, 0);

  } //"�������" while
}

void deadloop_no_hangerup( void) {
  //��������� ������ �������� ������������
#ifdef DEBUG
  printf("DEBUG: NO HANGER VIBRATION! DEADLOOP.\n");
#endif
  //���������� ��� ������
  gl_c_EmergencyCode = ERROR_INITIAL_AMPL_ANG_TEST_FAIL;

  ADCCP = 0x06;     //�� ����� �������� ������ AmplAng
  ADCCON |= 0x80;   //������ ��������������

  //������� ����������� ����������
  send_pack( 0, 7, flashParamAmplitudeCode);
  send_pack( 0, 8, flashParamTactCode);
  send_pack( 0, 9, flashParamMCoeff);
  send_pack( 0, 10, flashParamStartMode);
  send_pack( 0, 11, flashParamI1min);
  send_pack( 0, 12, flashParamI2min);
  send_pack( 0, 13, flashParamAmplAngMin1);
  send_pack( 0, 14, flashParamDecCoeff);
  send_pack( 0, 15, flashParamSignCoeff);
  send_pack( 0, 16, (VERSION_MINOR * 16 << 12) + VERSION_MAJOR * 16 + VERSION_MIDDLE);

  gl_ssh_prT1VAL = T1VAL;
  while( 1) {
    //����� 0,1 �������
    pause( 327);

    gl_ssh_SA_time = ( T1LD + gl_ssh_prT1VAL - T1VAL) % T1LD;
    gl_ssh_prT1VAL = T1VAL;

    //��������� AmplAng (� ������ ���)
    while (!( ADCSTA & 0x01)){}     // ������� ����� �������������� ��� (������������ ����� �� �������� ���� �� ��� ������ ���� �����)
    gl_ssh_ampl_angle = (ADCDAT >> 16);
    ADCCON |= 0x80;                 //������ ��������������

    //**********************************************************************
    // ��������� ������ �������� ������
    //**********************************************************************
    if( pos_in_in_buf == IN_COMMAND_BUF_LEN) {
      switch( input_buffer[0]) {
        case 0: //���������� ��� ���������
          flashParamAmplitudeCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 7, flashParamAmplitudeCode);
        break;

        case 1: //���������� ��� ����� ���������
          flashParamTactCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 8, flashParamTactCode);
        break;

        case 2: //���������� ����������� M
          flashParamMCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 9, flashParamMCoeff);
        break;

        case 3: //���������� ��������� ����
          flashParamStartMode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 10, flashParamStartMode);
        break;
        
        case 4: //���������� ����������� ��� I1
          flashParamI1min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 11, flashParamI1min);
        break;

        case 5: //���������� ����������� ��� I2
          flashParamI2min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 12, flashParamI2min);
        break;

        case 6: //���������� 1�� ������� ������� AmplAng
          flashParamAmplAngMin1 = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 13, flashParamAmplAngMin1);
        break;

        case 7: //���������� ����������� ������
          flashParamDecCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 14, flashParamDecCoeff);
        break;

        case 8: //���������� SA ����
          flashParamSignCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 15, flashParamSignCoeff);
        break;

        /*
        case 9: //� ������. ������ �������� ����� SA (�������� ����� dU)
          bAsyncDu = 1;
        break;

        case 10: //� ������. ������ ��������� ����� dU (�������� ����� SA)
          bAsyncDu = 0;
        break;
        */

        case 49: //������ ����������
          send_pack( 0, 7, flashParamAmplitudeCode);
          send_pack( 0, 8, flashParamTactCode);
          send_pack( 0, 9, flashParamMCoeff);
          send_pack( 0, 10, flashParamStartMode);
          send_pack( 0, 11, flashParamI1min);
          send_pack( 0, 12, flashParamI2min);
          send_pack( 0, 13, flashParamAmplAngMin1);
          send_pack( 0, 14, flashParamDecCoeff);
          send_pack( 0, 15, flashParamSignCoeff);
          send_pack( 0, 16, (VERSION_MINOR * 16 << 12) + VERSION_MAJOR * 16 + VERSION_MIDDLE);  
        break;

        case 50: //��������� ��������� �� ���� ������
          save_params(); break;

        case 51: //������������� ��������� �� ����-������ � �������� ��
          load_params(); break;
      }

      pos_in_in_buf = 0;
    }
    else
      //���� �������� ������ �� ����, ��
      //������� ������� ��������� � �������
      send_pack( 0, 6, gl_ssh_ampl_angle);

  } //"�������" ������ ������ �������� ������������
}

void deadloop_no_firing( void) {
  //��������� ������ �������
#ifdef DEBUG
  printf("DEBUG: NO LASER FIREUP! DEADLOOP.\n");
#endif

  //���������� ��� ������
  gl_c_EmergencyCode = ERROR_NO_LASER_FIRING;

  //������� ����������� ����������
  send_pack( 0, 7, flashParamAmplitudeCode);
  send_pack( 0, 8, flashParamTactCode);
  send_pack( 0, 9, flashParamMCoeff);
  send_pack( 0, 10, flashParamStartMode);
  send_pack( 0, 11, flashParamI1min);
  send_pack( 0, 12, flashParamI2min);
  send_pack( 0, 13, flashParamAmplAngMin1);
  send_pack( 0, 14, flashParamDecCoeff);
  send_pack( 0, 15, flashParamSignCoeff);
  send_pack( 0, 16, (VERSION_MINOR * 16 << 12) + VERSION_MAJOR * 16 + VERSION_MIDDLE);

  gl_ssh_prT1VAL = T1VAL;
  while( 1) {
    //����� 0,1 �������
    pause( 327);

    gl_ssh_SA_time = ( T1LD + gl_ssh_prT1VAL - T1VAL) % T1LD;
    gl_ssh_prT1VAL = T1VAL;

    //**********************************************************************
    // ��������� ������ �������� ������
    //**********************************************************************
    if( pos_in_in_buf == IN_COMMAND_BUF_LEN) {
      switch( input_buffer[0]) {
        case 0: //���������� ��� ���������
          flashParamAmplitudeCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 7, flashParamAmplitudeCode);
        break;

        case 1: //���������� ��� ����� ���������
          flashParamTactCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 8, flashParamTactCode);
        break;

        case 2: //���������� ����������� M
          flashParamMCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 9, flashParamMCoeff);
        break;

        case 3: //���������� ��������� ����
          flashParamStartMode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 10, flashParamStartMode);
        break;
        
        case 4: //���������� ����������� ��� I1
          flashParamI1min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 10, flashParamI1min);
        break;

        case 5: //���������� ����������� ��� I2
          flashParamI2min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 11, flashParamI2min);
        break;

        case 6: //���������� 1�� ������� ������� AmplAng
          flashParamAmplAngMin1 = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 12, flashParamAmplAngMin1);
        break;

        case 7: //���������� ����������� ������
          flashParamDecCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 14, flashParamDecCoeff);
        break;

        case 8: //���������� �������� �����������
          flashParamSignCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 15, flashParamSignCoeff);
        break;

        /*
        case 9: //� ������. ������ �������� ����� SA (�������� ����� dU)
        	bAsyncDu = 1;
        break;

        case 10: //� ������. ������ ��������� ����� dU (�������� ����� SA)
        	bAsyncDu = 0;
        break; */

        case 49: //������ ����������
          send_pack( 0, 7, flashParamAmplitudeCode);
          send_pack( 0, 8, flashParamTactCode);
          send_pack( 0, 9, flashParamMCoeff);
          send_pack( 0, 10, flashParamStartMode);
          send_pack( 0, 11, flashParamI1min);
          send_pack( 0, 12, flashParamI2min);
          send_pack( 0, 13, flashParamAmplAngMin1);
          send_pack( 0, 14, flashParamDecCoeff);
          send_pack( 0, 15, flashParamSignCoeff);
          send_pack( 0, 16, (VERSION_MINOR * 16 << 12) + VERSION_MAJOR * 16 + VERSION_MIDDLE);
        break;

        case 50: //��������� ��������� �� ���� ������
          save_params(); break;

        case 51: //������������� ��������� �� ����-������ � �������� ��
          load_params(); break;
      }
      
      pos_in_in_buf = 0;
    }
    else
      //���� �������� ������ �� ����, ��
      //������� ������� ��������� � �������
      send_pack( 0, 0, 0);

  } //"�������" while
}