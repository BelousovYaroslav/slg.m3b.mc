#include <stdio.h>
#include "settings.h"
#include "errors.h"
#include "flashEE.h"

#include "debug.h"

//declared in Main.c
extern char gl_c_EmergencyCode;

//flash-stored params declared in Main.c
extern unsigned short flashParamAmplitudeCode;      //��������� ��������� ������������ (��������)
extern unsigned short flashParamTactCode;           //��� ����� ���������
extern unsigned short flashParamMCoeff;             //����������� ���������
extern unsigned short flashParamStartMode;          //��������� ���� ������� ����������� ���������
extern unsigned short flashParamDecCoeff;           //����������� ������
extern unsigned short flashLockDev;                 //���� ���������� ����������

extern unsigned short flashParamI1min;              //����������� �������� ���� ������� I1
extern unsigned short flashParamI2min;              //����������� �������� ���� ������� I2
extern unsigned short flashParamAmplAngMin1;        //����������� �������� ��� ����������� �������� �� ������� � ����
//extern unsigned short flashParamHvApplyCount;     //���������� ������� ���������� 3kV ��� ������� � �����
//extern unsigned short flashParamHvApplyDurat;     //������������ ������� ���������� 3kV ��� ������� [����]
//extern unsigned short flashParamHvApplyPacks;     //���������� ����� ������� �������

extern unsigned short flashParamSignCoeff;          //�������� �����������
extern unsigned short flashParamDeviceId;           //ID ����������
extern unsigned short flashParamDateYear;           //���� ?? �������: ���
extern unsigned short flashParamDateMonth;          //���� ?? �������: �����
extern unsigned short flashParamDateDay;            //���� ?? �������: ����
extern char flashParamOrg[];                        //�������� �����������

//OBSOLETE? ��������� �� �����������
extern unsigned short gl_ushFlashParamLastRULA;
extern unsigned short gl_ushFlashParamLastRULM;

//���������� �������������
extern signed short flashParam_calibT1;
extern unsigned short flashParamT1_TD1_val, flashParamT1_TD2_val, flashParamT1_TD3_val;
extern signed short flashParam_calibT2;
extern unsigned short flashParamT2_TD1_val, flashParamT2_TD2_val, flashParamT2_TD3_val;




void load_params_p1( void) {
  //��� ���������
  if( flashEE_load_short( ADDR_AMPLITUDE, &flashParamAmplitudeCode))  gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //��� ����� ���������
  if( flashEE_load_short( ADDR_TACT_CODE, &flashParamTactCode))       gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //����������� �
  if( flashEE_load_short( ADDR_M_COEFF, &flashParamMCoeff))           gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //��������� ����
  if( flashEE_load_short( ADDR_START_MODE, &flashParamStartMode))     gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //����������� ������
  if( flashEE_load_short( ADDR_DEC_COEFF, &flashParamDecCoeff))       gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //���� ���������� ����������
  if( flashEE_load_short( ADDR_LOCK_DEV, &flashLockDev))              gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //��������� RULA
  if( flashEE_load_short( ADDR_LAST_RULA, &gl_ushFlashParamLastRULA)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //��������� RULM
  if( flashEE_load_short( ADDR_LAST_RULM, &gl_ushFlashParamLastRULM)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;


#ifdef DEBUG
  printf("DBG: load_params(): params loaded from flash memory. Here they are:\n");
  printf("DBG:   Amplitude Code: 0x%04x (%04d)\n", flashParamAmplitudeCode, flashParamAmplitudeCode); //��� ���������
  printf("DBG:   Base Tact Code: 0x%04x (%04d)\n", flashParamTactCode, flashParamTactCode);   //��� ����� ���������
  printf("DBG:   M Coefficient:  0x%04x (%04d)\n", flashParamMCoeff, flashParamMCoeff);       //����������� �
  printf("DBG:   Start Mode:     0x%04x (%04d)\n", flashParamStartMode, flashParamStartMode); //��������� ����
  printf("DBG:   Dec. Coeff:     0x%04x (%04d)\n", flashParamDecCoeff, flashParamDecCoeff);   //����������� ������
  printf("DBG:   Dev Lock:       0x%04x (%04d)\n", flashLockDev, flashLockDev);               //���� ���������� ����������
  printf("DBG:   Last RULA:      0x%04x (%04d)\n", gl_ushFlashParamLastRULA, gl_ushFlashParamLastRULA);   //��������� RULA
  printf("DBG:   Last RULM:      0x%04x (%04d)\n", gl_ushFlashParamLastRULM, gl_ushFlashParamLastRULM);   //��������� RULM
#endif
}

void load_params_p2( void) {
  //����������� ��� I1
  if( flashEE_load_short( ADDR_CONTROL_I1, &flashParamI1min)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����������� ��� I2
  if( flashEE_load_short( ADDR_CONTROL_I2, &flashParamI2min)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����������� AmplAng
  if( flashEE_load_short( ADDR_CONTROL_AA, &flashParamAmplAngMin1)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  /*
  //���������� ������� ���������� 3kV ��� ������� � �����
  if( flashEE_load_short( ADDR_HV_APPLY_C, &flashParamHvApplyCount)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������������ ������� ���������� 3kV ��� �������
  if( flashEE_load_short( ADDR_HV_APPLY_D, &flashParamHvApplyDurat)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //���������� ����� ������� �������
  if( flashEE_load_short( ADDR_HV_APPLY_P, &flashParamHvApplyPacks)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  */

#ifdef DEBUG
  printf("DBG:load_params_p2()\n");
  printf("DBG:   Control I1:     0x%04x (%04d)\n", flashParamI1min, flashParamI1min);                 //����������� ��� I1
  printf("DBG:   Control I2:     0x%04x (%04d)\n", flashParamI2min, flashParamI2min);                 //����������� ��� I2
  printf("DBG:   Control AA:     0x%04x (%04d)\n", flashParamAmplAngMin1, flashParamAmplAngMin1);     //����������� AmplAng
  //printf("DBG:   HV_count:       0x%04x (%04d)\n", flashParamHvApplyCount, flashParamHvApplyCount);   //HV_applies tries amount in pack
  //printf("DBG:   HV_duration:    0x%04x (%04d)\n", flashParamHvApplyDurat, flashParamHvApplyDurat);   //HV_applies tries duration
  //printf("DBG:   HV_packs:       0x%04x (%04d)\n", flashParamHvApplyPacks, flashParamHvApplyPacks);   //HV_applies tries packs
#endif
}

void load_params_p3( void) {
  //�������� �����������
  if( flashEE_load_short( ADDR_SIGN_COEFF, &flashParamSignCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //�������� �����
  if( flashEE_load_short( ADDR_DEVICE_ID, &flashParamDeviceId)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����.���
  if( flashEE_load_short( ADDR_DATE_Y, &flashParamDateYear)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����.�����
  if( flashEE_load_short( ADDR_DATE_M, &flashParamDateMonth)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����.����
  if( flashEE_load_short( ADDR_DATE_D, &flashParamDateDay)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //�����������
  if( flashEE_load_text( ADDR_ORG, flashParamOrg, 16)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

#ifdef DEBUG
  printf("DBG:load_params_p3()\n");
  printf("DBG:   Sign coeff:      0x%04x (%04d)\n", flashParamSignCoeff, flashParamSignCoeff); //�������� �����������
  printf("DBG:   Serial number:   0x%04x (%04d)\n", flashParamDeviceId, flashParamDeviceId);   //�������� �����
  printf("DBG:   Organization:    '%s'\n", flashParamOrg);                                     //�����������
  printf("DBG:   Year:            0x%04x (%04d)\n", flashParamDateYear, flashParamDateYear);   //���
  printf("DBG:   Month:           0x%04x (%04d)\n", flashParamDateMonth, flashParamDateMonth); //�����
  printf("DBG:   Day:             0x%04x (%04d)\n", flashParamDateDay, flashParamDateDay);     //����
#endif
}

void load_params_p4( void) {
  //����������� ����������� ����� ����������
  if( flashEE_load_short( ADDR_TCALIB_T1, ( unsigned short *) &flashParam_calibT1)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������� ������� ������������ ��� ����������� ����������� ����������
  if( flashEE_load_short( ADDR_TCALIB_T1_TD1, &flashParamT1_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������� ������� ������������ ��� ����������� ����������� ����������
  if( flashEE_load_short( ADDR_TCALIB_T1_TD2, &flashParamT1_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������� �������� ������������ ��� ����������� ����������� ����������
  if( flashEE_load_short( ADDR_TCALIB_T1_TD3, &flashParamT1_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  //����������� ������������ ����� ����������
  if( flashEE_load_short( ADDR_TCALIB_T2, ( unsigned short *) &flashParam_calibT2)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������� ������� ������������ ��� ������������ ����������� ����������
  if( flashEE_load_short( ADDR_TCALIB_T2_TD1, &flashParamT2_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������� ������� ������������ ��� ������������ ����������� ����������
  if( flashEE_load_short( ADDR_TCALIB_T2_TD2, &flashParamT2_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������� ������� ������������ ��� ������������ ����������� ����������
  if( flashEE_load_short( ADDR_TCALIB_T2_TD3, &flashParamT2_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
#ifdef DEBUG
  printf("DBG:load_params_p4()\n");
  printf("DBG:   T-Calibration T1=0x%04x (%04d)\n", flashParam_calibT1, flashParam_calibT1);      //������������� ����������: ����������� ������ �����
  printf("DBG:   T1_TD1:          0x%04x (%04d)\n", flashParamT1_TD1_val, flashParamT1_TD1_val);  //������������� ����������: ��������� ������������ TD1 � ������ ������������� �����
  printf("DBG:   T1_TD2:          0x%04x (%04d)\n", flashParamT1_TD2_val, flashParamT1_TD2_val);  //������������� ����������: ��������� ������������ TD2 � ������ ������������� �����
  printf("DBG:   T1_TD3:          0x%04x (%04d)\n", flashParamT1_TD3_val, flashParamT1_TD3_val);  //������������� ����������: ��������� ������������ TD3 � ������ ������������� �����

  printf("DBG:   T-Calibration T2=0x%04x (%04d)\n", flashParam_calibT2, flashParam_calibT2);      //������������� ����������: ����������� ������� �����
  printf("DBG:   T1_TD1:          0x%04x (%04d)\n", flashParamT2_TD1_val, flashParamT2_TD1_val);  //������������� ����������: ��������� ������������ TD1 � ������� ������������� �����
  printf("DBG:   T1_TD2:          0x%04x (%04d)\n", flashParamT2_TD2_val, flashParamT2_TD2_val);  //������������� ����������: ��������� ������������ TD2 � ������� ������������� �����
  printf("DBG:   T1_TD3:          0x%04x (%04d)\n", flashParamT2_TD3_val, flashParamT2_TD3_val);  //������������� ����������: ��������� ������������ TD3 � ������� ������������� �����
#endif
}

void check_params_p1( void) {
  //��� ��������� [0-255]. ��������� �������� 90
  //90 ��� �������� 35 ��� ����������
  if( flashParamAmplitudeCode > 255)
    flashParamAmplitudeCode = 35;

  //��� ����� ��������� [0-3]. ��������� �������� 0
  if( flashParamTactCode > 3)
    flashParamTactCode = 0;

  //����������� �[0-1] = �������� ��������� [0-250]
  //��������� �������� 200 (��� �������� M=0.8 � DAC1 = 0.8 * DAC0)
  if( flashParamMCoeff > 250)
    flashParamMCoeff = 200;

  //��������� ���� [0-250]. ��������� �������� 125 (��� �������� 1,25� �� DAC2)
  if( flashParamStartMode > 250)
    flashParamStartMode = 125;

  //����������� ������
  //default �������� 0,4 ��������!
  if( flashParamDecCoeff == 0xffff)
    flashParamDecCoeff = ( int) ( 0.4 * 65535.);

  //���� ���������� ����������
  //default �������� 0 - ����� �������������
  if( flashLockDev != 1)
    flashLockDev = 0;

  //��������� RULA [0-4095]
  if( gl_ushFlashParamLastRULA > 4095) {
    gl_ushFlashParamLastRULA = 0;
  }

  //��������� RULM [0-4095]
  if( gl_ushFlashParamLastRULM > 4095) {
    gl_ushFlashParamLastRULM = 0;
  }

#ifdef DEBUG
  printf("DBG: check_params_p1(): params checked for the range. Here they are:\n");
  printf("DBG:   Amplitude Code: 0x%04x (%04d)\n", flashParamAmplitudeCode, flashParamAmplitudeCode); //��� ���������
  printf("DBG:   Base Tact Code: 0x%04x (%04d)\n", flashParamTactCode, flashParamTactCode);   //��� ����� ���������
  printf("DBG:   M Coefficient:  0x%04x (%04d)\n", flashParamMCoeff, flashParamMCoeff);       //����������� �
  printf("DBG:   Start Mode:     0x%04x (%04d)\n", flashParamStartMode, flashParamStartMode); //��������� ����
  printf("DBG:   Dec. Coeff:     0x%04x (%04d)\n", flashParamDecCoeff, flashParamDecCoeff);   //����������� ������
  printf("DBG:   Dev Lock:       0x%04x (%04d)\n", flashLockDev, flashLockDev);               //���� ���������� ����������
#endif
}

void check_params_p2( void) {
  //����������� ��� ������� I1 [0-0.750 mA] = �������� ��������� [ 0 - 65534]
  //default �������� 0.4 mA
  if( flashParamI1min == 0xffff)
    flashParamI1min = ( short) ( 65535. * 0.4 / 0.75);

  //����������� ��� ������� I2 [0-0.750 mA] = �������� ��������� [ 0 - 65534]
  //default �������� 0.4 mA
  if( flashParamI2min == 0xffff)
    flashParamI2min = ( short) ( 65535. * 0.4 / 0.75);

  //����������� �������� �������� - ��������� ���� (0-3�) = �������� ��������� [ 0 - 65534]
  //default �������� 1.0�
  if( flashParamAmplAngMin1 == 0xffff)
    flashParamAmplAngMin1 = ( int) ( 1.0 / 3. * 65535.);

  /*
  //���������� ������� ���������� 3kV ��� ������� ( 1 - 20)
  //default value = 10
  if( flashParamHvApplyCount < 1 || flashParamHvApplyCount > 20) {
    flashParamHvApplyCount = 10;
  }

  //HV_applies tries duration [0 - 5 sec] = values [0 - 5000]
  //default value = 1000
  if( flashParamHvApplyDurat < 1 || flashParamHvApplyDurat > 5000) {
    flashParamHvApplyDurat = 1000;
  }

  //HV_applies tries packs [1 - 10] = values [1 - 10]
  //default value = 5
  if( flashParamHvApplyPacks < 1 || flashParamHvApplyPacks > 10) {
    flashParamHvApplyPacks = 5;
  }
  */

#ifdef DEBUG
  printf("DBG: check_params_p2(): params checked for the range. Here they are:\n");
  printf("DBG:   Control I1:     0x%04x (%04d)\n", flashParamI1min, flashParamI1min);                 //����������� ��� I1
  printf("DBG:   Control I2:     0x%04x (%04d)\n", flashParamI2min, flashParamI2min);                 //����������� ��� I2
  printf("DBG:   Control AA:     0x%04x (%04d)\n", flashParamAmplAngMin1, flashParamAmplAngMin1);     //����������� AmplAng
  //printf("DBG:   HV_count:       0x%04x (%04d)\n", flashParamHvApplyCount, flashParamHvApplyCount);   //HV_applies tries amount
  //printf("DBG:   HV_duration:    0x%04x (%04d)\n", flashParamHvApplyDurat, flashParamHvApplyDurat);   //HV_applies tries duration
  //printf("DBG:   HV_packs:       0x%04x (%04d)\n", flashParamHvApplyPacks, flashParamHvApplyPacks);   //HV_applies tries packs
#endif
}

void check_params_p3( void) {
  int i;
  //�������� �����������. [-1; +1]
  //default value = 1
  if( flashParamSignCoeff > 2)
    flashParamSignCoeff = 2;

  //ID ����������
  if( flashParamDeviceId == 65535)
    flashParamDeviceId = 0;

  //���� ?? ����������
  //default value = 2016.01.01
  if( flashParamDateYear < 2000 || flashParamDateYear > 2200)
    flashParamDateYear = 2016;

  if( flashParamDateMonth > 12)
    flashParamDateMonth = 1;

  if( flashParamDateDay > 31)
    flashParamDateDay = 1;

  //�������� �����������
  //default - ��� ������
  for( i=0; i<17; i++) {
    if( flashParamOrg[i] < 33 || flashParamOrg[i] > 126)
      flashParamOrg[i] = ' ';
  }



#ifdef DEBUG
  printf("DBG: check_params_p3(): params checked for the range. Here they are:\n");
  printf("DBG:   Sign coeff:     0x%04x (%04d)\n", flashParamSignCoeff, flashParamSignCoeff); //�������� �����������
  printf("DBG:   Serial number:  0x%04x (%04d)\n", flashParamDeviceId, flashParamDeviceId);   //�������� �����
  printf("DBG:   Year:           0x%04x (%04d)\n", flashParamDateYear, flashParamDateYear);   //���
  printf("DBG:   Month:          0x%04x (%04d)\n", flashParamDateMonth, flashParamDateMonth); //�����
  printf("DBG:   Day:            0x%04x (%04d)\n", flashParamDateDay, flashParamDateDay);     //����
  printf("DBG:   Organization:   '%s'\n", flashParamOrg);                                     //�����������
#endif
}

void check_params_p4( void) {
  if( flashParam_calibT1 < ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION)  ||
      flashParam_calibT1 > ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
    flashParam_calibT1 = 0;
    flashParamT1_TD1_val = 0;
    flashParamT1_TD2_val = 1;
    flashParamT1_TD3_val = 2;
  }

  if( flashParam_calibT2 < ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) ||
      flashParam_calibT2 > ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
    flashParam_calibT2 = 0;
    flashParamT2_TD1_val = 0;
    flashParamT2_TD2_val = 1;
    flashParamT2_TD3_val = 2;
  }
#ifdef DEBUG
  printf("DBG: check_params_p4(): params checked for the range. Here they are:\n");
#endif
}

void load_params( void) {
  load_params_p1();
  if( gl_c_EmergencyCode != ERROR_FLASH_LOAD_PARAMS_FAIL) load_params_p2();
  if( gl_c_EmergencyCode != ERROR_FLASH_LOAD_PARAMS_FAIL) load_params_p3();
  if( gl_c_EmergencyCode != ERROR_FLASH_LOAD_PARAMS_FAIL) load_params_p4();

  check_params_p1();
  check_params_p2();
  check_params_p3();
  check_params_p4();
}

void save_params_p1( void) {
#ifdef DEBUG
  printf("DBG: save_params_p1(): params to be saved are:\n");
  printf("DBG:   Amplitude Code: 0x%04x (%04d)\n", flashParamAmplitudeCode, flashParamAmplitudeCode); //��� ���������
  printf("DBG:   Base Tact Code: 0x%04x (%04d)\n", flashParamTactCode, flashParamTactCode);   //��� ����� ���������
  printf("DBG:   M Coefficient:  0x%04x (%04d)\n", flashParamMCoeff, flashParamMCoeff);       //����������� �
  printf("DBG:   Start Mode:     0x%04x (%04d)\n", flashParamStartMode, flashParamStartMode); //��������� ����
  printf("DBG:   Dec. Coeff:     0x%04x (%04d)\n", flashParamDecCoeff, flashParamDecCoeff);   //����������� ������
  printf("DBG:   Dev Lock:       0x%04x (%04d)\n", flashLockDev, flashLockDev);               //���� ���������� ����������
  printf("DBG:   Last RULA:      0x%04x (%04d)\n", gl_ushFlashParamLastRULA, gl_ushFlashParamLastRULA);   //��������� RULA
  printf("DBG:   Last RULM:      0x%04x (%04d)\n", gl_ushFlashParamLastRULM, gl_ushFlashParamLastRULM);   //��������� RULM
#endif

  if( flashEE_erase_page( ADDR_PAGE1)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }

  if( flashEE_save_short( ADDR_AMPLITUDE, flashParamAmplitudeCode)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TACT_CODE, flashParamTactCode)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_M_COEFF, flashParamMCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_START_MODE, flashParamStartMode)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_DEC_COEFF, flashParamDecCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
#ifdef DEBUG
  if( flashLockDev == 1) {
    printf( "DBG: device will be locked\n");
  }
#endif
  if( flashEE_save_short( ADDR_LOCK_DEV, flashLockDev)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_LAST_RULA, gl_ushFlashParamLastRULA)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_LAST_RULM, gl_ushFlashParamLastRULM)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
}

void save_params_p2( void) {
#ifdef DEBUG
  printf("DBG: save_params_p2(): params to be saved are:\n");
  printf("DBG:   Control I1:     0x%04x (%04d)\n", flashParamI1min, flashParamI1min);                 //����������� ��� I1
  printf("DBG:   Control I2:     0x%04x (%04d)\n", flashParamI2min, flashParamI2min);                 //����������� ��� I2
  printf("DBG:   Control AA:     0x%04x (%04d)\n", flashParamAmplAngMin1, flashParamAmplAngMin1);     //����������� AmplAng
  //printf("DBG:   HV_count:       0x%04x (%04d)\n", flashParamHvApplyCount, flashParamHvApplyCount);   //HV_applies tries amount
  //printf("DBG:   HV_duration:    0x%04x (%04d)\n", flashParamHvApplyDurat, flashParamHvApplyDurat);   //HV_applies tries duration
  //printf("DBG:   HV_packs:       0x%04x (%04d)\n", flashParamHvApplyPacks, flashParamHvApplyPacks);   //HV_applies tries packs
#endif

  if( flashEE_erase_page( ADDR_PAGE2)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }

  if( flashEE_save_short( ADDR_CONTROL_I1, flashParamI1min)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_CONTROL_I2, flashParamI2min)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_CONTROL_AA, flashParamAmplAngMin1)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  /*
  if( flashEE_save_short( ADDR_HV_APPLY_C, flashParamHvApplyCount)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_HV_APPLY_D, flashParamHvApplyDurat)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_HV_APPLY_P, flashParamHvApplyPacks)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  */
}

void save_params_p3( void) {
#ifdef DEBUG
  printf("DBG: save_params_p3(): params to be saved are:\n");
  printf("DBG:   Sign coeff:     0x%04x (%04d)\n", flashParamSignCoeff, flashParamSignCoeff); //�������� �����������
  printf("DBG:   Serial number:  0x%04x (%04d)\n", flashParamDeviceId, flashParamDeviceId);   //�������� �����
  printf("DBG:   Organization:   '%s'\n", flashParamOrg);                                     //�����������
  printf("DBG:   Year:           0x%04x (%04d)\n", flashParamDateYear, flashParamDateYear);   //���
  printf("DBG:   Month:          0x%04x (%04d)\n", flashParamDateMonth, flashParamDateMonth); //�����
  printf("DBG:   Day:            0x%04x (%04d)\n", flashParamDateDay, flashParamDateDay);     //����
#endif

  if( flashEE_erase_page( ADDR_PAGE3)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }

  if( flashEE_save_short( ADDR_SIGN_COEFF, flashParamSignCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_DEVICE_ID, flashParamDeviceId)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_DATE_Y, flashParamDateYear)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_DATE_M, flashParamDateMonth)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_DATE_D, flashParamDateDay)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_text( ADDR_ORG, flashParamOrg, 16)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
}

void save_params_p4( void) {
#ifdef DEBUG
  printf("DBG: save_params_p4(): params to be saved are:\n");
#endif

  if( flashEE_save_short( ADDR_TCALIB_T1, flashParam_calibT1)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T1_TD1, flashParamT1_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T1_TD2, flashParamT1_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T1_TD3, flashParamT1_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T2, flashParam_calibT2)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T2_TD1, flashParamT2_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T2_TD2, flashParamT2_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T2_TD3, flashParamT2_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
}

void save_params( void) {
  save_params_p1();
  if( gl_c_EmergencyCode != ERROR_FLASH_SAVE_PARAMS_FAIL) save_params_p2();
  if( gl_c_EmergencyCode != ERROR_FLASH_SAVE_PARAMS_FAIL) save_params_p3();
  if( gl_c_EmergencyCode != ERROR_FLASH_SAVE_PARAMS_FAIL) save_params_p4();
}