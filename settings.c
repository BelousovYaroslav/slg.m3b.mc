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
//extern unsigned short flashLockDev;               //���� ���������� ����������

extern unsigned short flashParamI1min;              //����������� �������� ���� ������� I1
extern unsigned short flashParamI2min;              //����������� �������� ���� ������� I2
extern unsigned short flashParamAmplAngMin1;        //����������� �������� ��� ����������� �������� �� ������� � ����
//extern unsigned short flashParamHvApplyCount;     //���������� ������� ���������� 3kV ��� ������� � �����
//extern unsigned short flashParamHvApplyDurat;     //������������ ������� ���������� 3kV ��� ������� [����]
//extern unsigned short flashParamHvApplyPacks;     //���������� ����� ������� �������

extern unsigned short flashParamSignCoeff;          //�������� �����������
extern unsigned int   flashParamDeviceId;           //ID ����������
extern unsigned short flashParamDateYear;           //���� ?? �������: ���
extern unsigned short flashParamDateMonth;          //���� ?? �������: �����
extern unsigned short flashParamDateDay;            //���� ?? �������: ����
extern char flashParamOrg[];                        //�������� �����������

//OBSOLETE? ��������� �� �����������?
extern unsigned short flashParamPhaseShift;
extern unsigned short gl_ushFlashParamLastRULA;
extern unsigned short gl_ushFlashParamLastRULM;

//���������� �������������
extern signed short flashParam_calibT1;
extern unsigned short flashParamT1_TD1_val, flashParamT1_TD2_val, flashParamT1_TD3_val;
extern signed short flashParam_calibT2;
extern unsigned short flashParamT2_TD1_val, flashParamT2_TD2_val, flashParamT2_TD3_val;



void load_params( void) {
  //��� ���������
  if( flashEE_load_short( 0xf000, &flashParamAmplitudeCode)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //��� ����� ���������
  if( flashEE_load_short( 0xf002, &flashParamTactCode)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����������� �
  if( flashEE_load_short( 0xf004, &flashParamMCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //��������� ����
  if( flashEE_load_short( 0xf006, &flashParamStartMode)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //�������� �����
  if( flashEE_load_int( 0xf008, &flashParamDeviceId)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //�����������
  if( flashEE_load_text( 0xf00C, flashParamOrg, 16)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  } 
  //���
  if( flashEE_load_short( 0xf02C, &flashParamDateYear)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //�����
  if( flashEE_load_short( 0xf02E, &flashParamDateMonth)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����
  if( flashEE_load_short( 0xf030, &flashParamDateDay)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����������� ��� I1
  if( flashEE_load_short( 0xf032, &flashParamI1min)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����������� ��� I2
  if( flashEE_load_short( 0xf034, &flashParamI2min)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����������� AmplAng
  if( flashEE_load_short( 0xf036, &flashParamAmplAngMin1)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����������� ������
  if( flashEE_load_short( 0xf038, &flashParamDecCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //�������� �����������
  if( flashEE_load_short( 0xf03A, &flashParamSignCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  //**************************************************************************************
  // ���������� �������������
  //**************************************************************************************
  //����������� ����������� ����� ����������
  if( flashEE_load_short( 0xf03C, ( unsigned short *) &flashParam_calibT1)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������� ������� ������������ ��� ����������� ����������� ����������
  if( flashEE_load_short( 0xf03E, &flashParamT1_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������� ������� ������������ ��� ����������� ����������� ����������
  if( flashEE_load_short( 0xf040, &flashParamT1_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����������� ������������ ����� ����������
  if( flashEE_load_short( 0xf042, ( unsigned short *) &flashParam_calibT2)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������� ������� ������������ ��� ������������ ����������� ����������
  if( flashEE_load_short( 0xf044, &flashParamT2_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������� ������� ������������ ��� ������������ ����������� ����������
  if( flashEE_load_short( 0xf046, &flashParamT2_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  //********************************************************************
  // �������� ������
  if( flashEE_load_short( 0xf048, &flashParamPhaseShift)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  //��������� RULA
  if( flashEE_load_short( 0xf04A, &gl_ushFlashParamLastRULA)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  //��������� RULM
  if( flashEE_load_short( 0xf04C, &gl_ushFlashParamLastRULM)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

#ifdef DEBUG
  printf("DEBUG: load_params(): params loaded from flash memory. Here they are:\n");
  printf("DEBUG:   Amplitude Code: 0x%04x (%04d)\n", flashParamAmplitudeCode, flashParamAmplitudeCode); //��� ���������
  printf("DEBUG:   Base Tact Code: 0x%04x (%04d)\n", flashParamTactCode, flashParamTactCode);   //��� ����� ���������
  printf("DEBUG:   M Coefficient:  0x%04x (%04d)\n", flashParamMCoeff, flashParamMCoeff);       //����������� �
  printf("DEBUG:   Start Mode:     0x%04x (%04d)\n", flashParamStartMode, flashParamStartMode); //��������� ����
  printf("DEBUG:   Serial number:  0x%04x (%04d)\n", flashParamDeviceId, flashParamDeviceId);   //�������� �����
  printf("DEBUG:   Organization:   '%s'\n", flashParamOrg);                                     //�����������
  printf("DEBUG:   Year:           0x%04x (%04d)\n", flashParamDateYear, flashParamDateYear);   //���
  printf("DEBUG:   Month:          0x%04x (%04d)\n", flashParamDateMonth, flashParamDateMonth); //�����
  printf("DEBUG:   Day:            0x%04x (%04d)\n", flashParamDateDay, flashParamDateDay);     //����
  printf("DEBUG:   Control I1:     0x%04x (%04d)\n", flashParamI1min, flashParamI1min);         //����������� ��� I1
  printf("DEBUG:   Control I2:     0x%04x (%04d)\n", flashParamI2min, flashParamI2min);         //����������� ��� I2
  printf("DEBUG:   Control AA:     0x%04x (%04d)\n", flashParamAmplAngMin1, flashParamAmplAngMin1); //����������� AmplAng
  printf("DEBUG:   Dec. Coeff:     0x%04x (%04d)\n", flashParamDecCoeff, flashParamDecCoeff);   //����������� ������
  printf("DEBUG:   Sign coeff:     0x%04x (%04d)\n", flashParamSignCoeff, flashParamSignCoeff); //�������� �����������
  printf("DEBUG:   Phase shift:    0x%04x (%04d)\n", flashParamPhaseShift, flashParamPhaseShift); //������� �����
  printf("DEBUG:   Last RULA:      0x%04x (%04d)\n", gl_ushFlashParamLastRULA, gl_ushFlashParamLastRULA);   //��������� RULA
  printf("DEBUG:   Last RULM:      0x%04x (%04d)\n", gl_ushFlashParamLastRULM, gl_ushFlashParamLastRULM);   //��������� RULM
#endif

  //PARAMS CHECKING
  if( flashParamAmplitudeCode > 25500)     //��� ��������� [0-25500]. ��������� �������� 9000
    flashParamAmplitudeCode = 9000;        //9000 ��� �������� 3500 ��� ����������

  if( flashParamTactCode > 3)       //��� ����� ��������� [0-3]. ��������� �������� 0
    flashParamTactCode = 0;

  if( flashParamMCoeff > 250)     //����������� �[0-1] = �������� ��������� [0-250].
    flashParamMCoeff = 125;       //��������� �������� 125 (��� �������� M=0.5 � DAC1 = 0.5 * DAC0)  

  if( flashParamStartMode > 250)     //��������� ���� [0-250]. ��������� �������� 125 (��� �������� 1,25� �� DAC2)
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
  printf("DEBUG:   Amplitude Code: 0x%04x (%04d)\n", flashParamAmplitudeCode, flashParamAmplitudeCode); //��� ���������
  printf("DEBUG:   Base Tact Code: 0x%04x (%04d)\n", flashParamTactCode, flashParamTactCode);   //��� ����� ���������
  printf("DEBUG:   M Coefficient:  0x%04x (%04d)\n", flashParamMCoeff, flashParamMCoeff);       //����������� �
  printf("DEBUG:   Start Mode:     0x%04x (%04d)\n", flashParamStartMode, flashParamStartMode); //��������� ����
  printf("DEBUG:   Serial number:  0x%04x (%04d)\n", flashParamDeviceId, flashParamDeviceId);   //�������� �����
  printf("DEBUG:   Organization:   '%s'\n", flashParamOrg);                                     //�����������
  printf("DEBUG:   Year:           0x%04x (%04d)\n", flashParamDateYear, flashParamDateYear);   //���
  printf("DEBUG:   Month:          0x%04x (%04d)\n", flashParamDateMonth, flashParamDateMonth); //�����
  printf("DEBUG:   Day:            0x%04x (%04d)\n", flashParamDateDay, flashParamDateDay);     //����
  printf("DEBUG:   Control I1:     0x%04x (%04d)\n", flashParamI1min, flashParamI1min);         //����������� ��� I1
  printf("DEBUG:   Control I2:     0x%04x (%04d)\n", flashParamI2min, flashParamI2min);         //����������� ��� I2
  printf("DEBUG:   Control AA:     0x%04x (%04d)\n", flashParamAmplAngMin1, flashParamAmplAngMin1); //����������� AmplAng
  printf("DEBUG:   Dec. Coeff:     0x%04x (%04d)\n", flashParamDecCoeff, flashParamDecCoeff);   //����������� ������
  printf("DEBUG:   Sign coeff:     0x%04x (%04d)\n", flashParamSignCoeff, flashParamSignCoeff); //�������� �����������
  printf("DEBUG:   Phase shift:    0x%04x (%04d)\n", flashParamPhaseShift, flashParamPhaseShift); //������� �����
  printf("DEBUG:   Last RULA:      0x%04x (%04d)\n", gl_ushFlashParamLastRULA, gl_ushFlashParamLastRULA);   //��������� RULA
  printf("DEBUG:   Last RULM:      0x%04x (%04d)\n", gl_ushFlashParamLastRULM, gl_ushFlashParamLastRULM);   //��������� RULM
#endif
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
  printf("DEBUG:   Amplitude Code: 0x%04x (%04d)\n", flashParamAmplitudeCode, flashParamAmplitudeCode); //��� ���������
  printf("DEBUG:   Base Tact Code: 0x%04x (%04d)\n", flashParamTactCode, flashParamTactCode);   //��� ����� ���������
  printf("DEBUG:   M Coefficient:  0x%04x (%04d)\n", flashParamMCoeff, flashParamMCoeff);       //����������� �
  printf("DEBUG:   Start Mode:     0x%04x (%04d)\n", flashParamStartMode, flashParamStartMode); //��������� ����
  printf("DEBUG:   Serial number:  0x%04x (%04d)\n", flashParamDeviceId, flashParamDeviceId);   //�������� �����
  printf("DEBUG:   Organization:   '%s'\n", flashParamOrg);                                     //�����������
  printf("DEBUG:   Year:           0x%04x (%04d)\n", flashParamDateYear, flashParamDateYear);   //���
  printf("DEBUG:   Month:          0x%04x (%04d)\n", flashParamDateMonth, flashParamDateMonth); //�����
  printf("DEBUG:   Day:            0x%04x (%04d)\n", flashParamDateDay, flashParamDateDay);     //����
  printf("DEBUG:   Control I1:     0x%04x (%04d)\n", flashParamI1min, flashParamI1min);         //����������� ��� I1
  printf("DEBUG:   Control I2:     0x%04x (%04d)\n", flashParamI2min, flashParamI2min);         //����������� ��� I2
  printf("DEBUG:   Control AA:     0x%04x (%04d)\n", flashParamAmplAngMin1, flashParamAmplAngMin1); //����������� AmplAng
  printf("DEBUG:   Dec. Coeff:     0x%04x (%04d)\n", flashParamDecCoeff, flashParamDecCoeff);   //����������� ������
  printf("DEBUG:   Sign coeff:     0x%04x (%04d)\n", flashParamSignCoeff, flashParamSignCoeff); //�������� �����������
  printf("DEBUG:   Phase shift:    0x%04x (%04d)\n", flashParamPhaseShift, flashParamPhaseShift); //������� �����
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
  //0xf03C - 0xf046 ������������ - ������������� ������ �������������
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