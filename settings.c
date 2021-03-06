#include <stdio.h>
#include "settings.h"
#include "errors.h"
#include "flashEE.h"
#include "AnalogueParamsConstList.h"
#include "debug.h"

//declared in Main.c
extern char gl_c_EmergencyCode;

//flash-stored params declared in Main.c
extern unsigned short gl_ush_flashParamAmplitudeCode;       //��������� ��������� ������������ (��������)
extern unsigned short gl_ush_flashParamTactCode;            //��� ����� ���������
extern unsigned short gl_ush_flashParamMCoeff;              //����������� ���������
extern unsigned short gl_ush_flashParamStartMode;           //��������� ���� ������� ����������� ���������
extern unsigned short gl_ush_flashParamStartDecCoeff;       //��������� ����������� ������
extern unsigned short gl_ush_flashLockDev;                  //���� ���������� ����������

extern unsigned short gl_ush_flashParamI1min;               //����������� �������� ���� ������� I1
extern unsigned short gl_ush_flashParamI2min;               //����������� �������� ���� ������� I2
extern unsigned short gl_ush_flashParamAmplAngMin1;         //����������� �������� ��� ����������� �������� �� ������� � ����
//extern unsigned short flashParamHvApplyCount;     //���������� ������� ���������� 3kV ��� ������� � �����
//extern unsigned short flashParamHvApplyDurat;     //������������ ������� ���������� 3kV ��� ������� [����]
//extern unsigned short flashParamHvApplyPacks;     //���������� ����� ������� �������

extern unsigned short gl_ush_flashParamSignCoeff;          //�������� �����������
extern unsigned short gl_ush_flashParamDeviceId;           //ID ����������
extern unsigned short gl_ush_flashParamDateYear;           //���� ?? �������: ���
extern unsigned short gl_ush_flashParamDateMonth;          //���� ?? �������: �����
extern unsigned short gl_ush_flashParamDateDay;            //���� ?? �������: ����
extern char gl_ac_flashParamOrg[];                         //�������� �����������

//OBSOLETE? ��������� �� �����������
extern unsigned short gl_ushFlashParamLastRULA;
extern unsigned short gl_ushFlashParamLastRULM;

//������ ���������� ���������� (���.) ����������
extern unsigned short gl_aushListOutputAddParams[];

//���������� �������������
extern signed short   gl_ssh_flashParam_calibT1;
extern unsigned short gl_ush_flashParamT1_TD1_val, gl_ush_flashParamT1_TD2_val, gl_ush_flashParamT1_TD3_val;
extern signed short   gl_ssh_flashParam_calibT2;
extern unsigned short gl_ush_flashParamT2_TD1_val, gl_ush_flashParamT2_TD2_val, gl_ush_flashParamT2_TD3_val;
extern short gl_shFlashParamTCalibUsage;          //���� ������������� ���������� �������������: 0 - ������������, REST (����������� 0xFF) - �� ������������

//���������� �������� ������
extern char gl_ac_calib_phsh_t[];                 //��������� �� ������ ����� ���������� �������� ������
extern char gl_ac_calib_phsh_phsh[];              //��������� �� ������ ����� �������� �������� ������, ��������������� ������������, ��������� ����
extern char gl_cFlashParamPhaseShiftUsage;        //���� ������������� �������� ������:
                                                  //      0 - ������������
                                                  //      REST (����������� 0xFF) - �� ������������

//���������� ������������ ������
extern char gl_ac_calib_dc_t[];                   //������ ����� ���������� ���������� ������������ ������
extern unsigned short gl_ush_calib_dc_dc[];       //������ ����� �������� ������������ ������, ��������������� ������������, ��������� ����
extern unsigned char  gl_ucDcUsageStartSetting;   //���� ��� ����� � �������� ���������� �������: 0=DC_START; REST=�� ������� ����������;
extern unsigned char  gl_ucDcUsageRecalc;         //���� ��� ������������� �������: 0=��������������;1=�� ������� ����������(�����);2=�� ������� ����������(�������);REST=������ �����;
extern unsigned short gl_ushDcUsageRecalcPeriod;  //������ �������������� ������� (� ��������)


void load_params_p1( void) {
  //��� ���������
  if( flashEE_load_short( ADDR_AMPLITUDE, &gl_ush_flashParamAmplitudeCode))  gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //��� ����� ���������
  if( flashEE_load_short( ADDR_TACT_CODE, &gl_ush_flashParamTactCode))       gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //����������� �
  if( flashEE_load_short( ADDR_M_COEFF, &gl_ush_flashParamMCoeff))           gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //��������� ����
  if( flashEE_load_short( ADDR_START_MODE, &gl_ush_flashParamStartMode))     gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //��������� ����������� ������
  if( flashEE_load_short( ADDR_DEC_COEFF_START, &gl_ush_flashParamStartDecCoeff))  gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //���� ���������� ����������
  if( flashEE_load_short( ADDR_LOCK_DEV, &gl_ush_flashLockDev))              gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //��������� RULA
  if( flashEE_load_short( ADDR_LAST_RULA, &gl_ushFlashParamLastRULA))        gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //��������� RULM
  if( flashEE_load_short( ADDR_LAST_RULM, &gl_ushFlashParamLastRULM))        gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;


#ifdef DEBUG
  printf("DBG: load_params(): params loaded from flash memory. Here they are:\n");
  printf("DBG:   Amplitude Code: 0x%04x (%04d)\n", gl_ush_flashParamAmplitudeCode,  gl_ush_flashParamAmplitudeCode);  //��� ���������
  printf("DBG:   Base Tact Code: 0x%04x (%04d)\n", gl_ush_flashParamTactCode,       gl_ush_flashParamTactCode);       //��� ����� ���������
  printf("DBG:   M Coefficient:  0x%04x (%04d)\n", gl_ush_flashParamMCoeff,         gl_ush_flashParamMCoeff);         //����������� �
  printf("DBG:   Start Mode:     0x%04x (%04d)\n", gl_ush_flashParamStartMode,      gl_ush_flashParamStartMode);      //��������� ����
  printf("DBG:   StartDecCoeff:  0x%04x (%04d)\n", gl_ush_flashParamStartDecCoeff,  gl_ush_flashParamStartDecCoeff);  //����������� ������
  printf("DBG:   Dev Lock:       0x%04x (%04d)\n", gl_ush_flashLockDev,             gl_ush_flashLockDev);             //���� ���������� ����������
  printf("DBG:   Last RULA:      0x%04x (%04d)\n", gl_ushFlashParamLastRULA, gl_ushFlashParamLastRULA);   //��������� RULA
  printf("DBG:   Last RULM:      0x%04x (%04d)\n", gl_ushFlashParamLastRULM, gl_ushFlashParamLastRULM);   //��������� RULM
#endif
}

void load_params_p2( void) {
  //����������� ��� I1
  if( flashEE_load_short( ADDR_CONTROL_I1, &gl_ush_flashParamI1min)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����������� ��� I2
  if( flashEE_load_short( ADDR_CONTROL_I2, &gl_ush_flashParamI2min)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����������� AmplAng
  if( flashEE_load_short( ADDR_CONTROL_AA, &gl_ush_flashParamAmplAngMin1)) {
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
  printf("DBG:   Control I1:     0x%04x (%04d)\n", gl_ush_flashParamI1min, gl_ush_flashParamI1min);             //����������� ��� I1
  printf("DBG:   Control I2:     0x%04x (%04d)\n", gl_ush_flashParamI2min, gl_ush_flashParamI2min);             //����������� ��� I2
  printf("DBG:   Control AA:     0x%04x (%04d)\n", gl_ush_flashParamAmplAngMin1, gl_ush_flashParamAmplAngMin1); //����������� AmplAng
  //printf("DBG:   HV_count:       0x%04x (%04d)\n", flashParamHvApplyCount, flashParamHvApplyCount);   //HV_applies tries amount in pack
  //printf("DBG:   HV_duration:    0x%04x (%04d)\n", flashParamHvApplyDurat, flashParamHvApplyDurat);   //HV_applies tries duration
  //printf("DBG:   HV_packs:       0x%04x (%04d)\n", flashParamHvApplyPacks, flashParamHvApplyPacks);   //HV_applies tries packs
#endif
}

void load_params_p3( void) {
  //�������� �����������
  if( flashEE_load_short( ADDR_SIGN_COEFF, &gl_ush_flashParamSignCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //�������� �����
  if( flashEE_load_short( ADDR_DEVICE_ID, &gl_ush_flashParamDeviceId)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����.���
  if( flashEE_load_short( ADDR_DATE_Y, &gl_ush_flashParamDateYear)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����.�����
  if( flashEE_load_short( ADDR_DATE_M, &gl_ush_flashParamDateMonth)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����.����
  if( flashEE_load_short( ADDR_DATE_D, &gl_ush_flashParamDateDay)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //�����������
  if( flashEE_load_text( ADDR_ORG, gl_ac_flashParamOrg, 16)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

#ifdef DEBUG
  printf("DBG:load_params_p3()\n");
  printf("DBG:   Sign coeff:      0x%04x (%04d)\n", gl_ush_flashParamSignCoeff, gl_ush_flashParamSignCoeff);  //�������� �����������
  printf("DBG:   Serial number:   0x%04x (%04d)\n", gl_ush_flashParamDeviceId,  gl_ush_flashParamDeviceId);   //�������� �����
  printf("DBG:   Organization:    '%s'\n", gl_ac_flashParamOrg);                                              //�����������
  printf("DBG:   Year:            0x%04x (%04d)\n", gl_ush_flashParamDateYear,  gl_ush_flashParamDateYear);   //���
  printf("DBG:   Month:           0x%04x (%04d)\n", gl_ush_flashParamDateMonth, gl_ush_flashParamDateMonth);  //�����
  printf("DBG:   Day:             0x%04x (%04d)\n", gl_ush_flashParamDateDay,   gl_ush_flashParamDateDay);    //����
#endif
}

void load_params_p4( void) {
  unsigned short ush_tmp;
#ifdef DEBUG
  int i;
#endif
  //������ ���������� ������ ���������� (���.) ����������
  //������ ���������� ������ ���������� (�������������� ����������). ������� 0
  if( flashEE_load_short( ADDR_LIST_PARAM1, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_aushListOutputAddParams[0] = ush_tmp;
  //������ ���������� ������ ���������� (�������������� ����������). ������� 1
  if( flashEE_load_short( ADDR_LIST_PARAM2, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_aushListOutputAddParams[1] = ush_tmp;
  //������ ���������� ������ ���������� (�������������� ����������). ������� 2
  if( flashEE_load_short( ADDR_LIST_PARAM3, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_aushListOutputAddParams[2] = ush_tmp;
  //������ ���������� ������ ���������� (�������������� ����������). ������� 3
  if( flashEE_load_short( ADDR_LIST_PARAM4, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_aushListOutputAddParams[3] = ush_tmp;
  //������ ���������� ������ ���������� (�������������� ����������). ������� 4
  if( flashEE_load_short( ADDR_LIST_PARAM5, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_aushListOutputAddParams[4] = ush_tmp;
  //������ ���������� ������ ���������� (�������������� ����������). ������� 5
  if( flashEE_load_short( ADDR_LIST_PARAM6, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_aushListOutputAddParams[5] = ush_tmp;
  //������ ���������� ������ ���������� (�������������� ����������). ������� 6
  if( flashEE_load_short( ADDR_LIST_PARAM7, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_aushListOutputAddParams[6] = ush_tmp;
  //������ ���������� ������ ���������� (�������������� ����������). ������� 7
  if( flashEE_load_short( ADDR_LIST_PARAM8, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_aushListOutputAddParams[7] = ush_tmp;
  //������ ���������� ������ ���������� (�������������� ����������). ������� 8
  if( flashEE_load_short( ADDR_LIST_PARAM9, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_aushListOutputAddParams[8] = ush_tmp;
  //������ ���������� ������ ���������� (�������������� ����������). ������� 9
  if( flashEE_load_short( ADDR_LIST_PARAM10, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_aushListOutputAddParams[9] = ush_tmp;
  //������ ���������� ������ ���������� (�������������� ����������). ������� 10
  if( flashEE_load_short( ADDR_LIST_PARAM11, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_aushListOutputAddParams[10] = ush_tmp;
  //������ ���������� ������ ���������� (�������������� ����������). ������� 11
  if( flashEE_load_short( ADDR_LIST_PARAM12, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_aushListOutputAddParams[11] = ush_tmp;


  //���������� �������������
  //����������� ����������� ����� ����������
  if( flashEE_load_short( ADDR_TCALIB_T1, ( unsigned short *) &gl_ssh_flashParam_calibT1)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������� ������� ������������ ��� ����������� ����������� ����������
  if( flashEE_load_short( ADDR_TCALIB_T1_TD1, &gl_ush_flashParamT1_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������� ������� ������������ ��� ����������� ����������� ����������
  if( flashEE_load_short( ADDR_TCALIB_T1_TD2, &gl_ush_flashParamT1_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������� �������� ������������ ��� ����������� ����������� ����������
  if( flashEE_load_short( ADDR_TCALIB_T1_TD3, &gl_ush_flashParamT1_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //����������� ������������ ����� ����������
  if( flashEE_load_short( ADDR_TCALIB_T2, ( unsigned short *) &gl_ssh_flashParam_calibT2)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������� ������� ������������ ��� ������������ ����������� ����������
  if( flashEE_load_short( ADDR_TCALIB_T2_TD1, &gl_ush_flashParamT2_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������� ������� ������������ ��� ������������ ����������� ����������
  if( flashEE_load_short( ADDR_TCALIB_T2_TD2, &gl_ush_flashParamT2_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������� ������� ������������ ��� ������������ ����������� ����������
  if( flashEE_load_short( ADDR_TCALIB_T2_TD3, &gl_ush_flashParamT2_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //������������� ������������� ���������� �������������
  if( flashEE_load_short( ADDR_TCALIB_USAGE, ( unsigned short *) &gl_shFlashParamTCalibUsage)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  //���������� �������� ������
  //���������� �������� ������. ����� 1. �����������.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T1, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_t[0] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 1. ������� �����
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH1, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_phsh[0] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 2. �����������.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T2, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_t[1] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 2. ������� �����
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH2, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_phsh[1] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 3. �����������.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T3, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_t[2] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 3. ������� �����
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH3, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_phsh[2] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 4. �����������.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T4, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_t[3] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 4. ������� �����
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH4, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_phsh[3] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 5. �����������.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T5, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_t[4] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 5. ������� �����
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH5, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_phsh[4] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 6. �����������.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T6, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_t[5] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 6. ������� �����
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH6, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_phsh[5] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 7. �����������.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T7, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_t[6] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 7. ������� �����
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH7, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_phsh[6] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 8. �����������.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T8, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_t[7] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 8. ������� �����
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH8, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_phsh[7] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 9. �����������.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T9, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_t[8] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 9. ������� �����
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH9, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_phsh[8] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 10. �����������.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T10, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_t[9] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 10. ������� �����
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH10, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_phsh[9] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 11. �����������.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T11, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_t[10] = ush_tmp & 0xFF;
  //���������� �������� ������. ����� 11. ������� �����
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH11, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_phsh_phsh[10] = ush_tmp & 0xFF;
  //������������� ���������� �������� ������
  if( flashEE_load_short( ADDR_PHSH_CALIB_USAGE, ( unsigned short *) &gl_cFlashParamPhaseShiftUsage)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;


  //���������� ������������ ������
  //���������� ������������ ������. ����� 1. �����������.
  if( flashEE_load_short( ADDR_DC_CALIB_T1, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_dc_t[0] = ush_tmp & 0xFF;
  //���������� ������������ ������. ����� 1. ����������� ������
  if( flashEE_load_short( ADDR_DC_CALIB_DC1, ( unsigned short *) &gl_ush_calib_dc_dc[0])) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;

  //���������� ������������ ������. ����� 2. �����������.
  if( flashEE_load_short( ADDR_DC_CALIB_T2, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_dc_t[1] = ush_tmp & 0xFF;
  //���������� ������������ ������. ����� 2. ����������� ������
  if( flashEE_load_short( ADDR_DC_CALIB_DC2, ( unsigned short *) &gl_ush_calib_dc_dc[1])) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;

  //���������� ������������ ������. ����� 3. �����������.
  if( flashEE_load_short( ADDR_DC_CALIB_T3, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_dc_t[2] = ush_tmp & 0xFF;
  //���������� ������������ ������. ����� 3. ����������� ������
  if( flashEE_load_short( ADDR_DC_CALIB_DC3, ( unsigned short *) &gl_ush_calib_dc_dc[2])) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;

  //���������� ������������ ������. ����� 4. �����������.
  if( flashEE_load_short( ADDR_DC_CALIB_T4, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_dc_t[3] = ush_tmp & 0xFF;
  //���������� ������������ ������. ����� 4. ����������� ������
  if( flashEE_load_short( ADDR_DC_CALIB_DC4, ( unsigned short *) &gl_ush_calib_dc_dc[3])) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;

  //���������� ������������ ������. ����� 5. �����������.
  if( flashEE_load_short( ADDR_DC_CALIB_T5, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_dc_t[4] = ush_tmp & 0xFF;
  //���������� ������������ ������. ����� 5. ����������� ������
  if( flashEE_load_short( ADDR_DC_CALIB_DC5, ( unsigned short *) &gl_ush_calib_dc_dc[4])) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;

  //���������� ������������ ������. ����� 6. �����������.
  if( flashEE_load_short( ADDR_DC_CALIB_T6, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_dc_t[5] = ush_tmp & 0xFF;
  //���������� ������������ ������. ����� 6. ����������� ������
  if( flashEE_load_short( ADDR_DC_CALIB_DC6, ( unsigned short *) &gl_ush_calib_dc_dc[5])) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;

  //���������� ������������ ������. ����� 7. �����������.
  if( flashEE_load_short( ADDR_DC_CALIB_T7, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_dc_t[6] = ush_tmp & 0xFF;
  //���������� ������������ ������. ����� 7. ����������� ������
  if( flashEE_load_short( ADDR_DC_CALIB_DC7, ( unsigned short *) &gl_ush_calib_dc_dc[6])) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;

  //���������� ������������ ������. ����� 8. �����������.
  if( flashEE_load_short( ADDR_DC_CALIB_T8, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_dc_t[7] = ush_tmp & 0xFF;
  //���������� ������������ ������. ����� 8. ����������� ������
  if( flashEE_load_short( ADDR_DC_CALIB_DC8, ( unsigned short *) &gl_ush_calib_dc_dc[7])) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;

  //���������� ������������ ������. ����� 9. �����������.
  if( flashEE_load_short( ADDR_DC_CALIB_T9, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_dc_t[8] = ush_tmp & 0xFF;
  //���������� ������������ ������. ����� 9. ����������� ������
  if( flashEE_load_short( ADDR_DC_CALIB_DC9, ( unsigned short *) &gl_ush_calib_dc_dc[8])) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;

  //���������� ������������ ������. ����� 10. �����������.
  if( flashEE_load_short( ADDR_DC_CALIB_T10, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_dc_t[9] = ush_tmp & 0xFF;
  //���������� ������������ ������. ����� 10. ����������� ������
  if( flashEE_load_short( ADDR_DC_CALIB_DC10, ( unsigned short *) &gl_ush_calib_dc_dc[9])) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;

  //���������� ������������ ������. ����� 11. �����������.
  if( flashEE_load_short( ADDR_DC_CALIB_T11, ( unsigned short *) &ush_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ac_calib_dc_t[10] = ush_tmp & 0xFF;
  //���������� ������������ ������. ����� 11. ����������� ������
  if( flashEE_load_short( ADDR_DC_CALIB_DC11, ( unsigned short *) &gl_ush_calib_dc_dc[10])) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;


  //��� ����� � �������� ���������� �������
  if( flashEE_load_short( ADDR_DC_START_DEF, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ucDcUsageStartSetting = ush_tmp & 0xFF;

  //��� �������������� ������� � �������� ������
  if( flashEE_load_short( ADDR_DC_RECALC, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_ucDcUsageRecalc = ush_tmp & 0xFF;

  //������ �������������� �������
  if( flashEE_load_short( ADDR_DC_RECALC_PERIOD, ( unsigned short *) &gl_ushDcUsageRecalcPeriod)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;

#ifdef DEBUG
  printf("DBG:load_params_p4()\n");
  for( i=0; i<12; i++) {
    printf("DBG:  ADDR_LIST_PARAM%d:    0x%04x (%04d)\n",   i+1, gl_aushListOutputAddParams[i], gl_aushListOutputAddParams[i]);
  }

  printf("DBG:   T-Calibration T1=0x%04x (%04d)\n", gl_ssh_flashParam_calibT1,   gl_ssh_flashParam_calibT1);    //������������� ����������: ����������� ������ �����
  printf("DBG:   T1_TD1:          0x%04x (%04d)\n", gl_ush_flashParamT1_TD1_val, gl_ush_flashParamT1_TD1_val);  //������������� ����������: ��������� ������������ TD1 � ������ ������������� �����
  printf("DBG:   T1_TD2:          0x%04x (%04d)\n", gl_ush_flashParamT1_TD2_val, gl_ush_flashParamT1_TD2_val);  //������������� ����������: ��������� ������������ TD2 � ������ ������������� �����
  printf("DBG:   T1_TD3:          0x%04x (%04d)\n", gl_ush_flashParamT1_TD3_val, gl_ush_flashParamT1_TD3_val);  //������������� ����������: ��������� ������������ TD3 � ������ ������������� �����
  printf("DBG:   T-Calibration T2=0x%04x (%04d)\n", gl_ssh_flashParam_calibT2,   gl_ssh_flashParam_calibT2);    //������������� ����������: ����������� ������� �����
  printf("DBG:   T1_TD1:          0x%04x (%04d)\n", gl_ush_flashParamT2_TD1_val, gl_ush_flashParamT2_TD1_val);  //������������� ����������: ��������� ������������ TD1 � ������� ������������� �����
  printf("DBG:   T1_TD2:          0x%04x (%04d)\n", gl_ush_flashParamT2_TD2_val, gl_ush_flashParamT2_TD2_val);  //������������� ����������: ��������� ������������ TD2 � ������� ������������� �����
  printf("DBG:   T1_TD3:          0x%04x (%04d)\n", gl_ush_flashParamT2_TD3_val, gl_ush_flashParamT2_TD3_val);  //������������� ����������: ��������� ������������ TD3 � ������� ������������� �����
  printf("DBG:   USAGE:           0x%04x (%04d)\n", gl_shFlashParamTCalibUsage, gl_shFlashParamTCalibUsage);  //���� ������������� ���������� �������������: 0 - ������������, REST (����������� 0xFF) - �� ������������

  for( i=0; i<11; i++) {
    printf("DBG:  ADDR_PHSH_CALIB_T%d:    0x%04x (%04d)\n",   i+1, gl_ac_calib_phsh_t[i], gl_ac_calib_phsh_t[i]);
    printf("DBG:  ADDR_PHSH_CALIB_PHSH%d: 0x%04x (%04d)\n\n", i+1, gl_ac_calib_phsh_phsh[i], gl_ac_calib_phsh_phsh[i]);
  }
  printf("DBG:  PHASE_SHIFT_USAGE:      0x%04x (%04d)\n\n",   gl_cFlashParamPhaseShiftUsage, gl_cFlashParamPhaseShiftUsage);

  for( i=0; i<11; i++) {
    printf("DBG:  ADDR_DC_CALIB_T%d:    0x%04x (%04d)\n",     i+1, gl_ac_calib_dc_t[i], gl_ac_calib_dc_t[i]);
    printf("DBG:  ADDR_DC_CALIB_DC%d:   0x%04x (%04d)\n\n",   i+1, gl_ush_calib_dc_dc[i], gl_ush_calib_dc_dc[i]);
  }

  printf("DBG:  ADDR_DC_START_DEF:    0x%04x (%04d)\n\n",     gl_ucDcUsageStartSetting, gl_ucDcUsageStartSetting);
  printf("DBG:  ADDR_DC_RECALC:       0x%04x (%04d)\n\n",     gl_ucDcUsageRecalc, gl_ucDcUsageRecalc);
  printf("DBG:  ADDR_DC_RECALC_PERIOD:0x%04x (%04d)\n\n",     gl_ushDcUsageRecalcPeriod, gl_ushDcUsageRecalcPeriod);

  printf("DBG:load_params_p4(): out\n");
#endif
}

void check_params_p1( void) {
  //��� ��������� [0-255]. ��������� �������� 90
  //90 ��� �������� 35 ��� ����������
  if( gl_ush_flashParamAmplitudeCode > 255)
    gl_ush_flashParamAmplitudeCode = 35;

  //��� ����� ��������� [0-3]. ��������� �������� 0
  if( gl_ush_flashParamTactCode > 3)
    gl_ush_flashParamTactCode = 0;

  //����������� �[0-1] = �������� ��������� [0-250]
  //��������� �������� 200 (��� �������� M=0.8 � DAC1 = 0.8 * DAC0)
  if( gl_ush_flashParamMCoeff > 250)
    gl_ush_flashParamMCoeff = 200;

  //��������� ���� [0-250]. ��������� �������� 125 (��� �������� 1,25� �� DAC2)
  if( gl_ush_flashParamStartMode > 250)
    gl_ush_flashParamStartMode = 125;

  //����������� ������
  //default �������� 0,04 ��������!
  if( gl_ush_flashParamStartDecCoeff == 0xffff)
    gl_ush_flashParamStartDecCoeff = ( int) ( 0.04 * 655350.);

  //���� ���������� ����������
  //default �������� 0 - ����� �������������
  if( gl_ush_flashLockDev != 1)
    gl_ush_flashLockDev = 0;

  //��������� RULA [0-4095]
  if( gl_ushFlashParamLastRULA > 4095) {
    gl_ushFlashParamLastRULA = 3000;
  }

  //��������� RULM [0-4095]
  if( gl_ushFlashParamLastRULM > 4095) {
    gl_ushFlashParamLastRULM = 0;
  }

#ifdef DEBUG
  printf("DBG: check_params_p1(): params checked for the range. Here they are:\n");
  printf("DBG:   Amplitude Code: 0x%04x (%04d)\n", gl_ush_flashParamAmplitudeCode,  gl_ush_flashParamAmplitudeCode);  //��� ���������
  printf("DBG:   Base Tact Code: 0x%04x (%04d)\n", gl_ush_flashParamTactCode,       gl_ush_flashParamTactCode);       //��� ����� ���������
  printf("DBG:   M Coefficient:  0x%04x (%04d)\n", gl_ush_flashParamMCoeff,         gl_ush_flashParamMCoeff);         //����������� �
  printf("DBG:   Start Mode:     0x%04x (%04d)\n", gl_ush_flashParamStartMode,      gl_ush_flashParamStartMode);      //��������� ����
  printf("DBG:   StartDecCoeff:  0x%04x (%04d)\n", gl_ush_flashParamStartDecCoeff,  gl_ush_flashParamStartDecCoeff);  //��������� ����������� ������
  printf("DBG:   Dev Lock:       0x%04x (%04d)\n", gl_ush_flashLockDev,             gl_ush_flashLockDev);             //���� ���������� ����������
#endif
}

void check_params_p2( void) {
  //����������� ��� ������� I1 [0-0.750 mA] = �������� ��������� [ 0 - 65534]
  //default �������� 0.4 mA
  if( gl_ush_flashParamI1min == 0xffff)
    gl_ush_flashParamI1min = ( short) ( 65535. * 0.4 / 0.75);

  //����������� ��� ������� I2 [0-0.750 mA] = �������� ��������� [ 0 - 65534]
  //default �������� 0.4 mA
  if( gl_ush_flashParamI2min == 0xffff)
    gl_ush_flashParamI2min = ( short) ( 65535. * 0.4 / 0.75);

  //����������� �������� �������� - ��������� ���� (0-3�) = �������� ��������� [ 0 - 65534]
  //default �������� 1.0�
  if( gl_ush_flashParamAmplAngMin1 == 0xffff)
    gl_ush_flashParamAmplAngMin1 = ( int) ( 1.0 / 3. * 65535.);

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
  printf("DBG:   Control I1:     0x%04x (%04d)\n", gl_ush_flashParamI1min, gl_ush_flashParamI1min);               //����������� ��� I1
  printf("DBG:   Control I2:     0x%04x (%04d)\n", gl_ush_flashParamI2min, gl_ush_flashParamI2min);               //����������� ��� I2
  printf("DBG:   Control AA:     0x%04x (%04d)\n", gl_ush_flashParamAmplAngMin1, gl_ush_flashParamAmplAngMin1);   //����������� AmplAng
  //printf("DBG:   HV_count:       0x%04x (%04d)\n", flashParamHvApplyCount, flashParamHvApplyCount);   //HV_applies tries amount
  //printf("DBG:   HV_duration:    0x%04x (%04d)\n", flashParamHvApplyDurat, flashParamHvApplyDurat);   //HV_applies tries duration
  //printf("DBG:   HV_packs:       0x%04x (%04d)\n", flashParamHvApplyPacks, flashParamHvApplyPacks);   //HV_applies tries packs
#endif
}

void check_params_p3( void) {
  int i;
  //�������� �����������. [-1; +1]
  //default value = 1
  if( gl_ush_flashParamSignCoeff > 2)
    gl_ush_flashParamSignCoeff = 2;

  //ID ����������
  if( gl_ush_flashParamDeviceId == 65535)
    gl_ush_flashParamDeviceId = 0;

  //���� ?? ����������
  //default value = 2016.01.01
  if( gl_ush_flashParamDateYear < 2000 || gl_ush_flashParamDateYear > 2200)
    gl_ush_flashParamDateYear = 2016;

  if( gl_ush_flashParamDateMonth > 12)
    gl_ush_flashParamDateMonth = 1;

  if( gl_ush_flashParamDateDay > 31)
    gl_ush_flashParamDateDay = 1;

  //�������� �����������
  //default - ��� ������
  for( i=0; i<17; i++) {
    if( gl_ac_flashParamOrg[i] < 33 || gl_ac_flashParamOrg[i] > 126)
      gl_ac_flashParamOrg[i] = ' ';
  }



#ifdef DEBUG
  printf("DBG: check_params_p3(): params checked for the range. Here they are:\n");
  printf("DBG:   Sign coeff:     0x%04x (%04d)\n", gl_ush_flashParamSignCoeff, gl_ush_flashParamSignCoeff);   //�������� �����������
  printf("DBG:   Serial number:  0x%04x (%04d)\n", gl_ush_flashParamDeviceId,  gl_ush_flashParamDeviceId);    //�������� �����
  printf("DBG:   Year:           0x%04x (%04d)\n", gl_ush_flashParamDateYear,  gl_ush_flashParamDateYear);    //���
  printf("DBG:   Month:          0x%04x (%04d)\n", gl_ush_flashParamDateMonth, gl_ush_flashParamDateMonth);   //�����
  printf("DBG:   Day:            0x%04x (%04d)\n", gl_ush_flashParamDateDay,   gl_ush_flashParamDateDay);     //����
  printf("DBG:   Organization:   '%s'\n", gl_ac_flashParamOrg);                                               //�����������
#endif
}

void check_params_p4( void) {
  unsigned short aush_tmp[12];
  int i, j, nCnt;

  char c_t;
  char c_phsh;
  unsigned short ush_dc;

  //������ ���������� (���.) ����������
  for( i=0; i<12; i++) {
    aush_tmp[i] = gl_aushListOutputAddParams[i];
    gl_aushListOutputAddParams[i] == 0xFF;
  }

  nCnt = 0;
  for( i=0; i<12; i++) {
    if( aush_tmp[i] != 0xFFFF && aush_tmp[i] != 0xFF) {
      gl_aushListOutputAddParams[nCnt++] = aush_tmp[i];
    }
  }

  if( nCnt == 0) {
    gl_aushListOutputAddParams[0] =  UTD1;
    gl_aushListOutputAddParams[1] =  AMPLANG_ALTERA;
    gl_aushListOutputAddParams[2] =  UTD3;
    gl_aushListOutputAddParams[3] =  I1;
    gl_aushListOutputAddParams[4] =  CNTRPC;
    gl_aushListOutputAddParams[5] =  I2;
    gl_aushListOutputAddParams[6] =  UTD2;
  }

  //������������� ����������
  if( gl_ssh_flashParam_calibT1 < ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION)  ||
      gl_ssh_flashParam_calibT1 > ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
    gl_ssh_flashParam_calibT1 = 0;
    gl_ush_flashParamT1_TD1_val = 0;
    gl_ush_flashParamT1_TD2_val = 1;
    gl_ush_flashParamT1_TD3_val = 2;
  }

  if( gl_ssh_flashParam_calibT2 < ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) ||
      gl_ssh_flashParam_calibT2 > ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
    gl_ssh_flashParam_calibT2 = 0;
    gl_ush_flashParamT2_TD1_val = 0;
    gl_ush_flashParamT2_TD2_val = 1;
    gl_ush_flashParamT2_TD3_val = 2;
  }

  //���������� ������
  for( i=0; i<10; i++) {
    for( j=0; j<10; j++) {

      //���������� �������� ������
      if( gl_ac_calib_phsh_t[ j] > gl_ac_calib_phsh_t[ j + 1]) {
        c_t = gl_ac_calib_phsh_t[ j + 1];
        gl_ac_calib_phsh_t[ j + 1] = gl_ac_calib_phsh_t[ j];
        gl_ac_calib_phsh_t[ j] = c_t;

        c_phsh = gl_ac_calib_phsh_phsh[ j + 1];
        gl_ac_calib_phsh_phsh[ j + 1] = gl_ac_calib_phsh_phsh[ j];
        gl_ac_calib_phsh_phsh[ j] = c_phsh;
      }

      //���������� ������������ ������
      if( gl_ac_calib_dc_t[ j] > gl_ac_calib_dc_t[ j + 1]) {
        c_t = gl_ac_calib_dc_t[ j + 1];
        gl_ac_calib_dc_t[ j + 1] = gl_ac_calib_dc_t[ j];
        gl_ac_calib_dc_t[ j] = c_t;

        ush_dc = gl_ush_calib_dc_dc[ j + 1];
        gl_ush_calib_dc_dc[ j + 1] = gl_ush_calib_dc_dc[ j];
        gl_ush_calib_dc_dc[ j] = ush_dc;
      }
    }
  }

  //������� �������� ��� �� ����� ������������ ������� ����� (�.�. � ��� ���� ���� �� ���� ����� ����������)
  /*
  for( i=0; i<11; i++) {
    if( gl_ac_calib_phsh_t[i] != 0xFFFF && gl_ac_calib_phsh_phsh[i] != 0xFF) {
      gl_cPhaseShiftCalibrated = 1;
      break;
    }
  }
  */

  //���� ������������� �������� ������: 0 - ������������, REST - ������ �����
  if( gl_cFlashParamPhaseShiftUsage > 1) gl_cFlashParamPhaseShiftUsage = 0x01;

  //���� ��� ����� � �������� ���������� �������: 0=DC_START; REST=�� ������� ����������;
  if( gl_ucDcUsageStartSetting > 1) gl_ucDcUsageStartSetting = 0x00;

  //���� ��� ������������� �������: 0=��������������;1=�� ������� ����������(�����);2=�� ������� ����������(�������);REST=������ �����;
  if( gl_ucDcUsageRecalc > 3) gl_ucDcUsageRecalc = 0x00;

  //������ �������������� ������� (� ��������) (min 1 max 600)
  if( gl_ushDcUsageRecalcPeriod < 1) gl_ushDcUsageRecalcPeriod = 1;
  if( gl_ushDcUsageRecalcPeriod > 600) gl_ushDcUsageRecalcPeriod = 600;


#ifdef DEBUG
  printf("DBG: check_params_p4(): add params list checked for the range. Here they are:\n");
  printf("DBG:   ADD_PARAM_LIST_01:     0x%04x (%04d)\n", gl_aushListOutputAddParams[0],  gl_aushListOutputAddParams[0]);  //������ ���������� ���������� (���. ����������)
  printf("DBG:   ADD_PARAM_LIST_02:     0x%04x (%04d)\n", gl_aushListOutputAddParams[1],  gl_aushListOutputAddParams[1]);  //������ ���������� ���������� (���. ����������)
  printf("DBG:   ADD_PARAM_LIST_03:     0x%04x (%04d)\n", gl_aushListOutputAddParams[2],  gl_aushListOutputAddParams[2]);  //������ ���������� ���������� (���. ����������)
  printf("DBG:   ADD_PARAM_LIST_04:     0x%04x (%04d)\n", gl_aushListOutputAddParams[3],  gl_aushListOutputAddParams[3]);  //������ ���������� ���������� (���. ����������)
  printf("DBG:   ADD_PARAM_LIST_05:     0x%04x (%04d)\n", gl_aushListOutputAddParams[4],  gl_aushListOutputAddParams[4]);  //������ ���������� ���������� (���. ����������)
  printf("DBG:   ADD_PARAM_LIST_06:     0x%04x (%04d)\n", gl_aushListOutputAddParams[5],  gl_aushListOutputAddParams[5]);  //������ ���������� ���������� (���. ����������)
  printf("DBG:   ADD_PARAM_LIST_07:     0x%04x (%04d)\n", gl_aushListOutputAddParams[6],  gl_aushListOutputAddParams[6]);  //������ ���������� ���������� (���. ����������)
  printf("DBG:   ADD_PARAM_LIST_08:     0x%04x (%04d)\n", gl_aushListOutputAddParams[7],  gl_aushListOutputAddParams[7]);  //������ ���������� ���������� (���. ����������)
  printf("DBG:   ADD_PARAM_LIST_09:     0x%04x (%04d)\n", gl_aushListOutputAddParams[8],  gl_aushListOutputAddParams[8]);  //������ ���������� ���������� (���. ����������)
  printf("DBG:   ADD_PARAM_LIST_10:     0x%04x (%04d)\n", gl_aushListOutputAddParams[9],  gl_aushListOutputAddParams[9]);  //������ ���������� ���������� (���. ����������)
  printf("DBG:   ADD_PARAM_LIST_11:     0x%04x (%04d)\n", gl_aushListOutputAddParams[10], gl_aushListOutputAddParams[10]); //������ ���������� ���������� (���. ����������)
  printf("DBG:   ADD_PARAM_LIST_12:     0x%04x (%04d)\n", gl_aushListOutputAddParams[11], gl_aushListOutputAddParams[11]); //������ ���������� ���������� (���. ����������)

  printf("DBG: Temperature calibration parameters:\n");
  printf("DBG:   flashParam_calibT1:     0x%04x (%04d)\n", gl_ssh_flashParam_calibT1,   gl_ssh_flashParam_calibT1);
  printf("DBG:   flashParam_calibT1_TD1: 0x%04x (%04d)\n", gl_ush_flashParamT1_TD1_val, gl_ush_flashParamT1_TD1_val);
  printf("DBG:   flashParam_calibT1_TD2: 0x%04x (%04d)\n", gl_ush_flashParamT1_TD2_val, gl_ush_flashParamT1_TD2_val);
  printf("DBG:   flashParam_calibT1_TD3: 0x%04x (%04d)\n", gl_ush_flashParamT1_TD3_val, gl_ush_flashParamT1_TD3_val);
  printf("DBG:   flashParam_calibT2:     0x%04x (%04d)\n", gl_ssh_flashParam_calibT2,   gl_ssh_flashParam_calibT2);
  printf("DBG:   flashParam_calibT2_TD1: 0x%04x (%04d)\n", gl_ush_flashParamT2_TD1_val, gl_ush_flashParamT2_TD1_val);
  printf("DBG:   flashParam_calibT2_TD2: 0x%04x (%04d)\n", gl_ush_flashParamT2_TD2_val, gl_ush_flashParamT2_TD2_val);
  printf("DBG:   flashParam_calibT2_TD3: 0x%04x (%04d)\n", gl_ush_flashParamT2_TD3_val, gl_ush_flashParamT2_TD3_val);
  printf("DBG:   flashParam_calibUsage:  0x%04x (%04d)\n", gl_shFlashParamTCalibUsage,  gl_shFlashParamTCalibUsage);

  printf("DBG: Phase shift calibration parameters:\n");
  for( i=0; i<11; i++) {
    printf("DBG:   ADDR_PHSH_CALIB_T%d:    0x%04x (%04d)\n",   i+1, gl_ac_calib_phsh_t[i], gl_ac_calib_phsh_t[i]);
    printf("DBG:   ADDR_PHSH_CALIB_PHSH%d: 0x%04x (%04d)\n\n", i+1, gl_ac_calib_phsh_phsh[i], gl_ac_calib_phsh_phsh[i]);
  }
  printf("DBG:   flashParam_calibUsage:  0x%04x (%04d)\n", gl_cFlashParamPhaseShiftUsage, gl_cFlashParamPhaseShiftUsage);

  printf("DBG: Dec coeff calibration parameters:\n");
  for( i=0; i<11; i++) {
    printf("DBG:  ADDR_DC_CALIB_T%d:    0x%04x (%04d)\n",     i+1, gl_ac_calib_dc_t[i], gl_ac_calib_dc_t[i]);
    printf("DBG:  ADDR_DC_CALIB_DC%d:   0x%04x (%04d)\n\n",   i+1, gl_ush_calib_dc_dc[i], gl_ush_calib_dc_dc[i]);
  }

  printf("DBG:  ADDR_DC_START_DEF:    0x%04x (%04d)\n\n",     gl_ucDcUsageStartSetting, gl_ucDcUsageStartSetting);
  printf("DBG:  ADDR_DC_RECALC:       0x%04x (%04d)\n\n",     gl_ucDcUsageRecalc, gl_ucDcUsageRecalc);
  printf("DBG:  ADDR_DC_RECALC_PERIOD:0x%04x (%04d)\n\n",     gl_ushDcUsageRecalcPeriod, gl_ushDcUsageRecalcPeriod);

#endif
}

void load_params( void) {

#ifdef DEBUG
  printf("DBG: blp1: gl_c_EmergencyCode=0x%02X\n", gl_c_EmergencyCode);
#endif
  load_params_p1();

#ifdef DEBUG
  printf("DBG: blp2: gl_c_EmergencyCode=0x%02X\n", gl_c_EmergencyCode);
#endif
  if( gl_c_EmergencyCode != ERROR_FLASH_LOAD_PARAMS_FAIL) load_params_p2();

#ifdef DEBUG
  printf("DBG: blp3: gl_c_EmergencyCode=0x%02X\n", gl_c_EmergencyCode);
#endif
  if( gl_c_EmergencyCode != ERROR_FLASH_LOAD_PARAMS_FAIL) load_params_p3();

#ifdef DEBUG
  printf("DBG: blp4: gl_c_EmergencyCode=0x%02X\n", gl_c_EmergencyCode);
#endif
  if( gl_c_EmergencyCode != ERROR_FLASH_LOAD_PARAMS_FAIL) load_params_p4();



#ifdef DEBUG
  printf("DBG: bcp1: gl_c_EmergencyCode=0x%02X\n", gl_c_EmergencyCode);
#endif
  check_params_p1();

#ifdef DEBUG
  printf("DBG: bcp2: gl_c_EmergencyCode=0x%02X\n", gl_c_EmergencyCode);
#endif
  check_params_p2();

#ifdef DEBUG
  printf("DBG: bcp3: gl_c_EmergencyCode=0x%02X\n", gl_c_EmergencyCode);
#endif
  check_params_p3();

#ifdef DEBUG
  printf("DBG: bcp4: gl_c_EmergencyCode=0x%02X\n", gl_c_EmergencyCode);
#endif
  check_params_p4();
}

void save_params_p1( void) {
#ifdef DEBUG
  printf("DBG: save_params_p1(): params to be saved are:\n");
  printf("DBG:   Amplitude Code: 0x%04x (%04d)\n", gl_ush_flashParamAmplitudeCode,  gl_ush_flashParamAmplitudeCode);  //��� ���������
  printf("DBG:   Base Tact Code: 0x%04x (%04d)\n", gl_ush_flashParamTactCode,       gl_ush_flashParamTactCode);       //��� ����� ���������
  printf("DBG:   M Coefficient:  0x%04x (%04d)\n", gl_ush_flashParamMCoeff,         gl_ush_flashParamMCoeff);         //����������� �
  printf("DBG:   Start Mode:     0x%04x (%04d)\n", gl_ush_flashParamStartMode,      gl_ush_flashParamStartMode);      //��������� ����
  printf("DBG:   StartDecCoeff:  0x%04x (%04d)\n", gl_ush_flashParamStartDecCoeff,  gl_ush_flashParamStartDecCoeff);  //��������� ����������� ������
  printf("DBG:   Dev Lock:       0x%04x (%04d)\n", gl_ush_flashLockDev,             gl_ush_flashLockDev);             //���� ���������� ����������
  printf("DBG:   Last RULA:      0x%04x (%04d)\n", gl_ushFlashParamLastRULA, gl_ushFlashParamLastRULA);   //��������� RULA
  printf("DBG:   Last RULM:      0x%04x (%04d)\n", gl_ushFlashParamLastRULM, gl_ushFlashParamLastRULM);   //��������� RULM
#endif

  if( flashEE_erase_page( ADDR_PAGE1)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }

  if( flashEE_save_short( ADDR_AMPLITUDE, gl_ush_flashParamAmplitudeCode)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TACT_CODE, gl_ush_flashParamTactCode)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_M_COEFF, gl_ush_flashParamMCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_START_MODE, gl_ush_flashParamStartMode)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_DEC_COEFF_START, gl_ush_flashParamStartDecCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
#ifdef DEBUG
  if( gl_ush_flashLockDev == 1) {
    printf( "DBG: device will be locked\n");
  }
#endif
  if( flashEE_save_short( ADDR_LOCK_DEV, gl_ush_flashLockDev)) {
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
  printf("DBG:   Control I1:     0x%04x (%04d)\n", gl_ush_flashParamI1min, gl_ush_flashParamI1min);               //����������� ��� I1
  printf("DBG:   Control I2:     0x%04x (%04d)\n", gl_ush_flashParamI2min, gl_ush_flashParamI2min);               //����������� ��� I2
  printf("DBG:   Control AA:     0x%04x (%04d)\n", gl_ush_flashParamAmplAngMin1, gl_ush_flashParamAmplAngMin1);   //����������� AmplAng
  //printf("DBG:   HV_count:       0x%04x (%04d)\n", flashParamHvApplyCount, flashParamHvApplyCount);   //HV_applies tries amount
  //printf("DBG:   HV_duration:    0x%04x (%04d)\n", flashParamHvApplyDurat, flashParamHvApplyDurat);   //HV_applies tries duration
  //printf("DBG:   HV_packs:       0x%04x (%04d)\n", flashParamHvApplyPacks, flashParamHvApplyPacks);   //HV_applies tries packs
#endif

  if( flashEE_erase_page( ADDR_PAGE2)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }

  if( flashEE_save_short( ADDR_CONTROL_I1, gl_ush_flashParamI1min)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_CONTROL_I2, gl_ush_flashParamI2min)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_CONTROL_AA, gl_ush_flashParamAmplAngMin1)) {
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
  printf("DBG:   Sign coeff:     0x%04x (%04d)\n", gl_ush_flashParamSignCoeff, gl_ush_flashParamSignCoeff);   //�������� �����������
  printf("DBG:   Serial number:  0x%04x (%04d)\n", gl_ush_flashParamDeviceId,  gl_ush_flashParamDeviceId);    //�������� �����
  printf("DBG:   Organization:   '%s'\n", gl_ac_flashParamOrg);                                               //�����������
  printf("DBG:   Year:           0x%04x (%04d)\n", gl_ush_flashParamDateYear,  gl_ush_flashParamDateYear);    //���
  printf("DBG:   Month:          0x%04x (%04d)\n", gl_ush_flashParamDateMonth, gl_ush_flashParamDateMonth);   //�����
  printf("DBG:   Day:            0x%04x (%04d)\n", gl_ush_flashParamDateDay,   gl_ush_flashParamDateDay);     //����
#endif

  if( flashEE_erase_page( ADDR_PAGE3)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }

  if( flashEE_save_short( ADDR_SIGN_COEFF, gl_ush_flashParamSignCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_DEVICE_ID, gl_ush_flashParamDeviceId)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_DATE_Y, gl_ush_flashParamDateYear)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_DATE_M, gl_ush_flashParamDateMonth)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_DATE_D, gl_ush_flashParamDateDay)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_text( ADDR_ORG, gl_ac_flashParamOrg, 16)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
}

void save_params_p4( void) {
  int i;
  unsigned short ush_tmp;

#ifdef DEBUG
  printf("DBG: save_params_p4(): params to be saved are:\n");
  for( i=0; i<12; i++) {
    printf("DBG:  ADDR_LIST_PARAM%d:    0x%04x (%04d)\n",   i+1, gl_aushListOutputAddParams[i], gl_aushListOutputAddParams[i]);
  }

  printf("DBG: temperature calibration parameters:\n");

  printf("DBG: Phase shift calibration parameters:\n");
  for( i=0; i<11; i++) {
    printf("DBG:   ADDR_PHSH_CALIB_T%d:    0x%04x (%04d)\n",   i+1, gl_ac_calib_phsh_t[i], gl_ac_calib_phsh_t[i]);
    printf("DBG:   ADDR_PHSH_CALIB_PHSH%d: 0x%04x (%04d)\n\n", i+1, gl_ac_calib_phsh_phsh[i], gl_ac_calib_phsh_phsh[i]);
  }
  printf("DBG:   flashParam_calibUsage:  0x%04x (%04d)\n", gl_cFlashParamPhaseShiftUsage, gl_cFlashParamPhaseShiftUsage);

  printf("DBG: Dec coeff calibration parameters:\n");
  for( i=0; i<11; i++) {
    printf("DBG:  ADDR_DC_CALIB_T%d:    0x%04x (%04d)\n",     i+1, gl_ac_calib_dc_t[i], gl_ac_calib_dc_t[i]);
    printf("DBG:  ADDR_DC_CALIB_DC%d:   0x%04x (%04d)\n\n",   i+1, gl_ush_calib_dc_dc[i], gl_ush_calib_dc_dc[i]);
  }

  printf("DBG:  ADDR_DC_START_DEF:    0x%04x (%04d)\n\n",     gl_ucDcUsageStartSetting, gl_ucDcUsageStartSetting);
  printf("DBG:  ADDR_DC_RECALC:       0x%04x (%04d)\n\n",     gl_ucDcUsageRecalc, gl_ucDcUsageRecalc);
  printf("DBG:  ADDR_DC_RECALC_PERIOD:0x%04x (%04d)\n\n",     gl_ushDcUsageRecalcPeriod, gl_ushDcUsageRecalcPeriod);
#endif

  if( flashEE_erase_page( ADDR_PAGE4)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }

  //������ ���������� (���.) ����������
  for( i=0; i<12; i++) {
    if( flashEE_save_short( ADDR_LIST_PARAM1 + i * 2, gl_aushListOutputAddParams[ i])) {
      gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
      return;
    }
  }

  //���������� �������������
  if( flashEE_save_short( ADDR_TCALIB_T1, gl_ssh_flashParam_calibT1)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T1_TD1, gl_ush_flashParamT1_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T1_TD2, gl_ush_flashParamT1_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T1_TD3, gl_ush_flashParamT1_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T2, gl_ssh_flashParam_calibT2)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T2_TD1, gl_ush_flashParamT2_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T2_TD2, gl_ush_flashParamT2_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T2_TD3, gl_ush_flashParamT2_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_USAGE, gl_shFlashParamTCalibUsage)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }

  //���������� �������� ������
  for( i=0; i<11; i++) {
    ush_tmp = gl_ac_calib_phsh_t[ i] & 0xFF;
    if( flashEE_save_short( ADDR_PHSH_CALIB_T1 + i * 4, ush_tmp)) {
      gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
      return;
    }

    ush_tmp = gl_ac_calib_phsh_phsh[ i] & 0xFF;
    if( flashEE_save_short( ADDR_PHSH_CALIB_PHSH1 + i * 4, ush_tmp)) {
      gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
      return;
    }
  }

  if( flashEE_save_short( ADDR_PHSH_CALIB_USAGE, ( unsigned short) gl_cFlashParamPhaseShiftUsage)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }


  //���������� ������������ ������
  for( i=0; i<11; i++) {
    ush_tmp = gl_ac_calib_dc_t[ i] & 0xFF;
    if( flashEE_save_short( ADDR_DC_CALIB_T1 + i * 4, ush_tmp)) {
      gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
      return;
    }

    if( flashEE_save_short( ADDR_DC_CALIB_DC1 + i * 4, gl_ush_calib_dc_dc[ i])) {
      gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
      return;
    }
  }

  //��� ����� � �������� ���������� �������
  if( flashEE_save_short( ADDR_DC_START_DEF, ( unsigned short ) gl_ucDcUsageStartSetting)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }

  //��� �������������� ������� � �������� ������
  if( flashEE_save_short( ADDR_DC_RECALC, ( unsigned short ) gl_ucDcUsageRecalc)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }

  //������ �������������� �������
  if( flashEE_save_short( ADDR_DC_RECALC_PERIOD, ( unsigned short ) gl_ushDcUsageRecalcPeriod)) {
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