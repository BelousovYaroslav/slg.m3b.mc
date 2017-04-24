#include <ADuC7026.h>
#include <stdio.h>
#include "Main.h"
#include "settings.h"
#include "McCommands.h"
#include "AnalogueParamsConstList.h"
#include "serial.h"
#include "debug.h"

//����� �������� ������
extern char input_buffer[];                         //����� �������� ������
extern char pos_in_in_buf;                          //������� ������ � ������ �������� ������

//��������� �������� �� ����-������
extern unsigned short flashParamAmplitudeCode;      //��������� ��������� ������������
extern unsigned short flashParamTactCode;           //��� ����� ���������
extern unsigned short flashParamMCoeff;             //����������� ���������
extern unsigned short flashParamStartMode;          //��������� ���� ������� ����������� ���������
extern unsigned short flashParamDecCoeff;           //����������� ������
extern unsigned short flashLockDev;                 //���� ���������� ����������

extern unsigned short flashParamI1min;              //����������� �������� ���� ������� I1
extern unsigned short flashParamI2min;              //����������� �������� ���� ������� I2
extern unsigned short flashParamAmplAngMin1;        //����������� �������� ������� �������� � ����

extern unsigned short flashParamSignCoeff;          //�������� �����������
extern unsigned short flashParamDeviceId;             //ID ����������
extern unsigned short flashParamDateYear;           //���� ? �������.���
extern unsigned short flashParamDateMonth;          //���� ? �������.�����
extern unsigned short flashParamDateDay;            //���� ? �������.����
extern char flashParamOrg[];                        //�������� �����������

//���������� �������������
extern signed short flashParam_calibT1;
extern unsigned short flashParamT1_TD1_val, flashParamT1_TD2_val, flashParamT1_TD3_val;
extern signed short flashParam_calibT2;
extern unsigned short flashParamT2_TD1_val, flashParamT2_TD2_val, flashParamT2_TD3_val;
extern char gl_bCalibrated;
extern char gl_cCalibProcessState;

extern unsigned short gl_ushFlashParamLastRULA;     //��������� RULA (obsolete)
extern unsigned short gl_ushFlashParamLastRULM;     //��������� RULM (obsolete)

//������� ��������
extern int gl_nRppTimerT2;                          //������� ������� ��� ���������� ������ ����������� ������� ����������� ���������

extern char gl_c_EmergencyCode;                     //��� ������

extern int gl_snMeaningCounter;                     //Amplitude control module: counter of measured values
extern int gl_snMeaningCounterRound;                //Amplitude control module: round of measured values
extern int gl_snMeaningShift;                       //Amplitude control module: bits for shift to get mean
extern long gl_lnMeaningSumm;                       //Amplitude control module: summ of amplitudes
extern long gl_lnMeanImps;                          //Amplitude control module: mean (it's calculated shifted by 4 i.e. multiplied by 16)
extern int  gl_nActiveRegulationT2;                 //Amplitude control module: amplitude active regulation T2 intersection

//�����
extern char gl_b_SyncMode;                          //���� ������ ������ ���������:   0=�����. 1=������.
extern char gl_chAngleOutput;                       //���� ������ ���������� ����: 0 = dW (4b)         1 = dN (2b), dU(2b)
extern char gl_bSimpleDnDuRegime;                   //���� ������ "��������� �������"
                                                    //(������ dN,dU � ����������� ��������� ��������, ��� ������ ���������� ����������, ��� ���, ��� �������� ���������)
extern char gl_bCalibProcessState;                  //���� ���������� �������� ������������� ����������
extern char gl_bCalibrated;                         //���� ��������� ������������ ������������� ����������: 0=�� ����������, 1=����������
extern char gl_n_PerimeterReset;                    //���� ������� ������ ���������:
                                                    //0 = ������ ����������
                                                    //1 = ������ ������������, ������ ���������� �����������
                                                    //2 = ������ ������������, ������ ��������� �����������
extern char gl_chLockBit;                           //���� ������������ ����������: 0 - ����� "������������"   1 - ����� "������������"

extern unsigned int gl_un_RULAControl;              //��� ������� RULA ���������� � ���... [0 - 4095] ==> [0-2.5 �]


extern short gl_nSentPackIndex;                     //Analogue Parameter Index (what are we sending now)

//�������
extern void configure_hanger( void);                //������� ������� ���� ����� ���������
extern void DACConfiguration( void);                //������� �������� ���������� (�������� RULA, RULM) �� ��� 

//���������� ����������� � �������� ������������ ������ "�� ����"
extern double gl_dblMeanAbsDn;
extern double gl_dblMeanAbsDu;
extern int    gl_nMeanDecCoeffCounter;

void processIncomingCommand( void) {
  short in_param_temp;
  int i;

  //**********************************************************************
  // ��������� ������ �������� ������
  //**********************************************************************
#ifdef DEBUG
  if( pos_in_in_buf > 0) {
    //printf("DBG: PIC: in with %d\n", pos_in_in_buf);
    putchar_nocheck( '0');
    putchar_nocheck( '0' + pos_in_in_buf);
  }
#endif

  if( pos_in_in_buf == IN_COMMAND_BUF_LEN) {

#ifdef DEBUG
    putchar_nocheck( '1');
    printf("\nDBG: Incoming command: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                input_buffer[ 0], input_buffer[ 1], input_buffer[ 2],
                input_buffer[ 3], input_buffer[ 4], input_buffer[ 5]);
#endif

    //LOCK �������
    if( gl_chLockBit == 1) {
      switch( input_buffer[ 0]) {
        case MC_COMMAND_ACT_UNLOCK_DEVICE:
          if( input_buffer[ 1] == 0x5A &&
              input_buffer[ 2] == 0x55 &&
              input_buffer[ 3] == 0x5A) {
                gl_chLockBit = 0;
                flashLockDev = 0;
          }
        break;
        case MC_COMMAND_REQ:
          switch( input_buffer[ 1]) {
            case VERSION:     gl_nSentPackIndex = VERSION;      break;
          }
        break;
        default:
#ifdef DEBUG
          printf("DBG: Device is locked!\n");
#endif
        break;
      }
      pos_in_in_buf = 0;
      return;
    }

#ifdef DEBUG
    putchar_nocheck( '2');
#endif

    switch( input_buffer[0]) {
      case MC_COMMAND_SET:
        switch( input_buffer[1]) {
          case AMPLITUDE:   //Set Amplitude of Hangreup Vibration
            flashParamAmplitudeCode = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = AMPLITUDE;

            gl_snMeaningCounter = 0;
            gl_snMeaningCounterRound = 128;
            gl_snMeaningShift = 7;
            gl_lnMeaningSumm = 0;
            gl_lnMeanImps = 0;

            //�������� ���� �������� ����������� ��������� (������������� ���������)
            gl_nActiveRegulationT2 = T2VAL;
            if( gl_nActiveRegulationT2 == 0) gl_nActiveRegulationT2 = 1;

          break;

          case TACT_CODE:   //Set CodeTact
            flashParamTactCode = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            configure_hanger();
            gl_nSentPackIndex = TACT_CODE;

            //�������� ���� �������� ����������� ��������� (������������� ���������)
            gl_nActiveRegulationT2 = T2VAL;
            if( gl_nActiveRegulationT2 == 0) gl_nActiveRegulationT2 = 1;

          break;

          case M_COEFF: //Set NoiseCoefficient M
            flashParamMCoeff = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);

            //������������ �� ���
            DACConfiguration();
            gl_nSentPackIndex = M_COEFF;

            //�������� ���� �������� ����������� ��������� (������������� ���������)
            gl_nActiveRegulationT2 = T2VAL;
            if( gl_nActiveRegulationT2 == 0) gl_nActiveRegulationT2 = 1;
          break;

          case STARTMODE: //Set StartMode
            flashParamStartMode = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);

            GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1

            gl_n_PerimeterReset = 1;
            gl_nRppTimerT2 = T2VAL;
            gl_nSentPackIndex = STARTMODE;

            DACConfiguration();
          break;

          case DECCOEFF: //Set decrement coeff
            flashParamDecCoeff = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = DECCOEFF;

            //������� ���������� ���������� ����������� ��� ��������� �� "�� ����"
            gl_nMeanDecCoeffCounter = 0;
            gl_dblMeanAbsDn = 0.;
            gl_dblMeanAbsDu = 0.;
          break;

          case CONTROL_I1:  //Set control_i1
            flashParamI1min = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = CONTROL_I1;
          break;

          case CONTROL_I2:  //Set control_i2
            flashParamI2min = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = CONTROL_I2;
          break;

          case CONTROL_AA:  //Set control_aa
            flashParamAmplAngMin1 = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = CONTROL_AA;
          break;

          /*
          case HV_APPLY_COUNT_SET:  //Set HV_applies_count
            flashParamHvApplyCount = input_buffer[2];
            gl_nSentPackIndex = HV_APPLY_COUNT_SET;
          break;

          case HV_APPLY_DURAT_SET:  //Set HV_applies_duration
            flashParamHvApplyDurat = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = HV_APPLY_DURAT_SET;
          break;

          case HV_APPLY_PACKS:   //Set HV_applies_packs
            flashParamHvApplyPacks = input_buffer[2];
            gl_nSentPackIndex = HV_APPLY_PACKS;
          break;
          */

          case SIGNCOEFF:  //Set sign coeff
            flashParamSignCoeff = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = SIGNCOEFF;
          break;

          case DEVNUM:    //Set device
            flashParamDeviceId = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = DEVNUM;
          break;

          /*
          case DEVNUM_BH:  //Set device id high byte
            flashParamDeviceId &= ( 0xFF00);
            flashParamDeviceId &= ( ( ( short) input_buffer[2]) << 8);
            gl_nSentPackIndex = DEVNUM_BH;
          break;
          */

          case DATE_Y:    //Set Date.YEAR
            flashParamDateYear = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = DATE_Y;
          break;

          case DATE_M:    //Set Date.MONTH
            flashParamDateMonth = input_buffer[2];
            gl_nSentPackIndex = DATE_Y;
          break;

          case DATE_D:    //Set Date.DAY
            flashParamDateDay = input_buffer[2];
            gl_nSentPackIndex = DATE_Y;
          break;

          case ORG_B1:    //���������� Organization.byte1
            flashParamOrg[0] = input_buffer[2];  break;
          case ORG_B2:    //���������� Organization.byte2
            flashParamOrg[1] = input_buffer[2];  break;
          case ORG_B3:    //���������� Organization.byte3
            flashParamOrg[2] = input_buffer[2];  break;
          case ORG_B4:    //���������� Organization.byte4
            flashParamOrg[3] = input_buffer[2];  break;
          case ORG_B5:    //���������� Organization.byte5
            flashParamOrg[4] = input_buffer[2];  break;
          case ORG_B6:    //���������� Organization.byte6
            flashParamOrg[5] = input_buffer[2];  break;
          case ORG_B7:    //���������� Organization.byte7
            flashParamOrg[6] = input_buffer[2];  break;
          case ORG_B8:    //���������� Organization.byte8
            flashParamOrg[7] = input_buffer[2];  break;
          case ORG_B9:    //���������� Organization.byte9
            flashParamOrg[8] = input_buffer[2];  break;
          case ORG_B10:   //���������� Organization.byte10
            flashParamOrg[9] = input_buffer[2];  break;
          case ORG_B11:   //���������� Organization.byte11
            flashParamOrg[10] = input_buffer[2]; break;
          case ORG_B12:   //���������� Organization.byte12
            flashParamOrg[11] = input_buffer[2]; break;
          case ORG_B13:   //���������� Organization.byte13
            flashParamOrg[12] = input_buffer[2]; break;
          case ORG_B14:   //���������� Organization.byte14
            flashParamOrg[13] = input_buffer[2]; break;
          case ORG_B15:   //���������� Organization.byte15
            flashParamOrg[14] = input_buffer[2]; break;
          case ORG_B16:   //���������� Organization.byte16
            flashParamOrg[15] = input_buffer[2]; break;
        }
      break;

      case MC_COMMAND_REQ:
        switch( input_buffer[1]) {
          case AMPLITUDE:   gl_nSentPackIndex = AMPLITUDE;    break;
          case TACT_CODE:   gl_nSentPackIndex = TACT_CODE;    break;
          case M_COEFF:     gl_nSentPackIndex = M_COEFF;      break;
          case STARTMODE:   gl_nSentPackIndex = STARTMODE;    break;
          case DECCOEFF:    gl_nSentPackIndex = DECCOEFF;     break;
          case CONTROL_I1:  gl_nSentPackIndex = CONTROL_I1;   break;
          case CONTROL_I2:  gl_nSentPackIndex = CONTROL_I2;   break;
          case CONTROL_AA:  gl_nSentPackIndex = CONTROL_AA;   break;

          case HV_APPLY_COUNT_SET:  gl_nSentPackIndex = HV_APPLY_COUNT_SET;   break;
          case HV_APPLY_COUNT_TR:   gl_nSentPackIndex = HV_APPLY_COUNT_TR;    break;
          case HV_APPLY_DURAT_SET:  gl_nSentPackIndex = HV_APPLY_DURAT_SET;   break;
          case HV_APPLY_PACKS:      gl_nSentPackIndex = HV_APPLY_PACKS;       break;

          case SIGNCOEFF:   gl_nSentPackIndex = SIGNCOEFF;    break;
          case DEVNUM:      gl_nSentPackIndex = DEVNUM;       break;

          case DATE_Y:      gl_nSentPackIndex = DATE_Y;       break;
          case DATE_M:      gl_nSentPackIndex = DATE_M;       break;
          case DATE_D:      gl_nSentPackIndex = DATE_D;       break;

          case ORG:         gl_nSentPackIndex = ORG_B1;       break;

          case VERSION:
            //printf("DBG: PIC: REQ: VER\n");
            //putchar_nocheck( '3');
            gl_nSentPackIndex = VERSION;
          break;
        }
      break;



      case MC_COMMAND_ACT_SWC_DW_DNDU_OUTPUT: //Within async mode switch DN,DU or dW output
        if( gl_b_SyncMode == 1) {
          if( input_buffer[1] == 0) gl_chAngleOutput = 0; //switch to dW output
          if( input_buffer[1] == 1) gl_chAngleOutput = 1; //switch to dNdU output
        }
      break;

      case MC_COMMAND_ACT_T_CALIBRATION: //Thermo calibration (parameter here is current temperature)
        gl_bCalibrated = 0;
        in_param_temp  = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
        if( flashParam_calibT1 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) && 
            flashParam_calibT1 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
            //� ��� ���� ���������� ����������� ����� ����������

            //������� �� ��, ����� �� ������ ��� ����� ������ ����
            if( in_param_temp == flashParam_calibT1) {
              gl_nSentPackIndex = CALIB_T1;
              break;
            }

            if( flashParam_calibT2 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION)  &&
              flashParam_calibT2 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
              //� ��� ���� ���������� ����������� � ������������ ����� ����������
              //��������� ����� ���� ��������
              if( in_param_temp < flashParam_calibT1) {
                //���� �������� �����������
                gl_cCalibProcessState = 1;
                flashParam_calibT1 = in_param_temp;
              }
              else {
                //���� �������� ������������
                gl_cCalibProcessState = 4;
                flashParam_calibT2 = in_param_temp;
              }
            }
            else {
              //� ��� ���� ���������� ����������� ����� ����������, �� ��� ���������� ������������
              gl_cCalibProcessState = 3;
              flashParam_calibT2 = in_param_temp;
            }

        }
        else {
          //� ��� ��� ���� ����������� �����!! ������ ��� ����� ����������� :)
          flashParam_calibT1 = in_param_temp;
          gl_cCalibProcessState = 1;
        }
      break;

      case MC_COMMAND_ACT_RESET_T_CALIB:    //Reset thermo calibration data
        gl_bCalibrated = 0;
        flashParam_calibT1 = 0;
        flashParamT1_TD1_val = 0;
        flashParamT1_TD2_val = 0;
        flashParamT1_TD3_val = 0;

        flashParam_calibT2 = 0;
        flashParamT2_TD1_val = 1;
        flashParamT2_TD2_val = 1;
        flashParamT2_TD3_val = 0;

        save_params_p4();
        gl_nSentPackIndex = CALIB_T1;
      break;

      case MC_COMMAND_ACT_LASER_OFF:    //Laser turn off
        //��������� ������� 800V
        GP4DAT |= ( 1 << (16 + 0));   //ONHV   (p4.0) = 1
        //��������� ������ 3kV
        GP4DAT |= ( 1 << (16 + 1));   //OFF3KV (p4.1) = 1
      break;

      case MC_COMMAND_ACT_INTEGR_OFF: //Integrator turn off
        GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1 (����������)
        gl_n_PerimeterReset = 1;      //������������� ���� ��������������� ������ (1=������ ����������, ������ ������������)
        gl_nRppTimerT2 = 0;           //<-- ����� �� ��������! ���� �� ��������� ������� (������ �� ������� �������� ����������)
      break;

      case MC_COMMAND_ACT_INTEGR_ON:  //Integrator turn on
        GP0DAT &= ~( 1 << (16 + 5));  //RP_P   (p0.5) = 0 (���������)
        gl_n_PerimeterReset = 2;      //������������� ���� ��������������� ������ (2=������ ���������, ������ ������������)
        gl_nRppTimerT2 = T1VAL;       //<-- �������� �����! � ����� [����������� �����] ���� ����� ����
      break;

      case MC_COMMAND_ACT_INTEGR_RESET:    //Integrator reset
        GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1 (����������)

        gl_nRppTimerT2 = T1VAL;       //<-- �������� �����! � ����� [����������� �����] ���������� ���������, ���� ������� � ��������� 2, � ��� ����� [����������� �����] ���� ������� � ��������� 0 (������ ����������)
        gl_n_PerimeterReset = 1;      //������������� ���� ��������������� ������ (1=������ ����������, ������ ������������)
      break;

      case MC_COMMAND_ACT_SAVE_FLASH_PARAMS:
        switch( input_buffer[1]) {
          case 0: gl_ushFlashParamLastRULA = gl_un_RULAControl; save_params_p1(); break;
          case 1: save_params_p2(); break;
          case 2: save_params_p3(); break;
          case 3: save_params_p4(); break;
        }
      break;

      case MC_COMMAND_ACT_RELOAD_FLASH_PARAMS:
        switch( input_buffer[1]) {
          case 0: load_params_p1(); check_params_p1(); break;
          case 1: load_params_p2(); check_params_p2(); break;
          case 2: load_params_p3(); check_params_p3(); break;
          case 3: load_params_p4(); check_params_p4(); break;
          case 4: load_params();                       break;
        }
      break;

      case MC_COMMAND_ACT_LOCK_DEVICE:
        if( input_buffer[1] == 0x55 &&
            input_buffer[2] == 0x5A &&
            input_buffer[3] == 0x55) {
#ifdef DEBUG
  printf("DBG: After saving page0 device will be locked\n");
#endif
                  //gl_chLockBit = 1;     ���� ����� � ������� ������, � �� ������ ����� ��������� ����������! :)
                  flashLockDev = 1;
        }
        else {
#ifdef DEBUG
  printf("DBG: Lock command has wrong parameters!\n");
#endif
        }
      break;

      case MC_COMMAND_SWITCH_TO_MAX_RATE_DNDU:
        gl_bSimpleDnDuRegime ^= 1;
      break;
    }

    if( pos_in_in_buf != 0) {

#ifdef DEBUG
      putchar_nocheck( '4');
      //printf("DBG: PIC: out\n");
#endif

      for( i=0; i<6; input_buffer[ i++] = 0);
      pos_in_in_buf = 0;
    }
  }



  //pos_in_in_buf = 0;

//#ifdef DEBUG
  //printf("DBG: PIC: out\n");
//#endif
}
