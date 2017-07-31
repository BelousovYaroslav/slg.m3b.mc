#include <stdio.h>
#include "settings.h"
#include "errors.h"
#include "flashEE.h"

#include "debug.h"

//declared in Main.c
extern char gl_c_EmergencyCode;

//flash-stored params declared in Main.c
extern unsigned short flashParamAmplitudeCode;      //амплитуда колебаний виброподвеса (задатчик)
extern unsigned short flashParamTactCode;           //код такта ошумления
extern unsigned short flashParamMCoeff;             //коэффициент ошумления
extern unsigned short flashParamStartMode;          //стартовая мода системы регулировки периметра
extern unsigned short flashParamDecCoeff;           //коэффициент вычета
extern unsigned short flashLockDev;                 //флаг блокировки устройства

extern unsigned short flashParamI1min;              //контрольное значение тока поджига I1
extern unsigned short flashParamI2min;              //контрольное значение тока поджига I2
extern unsigned short flashParamAmplAngMin1;        //контрольное значение для определения раскачки по сигналу с ДУСа
//extern unsigned short flashParamHvApplyCount;     //количество попыток применения 3kV при поджиге в пачке
//extern unsigned short flashParamHvApplyDurat;     //Длительность попыток применения 3kV при поджиге [мсек]
//extern unsigned short flashParamHvApplyPacks;     //Количество пачек попыток поджига

extern unsigned short flashParamSignCoeff;          //знаковый коэффициент
extern unsigned short flashParamDeviceId;           //ID устройства
extern unsigned short flashParamDateYear;           //Дата ?? прибора: год
extern unsigned short flashParamDateMonth;          //Дата ?? прибора: месяц
extern unsigned short flashParamDateDay;            //Дата ?? прибора: день
extern char flashParamOrg[];                        //название организации

//OBSOLETE? Кандидаты на выкидывание
extern unsigned short gl_ushFlashParamLastRULA;
extern unsigned short gl_ushFlashParamLastRULM;

//калибровка термодатчиков
extern signed short flashParam_calibT1;
extern unsigned short flashParamT1_TD1_val, flashParamT1_TD2_val, flashParamT1_TD3_val;
extern signed short flashParam_calibT2;
extern unsigned short flashParamT2_TD1_val, flashParamT2_TD2_val, flashParamT2_TD3_val;

//калибровка фазового сдвига
extern signed short *gl_passh_calib_phsh_t;
extern unsigned short *gl_paush_calib_phsh_phsh;
extern char gl_cPhaseShiftCalibrated;


void load_params_p1( void) {
  //код амплитуды
  if( flashEE_load_short( ADDR_AMPLITUDE, &flashParamAmplitudeCode))  gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //код такта подставки
  if( flashEE_load_short( ADDR_TACT_CODE, &flashParamTactCode))       gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //коэффициент М
  if( flashEE_load_short( ADDR_M_COEFF, &flashParamMCoeff))           gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //Начальная мода
  if( flashEE_load_short( ADDR_START_MODE, &flashParamStartMode))     gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //коэффициент вычета
  if( flashEE_load_short( ADDR_DEC_COEFF, &flashParamDecCoeff))       gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //флаг блокировки устройства
  if( flashEE_load_short( ADDR_LOCK_DEV, &flashLockDev))              gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //последнее RULA
  if( flashEE_load_short( ADDR_LAST_RULA, &gl_ushFlashParamLastRULA)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //последнее RULM
  if( flashEE_load_short( ADDR_LAST_RULM, &gl_ushFlashParamLastRULM)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;


#ifdef DEBUG
  printf("DBG: load_params(): params loaded from flash memory. Here they are:\n");
  printf("DBG:   Amplitude Code: 0x%04x (%04d)\n", flashParamAmplitudeCode, flashParamAmplitudeCode); //код амплитуды
  printf("DBG:   Base Tact Code: 0x%04x (%04d)\n", flashParamTactCode, flashParamTactCode);   //код такта подставки
  printf("DBG:   M Coefficient:  0x%04x (%04d)\n", flashParamMCoeff, flashParamMCoeff);       //коэффициент М
  printf("DBG:   Start Mode:     0x%04x (%04d)\n", flashParamStartMode, flashParamStartMode); //Начальная мода
  printf("DBG:   Dec. Coeff:     0x%04x (%04d)\n", flashParamDecCoeff, flashParamDecCoeff);   //коэффициент вычета
  printf("DBG:   Dev Lock:       0x%04x (%04d)\n", flashLockDev, flashLockDev);               //флаг блокировки устройства
  printf("DBG:   Last RULA:      0x%04x (%04d)\n", gl_ushFlashParamLastRULA, gl_ushFlashParamLastRULA);   //последнее RULA
  printf("DBG:   Last RULM:      0x%04x (%04d)\n", gl_ushFlashParamLastRULM, gl_ushFlashParamLastRULM);   //последнее RULM
#endif
}

void load_params_p2( void) {
  //минимальный ток I1
  if( flashEE_load_short( ADDR_CONTROL_I1, &flashParamI1min)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //минимальный ток I2
  if( flashEE_load_short( ADDR_CONTROL_I2, &flashParamI2min)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //минимальный AmplAng
  if( flashEE_load_short( ADDR_CONTROL_AA, &flashParamAmplAngMin1)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  /*
  //Количество попыток применения 3kV при поджиге в пачке
  if( flashEE_load_short( ADDR_HV_APPLY_C, &flashParamHvApplyCount)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Длительность попыток применения 3kV при поджиге
  if( flashEE_load_short( ADDR_HV_APPLY_D, &flashParamHvApplyDurat)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Количество пачек попыток поджига
  if( flashEE_load_short( ADDR_HV_APPLY_P, &flashParamHvApplyPacks)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  */

#ifdef DEBUG
  printf("DBG:load_params_p2()\n");
  printf("DBG:   Control I1:     0x%04x (%04d)\n", flashParamI1min, flashParamI1min);                 //минимальный ток I1
  printf("DBG:   Control I2:     0x%04x (%04d)\n", flashParamI2min, flashParamI2min);                 //минимальный ток I2
  printf("DBG:   Control AA:     0x%04x (%04d)\n", flashParamAmplAngMin1, flashParamAmplAngMin1);     //минимальный AmplAng
  //printf("DBG:   HV_count:       0x%04x (%04d)\n", flashParamHvApplyCount, flashParamHvApplyCount);   //HV_applies tries amount in pack
  //printf("DBG:   HV_duration:    0x%04x (%04d)\n", flashParamHvApplyDurat, flashParamHvApplyDurat);   //HV_applies tries duration
  //printf("DBG:   HV_packs:       0x%04x (%04d)\n", flashParamHvApplyPacks, flashParamHvApplyPacks);   //HV_applies tries packs
#endif
}

void load_params_p3( void) {
  //знаковый коэффициент
  if( flashEE_load_short( ADDR_SIGN_COEFF, &flashParamSignCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //серийный номер
  if( flashEE_load_short( ADDR_DEVICE_ID, &flashParamDeviceId)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //дата.год
  if( flashEE_load_short( ADDR_DATE_Y, &flashParamDateYear)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //дата.месяц
  if( flashEE_load_short( ADDR_DATE_M, &flashParamDateMonth)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //дата.день
  if( flashEE_load_short( ADDR_DATE_D, &flashParamDateDay)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //организация
  if( flashEE_load_text( ADDR_ORG, flashParamOrg, 16)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

#ifdef DEBUG
  printf("DBG:load_params_p3()\n");
  printf("DBG:   Sign coeff:      0x%04x (%04d)\n", flashParamSignCoeff, flashParamSignCoeff); //знаковый коэффициент
  printf("DBG:   Serial number:   0x%04x (%04d)\n", flashParamDeviceId, flashParamDeviceId);   //серийный номер
  printf("DBG:   Organization:    '%s'\n", flashParamOrg);                                     //организация
  printf("DBG:   Year:            0x%04x (%04d)\n", flashParamDateYear, flashParamDateYear);   //год
  printf("DBG:   Month:           0x%04x (%04d)\n", flashParamDateMonth, flashParamDateMonth); //месяц
  printf("DBG:   Day:             0x%04x (%04d)\n", flashParamDateDay, flashParamDateDay);     //день
#endif
}

void load_params_p4( void) {
  //Температура минимальной точки калибровки
  if( flashEE_load_short( ADDR_TCALIB_T1, ( unsigned short *) &flashParam_calibT1)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты первого термодатчика при минимальной температуре калибровки
  if( flashEE_load_short( ADDR_TCALIB_T1_TD1, &flashParamT1_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты второго термодатчика при минимальной температуре калибровки
  if( flashEE_load_short( ADDR_TCALIB_T1_TD2, &flashParamT1_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты третьего термодатчика при минимальной температуре калибровки
  if( flashEE_load_short( ADDR_TCALIB_T1_TD3, &flashParamT1_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  //Температура максимальной точки калибровки
  if( flashEE_load_short( ADDR_TCALIB_T2, ( unsigned short *) &flashParam_calibT2)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты первого термодатчика при максимальной температуре калибровки
  if( flashEE_load_short( ADDR_TCALIB_T2_TD1, &flashParamT2_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты второго термодатчика при максимальной температуре калибровки
  if( flashEE_load_short( ADDR_TCALIB_T2_TD2, &flashParamT2_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты второго термодатчика при максимальной температуре калибровки
  if( flashEE_load_short( ADDR_TCALIB_T2_TD3, &flashParamT2_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
#ifdef DEBUG
  printf("DBG:load_params_p4()\n");
  printf("DBG:   T-Calibration T1=0x%04x (%04d)\n", flashParam_calibT1, flashParam_calibT1);      //Температурная калибровка: температура нижней точки
  printf("DBG:   T1_TD1:          0x%04x (%04d)\n", flashParamT1_TD1_val, flashParamT1_TD1_val);  //Температурная калибровка: показания термодатчика TD1 в нижней температурной точке
  printf("DBG:   T1_TD2:          0x%04x (%04d)\n", flashParamT1_TD2_val, flashParamT1_TD2_val);  //Температурная калибровка: показания термодатчика TD2 в нижней температурной точке
  printf("DBG:   T1_TD3:          0x%04x (%04d)\n", flashParamT1_TD3_val, flashParamT1_TD3_val);  //Температурная калибровка: показания термодатчика TD3 в нижней температурной точке

  printf("DBG:   T-Calibration T2=0x%04x (%04d)\n", flashParam_calibT2, flashParam_calibT2);      //Температурная калибровка: температура верхней точки
  printf("DBG:   T1_TD1:          0x%04x (%04d)\n", flashParamT2_TD1_val, flashParamT2_TD1_val);  //Температурная калибровка: показания термодатчика TD1 в верхней температурной точке
  printf("DBG:   T1_TD2:          0x%04x (%04d)\n", flashParamT2_TD2_val, flashParamT2_TD2_val);  //Температурная калибровка: показания термодатчика TD2 в верхней температурной точке
  printf("DBG:   T1_TD3:          0x%04x (%04d)\n", flashParamT2_TD3_val, flashParamT2_TD3_val);  //Температурная калибровка: показания термодатчика TD3 в верхней температурной точке
#endif
}

void load_params_p5( void) {
  signed short sh_tmp;
  unsigned short ush_tmp;
#ifdef DEBUG
  int i;
#endif

  //калибровка фазового сдвига. Точка 1. Температура.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T1, ( unsigned short *) &sh_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_passh_calib_phsh_t[0] = sh_tmp;
  //калибровка фазового сдвига. Точка 1. Фазовый сдвиг
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH1, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_paush_calib_phsh_phsh[0] = ush_tmp;

  //калибровка фазового сдвига. Точка 2. Температура.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T2, ( unsigned short *) &sh_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_passh_calib_phsh_t[1] = sh_tmp;
  //калибровка фазового сдвига. Точка 2. Фазовый сдвиг
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH2, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_paush_calib_phsh_phsh[1] = ush_tmp;

  //калибровка фазового сдвига. Точка 3. Температура.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T3, ( unsigned short *) &sh_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_passh_calib_phsh_t[2] = sh_tmp;
  //калибровка фазового сдвига. Точка 3. Фазовый сдвиг
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH3, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_paush_calib_phsh_phsh[2] = ush_tmp;

  //калибровка фазового сдвига. Точка 4. Температура.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T4, ( unsigned short *) &sh_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_passh_calib_phsh_t[3] = sh_tmp;
  //калибровка фазового сдвига. Точка 4. Фазовый сдвиг
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH4, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_paush_calib_phsh_phsh[3] = ush_tmp;

  //калибровка фазового сдвига. Точка 5. Температура.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T5, ( unsigned short *) &sh_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_passh_calib_phsh_t[4] = sh_tmp;
  //калибровка фазового сдвига. Точка 5. Фазовый сдвиг
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH5, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_paush_calib_phsh_phsh[4] = ush_tmp;


  //калибровка фазового сдвига. Точка 6. Температура.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T6, ( unsigned short *) &sh_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_passh_calib_phsh_t[5] = sh_tmp;
  //калибровка фазового сдвига. Точка 6. Фазовый сдвиг
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH6, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_paush_calib_phsh_phsh[5] = ush_tmp;

  //калибровка фазового сдвига. Точка 7. Температура.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T7, ( unsigned short *) &sh_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_passh_calib_phsh_t[6] = sh_tmp;
  //калибровка фазового сдвига. Точка 7. Фазовый сдвиг
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH7, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_paush_calib_phsh_phsh[6] = ush_tmp;

  //калибровка фазового сдвига. Точка 8. Температура.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T8, ( unsigned short *) &sh_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_passh_calib_phsh_t[7] = sh_tmp;
  //калибровка фазового сдвига. Точка 8. Фазовый сдвиг
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH8, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_paush_calib_phsh_phsh[7] = ush_tmp;

  //калибровка фазового сдвига. Точка 9. Температура.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T9, ( unsigned short *) &sh_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_passh_calib_phsh_t[8] = sh_tmp;
  //калибровка фазового сдвига. Точка 9. Фазовый сдвиг
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH9, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_paush_calib_phsh_phsh[8] = ush_tmp;

  //калибровка фазового сдвига. Точка 10. Температура.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T10, ( unsigned short *) &sh_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_passh_calib_phsh_t[9] = sh_tmp;
  //калибровка фазового сдвига. Точка 10. Фазовый сдвиг
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH10, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_paush_calib_phsh_phsh[9] = ush_tmp;


  //калибровка фазового сдвига. Точка 11. Температура.
  if( flashEE_load_short( ADDR_PHSH_CALIB_T11, ( unsigned short *) &sh_tmp))    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_passh_calib_phsh_t[10] = sh_tmp;
  //калибровка фазового сдвига. Точка 11. Фазовый сдвиг
  if( flashEE_load_short( ADDR_PHSH_CALIB_PHSH11, ( unsigned short *) &ush_tmp)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  gl_paush_calib_phsh_phsh[10] = ush_tmp;


#ifdef DEBUG
  printf("DBG:load_params_p5()\n");
  for( i=0; i<11; i++) {
    printf("DBG:  ADDR_PHSH_CALIB_T%d:    0x%04x (%04d)\n",   i+1, gl_passh_calib_phsh_t[i], gl_passh_calib_phsh_t[i]);
    printf("DBG:  ADDR_PHSH_CALIB_PHSH%d: 0x%04x (%04d)\n\n", i+1, gl_paush_calib_phsh_phsh[i], gl_paush_calib_phsh_phsh[i]);
  }
#endif
}

void check_params_p1( void) {
  //Код амплитуды [0-255]. дефолтное значение 90
  //90 для большого 35 для маленького
  if( flashParamAmplitudeCode > 255)
    flashParamAmplitudeCode = 35;

  //Код такта подставки [0-3]. дефолтное значение 0
  if( flashParamTactCode > 3)
    flashParamTactCode = 0;

  //Коэффициент М[0-1] = значения параметра [0-250]
  //дефолтное значение 200 (что означает M=0.8 и DAC1 = 0.8 * DAC0)
  if( flashParamMCoeff > 250)
    flashParamMCoeff = 200;

  //Начальная мода [0-250]. дефолтное значение 125 (что означает 1,25В на DAC2)
  if( flashParamStartMode > 250)
    flashParamStartMode = 125;

  //коэффициент вычета
  //default значение 0,4 УТОЧНИТЬ!
  if( flashParamDecCoeff == 0xffff)
    flashParamDecCoeff = ( int) ( 0.4 * 65535.);

  //флаг блокировки устройства
  //default значение 0 - режим разработчиков
  if( flashLockDev != 1)
    flashLockDev = 0;

  //последний RULA [0-4095]
  if( gl_ushFlashParamLastRULA > 4095) {
    gl_ushFlashParamLastRULA = 0;
  }

  //последний RULM [0-4095]
  if( gl_ushFlashParamLastRULM > 4095) {
    gl_ushFlashParamLastRULM = 0;
  }

#ifdef DEBUG
  printf("DBG: check_params_p1(): params checked for the range. Here they are:\n");
  printf("DBG:   Amplitude Code: 0x%04x (%04d)\n", flashParamAmplitudeCode, flashParamAmplitudeCode); //код амплитуды
  printf("DBG:   Base Tact Code: 0x%04x (%04d)\n", flashParamTactCode, flashParamTactCode);   //код такта подставки
  printf("DBG:   M Coefficient:  0x%04x (%04d)\n", flashParamMCoeff, flashParamMCoeff);       //коэффициент М
  printf("DBG:   Start Mode:     0x%04x (%04d)\n", flashParamStartMode, flashParamStartMode); //Начальная мода
  printf("DBG:   Dec. Coeff:     0x%04x (%04d)\n", flashParamDecCoeff, flashParamDecCoeff);   //коэффициент вычета
  printf("DBG:   Dev Lock:       0x%04x (%04d)\n", flashLockDev, flashLockDev);               //флаг блокировки устройства
#endif
}

void check_params_p2( void) {
  //контрольный ток поджига I1 [0-0.750 mA] = значения параметра [ 0 - 65534]
  //default значение 0.4 mA
  if( flashParamI1min == 0xffff)
    flashParamI1min = ( short) ( 65535. * 0.4 / 0.75);

  //контрольный ток поджига I2 [0-0.750 mA] = значения параметра [ 0 - 65534]
  //default значение 0.4 mA
  if( flashParamI2min == 0xffff)
    flashParamI2min = ( short) ( 65535. * 0.4 / 0.75);

  //контрольное значение раскачки - амплитуда ДУПа (0-3В) = значения параметра [ 0 - 65534]
  //default значение 1.0В
  if( flashParamAmplAngMin1 == 0xffff)
    flashParamAmplAngMin1 = ( int) ( 1.0 / 3. * 65535.);

  /*
  //количество попыток применения 3kV при поджиге ( 1 - 20)
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
  printf("DBG:   Control I1:     0x%04x (%04d)\n", flashParamI1min, flashParamI1min);                 //минимальный ток I1
  printf("DBG:   Control I2:     0x%04x (%04d)\n", flashParamI2min, flashParamI2min);                 //минимальный ток I2
  printf("DBG:   Control AA:     0x%04x (%04d)\n", flashParamAmplAngMin1, flashParamAmplAngMin1);     //минимальный AmplAng
  //printf("DBG:   HV_count:       0x%04x (%04d)\n", flashParamHvApplyCount, flashParamHvApplyCount);   //HV_applies tries amount
  //printf("DBG:   HV_duration:    0x%04x (%04d)\n", flashParamHvApplyDurat, flashParamHvApplyDurat);   //HV_applies tries duration
  //printf("DBG:   HV_packs:       0x%04x (%04d)\n", flashParamHvApplyPacks, flashParamHvApplyPacks);   //HV_applies tries packs
#endif
}

void check_params_p3( void) {
  int i;
  //знаковый коэффициент. [-1; +1]
  //default value = 1
  if( flashParamSignCoeff > 2)
    flashParamSignCoeff = 2;

  //ID устройства
  if( flashParamDeviceId == 65535)
    flashParamDeviceId = 0;

  //Дата ?? устройства
  //default value = 2016.01.01
  if( flashParamDateYear < 2000 || flashParamDateYear > 2200)
    flashParamDateYear = 2016;

  if( flashParamDateMonth > 12)
    flashParamDateMonth = 1;

  if( flashParamDateDay > 31)
    flashParamDateDay = 1;

  //название организации
  //default - все минусы
  for( i=0; i<17; i++) {
    if( flashParamOrg[i] < 33 || flashParamOrg[i] > 126)
      flashParamOrg[i] = ' ';
  }



#ifdef DEBUG
  printf("DBG: check_params_p3(): params checked for the range. Here they are:\n");
  printf("DBG:   Sign coeff:     0x%04x (%04d)\n", flashParamSignCoeff, flashParamSignCoeff); //знаковый коэффициент
  printf("DBG:   Serial number:  0x%04x (%04d)\n", flashParamDeviceId, flashParamDeviceId);   //серийный номер
  printf("DBG:   Year:           0x%04x (%04d)\n", flashParamDateYear, flashParamDateYear);   //год
  printf("DBG:   Month:          0x%04x (%04d)\n", flashParamDateMonth, flashParamDateMonth); //месяц
  printf("DBG:   Day:            0x%04x (%04d)\n", flashParamDateDay, flashParamDateDay);     //день
  printf("DBG:   Organization:   '%s'\n", flashParamOrg);                                     //организация
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
  printf("DBG:   flashParam_calibT1:     0x%04x (%04d)\n", flashParam_calibT1, flashParam_calibT1);
  printf("DBG:   flashParam_calibT1_TD1: 0x%04x (%04d)\n", flashParamT1_TD1_val, flashParamT1_TD1_val);
  printf("DBG:   flashParam_calibT1_TD2: 0x%04x (%04d)\n", flashParamT1_TD2_val, flashParamT1_TD2_val);
  printf("DBG:   flashParam_calibT1_TD3: 0x%04x (%04d)\n", flashParamT1_TD3_val, flashParamT1_TD3_val);
  printf("DBG:   flashParam_calibT2:     0x%04x (%04d)\n", flashParam_calibT2, flashParam_calibT2);
  printf("DBG:   flashParam_calibT2_TD1: 0x%04x (%04d)\n", flashParamT2_TD1_val, flashParamT2_TD1_val);
  printf("DBG:   flashParam_calibT2_TD2: 0x%04x (%04d)\n", flashParamT2_TD2_val, flashParamT2_TD2_val);
  printf("DBG:   flashParam_calibT2_TD3: 0x%04x (%04d)\n", flashParamT2_TD3_val, flashParamT2_TD3_val);
#endif
}

void check_params_p5( void) {
  int i, j;
  signed short ssh_t;
  unsigned short ush_phsh;

  for( i=0; i<10; i++) {
    for( j=0; j<10; i++) {

      if( gl_passh_calib_phsh_t[ j] > gl_passh_calib_phsh_t[ j + 1]) {
        ssh_t = gl_passh_calib_phsh_t[ j + 1];
        gl_passh_calib_phsh_t[ j + 1] = gl_passh_calib_phsh_t[ j];
        gl_passh_calib_phsh_t[ j] = ssh_t;

        ush_phsh = gl_paush_calib_phsh_phsh[ j + 1];
        gl_paush_calib_phsh_phsh[ j + 1] = gl_paush_calib_phsh_phsh[ j];
        gl_paush_calib_phsh_phsh[ j] = ush_phsh;
      }

    }
  }

  for( i=0; i<11; i++) {
    if( gl_passh_calib_phsh_t[i] != 0xFFFF && gl_paush_calib_phsh_phsh[i] != 0xFFFF) {
      gl_cPhaseShiftCalibrated = 1;
      break;
    }
  }

#ifdef DEBUG
  printf("DBG: check_params_p5(): loaded params processed. Here they are:\n");
  for( i=0; i<11; i++) {
    printf("DBG:  ADDR_PHSH_CALIB_T%d:    0x%04x (%04d)\n",   i+1, gl_passh_calib_phsh_t[i], gl_passh_calib_phsh_t[i]);
    printf("DBG:  ADDR_PHSH_CALIB_PHSH%d: 0x%04x (%04d)\n\n", i+1, gl_passh_calib_phsh_phsh[i], gl_passh_calib_phsh_phsh[i]);
  }
#endif
}

void load_params( void) {
  load_params_p1();
  if( gl_c_EmergencyCode != ERROR_FLASH_LOAD_PARAMS_FAIL) load_params_p2();
  if( gl_c_EmergencyCode != ERROR_FLASH_LOAD_PARAMS_FAIL) load_params_p3();
  if( gl_c_EmergencyCode != ERROR_FLASH_LOAD_PARAMS_FAIL) load_params_p4();
  if( gl_c_EmergencyCode != ERROR_FLASH_LOAD_PARAMS_FAIL) load_params_p5();

  check_params_p1();
  check_params_p2();
  check_params_p3();
  check_params_p4();
  check_params_p5();
}

void save_params_p1( void) {
#ifdef DEBUG
  printf("DBG: save_params_p1(): params to be saved are:\n");
  printf("DBG:   Amplitude Code: 0x%04x (%04d)\n", flashParamAmplitudeCode, flashParamAmplitudeCode); //код амплитуды
  printf("DBG:   Base Tact Code: 0x%04x (%04d)\n", flashParamTactCode, flashParamTactCode);   //код такта подставки
  printf("DBG:   M Coefficient:  0x%04x (%04d)\n", flashParamMCoeff, flashParamMCoeff);       //коэффициент М
  printf("DBG:   Start Mode:     0x%04x (%04d)\n", flashParamStartMode, flashParamStartMode); //Начальная мода
  printf("DBG:   Dec. Coeff:     0x%04x (%04d)\n", flashParamDecCoeff, flashParamDecCoeff);   //коэффициент вычета
  printf("DBG:   Dev Lock:       0x%04x (%04d)\n", flashLockDev, flashLockDev);               //флаг блокировки устройства
  printf("DBG:   Last RULA:      0x%04x (%04d)\n", gl_ushFlashParamLastRULA, gl_ushFlashParamLastRULA);   //последнее RULA
  printf("DBG:   Last RULM:      0x%04x (%04d)\n", gl_ushFlashParamLastRULM, gl_ushFlashParamLastRULM);   //последнее RULM
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
  printf("DBG:   Control I1:     0x%04x (%04d)\n", flashParamI1min, flashParamI1min);                 //минимальный ток I1
  printf("DBG:   Control I2:     0x%04x (%04d)\n", flashParamI2min, flashParamI2min);                 //минимальный ток I2
  printf("DBG:   Control AA:     0x%04x (%04d)\n", flashParamAmplAngMin1, flashParamAmplAngMin1);     //минимальный AmplAng
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
  printf("DBG:   Sign coeff:     0x%04x (%04d)\n", flashParamSignCoeff, flashParamSignCoeff); //Знаковый коэффициент
  printf("DBG:   Serial number:  0x%04x (%04d)\n", flashParamDeviceId, flashParamDeviceId);   //серийный номер
  printf("DBG:   Organization:   '%s'\n", flashParamOrg);                                     //организация
  printf("DBG:   Year:           0x%04x (%04d)\n", flashParamDateYear, flashParamDateYear);   //год
  printf("DBG:   Month:          0x%04x (%04d)\n", flashParamDateMonth, flashParamDateMonth); //месяц
  printf("DBG:   Day:            0x%04x (%04d)\n", flashParamDateDay, flashParamDateDay);     //день
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

void save_params_p5( void) {
  int i;

#ifdef DEBUG
  printf("DBG: save_params_p5(): params to be saved are:\n");
  for( i=0; i<11; i++) {
    printf("DBG:  ADDR_PHSH_CALIB_T%d:    0x%04x (%04d)\n",   i+1, gl_passh_calib_phsh_t[i], gl_passh_calib_phsh_t[i]);
    printf("DBG:  ADDR_PHSH_CALIB_PHSH%d: 0x%04x (%04d)\n\n", i+1, gl_passh_calib_phsh_phsh[i], gl_passh_calib_phsh_phsh[i]);
  }
#endif

  for( i=0; i<11; i++) {
    if( flashEE_save_short( ADDR_PHSH_CALIB_T1 + i * 2, gl_passh_calib_phsh_t[ i])) {
      gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
      return;
    }

    if( flashEE_save_short( ADDR_PHSH_CALIB_PHSH1 + i * 2, gl_paush_calib_phsh_phsh[ i])) {
      gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
      return;
    }

  }
}

void save_params( void) {
  save_params_p1();
  if( gl_c_EmergencyCode != ERROR_FLASH_SAVE_PARAMS_FAIL) save_params_p2();
  if( gl_c_EmergencyCode != ERROR_FLASH_SAVE_PARAMS_FAIL) save_params_p3();
  if( gl_c_EmergencyCode != ERROR_FLASH_SAVE_PARAMS_FAIL) save_params_p4();
  if( gl_c_EmergencyCode != ERROR_FLASH_SAVE_PARAMS_FAIL) save_params_p5();
}