#include <ADuC7026.h>
#include <stdio.h>
#include <string.h>
#include "Main.h"
#include "settings.h"
#include "McCommands.h"
#include "AnalogueParamsConstList.h"
#include "serial.h"
#include "debug.h"

//Буфер входящих команд
extern char gl_acInputBuffer[];                     //буфер входящих команд
extern char gl_cPos_in_in_buf;                      //позиция записи в буфере входящих команд

extern unsigned short gl_ushCurrentDecCoeff;        //текущий коэффициент вычета

//ПАРАМЕТРЫ ХРАНИМЫЕ ВО ФЛЭШ-ПАМЯТИ
extern unsigned short gl_ush_flashParamAmplitudeCode;      //амплитуда колебаний виброподвеса
extern unsigned short gl_ush_flashParamTactCode;           //код такта ошумления
extern unsigned short gl_ush_flashParamMCoeff;             //коэффициент ошумления
extern unsigned short gl_ush_flashParamStartMode;          //начальная мода Системы Регулировки Периметра
extern unsigned short gl_ush_flashParamStartDecCoeff;      //стартовый коэффициент вычета
extern unsigned short gl_ush_flashLockDev;                 //флаг блокировки устройства

extern unsigned short gl_ush_flashParamI1min;       //контрольное значение тока поджига I1
extern unsigned short gl_ush_flashParamI2min;       //контрольное значение тока поджига I2
extern unsigned short gl_ush_flashParamAmplAngMin1; //контрольное значение сигнала раскачки с ДУСа

extern unsigned short gl_ush_flashParamSignCoeff;          //знаковый коэффициент
extern unsigned short gl_ush_flashParamDeviceId;           //ID устройства
extern unsigned short gl_ush_flashParamDateYear;           //дата ? прибора.год
extern unsigned short gl_ush_flashParamDateMonth;          //дата ? прибора.месяц
extern unsigned short gl_ush_flashParamDateDay;            //дата ? прибора.день
extern char gl_ac_flashParamOrg[];                         //название организации

//калибровка термодатчиков
extern signed short gl_ssh_flashParam_calibT1;
extern unsigned short gl_ush_flashParamT1_TD1_val, gl_ush_flashParamT1_TD2_val, gl_ush_flashParamT1_TD3_val;
extern signed short gl_ssh_flashParam_calibT2;
extern unsigned short gl_ush_flashParamT2_TD1_val, gl_ush_flashParamT2_TD2_val, gl_ush_flashParamT2_TD3_val;
extern char gl_bTDCalibrated;
extern char gl_cCalibProcessState;

//калибровка фазового сдвига
extern char gl_ac_calib_phsh_t[];                   //калибровка фазового сдвига. массив температур
extern char gl_ac_calib_phsh_phsh[];                //калибровка фазового сдвига. массив соответствующих фазовых сдвигов
extern char gl_cFlashParamPhaseShiftUsage;          //использование фазового сдвига
extern int  gl_nCurrentPhaseShift;                  //текущий (последний применённый) фазовый сдвиг. 0xFF-не задействован. default startup value = 0xFF

extern unsigned short gl_ushFlashParamLastRULA;     //последнее RULA (obsolete)
extern unsigned short gl_ushFlashParamLastRULM;     //последнее RULM (obsolete)


//калибровка коэффициента вычета
extern char gl_ac_calib_dc_t[];                     //массив точек температур калибровки коэффициента вычета
extern unsigned short gl_ush_calib_dc_dc[];         //массив точек значений коэффициента вычета, соответствующих температурам, описанным выше

extern unsigned char  gl_ucDcUsageStartSetting;     //флаг что брать в качестве стартового Квычета: 0=DC_START; REST=по таблице калибровки;
extern unsigned char  gl_ucDcUsageRecalc;           //флаг как перевычислять Квычета: 0=перевычисление;1=по таблице калибровки(грубо);2=по таблице калибровки(аппрокс);REST=ручной режим;
extern unsigned short gl_ushDcUsageRecalcPeriod;    //период перевычисления Квычета (в секундах)


//засечки таймеров
extern int gl_nRppTimerT2;                          //засечка таймера для проведения сброса интегратора Системы Регулировки Периметра

extern char gl_c_EmergencyCode;                     //код ошибки

extern int gl_snMeaningCounter;                     //Amplitude control module: counter of measured values
extern int gl_snMeaningCounterRound;                //Amplitude control module: round of measured values
extern int gl_snMeaningShift;                       //Amplitude control module: bits for shift to get mean
extern long gl_lnMeaningSumm;                       //Amplitude control module: summ of amplitudes
extern long gl_lnMeanImps;                          //Amplitude control module: mean (it's calculated shifted by 4 i.e. multiplied by 16)
extern int  gl_nActiveRegulationT2;                 //Amplitude control module: amplitude active regulation T2 intersection
extern char gl_cAmplRegulation;                     //Amplitude control module: amplitude regulation state (0=manual, 1=soft regulation,2=active regulation)

//Флаги
extern char gl_b_SyncMode;                          //флаг режима работы гироскопа:   0=синхр. 1=асинхр.
extern char gl_chAngleOutput;                       //флаг вывода приращения угла: 0 = dW (4b)         1 = dN (2b), dU(2b)
extern char gl_bSimpleDnDuRegime;                   //флаг режима "частотной нарезки"
                                                    //(выдачи dN,dU с максимально возможной частотой, без снятий аналоговых параметров, без СРП, без контроля амплитуды)
extern char gl_bCalibProcessState;                  //флаг проведения процесса температурной калибровки термодатчиков
extern char gl_bTDCalibrated;                       //флаг состояния температурной калибровки термодатчиков: 0=датчики не калиброваны, 1=датчики калиброваны
extern char gl_n_PerimeterReset;                    //флаг сигнала сброса периметра:
                                                    //0 = данные достоверны
                                                    //1 = данные НЕдостоверны, прошло выключение интегратора
                                                    //2 = данные НЕдостоверны, прошло включение интегратора
extern char gl_chLockBit;                           //флаг блокирования устройства: 0 - режим "разработчика"   1 - режим "пользователя"

extern unsigned int gl_un_RULAControl;              //код сигнала RULA выдаваемый с ЦАП... [0 - 4095] ==> [0-2.5 В]


extern short gl_nSentAddParamIndex;                 //Analogue Parameter Index (what are we sending now)
extern short gl_nSentAddParamSubIndex;              //Analogue Parameter sub-Index (what are we sending now)


//переменные участвующие в счётчике секунд относительно старта прибора
extern long gl_lSecondsFromStart;                   //собственно сами секунды
extern long gl_lCalibratedPhaseShiftApplySecs;      //секунда, на которой нам надо перевычислить фазовый сдвиг
extern long gl_lCalibratedDcApplySecs;              //секунда, на которой нам надо перевычислить Квычета

//функции
extern void configure_hanger( void);                //функция выстаки кода такта подставки
extern void DACConfiguration( void);                //функция выставки напряжений (сигналов RULA, RULM) на ЦАП 

//Переменные участвующие в рассчёте коэффициента вычета "на лету"
extern double gl_dblMeanAbsDn;
extern double gl_dblMeanAbsDu;
extern int    gl_nMeanDecCoeffCounter;

//список выдаваемых наружу аналоговых (доп.) параметров
extern unsigned short gl_aushListOutputAddParams[];


void processIncomingCommand( void) {
  short in_param_temp;
  int i, j;
  char c_t, c_phsh, c_cnt;
  unsigned short ush_dc;

  //**********************************************************************
  // Обработка буфера входящих команд
  //**********************************************************************
#ifdef DEBUG
  if( gl_cPos_in_in_buf > 0) {
    //printf("DBG: PIC: in with %d\n", gl_cPos_in_in_buf);
    putchar_nocheck( '0');
    putchar_nocheck( '0' + gl_cPos_in_in_buf);
  }
#endif

  if( gl_cPos_in_in_buf == IN_COMMAND_BUF_LEN) {

#ifdef DEBUG
    putchar_nocheck( '1');
    printf("\nDBG: Incoming command: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                gl_acInputBuffer[ 0], gl_acInputBuffer[ 1], gl_acInputBuffer[ 2],
                gl_acInputBuffer[ 3], gl_acInputBuffer[ 4], gl_acInputBuffer[ 5]);
#endif

    //LOCK прибора
    if( gl_chLockBit == 1) {
      switch( gl_acInputBuffer[ 0]) {
        case MC_COMMAND_ACT_UNLOCK_DEVICE:
          if( gl_acInputBuffer[ 1] == 0x5A &&
              gl_acInputBuffer[ 2] == 0x55 &&
              gl_acInputBuffer[ 3] == 0x5A) {
                gl_chLockBit = 0;
                gl_ush_flashLockDev = 0;
          }
        break;
        case MC_COMMAND_REQ:
          switch( gl_acInputBuffer[ 1]) {
            case VERSION:     gl_nSentAddParamIndex = VERSION;      break;
          }
        break;
        default:
#ifdef DEBUG
          printf("DBG: Device is locked!\n");
#endif
        break;
      }
      gl_cPos_in_in_buf = 0;
      return;
    }

#ifdef DEBUG
    putchar_nocheck( '2');
#endif

    switch( gl_acInputBuffer[0]) {
      //**************************************************************************** SET
      case MC_COMMAND_SET:
        switch( gl_acInputBuffer[1]) {
          case AMPLITUDE:   //Set Amplitude of Hangreup Vibration
            gl_ush_flashParamAmplitudeCode = gl_acInputBuffer[2] + ( ( ( short) gl_acInputBuffer[3]) << 8);
            gl_nSentAddParamIndex = AMPLITUDE;

            gl_snMeaningCounter = 0;
            gl_snMeaningCounterRound = 128;
            gl_snMeaningShift = 7;
            gl_lnMeaningSumm = 0;
            gl_lnMeanImps = 0;

            //включаем флаг активной регулировки амплитуды (перенастройка амплитуды)
            gl_cAmplRegulation = 2;
            gl_nActiveRegulationT2 = T2VAL;
            if( gl_nActiveRegulationT2 == 0) gl_nActiveRegulationT2 = 1;

          break;

          case TACT_CODE:   //Set CodeTact
            gl_ush_flashParamTactCode = gl_acInputBuffer[2] + ( ( ( short) gl_acInputBuffer[3]) << 8);
            configure_hanger();
            gl_nSentAddParamIndex = TACT_CODE;

            //включаем флаг активной регулировки амплитуды (перенастройка амплитуды)
            gl_cAmplRegulation = 2;
            gl_nActiveRegulationT2 = T2VAL;
            if( gl_nActiveRegulationT2 == 0) gl_nActiveRegulationT2 = 1;

          break;

          case M_COEFF: //Set NoiseCoefficient M
            gl_ush_flashParamMCoeff = gl_acInputBuffer[2] + ( ( ( short) gl_acInputBuffer[3]) << 8);

            //выставляемся на ЦАП
            DACConfiguration();
            gl_nSentAddParamIndex = M_COEFF;

            //включаем флаг активной регулировки амплитуды (перенастройка амплитуды)
            gl_cAmplRegulation = 2;
            gl_nActiveRegulationT2 = T2VAL;
            if( gl_nActiveRegulationT2 == 0) gl_nActiveRegulationT2 = 1;
          break;

          case STARTMODE: //Set StartMode
            gl_ush_flashParamStartMode = gl_acInputBuffer[2] + ( ( ( short) gl_acInputBuffer[3]) << 8);

            GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1

            gl_n_PerimeterReset = 1;
            gl_nRppTimerT2 = T2VAL;
            gl_nSentAddParamIndex = STARTMODE;

            DACConfiguration();
          break;

          case DECCOEFF_CURRENT: //Set decrement coeff (ТЕКУЩИЙ!)

            if( gl_ucDcUsageRecalc == 1 || gl_ucDcUsageRecalc == 2) {
              //режим использования калибровки. Ставим значение - и сбрасываем таймер перевычисления
              gl_ushCurrentDecCoeff = gl_acInputBuffer[2] + ( ( ( short) gl_acInputBuffer[3]) << 8);

              //засечём новую 1 мин
              gl_lCalibratedDcApplySecs = gl_lSecondsFromStart + gl_ushDcUsageRecalcPeriod;
            }
            else if( gl_ucDcUsageRecalc == 3) {

              //ручной режим. Просто ставим значение
              gl_ushCurrentDecCoeff = gl_acInputBuffer[2] + ( ( ( short) gl_acInputBuffer[3]) << 8);
            }
            else if( gl_ucDcUsageRecalc == 0) {
              //режим перевычисления на лету. Ставим заказанный, и сбрасываем накопленную статистику.
              gl_ushCurrentDecCoeff = gl_acInputBuffer[2] + ( ( ( short) gl_acInputBuffer[3]) << 8);

              //сбросим количество информации накопленное для пересчёта Кв "НА ЛЕТУ"
              gl_nMeanDecCoeffCounter = 0;
              gl_dblMeanAbsDn = 0.;
              gl_dblMeanAbsDu = 0.;
            }

            gl_nSentAddParamIndex = DECCOEFF_CURRENT;
          break;


          case CONTROL_I1:  //Set control_i1
            gl_ush_flashParamI1min = gl_acInputBuffer[2] + ( ( ( short) gl_acInputBuffer[3]) << 8);
            gl_nSentAddParamIndex = CONTROL_I1;
          break;

          case CONTROL_I2:  //Set control_i2
            gl_ush_flashParamI2min = gl_acInputBuffer[2] + ( ( ( short) gl_acInputBuffer[3]) << 8);
            gl_nSentAddParamIndex = CONTROL_I2;
          break;

          case CONTROL_AA:  //Set control_aa
            gl_ush_flashParamAmplAngMin1 = gl_acInputBuffer[2] + ( ( ( short) gl_acInputBuffer[3]) << 8);
            gl_nSentAddParamIndex = CONTROL_AA;
          break;

          /*
          case HV_APPLY_COUNT_SET:  //Set HV_applies_count
            flashParamHvApplyCount = gl_acInputBuffer[2];
            gl_nSentAddParamIndex = HV_APPLY_COUNT_SET;
          break;

          case HV_APPLY_DURAT_SET:  //Set HV_applies_duration
            flashParamHvApplyDurat = gl_acInputBuffer[2] + ( ( ( short) gl_acInputBuffer[3]) << 8);
            gl_nSentAddParamIndex = HV_APPLY_DURAT_SET;
          break;

          case HV_APPLY_PACKS:   //Set HV_applies_packs
            flashParamHvApplyPacks = gl_acInputBuffer[2];
            gl_nSentAddParamIndex = HV_APPLY_PACKS;
          break;
          */

          case SIGNCOEFF:  //Set sign coeff
            gl_ush_flashParamSignCoeff = gl_acInputBuffer[2] + ( ( ( short) gl_acInputBuffer[3]) << 8);
            gl_nSentAddParamIndex = SIGNCOEFF;
          break;

          case DEVNUM:    //Set device
            gl_ush_flashParamDeviceId = gl_acInputBuffer[2] + ( ( ( short) gl_acInputBuffer[3]) << 8);
            gl_nSentAddParamIndex = DEVNUM;
          break;

          /*
          case DEVNUM_BH:  //Set device id high byte
            gl_ush_flashParamDeviceId &= ( 0xFF00);
            gl_ush_flashParamDeviceId &= ( ( ( short) gl_acInputBuffer[2]) << 8);
            gl_nSentAddParamIndex = DEVNUM_BH;
          break;
          */

          case DATE_Y:    //Set Date.YEAR
            gl_ush_flashParamDateYear = gl_acInputBuffer[2] + ( ( ( short) gl_acInputBuffer[3]) << 8);
            gl_nSentAddParamIndex = DATE_Y;
          break;

          case DATE_M:    //Set Date.MONTH
            gl_ush_flashParamDateMonth = gl_acInputBuffer[2];
            gl_nSentAddParamIndex = DATE_Y;
          break;

          case DATE_D:    //Set Date.DAY
            gl_ush_flashParamDateDay = gl_acInputBuffer[2];
            gl_nSentAddParamIndex = DATE_Y;
          break;

          case ORG_B1:    //установить Organization.byte1
            gl_ac_flashParamOrg[0] = gl_acInputBuffer[2];  break;
          case ORG_B2:    //установить Organization.byte2
            gl_ac_flashParamOrg[1] = gl_acInputBuffer[2];  break;
          case ORG_B3:    //установить Organization.byte3
            gl_ac_flashParamOrg[2] = gl_acInputBuffer[2];  break;
          case ORG_B4:    //установить Organization.byte4
            gl_ac_flashParamOrg[3] = gl_acInputBuffer[2];  break;
          case ORG_B5:    //установить Organization.byte5
            gl_ac_flashParamOrg[4] = gl_acInputBuffer[2];  break;
          case ORG_B6:    //установить Organization.byte6
            gl_ac_flashParamOrg[5] = gl_acInputBuffer[2];  break;
          case ORG_B7:    //установить Organization.byte7
            gl_ac_flashParamOrg[6] = gl_acInputBuffer[2];  break;
          case ORG_B8:    //установить Organization.byte8
            gl_ac_flashParamOrg[7] = gl_acInputBuffer[2];  break;
          case ORG_B9:    //установить Organization.byte9
            gl_ac_flashParamOrg[8] = gl_acInputBuffer[2];  break;
          case ORG_B10:   //установить Organization.byte10
            gl_ac_flashParamOrg[9] = gl_acInputBuffer[2];  break;
          case ORG_B11:   //установить Organization.byte11
            gl_ac_flashParamOrg[10] = gl_acInputBuffer[2]; break;
          case ORG_B12:   //установить Organization.byte12
            gl_ac_flashParamOrg[11] = gl_acInputBuffer[2]; break;
          case ORG_B13:   //установить Organization.byte13
            gl_ac_flashParamOrg[12] = gl_acInputBuffer[2]; break;
          case ORG_B14:   //установить Organization.byte14
            gl_ac_flashParamOrg[13] = gl_acInputBuffer[2]; break;
          case ORG_B15:   //установить Organization.byte15
            gl_ac_flashParamOrg[14] = gl_acInputBuffer[2]; break;
          case ORG_B16:   //установить Organization.byte16
            gl_ac_flashParamOrg[15] = gl_acInputBuffer[2]; break;

          case PH_SH_CALIB_T:     //Калибровка фазового сдвига. Точка N. Температура            0x39  "9"
            gl_ac_calib_phsh_t[ gl_acInputBuffer[2]] = gl_acInputBuffer[3];
            gl_cFlashParamPhaseShiftUsage = 0xFF;
          break;

          case PH_SH_CALIB_PH_SH: //Калибровка фазового сдвига. Точка N. Соотв. фазовый сдвиг   0x3A  ":"
            gl_ac_calib_phsh_phsh[ gl_acInputBuffer[2]] = gl_acInputBuffer[3];
            gl_cFlashParamPhaseShiftUsage = 0xFF;
          break;

          case PH_SH_CURRENT_VAL: //Фазовый сдвиг. Текущее значение.                            0x3B  ";"
            gl_nCurrentPhaseShift = gl_acInputBuffer[2] + 0x100;
            gl_lCalibratedPhaseShiftApplySecs = gl_lSecondsFromStart;
          break;

          case PH_SH_USAGE:       //Калибровка фазового сдвига. Использование                   0x3D  "="
            if( gl_acInputBuffer[2] == 0x00) {
              for( i=0; i<10; i++) {
                for( j=0; j<10; j++) {

                  if( gl_ac_calib_phsh_t[ j] > gl_ac_calib_phsh_t[ j + 1]) {
                    c_t = gl_ac_calib_phsh_t[ j + 1];
                    gl_ac_calib_phsh_t[ j + 1] = gl_ac_calib_phsh_t[ j];
                    gl_ac_calib_phsh_t[ j] = c_t;

                    c_phsh = gl_ac_calib_phsh_phsh[ j + 1];
                    gl_ac_calib_phsh_phsh[ j + 1] = gl_ac_calib_phsh_phsh[ j];
                    gl_ac_calib_phsh_phsh[ j] = c_phsh;
                  }
                }
              }
            }

            for( i=0; i<11; i++) {
              if( gl_ac_calib_phsh_t[i] != 0xFF && gl_ac_calib_phsh_phsh[i] != 0xFF) {
                gl_cFlashParamPhaseShiftUsage = gl_acInputBuffer[2];
                break;
              }
            }
            gl_nSentAddParamIndex = PH_SH_USAGE;
          break;

          case SLG_ADDITIONAL_PARAM_LIST_ELEMENT: //элемент списка выдаваемых наружу аналоговых (доп.) параметров
            if( gl_acInputBuffer[2] >= 0 && gl_acInputBuffer[2] < 12) {
              gl_aushListOutputAddParams[ gl_acInputBuffer[2]] = gl_acInputBuffer[3];
              gl_nSentAddParamIndex = SLG_ADDITIONAL_PARAM_LIST_ELEMENT;
              gl_nSentAddParamSubIndex = gl_acInputBuffer[2];
            }
          break;

          case DC_CALIB_T:        //Калибровка коэффициента вычета. Точка N. Температура            0x3F  "?"
            gl_ac_calib_dc_t[ gl_acInputBuffer[2]] = gl_acInputBuffer[3];
            //reset setting: при старте брать DC_START, а перевычислять на лету
            gl_ucDcUsageStartSetting = 0;
            gl_ucDcUsageRecalc = 0;
          break;

          case DC_CALIB_DC_L:       //Калибровка коэффициента вычета. Точка N. Соотв. Квычета. мл. байт  0x40  "@"
            memcpy( &(gl_ush_calib_dc_dc[ gl_acInputBuffer[2]]), &gl_acInputBuffer[3], 1);
            //reset setting: при старте брать DC_START, а перевычислять на лету
            gl_ucDcUsageStartSetting = 0;
            gl_ucDcUsageRecalc = 0;
          break;

          case DC_CALIB_DC_H:       //Калибровка коэффициента вычета. Точка N. Соотв. Квычета. ст. байт  0x41  "A"
            memcpy( ( ( char *)&gl_ush_calib_dc_dc[ gl_acInputBuffer[2]])+1, &gl_acInputBuffer[3], 1);
            //reset setting: при старте брать DC_START, а перевычислять на лету
            gl_ucDcUsageStartSetting = 0;
            gl_ucDcUsageRecalc = 0;
          break;

          case DC_SETTINGS_START:   //Настройки использования коэффициента вычета: что брать при старте: 0=брать стартовый dc: REST=брать из таблицы калибровки
            gl_ucDcUsageStartSetting = gl_acInputBuffer[2];
            if( gl_ucDcUsageStartSetting > 1) gl_ucDcUsageStartSetting = 0;

            if( gl_ucDcUsageStartSetting == 1) {
              //на использование таблицы можно переключиться только если есть хотя бы одна валидная точка
              //сортируем таблицу по возрастанию температуры
              for( i=0; i<10; i++) {
                for( j=0; j<10; j++) {

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

              //если не найдём - то будем использовать стартовый DC
              gl_ucDcUsageStartSetting = 0;

              //ищем хотя бы одну не default-точку
              for( i=0; i<11; i++) {
                if( gl_ac_calib_dc_t[i] != 0xFF && gl_ush_calib_dc_dc[i] != 0xFFFF) {
                  //нашли! значит можно использовать калибровку!
                  gl_ucDcUsageStartSetting = 1;
                  break;
                }
              }
            }
            gl_nSentAddParamIndex = DC_SETTINGS_START;
          break;

          case DC_SETTINGS_RECALC:  //Настройки использования коэффициента вычета: как переопределять в процессе работы
                                    //  0=перевычисление на лету
                                    //  1=перевычисление ступенчатая
                                    //  2=калибровка сглаженная
                                    //  3=ручной режим

            if( gl_acInputBuffer[2] > 3) gl_acInputBuffer[2] = 0;

            if( gl_acInputBuffer[2] == 1 || gl_ucDcUsageRecalc == 2) {
              //на использование таблицы можно переключиться только если есть хотя бы две валидных точки
              //сортируем таблицу по возрастанию температуры
              for( i=0; i<10; i++) {
                for( j=0; j<10; j++) {

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

              c_cnt = 0;
              //ищем хотя бы две не default-точку
              for( i=0; i<11; i++) {
                if( gl_ac_calib_dc_t[i] != 0xFF && gl_ush_calib_dc_dc[i] != 0xFFFF) {
                  if( ++c_cnt == 2) {
                    //нашли! значит можно переключаться
                    gl_ucDcUsageRecalc = gl_acInputBuffer[2];
                    break;
                  }
                }
              }
            }
            else {
              //если заказывают manual regime или перевычисление
              gl_ucDcUsageRecalc = gl_acInputBuffer[2];
            }
            gl_nSentAddParamIndex = DC_SETTINGS_RECALC;
          break;

          case DECCOEFF_START:
            gl_ush_flashParamStartDecCoeff = gl_acInputBuffer[2] + ( ( ( short) gl_acInputBuffer[3]) << 8);
          break;

          case DC_SETTINGS_RECALC_PERIOD:
            gl_ushDcUsageRecalcPeriod = gl_acInputBuffer[2] + ( ( ( unsigned short) gl_acInputBuffer[3]) << 8);
            gl_nSentAddParamIndex = DC_SETTINGS_RECALC_PERIOD;
          break;

          case RULA:    //RULA
            if( gl_cAmplRegulation == 0) {
              gl_un_RULAControl = gl_acInputBuffer[2] + ( ( ( short) gl_acInputBuffer[3]) << 8);
            }
          break;

          case AMPL_HOLD_ACTIVE:
            if( gl_acInputBuffer[2] >= 0 && gl_acInputBuffer[2] <= 2) {
              gl_cAmplRegulation = gl_acInputBuffer[2];
            }
          break;
        }
      break;

      //**************************************************************************** REQ
      case MC_COMMAND_REQ:
        switch( gl_acInputBuffer[1]) {
          case ORG:
            gl_nSentAddParamIndex = ORG_B1;
          break;

          case SLG_ADDITIONAL_PARAM_LIST_ELEMENT: //элемент списка выдаваемых наружу аналоговых (доп.) параметров
            if( gl_acInputBuffer[2] >= 0 && gl_acInputBuffer[2] < 12) {
              gl_nSentAddParamIndex = SLG_ADDITIONAL_PARAM_LIST_ELEMENT;
              gl_nSentAddParamSubIndex = gl_acInputBuffer[2];
            }
          break;

          case PH_SH_CALIB_T: //калибровка фазового сдвига. температура
            if( gl_acInputBuffer[2] >= 0 && gl_acInputBuffer[2] < 11) {
              gl_nSentAddParamIndex = PH_SH_CALIB_T;
              gl_nSentAddParamSubIndex = gl_acInputBuffer[2];
            }
          break;

          case PH_SH_CALIB_PH_SH: //калибровка фазового сдвига. фазовый сдвиг
            if( gl_acInputBuffer[2] >= 0 && gl_acInputBuffer[2] < 11) {
              gl_nSentAddParamIndex = PH_SH_CALIB_PH_SH;
              gl_nSentAddParamSubIndex = gl_acInputBuffer[2];
            }
          break;

          case DC_CALIB_T:        //калибровка коэффициента вычета. температура
            if( gl_acInputBuffer[2] >= 0 && gl_acInputBuffer[2] < 11) {
              gl_nSentAddParamIndex = DC_CALIB_T;
              gl_nSentAddParamSubIndex = gl_acInputBuffer[2];
            }
          break;

          case DC_CALIB_DC_L:     //калибровка коэффициента вычета. коэффициент вычета. младший байт
            if( gl_acInputBuffer[2] >= 0 && gl_acInputBuffer[2] < 11) {
              gl_nSentAddParamIndex = DC_CALIB_DC_L;
              gl_nSentAddParamSubIndex = gl_acInputBuffer[2];
            }
          break;

          case DC_CALIB_DC_H:     //калибровка коэффициента вычета. коэффициент вычета. старший байт
            if( gl_acInputBuffer[2] >= 0 && gl_acInputBuffer[2] < 11) {
              gl_nSentAddParamIndex = DC_CALIB_DC_H;
              gl_nSentAddParamSubIndex = gl_acInputBuffer[2];
            }
          break;


          default:
            gl_nSentAddParamIndex = gl_acInputBuffer[1];
        }
      break;


      //**************************************************************************** REST
      case MC_COMMAND_ACT_SWC_DW_DNDU_OUTPUT: //Within async mode switch DN,DU or dW output
        if( gl_b_SyncMode == 1) {
          if( gl_acInputBuffer[1] == 0) gl_chAngleOutput = 0; //switch to dW output
          if( gl_acInputBuffer[1] == 1) gl_chAngleOutput = 1; //switch to dNdU output
        }
      break;

      case MC_COMMAND_ACT_T_CALIBRATION: //Thermo calibration (parameter here is current temperature)
        gl_bTDCalibrated = 0;
        in_param_temp  = gl_acInputBuffer[1] + ( ( ( short) gl_acInputBuffer[2]) << 8);
        if( gl_ssh_flashParam_calibT1 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) && 
            gl_ssh_flashParam_calibT1 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
            //у нас есть нормальная минимальная точка калибровки

            //затычка на то, чтобы не давали мин точку равной макс
            if( in_param_temp == gl_ssh_flashParam_calibT1) {
              gl_nSentAddParamIndex = CALIB_T1;
              break;
            }

            if( gl_ssh_flashParam_calibT2 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION)  &&
              gl_ssh_flashParam_calibT2 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
              //у нас есть нормальные минимальная и максимальная точка калибровки
              //определим какую надо заменить
              if( in_param_temp < gl_ssh_flashParam_calibT1) {
                //надо заменить минимальную
                gl_cCalibProcessState = 1;
                gl_ssh_flashParam_calibT1 = in_param_temp;
              }
              else {
                //надо заменить максимальную
                gl_cCalibProcessState = 4;
                gl_ssh_flashParam_calibT2 = in_param_temp;
              }
            }
            else {
              //у нас есть нормальная минимальная точка калибровки, но нет нормальной максимальной
              gl_cCalibProcessState = 3;
              gl_ssh_flashParam_calibT2 = in_param_temp;
            }

        }
        else {
          //у нас нет даже минимальной точки!! значит это будет минимальная :)
          gl_ssh_flashParam_calibT1 = in_param_temp;
          gl_cCalibProcessState = 1;
        }
      break;

      case MC_COMMAND_ACT_RESET_T_CALIB:    //Reset thermo calibration data
        gl_bTDCalibrated = 0;
        gl_ssh_flashParam_calibT1 = 0;
        gl_ush_flashParamT1_TD1_val = 0;
        gl_ush_flashParamT1_TD2_val = 0;
        gl_ush_flashParamT1_TD3_val = 0;

        gl_ssh_flashParam_calibT2 = 0;
        gl_ush_flashParamT2_TD1_val = 1;
        gl_ush_flashParamT2_TD2_val = 1;
        gl_ush_flashParamT2_TD3_val = 0;

        save_params_p4();
        gl_nSentAddParamIndex = CALIB_T1;
      break;

      case MC_COMMAND_ACT_RESET_PHSH_CALIB:    //Reset phase shift calibration
        gl_cFlashParamPhaseShiftUsage = 0xFF;
        for( i=0; i<11; i++) {
          gl_ac_calib_phsh_t[i] = 0xFF;
          gl_ac_calib_phsh_phsh[i] = 0xFF;
        }

        gl_nSentAddParamIndex = PH_SH_USAGE;
      break;


      case MC_COMMAND_ACT_RESET_DC_CALIB:
        //ставим стартовое - брать START
        //использование калибровки - перевычисление
        gl_ucDcUsageStartSetting= 0;
        gl_ucDcUsageRecalc = 0;

        for( i=0; i<11; i++) {
          gl_ac_calib_dc_t[i] = 0xFF;
          gl_ush_calib_dc_dc[i] = 0xFFFF;
        }
      break;

      case MC_COMMAND_ACT_LASER_OFF:    //Laser turn off
        //выключаем высокое 800V
        GP4DAT |= ( 1 << (16 + 0));   //ONHV   (p4.0) = 1
        //отключаем поджиг 3kV
        GP4DAT |= ( 1 << (16 + 1));   //OFF3KV (p4.1) = 1
      break;

      case MC_COMMAND_ACT_INTEGR_OFF: //Integrator turn off
        GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1 (выключение)
        gl_n_PerimeterReset = 1;      //устанавливаем флаг недостоверности данных (1=прошло выключение, данные НЕдостоверны)
        gl_nRppTimerT2 = 0;           //<-- ВРЕМЯ НЕ ЗАСЕКАЕМ! ФЛАГ НЕ ОПУСТИТСЯ НИКОГДА (только по команде включить интегратор)
      break;

      case MC_COMMAND_ACT_INTEGR_ON:  //Integrator turn on
        GP0DAT &= ~( 1 << (16 + 5));  //RP_P   (p0.5) = 0 (включение)
        gl_n_PerimeterReset = 2;      //устанавливаем флаг недостоверности данных (2=прошло включение, данные НЕдостоверны)
        gl_nRppTimerT2 = T1VAL;       //<-- ЗАСЕКАЕМ ВРЕМЯ! И ЧЕРЕЗ [определённое время] ФЛАГ БУДЕТ СНЯТ
      break;

      case MC_COMMAND_ACT_INTEGR_RESET:    //Integrator reset
        GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1 (выключение)

        gl_nRppTimerT2 = T1VAL;       //<-- ЗАСЕКАЕМ ВРЕМЯ! И ЧЕРЕЗ [определённое время] ИНТЕГРАТОР ВКЛЮЧИТСЯ, ФЛАГ перейдёт в состояние 2, а ещё через [определённое время] флаг перейдёт в состояние 0 (данные достоверны)
        gl_n_PerimeterReset = 1;      //устанавливаем флаг недостоверности данных (1=прошло выключение, данные НЕдостоверны)
      break;

      case MC_COMMAND_ACT_SAVE_FLASH_PARAMS:
        switch( gl_acInputBuffer[1]) {
          case 0: gl_ushFlashParamLastRULA = gl_un_RULAControl; save_params_p1(); break;
          case 1: save_params_p2(); break;
          case 2: save_params_p3(); break;
          case 3: save_params_p4(); break;
        }
      break;

      case MC_COMMAND_ACT_RELOAD_FLASH_PARAMS:
        switch( gl_acInputBuffer[1]) {
          case 0: load_params_p1(); check_params_p1(); break;
          case 1: load_params_p2(); check_params_p2(); break;
          case 2: load_params_p3(); check_params_p3(); break;
          case 3: load_params_p4(); check_params_p4(); break;
          case 4: load_params();                       break;
        }
      break;

      case MC_COMMAND_ACT_LOCK_DEVICE:
        if( gl_acInputBuffer[1] == 0x55 &&
            gl_acInputBuffer[2] == 0x5A &&
            gl_acInputBuffer[3] == 0x55) {
#ifdef DEBUG
  printf("DBG: After saving page0 device will be locked\n");
#endif
                  //gl_chLockBit = 1;     прям сразу её ставить нельзя, а то нельзя будет сохранить блокировку! :)
                  gl_ush_flashLockDev = 1;
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

    if( gl_cPos_in_in_buf != 0) {

#ifdef DEBUG
      putchar_nocheck( '4');
      //printf("DBG: PIC: out\n");
#endif

      for( i=0; i<6; gl_acInputBuffer[ i++] = 0);
      gl_cPos_in_in_buf = 0;
    }
  }



  //gl_cPos_in_in_buf = 0;

//#ifdef DEBUG
  //printf("DBG: PIC: out\n");
//#endif
}
