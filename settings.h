#ifndef SETTINGS_H
#define SETTINGS_H

#define MIN_T_THERMO_CALIBRATION -60
#define MAX_T_THERMO_CALIBRATION 60
#define THERMO_CALIB_PARAMS_BASE 10000

void load_params( void);                        //загрузить параметры из флэш-памяти в переменные
void load_params_p1( void);
void load_params_p2( void);
void load_params_p3( void);
void load_params_p4( void);

void save_params( void);                        //сохранить параметры из переменных во флэш-память
void save_params_p1( void);
void save_params_p2( void);
void save_params_p3( void);
void save_params_p4( void);


void check_params_p1( void);
void check_params_p2( void);
void check_params_p3( void);
void check_params_p4( void);

// ADDRESSES FOR FLASH-STORED PARAMS. PAGE 1
#define ADDR_PAGE1            0xF000
#define ADDR_AMPLITUDE        0xF000
#define ADDR_TACT_CODE        0xF002
#define ADDR_M_COEFF          0xF004
#define ADDR_START_MODE       0xF006
#define ADDR_DEC_COEFF_START  0xF008
#define ADDR_LOCK_DEV         0xF00A
#define ADDR_LAST_RULA        0xF00C
#define ADDR_LAST_RULM        0xF00E

// ADDRESSES FOR FLASH-STORED PARAMS. PAGE 2
#define ADDR_PAGE2            0xF200
#define ADDR_CONTROL_I1       0xF200
#define ADDR_CONTROL_I2       0xF202
#define ADDR_CONTROL_AA       0xF204
#define ADDR_HV_APPLY_C       0xF206
#define ADDR_HV_APPLY_D       0xF208
#define ADDR_HV_APPLY_P       0xF20A

// ADDRESSES FOR FLASH-STORED PARAMS. PAGE 3
#define ADDR_PAGE3            0xF400
#define ADDR_SIGN_COEFF       0xF400
#define ADDR_DEVICE_ID        0xF402
#define ADDR_DATE_Y           0xF404
#define ADDR_DATE_M           0xF406
#define ADDR_DATE_D           0xF408
#define ADDR_ORG              0xF40A    //16 bytes length

// ADDRESSES FOR FLASH-STORED PARAMS. PAGE 4
//список выдаваемых аналоговых (доп.) параметров
#define ADDR_PAGE4              0xF600
#define ADDR_LIST_PARAM1        0xF600
#define ADDR_LIST_PARAM2        0xF602
#define ADDR_LIST_PARAM3        0xF604
#define ADDR_LIST_PARAM4        0xF606
#define ADDR_LIST_PARAM5        0xF608
#define ADDR_LIST_PARAM6        0xF60A
#define ADDR_LIST_PARAM7        0xF60C
#define ADDR_LIST_PARAM8        0xF60E
#define ADDR_LIST_PARAM9        0xF610
#define ADDR_LIST_PARAM10       0xF612
#define ADDR_LIST_PARAM11       0xF614
#define ADDR_LIST_PARAM12       0xF616

//калибровочные данные термодатчиков
#define ADDR_TCALIB_T1          0xF618
#define ADDR_TCALIB_T1_TD1      0xF61A
#define ADDR_TCALIB_T1_TD2      0xF61C
#define ADDR_TCALIB_T1_TD3      0xF61E
#define ADDR_TCALIB_T2          0xF620
#define ADDR_TCALIB_T2_TD1      0xF622
#define ADDR_TCALIB_T2_TD2      0xF624
#define ADDR_TCALIB_T2_TD3      0xF626
#define ADDR_TCALIB_USAGE       0xF628

//калибровочные данные фазового сдвига
#define ADDR_PHSH_CALIB_T1      0xF62A
#define ADDR_PHSH_CALIB_PHSH1   0xF62C
#define ADDR_PHSH_CALIB_T2      0xF62E
#define ADDR_PHSH_CALIB_PHSH2   0xF630
#define ADDR_PHSH_CALIB_T3      0xF632
#define ADDR_PHSH_CALIB_PHSH3   0xF634
#define ADDR_PHSH_CALIB_T4      0xF636
#define ADDR_PHSH_CALIB_PHSH4   0xF638
#define ADDR_PHSH_CALIB_T5      0xF63A
#define ADDR_PHSH_CALIB_PHSH5   0xF63C
#define ADDR_PHSH_CALIB_T6      0xF63E
#define ADDR_PHSH_CALIB_PHSH6   0xF640
#define ADDR_PHSH_CALIB_T7      0xF642
#define ADDR_PHSH_CALIB_PHSH7   0xF644
#define ADDR_PHSH_CALIB_T8      0xF646
#define ADDR_PHSH_CALIB_PHSH8   0xF648
#define ADDR_PHSH_CALIB_T9      0xF64A
#define ADDR_PHSH_CALIB_PHSH9   0xF64C
#define ADDR_PHSH_CALIB_T10     0xF64E
#define ADDR_PHSH_CALIB_PHSH10  0xF650
#define ADDR_PHSH_CALIB_T11     0xF652
#define ADDR_PHSH_CALIB_PHSH11  0xF654
#define ADDR_PHSH_CALIB_USAGE   0xF656

//калибровочные данные коэффициента вычета
#define ADDR_DC_CALIB_T1        0xF658
#define ADDR_DC_CALIB_DC1       0xF65A
#define ADDR_DC_CALIB_T2        0xF65C
#define ADDR_DC_CALIB_DC2       0xF65E
#define ADDR_DC_CALIB_T3        0xF660
#define ADDR_DC_CALIB_DC3       0xF662
#define ADDR_DC_CALIB_T4        0xF664
#define ADDR_DC_CALIB_DC4       0xF666
#define ADDR_DC_CALIB_T5        0xF668
#define ADDR_DC_CALIB_DC5       0xF66A
#define ADDR_DC_CALIB_T6        0xF66C
#define ADDR_DC_CALIB_DC6       0xF66E
#define ADDR_DC_CALIB_T7        0xF670
#define ADDR_DC_CALIB_DC7       0xF672
#define ADDR_DC_CALIB_T8        0xF674
#define ADDR_DC_CALIB_DC8       0xF676
#define ADDR_DC_CALIB_T9        0xF678
#define ADDR_DC_CALIB_DC9       0xF67A
#define ADDR_DC_CALIB_T10       0xF67C
#define ADDR_DC_CALIB_DC10      0xF67E
#define ADDR_DC_CALIB_T11       0xF680
#define ADDR_DC_CALIB_DC11      0xF682
#define ADDR_DC_START_DEF       0xF684
#define ADDR_DC_RECALC          0xF686
#define ADDR_DC_RECALC_PERIOD   0xF688


// ADDRESSES FOR FLASH-STORED PARAMS. PAGE 5
// 2017.08.08 - у нас всего 4 страницы??



#endif