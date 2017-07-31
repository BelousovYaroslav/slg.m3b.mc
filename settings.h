#ifndef SETTINGS_H
#define SETTINGS_H

#define MIN_T_THERMO_CALIBRATION -60
#define MAX_T_THERMO_CALIBRATION 60
#define THERMO_CALIB_PARAMS_BASE 10000

void load_params( void);                        //��������� ��������� �� ����-������ � ����������
void load_params_p1( void);
void load_params_p2( void);
void load_params_p3( void);
void load_params_p4( void);

void save_params( void);                        //��������� ��������� �� ���������� �� ����-������
void save_params_p1( void);
void save_params_p2( void);
void save_params_p3( void);
void save_params_p4( void);                     //��������� ������������� ������ �������������

void check_params_p1( void);
void check_params_p2( void);
void check_params_p3( void);
void check_params_p4( void);

// ADRESSES FOR FLASH-STORED PARAMS. PAGE 1
#define ADDR_PAGE1            0xF000
#define ADDR_AMPLITUDE        0xF000
#define ADDR_TACT_CODE        0xF002
#define ADDR_M_COEFF          0xF004
#define ADDR_START_MODE       0xF006
#define ADDR_DEC_COEFF        0xF008
#define ADDR_LOCK_DEV         0xF00A
#define ADDR_LAST_RULA        0xF00C
#define ADDR_LAST_RULM        0xF00E

// ADRESSES FOR FLASH-STORED PARAMS. PAGE 2
#define ADDR_PAGE2            0xF200
#define ADDR_CONTROL_I1       0xF200
#define ADDR_CONTROL_I2       0xF202
#define ADDR_CONTROL_AA       0xF204
#define ADDR_HV_APPLY_C       0xF206
#define ADDR_HV_APPLY_D       0xF208
#define ADDR_HV_APPLY_P       0xF20A

// ADRESSES FOR FLASH-STORED PARAMS. PAGE 3
#define ADDR_PAGE3            0xF400
#define ADDR_SIGN_COEFF       0xF400
#define ADDR_DEVICE_ID        0xF402
#define ADDR_DATE_Y           0xF404
#define ADDR_DATE_M           0xF406
#define ADDR_DATE_D           0xF408
#define ADDR_ORG              0xF40A    //16 bytes length

// ADRESSES FOR FLASH-STORED PARAMS. PAGE 4   ���������� ������������� ��������
#define ADDR_PAGE4            0xF600
#define ADDR_TCALIB_T1        0xF600
#define ADDR_TCALIB_T1_TD1    0xF602
#define ADDR_TCALIB_T1_TD2    0xF604
#define ADDR_TCALIB_T1_TD3    0xF606
#define ADDR_TCALIB_T2        0xF608
#define ADDR_TCALIB_T2_TD1    0xF60A
#define ADDR_TCALIB_T2_TD2    0xF60C
#define ADDR_TCALIB_T2_TD3    0xF60E

// ADRESSES FOR FLASH-STORED PARAMS. PAGE 5   ���������� �������� ������
#define ADDR_PAGE5              0xF800
#define ADDR_PHSH_CALIB_T1      0xF800
#define ADDR_PHSH_CALIB_PHSH1   0xF802
#define ADDR_PHSH_CALIB_T2      0xF804
#define ADDR_PHSH_CALIB_PHSH2   0xF806
#define ADDR_PHSH_CALIB_T3      0xF808
#define ADDR_PHSH_CALIB_PHSH3   0xF80A
#define ADDR_PHSH_CALIB_T4      0xF80C
#define ADDR_PHSH_CALIB_PHSH4   0xF80E
#define ADDR_PHSH_CALIB_T5      0xF800
#define ADDR_PHSH_CALIB_PHSH5   0xF802
#define ADDR_PHSH_CALIB_T6      0xF804
#define ADDR_PHSH_CALIB_PHSH6   0xF806
#define ADDR_PHSH_CALIB_T7      0xF808
#define ADDR_PHSH_CALIB_PHSH7   0xF80A
#define ADDR_PHSH_CALIB_T8      0xF80C
#define ADDR_PHSH_CALIB_PHSH8   0xF80E
#define ADDR_PHSH_CALIB_T9      0xF810
#define ADDR_PHSH_CALIB_PHSH9   0xF812
#define ADDR_PHSH_CALIB_T10     0xF814
#define ADDR_PHSH_CALIB_PHSH10  0xF816
#define ADDR_PHSH_CALIB_T11     0xF818
#define ADDR_PHSH_CALIB_PHSH11  0xF81A
#endif