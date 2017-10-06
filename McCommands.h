#ifndef T39_M3A_MC_COMMAND_SET
#define T39_M3A_MC_COMMAND_SET

#define MC_COMMAND_SET                        33    //33 = 0x21 = "!"     ��������� �������� ���������. Param1: �������� ��������� -> ����� �������� ��������
#define MC_COMMAND_REQ                        34    //34 = 0x22 = """     ��������� �������� ���������. Param1: �������� ��������� -> ����� �������� ��������

#define MC_COMMAND_ACT_SAVE_FLASH_PARAMS      35    //35 = 0x23 = "#"    Param: 0-1-2-3  - page to save
#define MC_COMMAND_ACT_RELOAD_FLASH_PARAMS    36    //36 = 0x24 = "$"    Param: 0-1-2-3  - page to reload

#define MC_COMMAND_ACT_T_CALIBRATION          37    //37 = 0x25 = "%"     ��������� ���������� ������������� �� ������� �����������. �������� - ������� �����������
#define MC_COMMAND_ACT_RESET_T_CALIB          38    //38 = 0x26 = "&"     �������� ������ ���������� �������������. ��� ����������.

#define MC_COMMAND_ACT_LASER_OFF              39    //39 = 0x27 = "'"     ��������� ����� (HV_ON, HV_3KV  --> off)
#define MC_COMMAND_ACT_INTEGR_OFF             40    //40 = 0x28 = "("     ��������� ���������� ������� ����������� ���������
#define MC_COMMAND_ACT_INTEGR_ON              41    //41 = 0x29 = ")"     �������� ���������� ������� ����������� ���������
#define MC_COMMAND_ACT_INTEGR_RESET           42    //42 = 0x2A = "*"     �������� ���������� ������� ����������� ��������� (����, �����, ���, �����)

#define MC_COMMAND_ACT_SWC_DW_DNDU_OUTPUT     43    //43 = 0x2B = "+"     ����������� ���������� �������� ���������� ���� (���������� ���/���� ������������� ������������ ������)

#define MC_COMMAND_ACT_LOCK_DEVICE            44    //44 = 0x2C = ","     ��������� ������ � ����� ������������
#define MC_COMMAND_ACT_UNLOCK_DEVICE          45    //45 = 0x2D = "-"     ��������� ������ � ����� ������������

#define MC_COMMAND_SWITCH_TO_MAX_RATE_DNDU    46    //46 = 0x2E = "."     ������������� � ����� ������ dNdU � ����������� ��������� �������� (�������� ����������-�������)

#define MC_COMMAND_ACT_RESET_PHSH_CALIB       47    //47 = 0x2F = "/"     �������� ������ ���������� �������� ������
#define MC_COMMAND_ACT_RESET_DC_CALIB         48    //48 = 0x30 = "0"     �������� ������ ���������� ������������ ������
#endif