#ifndef T39_M3A_ANALOGUE_PARAMETERS_LIST
#define T39_M3A_ANALOGUE_PARAMETERS_LIST


#define UTD1                0   //����������� 1
#define UTD2                1   //����������� 2
#define UTD3                2   //����������� 3
#define I1                  3   //��������� ��� i1
#define I2                  4   //��������� ��� i2
#define CNTRPC              5   //���������� �� ����������������
#define AMPLANG_ALTERA      6   //��������� ���������� �� alter'�
#define AMPLANG_DUS         7   //��������� ���������� � ����
#define RULA                8   //���������� RULA

#define AMPLITUDE           9   //�������� ��������� ���������
#define TACT_CODE           10  //��� ����� ���������
#define M_COEFF             11  //����������� M
#define STARTMODE           12  //��������� ����
#define DECCOEFF            13  //����������� ������

#define CONTROL_I1          14  //����������� ��� ������� I1
#define CONTROL_I2          15  //����������� ��� ������� I2
#define CONTROL_AA          16  //����������� ������ �������� AmplAng

#define HV_APPLY_COUNT_SET  17  //�������� ���-�� HV ������
#define HV_APPLY_COUNT_TR   18  //���-�� HV ������ � ���� �������
#define HV_APPLY_DURAT_SET  19  //�������� ������������ HV ������
#define HV_APPLY_PACKS      52  //���������� ����� 3kV ��������� �������

#define SIGNCOEFF           20  //�������� �����������
#define DEVNUM              21  //����� �������
//#define DEVNUM              22  //����� �������. ������� ����

#define DATE_Y              23  //����.���
#define DATE_M              24  //����.�����
#define DATE_D              25  //����.����

#define ORG                 26  //����������� ������� (��� ������� �������)
#define ORG_B1              27  //�����������.����1
#define ORG_B2              28  //�����������.����2
#define ORG_B3              29  //�����������.����3
#define ORG_B4              30  //�����������.����4
#define ORG_B5              31  //�����������.����5
#define ORG_B6              32  //�����������.����6
#define ORG_B7              33  //�����������.����7
#define ORG_B8              34  //�����������.����8
#define ORG_B9              35  //�����������.����9
#define ORG_B10             36  //�����������.����10
#define ORG_B11             37  //�����������.����11
#define ORG_B12             38  //�����������.����12
#define ORG_B13             39  //�����������.����13
#define ORG_B14             40  //�����������.����14
#define ORG_B15             41  //�����������.����15
#define ORG_B16             42  //�����������.����16 ��� ������������ 0 �� �����!

#define VERSION             43  //������

#define CALIB_T1            44  //����������� ������ ������������� ����� ����������
#define T1_TD1              45  //��������� ������� TD1 �� ������ ����. �����
#define T1_TD2              46  //��������� ������� TD2 �� ������ ����. �����
#define T1_TD3              47  //��������� ������� TD3 �� ������ ����. �����

#define CALIB_T2            48  //����������� ������� ������������� ����� ����������
#define T2_TD1              49  //��������� ������� TD1 �� ������� ����. �����
#define T2_TD2              50  //��������� ������� TD2 �� ������� ����. �����
#define T2_TD3              51  //��������� ������� TD3 �� ������� ����. �����
//LAST                      52 - HV_APPLY_PACKS

/*

#define SENDING_UTD1            0
#define SENDING_UTD2            1
#define SENDING_UTD3            2
#define SENDING_I1              3
#define SENDING_I2              4
#define SENDING_CNTRPC          5
#define SENDING_AMPLANG_ALTERA  6
#define SENDING_AMPLANG_DUS     7

#define SENDING_AMPLITUDE       8 //��� ���������
#define SENDING_TACT_CODE       9 //��� ����� ���������
#define SENDING_M_COEFF         10 //����������� M
#define SENDING_STARTMODE       11 //��������� ����
#define SENDING_DECCOEFF        12 //����������� ������
#define SENDING_SIGNCOEFF       13 //�������� �����������

#define SENDING_VERSION         14 //������

#define SENDING_CONTROL_I1      15 //����������� ��� ������� I1
#define SENDING_CONTROL_I2      16 //����������� ��� ������� I2
#define SENDING_CONTROL_AA      17 //����������� ������ �������� AmplAng


#define SENDING_CALIB_T1        18 //����������� ������ ������������� ����� ����������
#define SENDING_T1_TD1          19 //��������� ������� TD1 �� ������ ����. �����
#define SENDING_T1_TD2          20 //��������� ������� TD2 �� ������ ����. �����
#define SENDING_T1_TD3          21 //��������� ������� TD3 �� ������ ����. �����

#define SENDING_CALIB_T2        22 //����������� ������� ������������� ����� ����������
#define SENDING_T2_TD1          23 //��������� ������� TD1 �� ������� ����. �����
#define SENDING_T2_TD2          24 //��������� ������� TD2 �� ������� ����. �����
#define SENDING_T2_TD3          25 //��������� ������� TD3 �� ������� ����. �����
#define SENDING_DEVNUM_BL       26 //����� �������. ������� ����
#define SENDING_DEVNUM_BH       27 //����� �������. ������� ����
#define SENDING_ORG_B1          28 //�����������.����1
#define SENDING_ORG_B2          29 //�����������.����2
#define SENDING_ORG_B3          30 //�����������.����3
#define SENDING_ORG_B4          31 //�����������.����4
#define SENDING_ORG_B5          32 //�����������.����5
#define SENDING_ORG_B6          33 //�����������.����6
#define SENDING_ORG_B7          34 //�����������.����7
#define SENDING_ORG_B8          35 //�����������.����8
#define SENDING_ORG_B9          36 //�����������.����9
#define SENDING_ORG_B10         37 //�����������.����10
#define SENDING_ORG_B11         38 //�����������.����11
#define SENDING_ORG_B12         39 //�����������.����12
#define SENDING_ORG_B13         40 //�����������.����13
#define SENDING_ORG_B14         41 //�����������.����14
#define SENDING_ORG_B15         42 //�����������.����15
#define SENDING_ORG_B16         43 //�����������.����16 ��� ������������ 0 �� �����!

#define SENDING_DATE_Y          44 //����.���
#define SENDING_DATE_M          45 //����.�����
#define SENDING_DATE_D          46 //����.����

#define SENDING_HV_APPLY_THIS_RUN 47 //������������ ���������� 3kV ��� ���������
#define SENDING_HV_APPLY_SET    48 //������� ������������ ���������� 3kV ��� ���������
*/
#endif