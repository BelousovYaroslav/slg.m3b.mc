#include <ADuC7026.h>
#include "Main.h"
#include "serial.h"

extern char gl_b_SA_Processed;                //���� ��������� ��������� ������� SA
extern signed short gl_ssh_angle_inc;         //���������� ����
extern signed short gl_ssh_angle_inc_prev;    //���������� ����
extern signed short gl_ssh_angle_hanger;      //���� ���������� �����-�������
extern signed short gl_ssh_angle_hanger_prev; //���������� �������� ���� ���������� �����-�������
extern char gl_b_SyncMode;                    //���� ������ ������ ���������:   0=�����. 1=������.


short gl_nSentPacksCounter;                   //������� �������

void SimpleMaxRateDnDuRegime( void) {
  char lb, hb;
  char cChkSm;
  signed short ssh_dN;
  signed short ssh_dU;

  gl_nSentPacksCounter = 0;

  while( !(GP0DAT & 0x10)) {}

    if( gl_b_SA_Processed == 0) {
      // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
      // 1. ����������� � ������� ��� �������� �������������� ���������
      //��� �������� ������ ������� ���������� CntReady (p4.2)
      //printf( "CNT_READY: %c", ( GP4DAT & 0x04) ? '1' : '0');
      while( !( GP4DAT & 0x04)) {}
      //putchar( ( GP4DAT & 0x04) ? '1' : '0');



      //����������� ������� ���� ���� �������� �������������� ���������
      //GP1SET = 1 << (16 + 3);  //RDHBC (p1.3) = 1			WAY1
      GP1DAT |= 1 << (16 + 3);  //RDHBC (p1.3) = 1			WAY2

      //pause( 1);                //�����

      //������
      hb = (( GP1DAT & BIT_5) >> 5) +
        ((( GP0DAT & BIT_7) >> 7) << 1) +
        ((( GP0DAT & BIT_1) >> 1) << 2) +
        ((( GP2DAT & BIT_3) >> 3) << 3) +
        ((( GP4DAT & BIT_6) >> 6) << 4) +
        ((( GP4DAT & BIT_7) >> 7) << 5) +
        ((( GP0DAT & BIT_6) >> 6) << 6) +
        ((( GP0DAT & BIT_2) >> 2) << 7);

      //GP1CLR = 1 << (16 + 3);  //RDHBC (p1.3) = 0				WAY1
      GP1DAT &= ~( 1 << (16 + 3));  //RDHBC (p1.3) = 0			WAY2


      //����������� ������� ���� ���� �������� �������������� ���������
      //GP1SET = 1 << (16 + 4);  //RDLBC (p1.4) = 1		WAY1
      GP1DAT |= 1 << (16 + 4);  //RDLBC (p1.4) = 1		WAY2

      //pause( 1);                //�����

      lb = (( GP1DAT & BIT_5) >> 5) +
          ((( GP0DAT & BIT_7) >> 7) << 1) +
          ((( GP0DAT & BIT_1) >> 1) << 2) +
          ((( GP2DAT & BIT_3) >> 3) << 3) +
          ((( GP4DAT & BIT_6) >> 6) << 4) +
          ((( GP4DAT & BIT_7) >> 7) << 5) +
          ((( GP0DAT & BIT_6) >> 6) << 6) +
          ((( GP0DAT & BIT_2) >> 2) << 7);

      //GP1CLR = 1 << (16 + 4);  //RDLBC (p1.4) = 0		WAY1
      GP1DAT &= ~( 1 << (16 + 4));  //RDLBC (p1.4) = 0		WAY2

      //���������� ��� �����
      gl_ssh_angle_inc_prev = gl_ssh_angle_inc;
      gl_ssh_angle_inc = lb + (hb << 8);

      // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
      // 2. � ������ ������ ������, ����������� � ������� ��� �������� �������������� ���������
      if( gl_b_SyncMode) {
        //��� �������� ������ ������� ���������� ANGLE_READY (p2.4)

        //printf( "\tANGLE_READY: %c", ( GP2DAT & 0x10) ? '1' : '0');
        while( !( GP2DAT & 0x10)) {}
        //putchar( ( GP2DAT & 0x10) ? '1' : '0');

        //����������� ������� ���� ���� �������� ���������
        GP0SET = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 1
        //pause( 1);                //�����

        //������
        hb =  (( GP1DAT & BIT_5) >> 5) +
              ((( GP0DAT & BIT_7) >> 7) << 1) +
               ((( GP0DAT & BIT_1) >> 1) << 2) +
               ((( GP2DAT & BIT_3) >> 3) << 3) +
               ((( GP4DAT & BIT_6) >> 6) << 4) +
               ((( GP4DAT & BIT_7) >> 7) << 5) +
               ((( GP0DAT & BIT_6) >> 6) << 6) +
               ((( GP0DAT & BIT_2) >> 2) << 7);

        GP0CLR = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 0


        //����������� ������� ���� ���� �������� �������������� ���������
        GP2SET = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 1
        //pause( 1);                //�����

        lb = (( GP1DAT & BIT_5) >> 5) +
              ((( GP0DAT & BIT_7) >> 7) << 1) +
             ((( GP0DAT & BIT_1) >> 1) << 2) +
             ((( GP2DAT & BIT_3) >> 3) << 3) +
             ((( GP4DAT & BIT_6) >> 6) << 4) +
             ((( GP4DAT & BIT_7) >> 7) << 5) +
             ((( GP0DAT & BIT_6) >> 6) << 6) +
             ((( GP0DAT & BIT_2) >> 2) << 7);

        GP2CLR = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 0

        //���������� ��� �����
        gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;
        gl_ssh_angle_hanger = lb + (hb << 8);

        if( gl_ssh_angle_hanger & 0x2000) {
          gl_ssh_angle_hanger = ( gl_ssh_angle_hanger & 0x3FFF) | 0xC000;
        }
        else
          gl_ssh_angle_hanger = ( gl_ssh_angle_hanger & 0x3FFF);

      }

      cChkSm = 0;

      putchar_nocheck( 0xCC);
      //putchar_nocheck( 0xDD);

      //N
      //putchar_nocheck( gl_ssh_angle_inc & 0xff);
      //putchar_nocheck( (gl_ssh_angle_inc & 0xff00) >> 8);

      

      ssh_dN = gl_ssh_angle_inc - gl_ssh_angle_inc_prev;
      ssh_dU = gl_ssh_angle_hanger- gl_ssh_angle_hanger_prev;

      //dN
      putchar_nocheck( ssh_dN & 0xff);              cChkSm += ( ssh_dN & 0xff);
      putchar_nocheck( ( ssh_dN & 0xff00) >> 8);    cChkSm += (( ssh_dN & 0xff00) >> 8);

      //U
      //putchar_nocheck( gl_ssh_angle_hanger & 0xff);
      //putchar_nocheck( ( gl_ssh_angle_hanger & 0xff00) >> 8);

      //dU
      putchar_nocheck( ssh_dU & 0xff);              cChkSm += ( ssh_dU & 0xff);
      putchar_nocheck( ( ssh_dU & 0xff00) >> 8);    cChkSm += (( ssh_dU & 0xff00) >> 8);

      //Counter
      gl_nSentPacksCounter = (++gl_nSentPacksCounter) % 0xFF;
      putchar_nocheck( gl_nSentPacksCounter & 0xff);   cChkSm += ( gl_nSentPacksCounter & 0xff);

      putchar_nocheck( cChkSm & 0xFF);

      //��������� ���� � ��� ��� ������� ������� ������� SA �� ����������
      gl_b_SA_Processed = 1;
    }
    else {
      while( (GP0DAT & 0x10)) {}
      gl_b_SA_Processed = 0;
    }
}