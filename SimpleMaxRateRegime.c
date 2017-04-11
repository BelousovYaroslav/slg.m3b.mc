#include <ADuC7026.h>
#include "Main.h"
#include "serial.h"

extern char gl_b_SA_Processed;                //флаг окончания обработки сигнала SA
extern signed short gl_ssh_angle_inc;         //приращение угла
extern signed short gl_ssh_angle_inc_prev;    //приращение угла
extern signed short gl_ssh_angle_hanger;      //угол отклонения вибро-подвеса
extern signed short gl_ssh_angle_hanger_prev; //предыдущее значение угла отклонения вибро-подвеса
extern char gl_b_SyncMode;                    //флаг режима работы гироскопа:   0=синхр. 1=асинхр.


short gl_nSentPacksCounter;                   //счетчик посылок

void SimpleMaxRateDnDuRegime( void) {
  char lb, hb;
  char cChkSm;
  signed short ssh_dN;
  signed short ssh_dU;

  gl_nSentPacksCounter = 0;

  while( !(GP0DAT & 0x10)) {}

    if( gl_b_SA_Processed == 0) {
      // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
      // 1. запрашиваем у альтеры код счётчика информационных импульсов
      //ждём высокого уровня сигнала готовности CntReady (p4.2)
      //printf( "CNT_READY: %c", ( GP4DAT & 0x04) ? '1' : '0');
      while( !( GP4DAT & 0x04)) {}
      //putchar( ( GP4DAT & 0x04) ? '1' : '0');



      //запрашиваем старший байт кода счётчика информационных импульсов
      //GP1SET = 1 << (16 + 3);  //RDHBC (p1.3) = 1			WAY1
      GP1DAT |= 1 << (16 + 3);  //RDHBC (p1.3) = 1			WAY2

      //pause( 1);                //пауза

      //чтение
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


      //запрашиваем младший байт кода счётчика информационных импульсов
      //GP1SET = 1 << (16 + 4);  //RDLBC (p1.4) = 1		WAY1
      GP1DAT |= 1 << (16 + 4);  //RDLBC (p1.4) = 1		WAY2

      //pause( 1);                //пауза

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

      //складываем два байта
      gl_ssh_angle_inc_prev = gl_ssh_angle_inc;
      gl_ssh_angle_inc = lb + (hb << 8);

      // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // ***** // *****
      // 2. в случае асинхр режима, запрашиваем у альтеры код счётчика информационных импульсов
      if( gl_b_SyncMode) {
        //ждём высокого уровня сигнала готовности ANGLE_READY (p2.4)

        //printf( "\tANGLE_READY: %c", ( GP2DAT & 0x10) ? '1' : '0');
        while( !( GP2DAT & 0x10)) {}
        //putchar( ( GP2DAT & 0x10) ? '1' : '0');

        //запрашиваем старший байт угла поворота вибратора
        GP0SET = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 1
        //pause( 1);                //пауза

        //чтение
        hb =  (( GP1DAT & BIT_5) >> 5) +
              ((( GP0DAT & BIT_7) >> 7) << 1) +
               ((( GP0DAT & BIT_1) >> 1) << 2) +
               ((( GP2DAT & BIT_3) >> 3) << 3) +
               ((( GP4DAT & BIT_6) >> 6) << 4) +
               ((( GP4DAT & BIT_7) >> 7) << 5) +
               ((( GP0DAT & BIT_6) >> 6) << 6) +
               ((( GP0DAT & BIT_2) >> 2) << 7);

        GP0CLR = 1 << (16 + 3);  //RDHBANGLE (p0.3) = 0


        //запрашиваем младший байт кода счётчика информационных импульсов
        GP2SET = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 1
        //pause( 1);                //пауза

        lb = (( GP1DAT & BIT_5) >> 5) +
              ((( GP0DAT & BIT_7) >> 7) << 1) +
             ((( GP0DAT & BIT_1) >> 1) << 2) +
             ((( GP2DAT & BIT_3) >> 3) << 3) +
             ((( GP4DAT & BIT_6) >> 6) << 4) +
             ((( GP4DAT & BIT_7) >> 7) << 5) +
             ((( GP0DAT & BIT_6) >> 6) << 6) +
             ((( GP0DAT & BIT_2) >> 2) << 7);

        GP2CLR = 1 << (16 + 5);  //RDLBANGLE (p2.5) = 0

        //складываем два байта
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

      //поднимаем флаг о том что текущий высокий уровень SA мы обработали
      gl_b_SA_Processed = 1;
    }
    else {
      while( (GP0DAT & 0x10)) {}
      gl_b_SA_Processed = 0;
    }
}