#ifndef T39_M3A_ANALOGUE_PARAMETERS_LIST
#define T39_M3A_ANALOGUE_PARAMETERS_LIST


#define UTD1                0   //термодатчик 1
#define UTD2                1   //термодатчик 2
#define UTD3                2   //термодатчик 3
#define I1                  3   //разрядный ток i1
#define I2                  4   //разрядный ток i2
#define CNTRPC              5   //напряжение на пьезокорректорах
#define AMPLANG_ALTERA      6   //амплитуда получаемая от alter'ы
#define AMPLANG_DUS         7   //амплитуда получаемая с ДУСа
#define RULA                8   //напряжение RULA

#define AMPLITUDE           9   //заданная амплитуда колебания
#define TACT_CODE           10  //код такта подставки
#define M_COEFF             11  //Коэффициент M
#define STARTMODE           12  //Начальная мода
#define DECCOEFF            13  //Коэффициент вычета

#define CONTROL_I1          14  //Контрольный ток поджига I1
#define CONTROL_I2          15  //Контрольный ток поджига I2
#define CONTROL_AA          16  //Контрольный сигнал раскачки AmplAng

#define HV_APPLY_COUNT_SET  17  //Заданное кол-во HV тычков
#define HV_APPLY_COUNT_TR   18  //кол-во HV тычков в этом запуске
#define HV_APPLY_DURAT_SET  19  //Заданная длительность HV тычков
#define HV_APPLY_PACKS      52  //Количество пачек 3kV импульсов поджига

#define SIGNCOEFF           20  //Знаковый коэффициент
#define DEVNUM              21  //Номер прибора
//#define DEVNUM              22  //Номер прибора. Старший байт

#define DATE_Y              23  //Дата.год
#define DATE_M              24  //Дата.месяц
#define DATE_D              25  //Дата.день

#define ORG                 26  //Организация целиком (для запроса колбасы)             0x14
#define ORG_B1              27  //Организация.Байт1                                     0x15
#define ORG_B2              28  //Организация.Байт2                                     0x16
#define ORG_B3              29  //Организация.Байт3                                     0x17
#define ORG_B4              30  //Организация.Байт4                                     0x18
#define ORG_B5              31  //Организация.Байт5                                     0x19
#define ORG_B6              32  //Организация.Байт6                                     0x20  " "
#define ORG_B7              33  //Организация.Байт7                                     0x21  "!"
#define ORG_B8              34  //Организация.Байт8                                     0x22  """
#define ORG_B9              35  //Организация.Байт9                                     0x23  "#"
#define ORG_B10             36  //Организация.Байт10                                    0x24  "$"
#define ORG_B11             37  //Организация.Байт11                                    0x25  "%"
#define ORG_B12             38  //Организация.Байт12                                    0x26  "&"
#define ORG_B13             39  //Организация.Байт13                                    0x27  "'"
#define ORG_B14             40  //Организация.Байт14                                    0x28  "("
#define ORG_B15             41  //Организация.Байт15                                    0x29  ")"
#define ORG_B16             42  //Организация.Байт16 БЕЗ завершающего 0 на конце!       0x2A  "*"

#define VERSION             43  //Версия                                                0x2B  "+"

#define CALIB_T1            44  //Температура нижней температурной точки калибровки     0x2C  ","
#define T1_TD1              45  //показания датчика TD1 на нижней темп. точке           0x2D  "-"
#define T1_TD2              46  //показания датчика TD2 на нижней темп. точке           0x2E  "."
#define T1_TD3              47  //показания датчика TD3 на нижней темп. точке           0x2F  "/"

#define CALIB_T2            48  //Температура верхней температурной точки калибровки    0x30  "0"
#define T2_TD1              49  //показания датчика TD1 на верхней темп. точке          0x31  "1"
#define T2_TD2              50  //показания датчика TD2 на верхней темп. точке          0x32  "2"
#define T2_TD3              51  //показания датчика TD3 на верхней темп. точке          0x33  "3"

#define AMPL_HOLD_MEAN      52  //Алгоритм стабилизации амплитуды: среднее
#define AMPL_HOLD_ROUND     53  //Алгоритм стабилизации амплитуды: круг (период среднего)
#define AMPL_HOLD_ACTIVE    54  //Алгоритм стабилизации амплитуды: флаг активной регулировки

#define SECONDS_FROM_START  55  //Секундный таймер от момента старта прибора
/*

#define SENDING_UTD1            0
#define SENDING_UTD2            1
#define SENDING_UTD3            2
#define SENDING_I1              3
#define SENDING_I2              4
#define SENDING_CNTRPC          5
#define SENDING_AMPLANG_ALTERA  6
#define SENDING_AMPLANG_DUS     7

#define SENDING_AMPLITUDE       8 //код амплитуды
#define SENDING_TACT_CODE       9 //код такта подставки
#define SENDING_M_COEFF         10 //Коэффициент M
#define SENDING_STARTMODE       11 //Начальная мода
#define SENDING_DECCOEFF        12 //Коэффициент вычета
#define SENDING_SIGNCOEFF       13 //Знаковый коэффициент

#define SENDING_VERSION         14 //Версия

#define SENDING_CONTROL_I1      15 //Контрольный ток поджига I1
#define SENDING_CONTROL_I2      16 //Контрольный ток поджига I2
#define SENDING_CONTROL_AA      17 //Контрольный сигнал раскачки AmplAng


#define SENDING_CALIB_T1        18 //Температура нижней температурной точки калибровки
#define SENDING_T1_TD1          19 //показания датчика TD1 на нижней темп. точке
#define SENDING_T1_TD2          20 //показания датчика TD2 на нижней темп. точке
#define SENDING_T1_TD3          21 //показания датчика TD3 на нижней темп. точке

#define SENDING_CALIB_T2        22 //Температура верхней температурной точки калибровки
#define SENDING_T2_TD1          23 //показания датчика TD1 на верхней темп. точке
#define SENDING_T2_TD2          24 //показания датчика TD2 на верхней темп. точке
#define SENDING_T2_TD3          25 //показания датчика TD3 на верхней темп. точке
#define SENDING_DEVNUM_BL       26 //Номер прибора. Младший байт
#define SENDING_DEVNUM_BH       27 //Номер прибора. Старший байт
#define SENDING_ORG_B1          28 //Организация.Байт1
#define SENDING_ORG_B2          29 //Организация.Байт2
#define SENDING_ORG_B3          30 //Организация.Байт3
#define SENDING_ORG_B4          31 //Организация.Байт4
#define SENDING_ORG_B5          32 //Организация.Байт5
#define SENDING_ORG_B6          33 //Организация.Байт6
#define SENDING_ORG_B7          34 //Организация.Байт7
#define SENDING_ORG_B8          35 //Организация.Байт8
#define SENDING_ORG_B9          36 //Организация.Байт9
#define SENDING_ORG_B10         37 //Организация.Байт10
#define SENDING_ORG_B11         38 //Организация.Байт11
#define SENDING_ORG_B12         39 //Организация.Байт12
#define SENDING_ORG_B13         40 //Организация.Байт13
#define SENDING_ORG_B14         41 //Организация.Байт14
#define SENDING_ORG_B15         42 //Организация.Байт15
#define SENDING_ORG_B16         43 //Организация.Байт16 БЕЗ завершающего 0 на конце!

#define SENDING_DATE_Y          44 //Дата.год
#define SENDING_DATE_M          45 //Дата.месяц
#define SENDING_DATE_D          46 //Дата.день

#define SENDING_HV_APPLY_THIS_RUN 47 //Длительность применения 3kV при включении
#define SENDING_HV_APPLY_SET    48 //Уставка длительности применения 3kV при включении
*/
#endif