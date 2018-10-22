#ifndef FSK_H
#define FSK_H

#include "stm32l4xx_hal.h"
#include "arm_math.h"
#include "arm_const_structs.h"

#define BASE_31 31
#define BASE_63 63
#define BASE_127 127
#define BASE_255 255
#define MAX_BASE BASE_255
#define FFT_Length (MAX_BASE + 1) // размер окна ОБПФ
#define SizeToFind 4              // в скольки ячейках искать максимум
#define Max_Equ_Size 20
#define EQULIZER
#define GILBERT_POR 32               // порядок фильтра
#define GILBERT_POR2 GILBERT_POR / 2 // половина от порядка фильтра

//const unsigned char StartPSP_31[BASE_31] = {1,0,0,0,0,1,0,0,1,0,1,1,0,0,1,1,1,1,1,0,0,0,1,1,0,1,1,1,0,1,0};
//const unsigned char StartPSP_63[BASE_63] = {1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1};
//const unsigned char StartPSP_127[BASE_127] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1};

//const unsigned char StopPSP_31[BASE_31]  = {1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0};
//const unsigned char StopPSP_63[BASE_63] = {1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1};
//const unsigned char StopPSP_127[BASE_127] = {1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0};

#define PAM1
//#define PAM2

#ifdef PAM1
extern int8_t psp_codInput[255];
extern int8_t psp_codOutput[255];
#else
#ifdef PAM2
extern int8_t psp_codInput[255];
extern int8_t psp_codOutput[255];
#else
#error "Choose PAM"
#endif
#endif

#define M_PI 3.1415926535897932384626433832795 // пи

//#define EQULIZER

/********************** параметры сигнала и его обнаружения *******************/
/************************ коментарии в psk.cpp ********************************/

class PSK
{
  float *delayLineTMP;
  float *delayLineDMA;
  float *delayLineCORR;
  int PeriodsToSymb; // число периодов на знак псп
  int FRate;         // отношение Fs к несущей частоте
  int Rate;          // величина децимации

  float Frequence; // несущая частота
  float Fsampling; // частота дискретизации
  int TimDiv;      // делитель для таймера
  float arg;       // аргумент для вычисления опорного колебания

  unsigned int time;           // время (для умножения)
  unsigned int CountSample; // счетчик для прореживания
  float norm;               // норма комплексного числа
  short adc_sample_0;       // отсчет АЦП центрированный
  float Sample[2];          // отсчет аналитического сигнала
  float CExp[2];            // отсчет опорного колебания
  float Avr[2];             // отсчет огибающей
                            // float AvrMiddle[2];     // среднее значение огибающей на квант
  float Avr_Norm_Dec[2];    // отсчет прореженной огибающей

  float DelayLine[2][2 * MAX_BASE]; // линия задержки ФНВ (четные - реальная часть, нечетные - мнимая)
  float max_FNV;                    // максимум ФНВ
  float Corr[2];

  float Porog_Moda;                     // порог обнаружения моды сигнала
  int Equ_Size;                         // длина окна эквалайзера
  float Equlizer_Line[Max_Equ_Size]; // окно эквалайзера
  uint32_t pos_put; // позиция элемента для +
  float equ_sum;
  float multiple_equ;  // множитель для элемента 
  float Equ_max;                        // максимум после эквалайзера
  float Porog;                          // порог обнаружения

  // текущие псп (четные - реальная часть (сама последовательность), нечетные - мнимая часть (0))
  float psp[2 * (MAX_BASE + 1)];
  float elem_zero_dma; // zero fo DMA

public:
  volatile short adc_sample; // отсчет АЦП
  unsigned int Base;
  PSK()
  {
    time = 0;
    CountSample = 0;
    delayLineDMA = &DelayLine[0][0];
    delayLineCORR = &DelayLine[1][0];
  };
  void SetParamFSK(float f, int S, int R, float p_m, int equ, float Prg);
  void loadBPSP(unsigned char *cod, unsigned int _base, int channel_num);
  void loadQPSP(unsigned char *cod, unsigned int _base, int channel_num);
  int AverageAndCorrelation();
  int CorrellationV2();
  void cleanInternalBuffers();
};

extern PSK psk;
/******************************************************************************/

#endif
