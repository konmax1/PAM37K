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
#define FFT_Length (MAX_BASE + 1) // ������ ���� ����
#define SizeToFind 4              // � ������� ������� ������ ��������
#define Max_Equ_Size 20
#define EQULIZER
#define GILBERT_POR 32               // ������� �������
#define GILBERT_POR2 GILBERT_POR / 2 // �������� �� ������� �������

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

#define M_PI 3.1415926535897932384626433832795 // ��

//#define EQULIZER

/********************** ��������� ������� � ��� ����������� *******************/
/************************ ���������� � psk.cpp ********************************/

class PSK
{
  float *delayLineTMP;
  float *delayLineDMA;
  float *delayLineCORR;
  int PeriodsToSymb; // ����� �������� �� ���� ���
  int FRate;         // ��������� Fs � ������� �������
  int Rate;          // �������� ���������

  float Frequence; // ������� �������
  float Fsampling; // ������� �������������
  int TimDiv;      // �������� ��� �������
  float arg;       // �������� ��� ���������� �������� ���������

  unsigned int time;           // ����� (��� ���������)
  unsigned int CountSample; // ������� ��� ������������
  float norm;               // ����� ������������ �����
  short adc_sample_0;       // ������ ��� ��������������
  float Sample[2];          // ������ �������������� �������
  float CExp[2];            // ������ �������� ���������
  float Avr[2];             // ������ ���������
                            // float AvrMiddle[2];     // ������� �������� ��������� �� �����
  float Avr_Norm_Dec[2];    // ������ ����������� ���������

  float DelayLine[2][2 * MAX_BASE]; // ����� �������� ��� (������ - �������� �����, �������� - ������)
  float max_FNV;                    // �������� ���
  float Corr[2];

  float Porog_Moda;                     // ����� ����������� ���� �������
  int Equ_Size;                         // ����� ���� �����������
  float Equlizer_Line[Max_Equ_Size]; // ���� �����������
  uint32_t pos_put; // ������� �������� ��� +
  float equ_sum;
  float multiple_equ;  // ��������� ��� �������� 
  float Equ_max;                        // �������� ����� �����������
  float Porog;                          // ����� �����������

  // ������� ��� (������ - �������� ����� (���� ������������������), �������� - ������ ����� (0))
  float psp[2 * (MAX_BASE + 1)];
  float elem_zero_dma; // zero fo DMA

public:
  volatile short adc_sample; // ������ ���
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
