#ifndef FSK_H
#define FSK_H


#define PAM1

#ifdef PAM1
    #define PAM_MIDDLE 1226
#endif
#ifdef PAM2
#endif
#ifdef PAM3
#endif

//#include "iir_filtr.h"
#include "stm32l4xx_hal.h"
#include "gilbert.h"
#include "arm_math.h"
#include "arm_const_structs.h"

#define BASE_31         31
#define BASE_63         63
#define BASE_127        127
#define MAX_BASE        BASE_127
#define FFT_Length      (MAX_BASE+1)         // ������ ���� ����
#define SizeToFind      4                  // � ������� ������� ������ ��������
#define Max_Equ_Size    20

const unsigned char StartPSP_31[BASE_31] = {1,0,0,0,0,1,0,0,1,0,1,1,0,0,1,1,1,1,1,0,0,0,1,1,0,1,1,1,0,1,0};
const unsigned char StartPSP_63[BASE_63] = {1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1};
const unsigned char StartPSP_127[BASE_127] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1};

const unsigned char StopPSP_31[BASE_31]  = {1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0};
const unsigned char StopPSP_63[BASE_63] = {1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1};
const unsigned char StopPSP_127[BASE_127] = {1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0};

#define M_PI   3.1415926535897932384626433832795        // ��


//#define EQULIZER

/********************** ��������� ������� � ��� ����������� *******************/
/************************ ���������� � psk.cpp ********************************/

class PSK {
  int PeriodsToSymb;      // ����� �������� �� ���� ���
  int FRate;              // ��������� Fs � ������� �������
  int Rate;               // �������� ���������

  float Frequence;        // ������� �������
  float Fsampling;        // ������� �������������
  int TimDiv;             // �������� ��� �������
  float arg;              // �������� ��� ���������� �������� ���������
  
  unsigned int t;     // ����� (��� ���������)
  unsigned int CountSample;   // ������� ��� ������������
  float norm;                     // ����� ������������ �����
  short adc_sample_0;         // ������ ��� ��������������
  float Sample[2];        // ������ �������������� �������
  float CExp[2];          // ������ �������� ���������
  float Avr[2];           // ������ ���������
 // float AvrMiddle[2];     // ������� �������� ��������� �� �����
  float Avr_Norm_Dec[2];  // ������ ����������� ���������
  
  float DelayLine[2*MAX_BASE];       // ����� �������� ��� (������ - �������� �����, �������� - ������)
  float FFT_Buf[2*FFT_Length];    // ������� ������ ��� ����
  float Mag_FNV[2*SizeToFind];    // ��������� ����
  float max_FNV;                  // �������� ���
  unsigned int index_max_FNV;     // ��������� �����    
  float Corr[2];

  float Porog_Moda;               // ����� ����������� ���� �������
  int Equ_Size;                   // ����� ���� �����������
  float Equlizer_Line[2][Max_Equ_Size]; // ���� �����������
  float Equ_max;                  // �������� ����� �����������
  float Porog;                    // ����� �����������

  
  // ������� ��� (������ - �������� ����� (���� ������������������), �������� - ������ ����� (0))
  float psp_0[2*(MAX_BASE+1)];
  float psp_1[2*(MAX_BASE+1)];
  float* psp;
  int Round(double x);
  
public:
  short adc_sample;         // ������ ���
  unsigned int Base;
  float phaze0[2];
  PSK() {
    t = 0;
    CountSample = 0;
  };
  void SetParamFSK(float f, int S, int R, float p_m, int equ, float Prg);
  void loadBPSP(unsigned char* cod, unsigned int _base, int channel_num);
  void loadQPSP(unsigned char* cod, unsigned int _base, int channel_num);
  void SetCurrentPSP(int channel_num);
  int Average();
  int Correllation();
  int Woodword();
  unsigned char GetBitFromAverage();
};


extern PSK psk;
/******************************************************************************/




  

#endif