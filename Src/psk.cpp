#include "psk.h"
#include "stm32l4xx_hal.h"
//#include "stm32f7xx_hal.h"
#ifdef PAM1
int8_t psp_codInput[255] = {1, -1, -1, -1, -1, -1, -1, -1, 1, -1, -1, -1, 1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1,
    1, 1, -1, -1, 1, -1, -1, -1, -1, -1, 1, -1, 1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1, -1, -1, -1, 1, 1, 1, 1, 1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, 1, 1, -1, 1,
    -1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, 1, -1, -1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1, -1, -1, 1, 1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1,
    -1, -1, 1, 1, 1, -1, 1, 1, 1, -1, 1, 1, -1, -1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1, 1, -1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, 1, 1, -1, -1, -1, -1, -1, 1, 1, 1, -1, 1, -1, 1,
    -1, 1, -1, 1, 1, 1, 1, 1, -1, -1, 1, -1, 1, -1, -1, -1, -1, 1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, 1, -1, 1, 1, 1, 1, -1, -1, -1, 1, 1, -1};
int8_t psp_codOutput[255] = {1, -1, -1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, 1, 1, 1, 1, -1, 1, -1, -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, -1, -1, 1, -1, 
    1, -1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, -1, -1, 1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, -1, -1, 1, -1, -1, -1, -1, 1, -1, -1, -1, -1, -1,
    1, -1, -1, 1, 1, -1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, -1, -1, 1, -1, 1, -1, 1, -1, -1, -1, -1, 1, 1, 1, -1, -1, 1, -1, 1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1, -1, 1, 1, 1, -1, 1, -1, 1,
    -1, -1, 1, 1, 1, 1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, -1, -1, 1, 1, 1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1, -1, -1, -1, 1, 1, 1, 1, -1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1,
    -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, 1, 1, -1, 1};
#endif
#ifdef PAM2
int8_t psp_codInt[255] = {-1, -1, -1, -1, -1, -1, -1, 1, -1, 1, -1, 1, -1, -1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1, -1, -1,
    1, 1, 1, 1, -1, -1, -1, 1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, 1, 1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, 1, 1, 1, 1,
    1, -1, -1, -1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, -1, 1, 1, -1, 1, -1,
    1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, 1, -1, 1, 1, 1, -1, -1, -1, 1, -1, -1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, -1, -1, -1,
    -1, 1, -1, -1, -1, 1, -1, -1, -1, -1, 1, 1, 1, 1, -1, 1, 1, -1, -1, -1, 1, -1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1};
int8_t psp_codOutput[255] = {-1, -1, -1, -1, -1, -1, -1, 1, 1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1, -1, -1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, -1,
    1, 1, 1, -1, -1, -1, -1, 1, 1, 1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1, -1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, 1,
    1, -1, -1, 1, 1, -1, -1, -1, 1, -1, -1, -1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, -1, 1, -1, -1, 1, 1, 1, -1, 1, -1, -1, -1, 1, -1, -1,
    1, -1, -1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, -1, 1, 1, -1, 1, 1, -1, 1, 1, -1, -1, 1, -1, 1, 1, 1, -1, -1, -1, 1, 1, 1, -1, 1, 1, -1, -1, -1, -1, 1, -1, 1, 1, 1, 1, -1,
    -1, -1, 1, -1, 1, -1, -1, 1, 1, 1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, -1, 1, 1, -1, 1, -1};
#endif

float GilbertBufer[GILBERT_POR + 1]; // линия задержки для преобразования (мнимая часть)
float GILBERT_COEFF[GILBERT_POR + 1] = {4.54486433486115e-05, -0.0319136045611685, -3.31277815639291e-05,
                                        -0.0260478358874455, -1.59427170179442e-06, -0.0373284021357407, 3.46942587567515e-06, -0.0531107459937533,
                                        -1.03378562696214e-06, -0.0766995375013512, -5.96683448033596e-06, -0.116853826766850, 1.66624629076986e-05,
                                        -0.205805810057242, -1.89302802971010e-05, -0.634472629392063, 0, 0.634472629392063, 1.89302802971010e-05,
                                        0.205805810057242, -1.66624629076986e-05, 0.116853826766850, 5.96683448033596e-06, 0.0766995375013512,
                                        1.03378562696214e-06, 0.0531107459937533, -3.46942587567515e-06, 0.0373284021357407, 1.59427170179442e-06,
                                        0.0260478358874455, 3.31277815639291e-05, 0.0319136045611685, -4.54486433486115e-05};

                                        
extern volatile unsigned int t1, t2, t3, t4, t5;
                                        
void PSK::SetParamFSK(float f, int S, int R, float p_m, int equ, float Prg)
{

  PeriodsToSymb = S;            //  периодов на знак псп
  FRate = R;                    // отношение частоты дискретизации к несущей R раз
  Rate = PeriodsToSymb * FRate; // прореживать в S*R раз

  Fsampling = f * FRate;                          // Fs = f*R Гц
  float Div = HAL_RCC_GetPCLK2Freq() / Fsampling; // делитель на таймер
  TimDiv = Round(Div);
  Fsampling = HAL_RCC_GetPCLK2Freq() / (float)TimDiv; // уточненная частота дискретизации
  Frequence = Fsampling / (float)R;                   // уточненная несущая частота
  arg = 2 * M_PI * Frequence / Fsampling;             // аргумент для вычисления отсчета комплексного колебания
  Porog_Moda = p_m;                                   // порог обнаружения моды сигнала
  Equ_Size = equ;                                     // ширина эквалайзера
  Porog = Prg;                                        // порог обнаружения
}

void PSK::loadBPSP(unsigned char *cod, unsigned int _base, int channel_num)
{
  Base = _base;
  float *p_psp;
  if (channel_num == 0)
  {
    p_psp = psp_0;
  }
  else
  {
    p_psp = psp_1;
  }
  volatile int i, j;
  for (i = 0, j = 0; i < Base; i++, j += 2)
  {
    if (cod[i] == 1)
    {
      p_psp[j] = 1;
    }
    else
    {
      p_psp[j] = -1;
    }
    p_psp[j + 1] = 0;
  }
}

void PSK::loadQPSP(unsigned char *cod, unsigned int _base, int channel_num)
{
  Base = _base;
  float *p_psp;
  if (channel_num == 0)
  {
    p_psp = psp_0;
  }
  else
  {
    p_psp = psp_1;
  }
  volatile int i, j;
  for (i = 0, j = 0; i < Base; i += 2, j += 2)
  {
    if (cod[i] == 1)
    {
      p_psp[j] = 1;
    }
    else
    {
      p_psp[j] = -1;
    }

    if (cod[i + 1] == 1)
    {
      p_psp[j + 1] = 1;
    }
    else
    {
      p_psp[j + 1] = -1;
    }
  }
}

void PSK::SetCurrentPSP(int channel_num)
{
  if (channel_num == 0)
  {
    psp = psp_0;
  }
  else
  {
    psp = psp_1;
  }
};

int PSK::Round(double x)
{
  int y = (int)x;
  if ((x - y) < 0.5)
    return y;
  else
    return y + 1;
}

int16_t adc_max = 0;
int PSK::AverageAndCorrelation()
{    
  t1= DWT->CYCCNT; 
  DMA1->IFCR |= DMA_IFCR_CTCIF1 | DMA_IFCR_CGIF1 | DMA_IFCR_CHTIF1;
  DMA1_Channel1->CCR &= ~DMA_CCR_EN;
  DMA1_Channel1->CPAR = (uint32_t)&GilbertBufer[1];
  DMA1_Channel1->CMAR = (uint32_t)&GilbertBufer[0];
  DMA1_Channel1->CNDTR = GILBERT_POR;
  DMA1_Channel1->CCR |= DMA_CCR_EN;

  adc_sample_0 = adc_sample - 2048;                // вычитаем из отсчета сигнала средний уровень
  if( adc_sample_0 > adc_max){
      adc_max = adc_sample_0;
  }
  Sample[0] = (float)adc_sample_0 * (1. / 2048.0); // нормируем отсчет (-1..1)    
  float argt = arg * t;                            // аргумента у синуса и косинуса
  CExp[0] =  arm_cos_f32(argt);                     // опорное колебание
  CExp[1] =  arm_sin_f32(argt);
  t++;
  //arm_sin_cos_f32(argt,&CExp[1],&CExp[0]);
  /*if (t == Fsampling * 100)
    t = 0;*/
  t = ( t + 1) % 3000000;
  CountSample++;  
  while ((DMA1->ISR & DMA_ISR_TCIF1) == 0)
  {
  }  
  GilbertBufer[GILBERT_POR] = Sample[0];
  Sample[0] = GilbertBufer[GILBERT_POR2];
  arm_dot_prod_f32(&GilbertBufer[0], &GILBERT_COEFF[0], GILBERT_POR+1, &Sample[1]);
  //end of gilbert
  arm_cmplx_mult_cmplx_f32(Sample, CExp, Avr, 1); // умножение
 
  if (CountSample == Rate)
  { // канал 2
    arm_cmplx_mag_f32(&Avr[0], &norm, 1);
    norm = 1. / norm;
    Avr_Norm_Dec[0] = Avr[0] * norm;
    Avr_Norm_Dec[1] = Avr[1] * norm;
    CountSample = 0;  
    //    CorrellationV2();
    return 1;
  }
  return 0;
}

volatile int32_t maxFNVstat = 0;
int PSK::CorrellationV2()
{
    
  t2= DWT->CYCCNT; 
  DMA1_Channel2->CCR &= ~DMA_CCR_EN;
  DMA1_Channel2->CPAR = (uint32_t)&delayLineDMA[2];
  DMA1_Channel2->CMAR = (uint32_t)&delayLineCORR[0];
  DMA1_Channel2->CNDTR = 2 * Base;
  DMA1->IFCR |= DMA_IFCR_CTCIF2 | DMA_IFCR_CGIF2 | DMA_IFCR_CHTIF2;
  DMA1_Channel2->CCR |= DMA_CCR_EN;

  int32_t retvalue = 0;
  delayLineTMP = delayLineCORR;
  delayLineCORR = delayLineDMA;
  delayLineDMA = delayLineTMP;    
    
  arm_cmplx_dot_prod_f32(delayLineCORR,psp,Base,&Corr[0],&Corr[1]); 
  delayLineDMA[2*Base-2] = Avr_Norm_Dec[0];
  delayLineDMA[2*Base-1] = Avr_Norm_Dec[1];
//  arm_cmplx_mult_cmplx_f32(delayLineCORR, psp, FFT_Buf, Base); // свертка с кодом

//  delayLineDMA[2*Base-2] = Avr_Norm_Dec[0];
//  delayLineDMA[2*Base-1] = Avr_Norm_Dec[1];
//  Corr[0] = 0;
//  Corr[1] = 0;  
//  float* p_cor1 = &Corr[0];
//  float* p_cor2 = &Corr[1];
//  float* p_fft1 = &FFT_Buf[0];
//  for (volatile int32_t i = 0; i < Base; i++)
//  {
//      *p_cor1 += *p_fft1;
//      p_fft1++;
//      *p_cor2 += *p_fft1;
//      p_fft1++;
//  }
  
//    volatile int32_t i, j;
//  for (i = 0; i < Base; i++)
//  {
//    j = 2 * i;
//    Corr[0] += FFT_Buf[j];
//    Corr[1] += FFT_Buf[j + 1];
//  }
  arm_cmplx_mag_f32(Corr, &max_FNV, 1); // Магнитуда
  //max_FNV /= Base;
  //DAC1->DHR12R1 = max_FNV * 4095;
  if( max_FNV > maxFNVstat){
      maxFNVstat = max_FNV;
  }
  if (max_FNV > (Porog) )
  {
    retvalue = max_FNV;
  }
  return retvalue;
}

PSK psk;
