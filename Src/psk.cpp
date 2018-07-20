#include "psk.h"

//#include "stm32f7xx_hal.h"
    
void PSK::SetParamFSK(float f, int S, int R, float p_m, int equ, float Prg) {
  
  PeriodsToSymb = S;   //  периодов на знак псп
  FRate = R;            // отношение частоты дискретизации к несущей R раз
  Rate = PeriodsToSymb * FRate; // прореживать в S*R раз
  
  Fsampling = f * FRate;        // Fs = f*R Гц
  float Div = HAL_RCC_GetPCLK2Freq()/Fsampling;     // делитель на таймер
  TimDiv = Round(Div);
  Fsampling = HAL_RCC_GetPCLK2Freq() /(float)TimDiv; // уточненная частота дискретизации
  Frequence = Fsampling/(float)R;       // уточненная несущая частота
  arg = 2*M_PI*Frequence/Fsampling;     // аргумент для вычисления отсчета комплексного колебания    
  Porog_Moda = p_m;                                    // порог обнаружения моды сигнала
  Equ_Size = equ;                                       // ширина эквалайзера
  Porog = Prg;                                          // порог обнаружения
}

void PSK::loadBPSP(unsigned char* cod, unsigned int _base, int channel_num) {
  Base = _base;
  float* p_psp;
  if(channel_num == 0) {
    p_psp = psp_0;
  }
  else {
    p_psp = psp_1;
  }
  volatile int i,j;
  for(i=0, j=0; i<Base; i++, j+=2) {
    if(cod[i] == 1) {
      p_psp[j] = 1;
    }
    else {
      p_psp[j] = -1;
    }
    p_psp[j+1] = 0;
  }
}

void PSK::loadQPSP(unsigned char* cod, unsigned int _base, int channel_num) {
  Base = _base;
  float* p_psp;
  if(channel_num == 0) {
    p_psp = psp_0;
  }
  else {
    p_psp = psp_1;
  }
  volatile int i,j;
  for(i=0, j=0; i<Base; i+=2, j+=2) 
  {
    if(cod[i] == 1)
    {
      p_psp[j] = 1;
    }
    else 
    {
      p_psp[j] = -1;
    }
    
    if(cod[i + 1] == 1)
    {
      p_psp[j+1] = 1;
    }
    else 
    {
      p_psp[j+1] = -1;
    }
  }
}

void PSK::SetCurrentPSP(int channel_num) {
  if(channel_num == 0) {
    psp = psp_0;
  }
  else {
    psp = psp_1;
  }
};

int PSK::Round(double x) {
  int y = (int)x;
  if((x-y) < 0.5)
    return y;
  else
    return y+1;
}

int PSK::Average() {
  float argt; 
  adc_sample_0 = adc_sample - 2048;              // вычитаем из отсчета сигнала средний уровень
  Sample[0] = (float)adc_sample_0/2048.0;        // нормируем отсчет (-1..1)
  Sample[1] = GilbertTransform(Sample[0]);     // переход к аналитическому сигналу   
  argt = arg*t;                          // аргумента у синуса и косинуса
  CExp[0] = arm_cos_f32(argt);                 // опорное колебание
  CExp[1] = -arm_sin_f32(argt);
  t++;
  if(t == Fsampling*100)
     t = 0;
  arm_cmplx_mult_cmplx_f32(Sample,CExp,Avr,1);         // умножение
  CountSample++;
  
  //DAC->DHR12R1 = 2047*Avr[0] + 2047;
  //DAC->DHR12R2 = 2047*Avr[1] + 2047;
  
  if(CountSample == Rate) {             // канал 2
    arm_sqrt_f32((Avr[0]*Avr[0] + Avr[1]*Avr[1]),&norm);
    Avr_Norm_Dec[0] = Avr[0]/norm;
    Avr_Norm_Dec[1] = Avr[1]/norm;
    CountSample = 0;
      
    return 1;
  }
  return 0;
}

int PSK::Correllation() {
    
      int retvalue = 0;
     
      /****************** перемещение линии задержки************************/
      volatile int i,j;                   
      for(i=0; i<Base-1; i++) {
        j = 2*i;
        DelayLine[j] = DelayLine[j+2];
        DelayLine[j+1] = DelayLine[j+3];
      }
      DelayLine[2*Base-2] = Avr_Norm_Dec[0];
      DelayLine[2*Base-1] = Avr_Norm_Dec[1];
     /************************************************************************/ 
      
      arm_cmplx_mult_cmplx_f32(DelayLine,psp,FFT_Buf,Base);   // свертка с кодом
      Corr[0] = 0;
      Corr[1] = 0;
      for(i=0; i<Base; i++) {
        j = 2*i;
        Corr[0] += FFT_Buf[j];
        Corr[1] += FFT_Buf[j+1];
      }
      arm_cmplx_mag_f32(Corr, &max_FNV, 1);        // Магнитуда
      max_FNV /= Base;
      
#ifdef EQULIZER
      
      for(int i=Equ_Size-1; i>0; i--) {               // сдвигаем окно эквалайзера
        Equlizer_Line[i] = Equlizer_Line[i-1]; 
      }
      if(max_FNV > Porog_Moda) {              // если обнаружена мода сигнала
         Equlizer_Line[0] = max_FNV*max_FNV;       // квадрат максимума фнв записываем в нулевой элемент окна эквалайзера
      }
      else {
         Equlizer_Line[0] = 0;     // если ничего не обнаружено, записываем 0 в в нулевой элемент окна эквалайзера
      }
      Equ_max = 0;            // складываем элементы окна эквалайзера (энергетическое сложение - квадраты энергии)
      for(int i=0; i<Equ_Size; i++) {
        Equ_max += Equlizer_Line[i];
      }
      Equ_max = sqrt(Equ_max);        // максимум "собранного" эквалайзером сигнала
        
      if(Equ_max > Porog) {              // превышение порога (если нет пропуска после обнаружения)
          retvalue = 1;
      }  
      
#else  
      if(max_FNV>1)
          max_FNV = 1;
      DAC1->DHR12R1 = max_FNV * 4095;
      if(max_FNV > Porog) { 
         phaze0[0] = (DelayLine[2] + DelayLine[4] + DelayLine[6] + DelayLine[8])/4;
         phaze0[1] = -(DelayLine[3] + DelayLine[5] + DelayLine[7] + DelayLine[9])/4;
         retvalue = 1;
      }
        
#endif
      return retvalue;
      
};


int PSK::Woodword() {
      int retvalue = 0;
     
      /****************** перемещение линии задержки************************/
      volatile int i,j;                   
      for(i=0; i<Base-1; i++) {
        j = 2*i;
        DelayLine[j] = DelayLine[j+2];
        DelayLine[j+1] = DelayLine[j+3];
      }
      DelayLine[2*Base-2] = Avr_Norm_Dec[0];
      DelayLine[2*Base-1] = Avr_Norm_Dec[1];
     /************************************************************************/ 
      
      arm_cmplx_mult_cmplx_f32(DelayLine,psp,FFT_Buf,Base);   // умножение линии задержки на код
      FFT_Buf[2*FFT_Length-2] = 0;
      FFT_Buf[2*FFT_Length-1] = 0;

      arm_cfft_f32(&arm_cfft_sR_f32_len32,FFT_Buf,1,1);      // ОБПФ
      
      
      arm_cmplx_mag_f32(FFT_Buf, Mag_FNV, SizeToFind);        // Магнитуда
      arm_cmplx_mag_f32(&FFT_Buf[2*(FFT_Length-SizeToFind)], &Mag_FNV[SizeToFind], SizeToFind);

      arm_max_f32(Mag_FNV, 2*SizeToFind, &max_FNV, &index_max_FNV);     // максимум и частотный сдвиг     
      
#ifdef EQULIZER
      
      for(int i=Equ_Size-1; i>0; i--) {               // сдвигаем окно эквалайзера
        Equlizer_Line[i] = Equlizer_Line[i-1]; 
      }
      if(max_FNV > Porog_Moda) {              // если обнаружена мода сигнала
         Equlizer_Line[0] = max_FNV*max_FNV;       // квадрат максимума фнв записываем в нулевой элемент окна эквалайзера
      }
      else {
         Equlizer_Line[0] = 0;     // если ничего не обнаружено, записываем 0 в в нулевой элемент окна эквалайзера
      }
      Equ_max = 0;            // складываем элементы окна эквалайзера (энергетическое сложение - квадраты энергии)
      for(int i=0; i<Equ_Size; i++) {
        Equ_max += Equlizer_Line[i];
      }
      Equ_max = sqrt(Equ_max);        // максимум "собранного" эквалайзером сигнала
        
      if(Equ_max > Porog) {              // превышение порога (если нет пропуска после обнаружения)
          retvalue = 1;
      }  
      
#else      
      if(max_FNV > Porog) { 
         phaze0[0] = (DelayLine[2] + DelayLine[4] + DelayLine[6] + DelayLine[8])/4;
         phaze0[1] = -(DelayLine[3] + DelayLine[5] + DelayLine[7] + DelayLine[9])/4;
         retvalue = 1;
      }
        
#endif
      return retvalue;
};

unsigned char PSK::GetBitFromAverage() {
   float correct[2];
   arm_cmplx_mult_cmplx_f32(Avr_Norm_Dec,phaze0,correct,1);
   if(correct[0] > 0)
     return 0;
   else
     return 1;
};

PSK psk;

