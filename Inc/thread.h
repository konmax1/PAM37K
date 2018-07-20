#ifndef THREADH
#define THREADH

#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "usb_device.h"
#include "math.h"
#include "arm_math.h"

#include "global.h"
#include "cmsis_compiler.h"
#include "rtx_os.h"

extern int Np;
extern float Fs;
extern int K;
extern float A;

void TimARR();
void ccr_set (uint32_t num);

void stopPWM ();
void startPWM ();
void changePWM (float Fnew, int Nnew);


#endif
