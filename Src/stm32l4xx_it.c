/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"

/* USER CODE BEGIN 0 */
#include "global.h"




volatile int32_t d_val = 0;
volatile osStatus_t stat;
volatile uint32_t ISR;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;

extern TIM_HandleTypeDef htim7;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles ADC1 global interrupt.
*/
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */
    ISR = ADC1->ISR;
    if( ISR & ADC_ISR_JEOS)
    {
        ADC1->ISR |= ADC_ISR_JEOS;    
        //adc = ADC1->JDR2; // Instrumental amplifier
        //adc = ADC1->JDR1;      // logariphmic amplifier   
        psk.adc_sample = ADC1->JDR1;      // logariphmic amplifier   
        if(psk.AverageAndCorrelation()) 
        {
            stat = osSemaphoreRelease(adcHandle);
            if( stat != osOK)
            { 
                d_val++;
            }
        }
    }
    else
    { 
        psk.adc_sample = ADC1->JDR2 + 1; 
    }
  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_IRQn 1 */
  /* USER CODE END ADC1_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles TIM1 break interrupt and TIM15 global interrupt.
*/
void TIM1_BRK_TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 1 */
}

/**
* @brief This function handles RTC alarm interrupt through EXTI line 18.
*/
void RTC_Alarm_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_Alarm_IRQn 0 */
  //xTaskResumeFromISR(Mode2Handle);
  osSemaphoreRelease(rtcIRQHandle);
  //xSemaphoreGiveFromISR(rtcIRQHandle,&xYeild);
  //taskYIELD();
  /* USER CODE END RTC_Alarm_IRQn 0 */
  HAL_RTC_AlarmIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_Alarm_IRQn 1 */

  /* USER CODE END RTC_Alarm_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt, DAC channel1 and channel2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  HAL_DAC_IRQHandler(&hdac1);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
* @brief This function handles USB event interrupt through EXTI line 17.
*/
void USB_IRQHandler(void)
{
  /* USER CODE BEGIN USB_IRQn 0 */

  /* USER CODE END USB_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_IRQn 1 */

  /* USER CODE END USB_IRQn 1 */
}

/* USER CODE BEGIN 1 */
extern "C" void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
    //t5 = TIM1->CNT;
    //t1=DWT->CYCCNT;
    //static long xHigherPriorityTaskWoken;
    if(TIM1->SR & TIM_SR_UIF)
    {
        TIM1->SR &= ~TIM_SR_UIF;
        /*if(xTaskResumeFromISR(calcTaskHandle))
        {
        taskYIELD();
    }*/        
        //t2=DWT->CYCCNT;  
        currSignal.emitingAudio();    
        //t3=DWT->CYCCNT;
        currSignal.calcNext();          
        //t4=DWT->CYCCNT;
        if(TIM1->SR & TIM_SR_UIF)
        {
            TIM1->SR &= ~TIM_SR_UIF;
        }
        /*xSemaphoreGiveFromISR(reCalcPWMHandle,&xHigherPriorityTaskWoken); 
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);*/
        
        /*if(xHigherPriorityTaskWoken == pdTRUE)
        taskYIELD();*/
        //taskYIELD();
        //TimARR();
        
    }else
    {
        if(TIM1->SR & TIM_SR_UIF)
        {
            TIM1->SR &= ~TIM_SR_UIF;
        }
    }
    //t6=DWT->CYCCNT;
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  //HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */
    //t7=DWT->CYCCNT;
  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}
volatile int hjf = 0;
extern "C" void TIM2_IRQHandler()
{ 
    if(TIM2->SR & TIM_SR_UIF)
    {
        CLEAR_BIT(TIM2->SR,TIM_SR_UIF);
        currSignal.changePSKbit();
        
    }else
    {    
        hjf = 1;
        CLEAR_BIT(TIM2->SR,TIM_SR_UIF);
    }
}
    
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
