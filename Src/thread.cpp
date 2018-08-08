#include "thread.h"
#include "communication.h"
#include "main.h"
/*struct data_signal
{    
    typeSignals typeSignal;
    uint16_t num_kvant_psp;
    float freq1;
    float freq2;
    float amplitude;
    uint16_t signal_pause;
    uint8_t  duration;
    uint8_t lenPSP;
    uint32_t psp[4];
    uint8_t repeat;
    uint8_t DD;
    uint8_t HH;
    uint8_t MM;
};*/
extern "C" void MX_RTC_Init(void);

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LED_But_Pin)
    {
        osSemaphoreRelease(ButtonHandle);
    }
}

extern "C" void iwdgThread(void *argument)
{
    for (;;)
    {
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(500);
    }
}

extern "C" void eButtonThread(void *argument)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
    memset(&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));
    memset(&RCC_ClkInitStruct, 0, sizeof(RCC_ClkInitStruct));
    osDelay(1000);
    //ждем нажатие на кнопку
    volatile uint32_t *curMode = &RTC->BKP10R;
    if (*curMode == 1)
    {
        //Первый запуск
        MX_RTC_Init();
        osThreadResume(defaultTaskHandle);
    }
    else if (*curMode == 0)
    {
        HAL_RCC_DeInit();
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_MSI | RCC_OSCILLATORTYPE_LSI;
        RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
        RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        RCC_OscInitStruct.PLL.PLLM = 2;
        RCC_OscInitStruct.PLL.PLLN = 12;
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
        RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
        RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

        RCC_OscInitStruct.LSIState = RCC_LSI_ON;

        RCC_OscInitStruct.MSICalibrationValue = 0;
        RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
        RCC_OscInitStruct.MSIState = RCC_MSI_ON;
        HAL_RCC_OscConfig(&RCC_OscInitStruct);

        /**Initializes the CPU, AHB and APB busses clocks
                */
        RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
        RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
        RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

        HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
        HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
        CLEAR_BIT(U_gen_GPIO_Port->ODR, U_gen_Pin);

        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USB | RCC_PERIPHCLK_ADC;
        PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
        PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
        PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
        HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
        MX_RTC_Init();
    }
    for (;;)
    {
        if (osSemaphoreAcquire(ButtonHandle, osWaitForever) == osOK)
        {
            osDelay(200);
            if (READ_BIT(GPIOA->IDR, GPIO_PIN_7) == 0)
            {
                switch (*curMode)
                {
                case 0:
                    HAL_RCC_DeInit();
                    SET_BIT(U_gen_GPIO_Port->ODR, U_gen_Pin);
                    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI;
                    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
                    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
                    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
                    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
                    RCC_OscInitStruct.PLL.PLLM = 2;
                    RCC_OscInitStruct.PLL.PLLN = 12;
                    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
                    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
                    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

                    RCC_OscInitStruct.LSIState = RCC_LSI_ON;

                    RCC_OscInitStruct.MSICalibrationValue = 0;
                    RCC_OscInitStruct.MSIClockRange = 6;
                    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
                    HAL_RCC_OscConfig(&RCC_OscInitStruct);

                    /**Initializes the CPU, AHB and APB busses clocks
                    */
                    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
                    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
                    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
                    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
                    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

                    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
                    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

                    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USB | RCC_PERIPHCLK_ADC;
                    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
                    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV32;
                    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
                    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
                    //Первый запуск

                    *curMode = 1;
                    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0);
                    MX_RTC_Init();
                    osSemaphoreAcquire(ButtonHandle, 0);
                    osThreadResume(defaultTaskHandle);
                    break;
                case 1:
                    //Нажатие кнопки во время работы
                    //выключить изделие
                    *curMode = 0;
                    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0);
                    osSemaphoreAcquire(ButtonHandle, 0);
                    HAL_NVIC_SystemReset();
                    break;
                }
            }
        }
    }
}

extern "C" void StartDefaultTask(void *argument)
{
    HAL_DisableDBGSleepMode();
    HAL_EnableDBGSleepMode();
    HAL_IWDG_Refresh(&hiwdg);
    //Switch to MSI

    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_TIM2_CLK_ENABLE();
    //TickType_t xLastWakeTime = xTaskGetTickCount();

    //      /*Configure GPIO pin : LED_But_Pin */
    //  GPIO_InitStruct.Pin = LED_But_Pin;
    //  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    //  GPIO_InitStruct.Pull = GPIO_NOPULL;
    //  HAL_GPIO_Init(LED_But_GPIO_Port, &GPIO_InitStruct);

    //HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
    if ((GPIOA->IDR & PWR_GPIO_BIT_10) > 0)
    {
        MX_USB_DEVICE_Init();
    }
    currSignal.StopAudio();
    HAL_IWDG_Refresh(&hiwdg);
    //Ожидание нажатия на кнопку включения
    osThreadSuspend(defaultTaskHandle);
    //currSignal.setSignalData(flashdata.cnfg.sig[0]);
    /*data_signal ds;
    ds.typeSignal = eQPSK;
    ds.num_kvant_psp = 10;
    ds.freq1 = 37500;
    ds.freq2 = 38000;
    ds.amplitude = 1;
    ds.signal_pause = 0;
    ds.duration = 200;
    ds.lenPSP = 31;
    ds.psp[0] = 0x7F72317E;
    ds.psp[1] = 0x55555555;
    ds.psp[2] = 0x55555555;
    ds.psp[3] = 0x55555555;
    ds.repeat = 1;
    ds.DD = 0;
    ds.HH = 0;
    ds.MM = 5;*/

    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    GPIO_InitStruct.Pin = LED_But_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    EXTI->IMR1 &= ~EXTI_IMR1_IM7;
    HAL_GPIO_Init(LED_But_GPIO_Port, &GPIO_InitStruct);

    for (volatile int i = 0; i < 5; i++)
    {
        SET_BIT(GPIOA->ODR, LED_But_Pin);
        osDelay(500);
        CLEAR_BIT(GPIOA->ODR, LED_But_Pin);
        osDelay(500);
    }

    GPIO_InitStruct.Pin = LED_But_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(LED_But_GPIO_Port, &GPIO_InitStruct);
    EXTI->PR1 |= EXTI_PR1_PIF7;
    NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_IWDG_Refresh(&hiwdg);
    currSignal.loadFromFlash();
    //currSignal.StartAudio();
    //osThreadSuspend(defaultTaskHandle);
    //currSignal.StartAudio();
    for (;;)
    {
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
        GPIO_InitStruct.Pin = LED_But_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        EXTI->IMR1 &= ~EXTI_IMR1_IM7;
        HAL_GPIO_Init(LED_But_GPIO_Port, &GPIO_InitStruct);

        CLEAR_BIT(GPIOA->ODR, LED_But_Pin);
        osDelay(500);
        GPIO_InitStruct.Pin = LED_But_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(LED_But_GPIO_Port, &GPIO_InitStruct);
        EXTI->PR1 |= EXTI_PR1_PIF7;
        NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        SET_BIT(GPIOA->ODR, LED_But_Pin);
        osDelay(30000);
    }
}

extern "C" void eLedToggle(void *argument)
{
    osThreadSuspend(LedToggleHandle);
    for (;;)
    {
        CLEAR_BIT(GPIOA->ODR, LED_But_Pin);
        osDelay(5);
        SET_BIT(GPIOA->ODR, LED_But_Pin);
        osDelay(5);
    }
}

extern "C" void eMode1(void *argument)
{
    osThreadSuspend(Mode1Handle);
    osThreadTerminate(ModeRxTxHandle);
    osThreadTerminate(Mode2Handle);
    //TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        //osDelayUntil(&xLastWakeTime,100);
        currSignal.emitSignal();
    }
}

extern "C" void eMode2(void *argument)
{
    /**/
    //TickType_t xLastWakeTime = xTaskGetTickCount();
    osThreadSuspend(Mode2Handle);
    osThreadTerminate(ModeRxTxHandle);
    osThreadTerminate(Mode1Handle);
    for (;;)
    {
        if (osSemaphoreAcquire(rtcIRQHandle, 1000) == osOK)
        {
            for (volatile int32_t i = 0; i < currSignal.getSignalData().repeat; i++)
            {
                currSignal.emitSignal();
            }
            currSignal.loadFromFlashNextSignal();
            //vTaskSuspend(NULL);
        }
        else
        {
            //vTaskSuspend(NULL);
        }
        //osDelayUntil(&xLastWakeTime,100);
    }
}

extern "C" void eModeRxTx(void *argument)
{
    osThreadSuspend(ModeRxTxHandle);
    //osThreadSuspend(iwdgThreadHandle);
    //osThreadTerminate(iwdgThreadHandle);
    osThreadTerminate(Mode1Handle);
    osThreadTerminate(Mode2Handle);
    HAL_IWDG_Refresh(&hiwdg);
    //TickType_t xLastWakeTime = xTaskGetTickCount();

    //temporally init w/o external program
    currSignal.getSignalData().lenPSP = 255;
    psk.loadBPSP((uint8_t *)&psp_codInput[0], currSignal.getSignalData().lenPSP , 0);
    memcpy(&currSignal.getdPSPOut(), &psp_codOutput[0], currSignal.getSignalData().lenPSP);
    int8_t *p_t = (int8_t *)&currSignal.getdPSPOut();
    for(volatile int32_t i = 0; i < currSignal.getSignalData().lenPSP; i++){
        p_t[i] = ( p_t[i] + 1) / 2;
    }
    currSignal.getSignalData().duration = 40;
    currSignal.getFindData().num_kvant_psp = 10;
    currSignal.getSignalData().num_kvant_psp = 5;
    //end of temp

    psk.SetCurrentPSP(0);
    //DAC1->CR |= DAC_CR_EN1;
    psk.SetParamFSK(currSignal.getFindData().freq1 - 30000,
                    currSignal.getFindData().num_kvant_psp / 5,
                    (currSignal.getFindData().freq1 / (currSignal.getFindData().freq1 - 30000)) - 1,
                    0.,
                    0.,
                    166.);
    //HAL_ADC_Start_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_TIM_Base_Start(&htim6);
    osThreadResume(defaultTaskHandle);
    uint16_t smpl;
    volatile uint16_t stateee;
    osStatus_t state;
    HAL_IWDG_Refresh(&hiwdg);
    for (;;)
    {
        //найти сигнал и излучить запомненный
        if (osSemaphoreAcquire(adcHandle, osWaitForever) == osOK)
        {
            stateee = psk.CorrellationV2();
            if(stateee)
            {
                //сигнал найден, излучить
                HAL_TIM_Base_Stop(&htim6);
                currSignal.emitSignal();
                HAL_TIM_Base_Start(&htim6);
            }
        }
    }
}

extern "C" __NO_RETURN void osRtxIdleThread(void *argument)
{
    volatile uint32_t *curMode = &RTC->BKP10R;
    for (;;)
    {
        HAL_IWDG_Refresh(&hiwdg);
        if (curMode == 0)
        {
            //wait button
            __WFI();
        }
        else
        {
            //runmode
            __WFI();
        }
    }
}

//OS Error Callback function
extern "C" uint32_t osRtxErrorNotify(uint32_t code, void *object_id)
{
    (void)object_id;

    switch (code)
    {
    case osRtxErrorStackUnderflow:
        // Stack underflow detected for thread (thread_id=object_id)
        break;
    case osRtxErrorISRQueueOverflow:
        // ISR Queue overflow detected when inserting object (object_id)
        break;
    case osRtxErrorTimerQueueOverflow:
        // User Timer Callback Queue overflow detected for timer (timer_id=object_id)
        break;
    case osRtxErrorClibSpace:
        // Standard C/C++ library libspace not available: increase OS_THREAD_LIBSPACE_NUM
        break;
    case osRtxErrorClibMutex:
        // Standard C/C++ library mutex initialization failed
        break;
    default:
        break;
    }
    for (;;)
    {
    }
    //return 0U;
}
