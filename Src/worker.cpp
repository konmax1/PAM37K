#include "global.h"

uint32_t statFlag = 0;
void fullPower();
void lowPower();

void setLedPort()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    GPIO_InitStruct.Pin = LED_But_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    EXTI->IMR1 &= ~EXTI_IMR1_IM7;
    HAL_GPIO_Init(LED_But_GPIO_Port, &GPIO_InitStruct);
}

void setButtonPort()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LED_But_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(LED_But_GPIO_Port, &GPIO_InitStruct);
    EXTI->PR1 |= EXTI_PR1_PIF7;
    NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    SET_BIT(GPIOA->ODR, LED_But_Pin);
}

extern "C" void setup()
{
    currSignal.StopAudio();
    HAL_DisableDBGSleepMode();
    HAL_EnableDBGSleepMode();
    fullPower();
    data_signal input, output;
    input.typeSignal = eBPSK;
    input.num_kvant_psp = 5;
    input.freq1 = 37500;
    input.freq2 = 37500;
    input.amplitude = 1;
    input.signal_pause = 0;
    input.duration = 10;
    input.lenPSP = 255;
    input.repeat = 1;
    input.DD = 0;
    input.HH = 0;
    input.MM = 0;

    output.typeSignal = eBPSK;
    output.num_kvant_psp = 5;
    output.freq1 = 37500;
    output.freq2 = 37500;
    output.amplitude = 1;
    output.signal_pause = 0;
    output.duration = 10;
    output.lenPSP = 255;
    output.repeat = 1;
    output.DD = 0;
    output.HH = 0;
    output.MM = 0;
    currSignal.initMode(input, output, eRxTx);
    psk.loadBPSP((uint8_t *)&psp_codInput[0], currSignal.getSignalData().lenPSP, 0);
    memcpy(&currSignal.getdPSPOut(), &psp_codOutput[0], currSignal.getSignalData().lenPSP);
    int8_t *p_t = (int8_t *)&currSignal.getdPSPOut();
    for (volatile int32_t i = 0; i < currSignal.getSignalData().lenPSP; i++)
    {
        p_t[i] = (p_t[i] + 1) / 2;
    }
    psk.SetParamFSK(currSignal.getFindData().freq1 - 30000,
                    currSignal.getFindData().num_kvant_psp / 5,
                    (currSignal.getFindData().freq1 / (currSignal.getFindData().freq1 - 30000)) - 1,
                    0.,
                    0.,
                    166.);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_TIM_Base_Start(&htim6);
}

extern "C" void loop()
{
    volatile int32_t par = 0;
    setLedPort();
    while (1)
    { 
        if (statFlag & AVR_SAMPLE_RDY)
        {
            CLEAR_BIT(statFlag,AVR_SAMPLE_RDY);
            
//            CLEAR_BIT(GPIOA->ODR, LED_But_Pin);
            if(psk.CorrellationV2()){
                CLEAR_BIT(GPIOA->ODR, LED_But_Pin);
                HAL_Delay(100);
                SET_BIT(GPIOA->ODR, LED_But_Pin);
                par++;
            }
//            SET_BIT(GPIOA->ODR, LED_But_Pin);
        }
    }
}

void fullPower()
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
    memset(&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));
    memset(&RCC_ClkInitStruct, 0, sizeof(RCC_ClkInitStruct));
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
}
void lowPower()
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
    memset(&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));
    memset(&RCC_ClkInitStruct, 0, sizeof(RCC_ClkInitStruct));
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
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
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
}