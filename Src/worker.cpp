#include "global.h"

volatile uint32_t statFlag = 0;
volatile uint32_t needBlink = 0;
void fullPower();
void lowPower();
void buttonClick();
void MX_RTC_InitMy();

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

extern "C" void HAL_SYSTICK_Callback(void)
{ 
	static volatile uint32_t mills = 0;
	static volatile uint32_t counter = 10;
	if(needBlink){
		if(mills == 0){
			mills = HAL_GetTick();  
		}
		if ((HAL_GetTick() - mills) > 500){
			mills = HAL_GetTick();
			counter--;
			if(counter > 0){
				if( (counter % 2 ) == 0){
					setLedPort();
					CLEAR_BIT(GPIOA->ODR, LED_But_Pin);
				}else{
					setButtonPort();
				}
			}else{
				needBlink = 0;
			}
		}
			
	}
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static volatile uint32_t mills = 0;
    if (GPIO_Pin == LED_But_Pin)
    {
        if ((HAL_GetTick() - mills) > 50)
        {
            mills = HAL_GetTick();
            buttonClick();
        }
        else
        {
        }
    }
}

extern "C" void setup()
{
    HAL_IWDG_Refresh(&hiwdg);
    currSignal.StopAudio();
    HAL_DisableDBGSleepMode();
    HAL_EnableDBGSleepMode();
    fullPower();
    setButtonPort();
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
		
    HAL_IWDG_Refresh(&hiwdg);
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
                    0.25,
                    20,
                    0.45);
		
    HAL_IWDG_Refresh(&hiwdg);
    lowPower();		
    HAL_IWDG_Refresh(&hiwdg);
    //MX_RTC_InitMy();
}

void ToogleLed(uint32_t _mills)
{
    static uint32_t mills;
    static uint32_t ledIsOn = 0;
    if (ledIsOn == 0)
    {
        if ((_mills - mills) > 29000)
        {
            mills = _mills;
            setLedPort();
            CLEAR_BIT(GPIOA->ODR, LED_But_Pin);
            ledIsOn = 1;
        }
    }
    else
    {
        if ((_mills - mills) > 1000)
        {
            mills = _mills;
            SET_BIT(GPIOA->ODR, LED_But_Pin);
            setButtonPort();
            ledIsOn = 0;
        }
    }
}

void findSignal()
{
    if (statFlag & AVR_SAMPLE_RDY)
    {
        CLEAR_BIT(statFlag, AVR_SAMPLE_RDY);

        //CLEAR_BIT(GPIOA->ODR, LED_But_Pin);
        if (psk.CorrellationV2())
        {
            setLedPort();
            CLEAR_BIT(GPIOA->ODR, LED_But_Pin);
            psk.cleanInternalBuffers();
            HAL_TIM_Base_Stop(&htim6);
						//HAL_Delay(100);
            currSignal.emitSignal();
            HAL_TIM_Base_Start(&htim6);
            SET_BIT(GPIOA->ODR, LED_But_Pin);
            setButtonPort();
        }
        //SET_BIT(GPIOA->ODR, LED_But_Pin);
    }
}

extern "C" void loop()
{
    uint32_t mills;
    //setLedPort();
    //SET_BIT(GPIOA->ODR, LED_But_Pin);
    while (1)
    {
        if (RTC->BKP10R == 1)
        {
            findSignal();
            ToogleLed(HAL_GetTick());
        }

        HAL_IWDG_Refresh(&hiwdg);
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    }
}

void buttonClick()
{
    if (RTC->BKP10R == 1)
    {
        // уйти в сон
        RTC->BKP10R = 0;
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0);
        HAL_IWDG_Refresh(&hiwdg);
        HAL_NVIC_SystemReset();
    }
    else
    {
        // начать работу
        HAL_IWDG_Refresh(&hiwdg);
        fullPower();
        HAL_IWDG_Refresh(&hiwdg);
        HAL_ADCEx_InjectedStart_IT(&hadc1);
        HAL_TIM_Base_Start(&htim6);
        MX_RTC_InitMy();
        RTC->BKP10R = 1;
				needBlink = 1;
    }
}

void fullPower()
{
    HAL_PWR_EnableBkUpAccess();
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
    HAL_PWR_EnableBkUpAccess();
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

void MX_RTC_InitMy(void)
{

    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC Only 
    */
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 124;
    hrtc.Init.SynchPrediv = 6249;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initialize RTC and set the Time and Date 
    */
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2)
    {
        sTime.Hours = 0x0;
        sTime.Minutes = 0x0;
        sTime.Seconds = 0x0;
        sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        sTime.StoreOperation = RTC_STOREOPERATION_SET;
        if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
        {
            _Error_Handler(__FILE__, __LINE__);
        }

        sDate.WeekDay = RTC_WEEKDAY_MONDAY;
        sDate.Month = RTC_MONTH_JANUARY;
        sDate.Date = 0x1;
        sDate.Year = 0x0;

        if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
        {
            _Error_Handler(__FILE__, __LINE__);
        }

        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2);
    }
    /**Enable the Alarm A 
    */
    sAlarm.AlarmTime.Hours = 0x0;
    sAlarm.AlarmTime.Minutes = 0x0;
    sAlarm.AlarmTime.Seconds = 0x0;
    sAlarm.AlarmTime.SubSeconds = 0x0;
    sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_SET;
    sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
    sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    sAlarm.AlarmDateWeekDay = 0x1;
    sAlarm.Alarm = RTC_ALARM_A;
    if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}
