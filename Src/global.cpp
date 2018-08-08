#include "global.h"
#include "usb_device.h"

SIGNAL currSignal;
flashDATA flashdata __attribute__((at(ADDRESS_FLASH_DATA))); //@ ADDRESS_FLASH_DATA;

volatile unsigned int t1, t2, t3, t4, t5;

extern "C" void iwdgThread(void *argument);

void SIGNAL::setSignalData(data_signal &_ds)
{
    memcpy(&dataOut, &_ds, sizeof(dataOut));
    prepareData();
}

data_signal &SIGNAL::getSignalData()
{
    return dataOut;
}

void SIGNAL::setFindData(data_signal &_ds)
{
    memcpy(&dataFind, &_ds, sizeof(dataFind));
    uint8_t mas[256];
    volatile int i, j;
    switch (dataFind.typeSignal)
    {
    case eTONE:
        break;
    case eBPSK:
        for (i = 0; i < 4; i++)
        {
            for (j = 0; j < 32; j++)
            {
                mas[i * 32 + j] = (dataFind.psp[i] & (uint32_t)(1 << j)) >> j;
            }
        }
        psk.loadBPSP(&mas[0], dataFind.lenPSP, 0);
        //Debug text
        //psk.loadBPSP((unsigned char*)&StartPSP_31[0],31,0);
        break;
    case eQPSK:
        for (i = 0; i < 4; i++)
        {
            for (j = 0; j < 32; j++)
            {
                mas[i * 32 + j] = (dataFind.psp[i] & (uint32_t)(3 << 2 * j)) >> (2 * j);
            }
        }
        psk.loadQPSP(&mas[0], dataFind.lenPSP / 2, 0);
        //dataOut.lenPSP = dataOut.lenPSP / 2;
        break;
    case eLFM:
        break;
    case eLFMv:
        break;
    case eHFM:
        break;
    case eFM:
        break;
    case eReserve:
        break;
    }

    //prepareData();
}

data_signal &SIGNAL::getFindData()
{
    return dataFind;
}

SIGNAL::SIGNAL()
{
    freqDiskr = 300000;
}

void SIGNAL::prepareData()
{
    initTimer();
    initSinTable();
    volatile int i, j;
    outVal = 0;
    timVal = 0;
    bpskCNT = 0;
    bpskNUM = 0;
    switch (dataOut.typeSignal)
    {
    case eTONE:
        break;
    case eBPSK:
        for (i = 0; i < 4; i++)
        {
            for (j = 0; j < 32; j++)
            {
                dPSPOut[i * 32 + j] = (dataOut.psp[i] & (uint32_t)(1 << j)) >> j;
            }
        }
        //Debug text
        /*for( j = 0; j < 31; j++)
    {
       dPSPOut[j] = StartPSP_31[j]; 
    }*/
        break;
    //end debug text
    case eQPSK:
        for (i = 0; i < 4; i++)
        {
            for (j = 0; j < 32; j++)
            {
                dPSPOut[i * 32 + j] = (dataOut.psp[i] & (uint32_t)(3 << 2 * j)) >> (2 * j);
            }
        }
        dataOut.lenPSP = dataOut.lenPSP / 2;
        break;
    case eLFM:
        break;
    case eLFMv:
        break;
    case eHFM:
        break;
    case eFM:
        for (i = 0; i < 4; i++)
        {
            for (j = 0; j < 32; j++)
            {
                dPSPOut[i * 32 + j] = (dataOut.psp[i] & (uint32_t)(1 << j)) >> j;
            }
        }

        Ftim = HAL_RCC_GetPCLK2Freq();
        freq1ARR = (float)Ftim / (dataOut.freq1 * Npoint) - 1;
        freq2ARR = (float)Ftim / (dataOut.freq2 * Npoint) - 1;
        TIM2->ARR = Ftim * ((float)dataOut.duration * 10. / 1000.) / 31.;
        break;
    case eReserve:
        break;
    }
}

void SIGNAL::calcNext()
{
    //return (this->*emitingfunc)();
    switch (dataOut.typeSignal)
    {
    case eTONE:
        tone();
        break;
    case eBPSK:
        bpsk();
        break;
    case eQPSK:
        qpsk();
        break;
    case eLFM:
        bpsk();
        break;
    case eLFMv:
        bpsk();
        break;
    case eHFM:
        bpsk();
    case eFM:
        bfm();
        break;
    case eReserve:
        break;
    }
    //(this->*emitingfunc)();
}

void SIGNAL::emitingAudio()
{
    TIM1->CCR2 = outVal;
    //DAC1->DHR12R1 = outVal*10;
}

void SIGNAL::initTimer()
{
    TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2NE;
    TIM1->BDTR |= /*TIM_BDTR_MOE |*/ TIM_BDTR_DTG_1;
    NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
    TIM1->DIER |= TIM_DIER_UIE;
    TIM1->SR = 0;
    TIM1->CNT = 0;
    NVIC_ClearPendingIRQ(TIM1_UP_TIM16_IRQn);
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

    //----Init TIM2 fo FM signals
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->SR = 0;
    TIM2->CNT = 0;
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    NVIC_EnableIRQ(TIM2_IRQn);

    //TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
}

void SIGNAL::initSinTable()
{
    uint32_t rARR;
    float step;
    Ftim = HAL_RCC_GetPCLK2Freq();
    rARR = (float)Ftim / (dataOut.freq1 * Npoint) - 1;
    TIM1->ARR = rARR;
    TIM1->CCR2 = rARR / 2;
    step = (float)(2 * PI) / Npoint;

    for (volatile int i = 0; i <= (2 * Npoint); i++)
    {
        sintable[i] = (arm_sin_f32(i * step) * dataOut.amplitude * rARR / 2) + (rARR / 2);
    }
}

void SIGNAL::tone()
{
    outVal = sintable[timVal];
    timVal++;
    if (timVal >= Npoint)
    {
        timVal = 0;
    }
}
//uint16_t bpsk_mas[255*5*8];
//uint16_t bpsk_mas_cnt = 0;
void SIGNAL::bpsk()
{
    outVal = sintable[timVal + dPSPOut[bpskNUM] * (Npoint / 2)];
    //bpsk_mas[bpsk_mas_cnt++] = outVal;
    timVal++;
    if (timVal >= Npoint)
    {
        bpskCNT++;
        if (bpskCNT >= dataOut.num_kvant_psp)
        {
            bpskCNT = 0;
            bpskNUM++;
            if (bpskNUM >= dataOut.lenPSP)
            {
                bpskNUM = 0;
                //bpsk_mas_cnt = 0;
                StopAudio();
            }
        }
        timVal = 0;
    }
}

void SIGNAL::qpsk()
{
    outVal = sintable[timVal + dPSPOut[bpskNUM] * (Npoint / 4)];
    timVal++;
    if (timVal >= Npoint)
    {
        bpskCNT++;
        if (bpskCNT >= dataOut.num_kvant_psp)
        {
            bpskCNT = 0;
            bpskNUM++;
            if (bpskNUM >= (dataOut.lenPSP))
            {
                bpskNUM = 0;
                StopAudio();
            }
        }
        timVal = 0;
    }
}

void SIGNAL::changePSKbit()
{
    if (dPSPOut[bpskNUM])
    {
        TIM1->ARR = freq1ARR;
    }
    else
    {
        TIM1->ARR = freq2ARR;
    }
    bpskNUM++;
    if (bpskCNT > currSignal.dataOut.lenPSP)
    {
        StopAudio();
    }
}

void SIGNAL::bfm()
{
    outVal = sintable[timVal];
    timVal++;
    if (timVal >= Npoint)
    {
        timVal = 0;
    }
    //    outVal = sintable[timVal % Npoint];
    //    timVal++;
    //    if(timVal >= Npoint * dataOut.num_kvant_psp)
    //    {
    //        bpskCNT++;
    //        if(bpskCNT >= dataOut.num_kvant_psp)
    //        {
    //            bpskCNT = 0;
    //            bpskNUM ++;
    //            if(bpskNUM >= (dataOut.lenPSP))
    //            {
    //                bpskNUM = 0;
    //                StopAudio();
    //            }
    //        }
    //        timVal = 0;
    //    }
}
/*void SIGNAL::setRunSetting(typeRUN &_tr)
{
    typerun = _tr;
}*/

void SIGNAL::StartAudio()
{
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LED_But_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(LED_But_GPIO_Port, &GPIO_InitStruct);
    EXTI->IMR1 &= ~EXTI_IMR1_IM7;
    osThreadResume(LedToggleHandle);

    TIM1->SR = 0;
    NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
    NVIC_ClearPendingIRQ(TIM1_UP_TIM16_IRQn);
    TIM1->CNT = 0;
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    TIM1->BDTR |= TIM_BDTR_MOE;
    if (dataOut.typeSignal == eFM)
    {
        bpskNUM = 0;
        changePSKbit();
        SET_BIT(TIM2->CR1, TIM_CR1_CEN);
    }
    calcNext();
    emitingAudio();

    calcNext();
    //emitingAudio();
    TIM1->CR1 |= TIM_CR1_CEN;
}

void SIGNAL::StopAudio()
{
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->BDTR &= ~TIM_BDTR_MOE;

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LED_But_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    HAL_GPIO_Init(LED_But_GPIO_Port, &GPIO_InitStruct);
    EXTI->PR1 |= EXTI_PR1_PIF7;
    CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN);
    NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    osThreadSuspend(LedToggleHandle);

    //DAC->DHR12R1 = 0;
}

void SIGNAL::initTxMode(data_signal &_ds)
{
    setSignalData(_ds);
}

void SIGNAL::initTxRxMode(data_signal &_dsFind, data_signal &_dsOut)
{
    setSignalData(_dsOut);
    setFindData(_dsFind);
}

void SIGNAL::initMode(data_signal &_dsFind, data_signal &_dsOut, typeRUN _typerun)
{
    typerun = _typerun;
    switch (typerun)
    {
    case eTx:
        initTxMode(_dsOut);
        break;
    case eRxTx:
        initTxRxMode(_dsFind, _dsOut);
        break;
    case eReserveRUN:
        break;
    };
}

void SIGNAL::loadFromFlash()
{
    /*if(GPIOA->ODR & PWR_GPIO_BIT_10) 
        return;*/
    RTC_AlarmTypeDef sAlarm;
    typerun = flashdata.cnfg.typerun;
    uint32_t pos;

    switch (typerun)
    {
    case eTx:
        //iwdgThreadHandle = osThreadNew(iwdgThread, NULL, NULL);
        if ((GPIOA->IDR & PWR_GPIO_BIT_10) == 0)
        {
            pos = HAL_RTCEx_BKUPRead(&hrtc, 1);
            if (pos > flashdata.cnfg.cntSIGNAL)
            {
                osThreadSuspend(Mode2Handle);
                osThreadSuspend(Mode1Handle);
                break;
                //pos = 9;
            }
            //подготавливаем данные
            initTxMode(flashdata.cnfg.sig[pos]);
            if (flashdata.cnfg.cntSIGNAL > 1)
            {
                //режим 2
                //установить будильник
                //проверка в случае перезагрузки на диапазон будильника
                sAlarm.AlarmTime.Hours = dataOut.HH;
                sAlarm.AlarmTime.Minutes = dataOut.MM;
                sAlarm.AlarmTime.Seconds = 0x0;
                sAlarm.AlarmTime.SubSeconds = 0x0;
                sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
                sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_SET;
                sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
                sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
                sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
                sAlarm.AlarmDateWeekDay = dataOut.DD;
                sAlarm.Alarm = RTC_ALARM_A;
                HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD);
                osThreadResume(Mode2Handle);
            }
            else
            {
                //режим 1
                //включаем поток и работаем пока не сядет батарейка
                osThreadResume(Mode1Handle);
            }
        }
        osThreadSuspend(defaultTaskHandle);
        break;
    case eRxTx:
        //подготавливаем данные
        SET_BIT(AnalogStage_GPIO_Port->ODR, AnalogStage_Pin);
        initTxRxMode(flashdata.cnfg.sig[0], flashdata.cnfg.sig[1]);
        if ((GPIOA->IDR & PWR_GPIO_BIT_10) == 0)
        {
            osThreadResume(ModeRxTxHandle);
        }
        break;
    case eReserveRUN:
        break;
    };
}

void SIGNAL::loadFromFlashNextSignal()
{
    uint32_t pos;
    pos = HAL_RTCEx_BKUPRead(&hrtc, 1);
    pos++;
    HAL_RTCEx_BKUPWrite(&hrtc, 1, pos);
    loadFromFlash();
}

void SIGNAL::emitSignal()
{
    osDelay(currSignal.getSignalData().signal_pause);
    currSignal.StartAudio();
    if (currSignal.getSignalData().duration > 0)
    {
        osDelay(currSignal.getSignalData().duration * 10);
        currSignal.StopAudio();
    }
}

uint8_t &SIGNAL::getdPSPOut()
{
    return dPSPOut[0];
}
