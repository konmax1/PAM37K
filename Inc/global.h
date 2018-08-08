#ifndef GLOBAL_H
#define GLOBAL_H

#include "stm32l4xx_hal.h"
#include "string.h"
#include "arm_math.h"
#include "cmsis_os2.h"
#include "psk.h"

#define SIZE_TABLE (30)
#define Npoint (8)
#define ADDRESS_FLASH_DATA (0x0803F800)
#define NUM_PAGE_FLASH_DATA (127)


enum typeRUN : uint16_t
{
    eTx = 1,
    eRxTx = 2,
    eReserveRUN = 0xFFFF
};

enum typeSignals : uint16_t{
    eTONE = 0,
    eBPSK = 1,
    eQPSK = 2,
    eLFM = 3,
    eLFMv = 4,
    eHFM = 5,
    eFM = 6,
    eReserve = 0xFFFF
};

enum typeCMD : uint16_t{
    eOK = 0x01,
    eERR = 0x0F,
    eSetCNFG = 0x10,
    eAskReadCNFG = 0x11,
    eSendCNFG_PAM = 0x12,
    eConnect = 0x20,
    eRsrvCMD = 0xFFFF,
};

struct data_signal
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
};

struct rtPacket
{
    uint16_t typeCMD;
    uint16_t lenDATA;
    uint32_t data;
};

struct configPACKET
{
    typeRUN typerun;
    uint16_t cntSIGNAL;
    data_signal sig[10];
};

union flashDATA
{
    uint8_t rawData[2048] ;
    configPACKET cnfg;
};

class SIGNAL
{
private:  
    //FM data
    uint32_t freq1ARR,freq2ARR;
    //end of FM data
	typeRUN typerun;
    uint32_t bpskCNT;
    uint32_t bpskNUM;
    uint32_t timVal;
    uint32_t outVal;
    float Ftim;
    uint8_t dPSPOut[256];
    uint16_t sintable[SIZE_TABLE * 2];
    float freqDiskr;
    data_signal dataOut;
    data_signal dataFind;
    void initTimer();
    void initSinTable();
    void prepareData();
    void setSignalData(data_signal& _ds);
    void setFindData(data_signal& _ds);
    
    void tone();
    void bpsk();
    void qpsk();
    void bfm();
public:
    
    SIGNAL();
    void changePSKbit();
    void emitSignal(); 
    uint8_t& getdPSPOut();
    data_signal& getSignalData();
    data_signal& getFindData();
    void initTxMode(data_signal& _ds);
    void initTxRxMode(data_signal& _dsFind,data_signal& _dsOut);
    void initMode(data_signal& _dsFind,data_signal& _dsOut, typeRUN _typerun);
    void loadFromFlash();
    void loadFromFlashNextSignal();
	//void setRunSetting(typeRUN &_tr);
	//void init
    void StartAudio();
    void StopAudio();
    void calcNext();
    void emitingAudio();
    
};

extern flashDATA flashdata;

extern SIGNAL currSignal;
extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;
extern RTC_HandleTypeDef hrtc;
//extern DAC_HandleTypeDef hdac1;

extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim1;

extern osThreadId_t defaultTaskHandle;
extern osThreadId_t Mode1Handle;
extern osThreadId_t Mode2Handle;
extern osThreadId_t ModeRxTxHandle;
extern osThreadId_t LedToggleHandle;
extern osThreadId_t ButtonThreadHandle;
//extern osThreadId_t iwdgThreadHandle;

extern osSemaphoreId_t rtcIRQHandle;
extern osSemaphoreId_t adcHandle;
extern osSemaphoreId_t ButtonHandle;

extern osMessageQueueId_t adcQueueHandle;

extern volatile unsigned int t1,t2,t3,t4,t5;






















#endif
