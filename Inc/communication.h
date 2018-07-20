#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#define APP_RX_DATA_SIZE  1000
#define APP_TX_DATA_SIZE  1000

#include "stm32l4xx_hal.h"
#include "string.h"
#include "arm_math.h"
#include "global.h"
#include "usbd_cdc_if.h"


class Communication
{
private:
    uint32_t currLen;
    void sendOK();
    void sendERR();
    void writeToFlashRunData(configPACKET* data);
    void sendToPCflashData();
public:
    uint32_t rxUSB(uint32_t _currLen);
    Communication();
    void parse();
};




extern Communication comm;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];














#endif
