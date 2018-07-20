#include "communication.h"

Communication comm;





Communication::Communication()
{
    currLen = 0;
}


uint32_t Communication::rxUSB(uint32_t _currLen)
{
    currLen += _currLen;  
    rtPacket *rt =  (rtPacket*)&UserRxBufferFS[0];
    if(_currLen < 64)
    {
        if( currLen  == ( rt->lenDATA + 4 ) )
        {
            parse();
			currLen = 0;
        }
        //parse command, end of block
    }
    return currLen;
}

void Communication::parse()
{    
    rtPacket *rt =  (rtPacket*)&UserRxBufferFS[0];
    switch(rt->typeCMD)
    {
    case eSetCNFG:
        writeToFlashRunData((configPACKET*)&rt->data);
        currSignal.loadFromFlash();
        sendOK();
        break;        
    case eAskReadCNFG:
        sendToPCflashData();
        break; 
    case eConnect:
        sendOK();
        break;
    };
    
    
}

void Communication::sendOK()
{
    rtPacket *rt =  (rtPacket*)&UserTxBufferFS[0]; 
    rt->typeCMD = eOK;
    rt->lenDATA = 0;
    while(CDC_Transmit_FS(&UserTxBufferFS[0],4)!= HAL_OK)
    {
    }
}

void Communication::sendERR()
{
    rtPacket *rt =  (rtPacket*)&UserTxBufferFS[0]; 
    rt->typeCMD = eERR;
    rt->lenDATA = 0;
    while(CDC_Transmit_FS(&UserTxBufferFS[0],4)!= HAL_OK)
    {
    }
}

void Communication::writeToFlashRunData(configPACKET* data)
{
    HAL_FLASH_Unlock();
    
    //----Erase old data-------------------
    uint32_t staterr;
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES; 
    erase.NbPages = 1;
    erase.Banks = FLASH_BANK_1;
    erase.Page = NUM_PAGE_FLASH_DATA;
    HAL_FLASHEx_Erase(&erase,&staterr);
    //----End of erase old data-------------
    //----Program new data------------------
    uint32_t sizetoprogram = ( (data->cntSIGNAL * sizeof(data_signal) + 4) / 8) + 1;
    uint64_t val;
    uint64_t *pval;
    for(volatile int32_t i=0; i < sizetoprogram; i++)
    {
        pval = (uint64_t*)((uint8_t*)data + i * 8);
        val = *pval;
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,ADDRESS_FLASH_DATA + i * 8, val);
    }
    //----End of program new data------------

    HAL_FLASH_Lock();
}

void Communication::sendToPCflashData()
{ 
    rtPacket *rt =  (rtPacket*)&UserTxBufferFS[0]; 
    rt->typeCMD = eSendCNFG_PAM;
    if(flashdata.cnfg.typerun == 0xFFFF)
    {
        sendERR();
        return;
    }
    rt->lenDATA = 4 + flashdata.cnfg.cntSIGNAL * sizeof(data_signal);    
    memcpy(&UserTxBufferFS[4],&flashdata,rt->lenDATA);
    while(CDC_Transmit_FS(&UserTxBufferFS[0],4 + rt->lenDATA )!= HAL_OK)
    {
    }
}











