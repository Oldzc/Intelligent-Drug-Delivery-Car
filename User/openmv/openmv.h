#ifndef __OPENMV_H
#define __OPENMV_H


#include "bsp_sys.h"

extern  uint8_t  Num, LoR, Finded_flag, FindTask;
extern u8 RoomNum, TargetNum ,TASK;
extern char TargetRoom;

extern u8 FindStartFlag;
extern u16 FindTimeCount;

extern uint8_t uart3_rxbuff;

void  Openmv_Receive_Data(uint8_t data);
void SetTargetRoom(void);
void SendDataToOpenmv(void);

#endif
