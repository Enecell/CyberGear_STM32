#ifndef __MI_CAN_H
#define __MI_CAN_H

/*----------------小米电机CAN初始化函数库-----------------*/

void MI_CAN_Init(void); //初始化CAN
void MI_CAN_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data);
uint8_t MI_CAN_ReceiveFlag(void);
void MI_CAN_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data);

#endif
