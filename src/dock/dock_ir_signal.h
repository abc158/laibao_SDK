//  Copyright (C) 2016-2020, Amicro, Inc.
//  permission of Amicro, Inc.  All rights reserved.
#ifndef _DOCK_IR_SIGNAL_H_
#define _DOCK_IR_SIGNAL_H_
#define BACK_LEFT_STRONG         0XF001
#define BACK_LEFT_WEAK           0XF002
#define BACK_RIGHT_STRONG        0XF004
#define BACK_RIGHT_WEAK          0XF008
#define LEFT_STRONG              0XF010
#define LEFT_WEAK                0XF020
#define RIGHT_STRONG             0XF040
#define RIGHT_WEAK               0XF080
#define MIDDLE_STRONG            0XF100
#define MIDDLE_WEAK              0XF200
typedef struct
{
  unsigned int     timer;
  unsigned char    count;
  unsigned char    data_bit;
} IR_CODE;
typedef enum
{
  IR_TX_BACK_LEFT=0,
  IR_TX_LEFT,
  IR_TX_MIDDLE,
  IR_TX_RIGHT, 
  IR_TX_BACK_RIGHT,
} IR_SEND_POSITION;
typedef enum
{
  IR_TX_STRONG=0,
  IR_TX_WEAK,
} IR_SEND_STRENGTH;
IR_CODE ir_decode[5][2];
int ir_send_gpio[5][2]={
      IR_TX_BACK_LEFT_STRONG,
      IR_TX_BACK_LEFT_WEAK,
      IR_TX_LEFT_STRONG,
      IR_TX_LEFT_WEAK,
      IR_TX_MIDDLE_STRONG,
      IR_TX_MIDDLE_WEAK,
      IR_TX_RIGHT_STRONG,
      IR_TX_RIGHT_WEAK,
      IR_TX_BACK_RIGHT_STRONG,
      IR_TX_BACK_RIGHT_WEAK,
};
extern void ir_send(void);

#endif //_DOCK_IR_SIGNAL_H_