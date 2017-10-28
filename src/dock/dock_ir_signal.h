//  Copyright (C) 2016-2020, Amicro, Inc.
//  permission of Amicro, Inc.  All rights reserved.
#ifndef _DOCK_IR_SIGNAL_H_
#define _DOCK_IR_SIGNAL_H_
#define BACK_LEFT_STRONG         0XF001
#define BACK_LEFT_WEAK           0XF002
#define LEFT_STRONG              0XF004
#define LEFT_WEAK                0XF008
#define MIDDLE_STRONG            0XF010
#define MIDDLE_WEAK              0XF020
#define RIGHT_STRONG             0XF040
#define RIGHT_WEAK               0XF080
#define BACK_RIGHT_STRONG        0XF100
#define BACK_RIGHT_WEAK          0XF200
typedef struct
{
  unsigned int     timer;
  unsigned char    count;
  unsigned char    data_bit;
} IR_CODE;
typedef enum{  
  AM_IR_REC_BACK_LEFT,
  AM_IR_REC_LEFT,
  AM_IR_REC_MIDDLE,
  AM_IR_REC_RIGHT,
  AM_IR_REC_BACK_RIGHT,
}IR_REC_POSITION;
typedef enum{  
  AM_SEND_BACK_LEFT,
  AM_SEND_LEFT,
  AM_SEND_MIDDLE,
  AM_SEND_RIGHT,
  AM_SEND_BACK_RIGHT,
}IR_SEND_POSITION;
typedef enum
{
  IR_TX_STRONG=0,
  IR_TX_WEAK,
} IR_SEND_STRENGTH;
extern void ir_send(void);

#endif //_DOCK_IR_SIGNAL_H_