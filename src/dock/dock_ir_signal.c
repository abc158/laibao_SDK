//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "am_type.h"
#include "am_robot_type.h"
#include "syscall_api.h"
#include "am_tm_api.h"
#include "docking-new.h"
#include "dock_ir_signal.h"
#define SIGNAL_INTERVAL_SEND        //强弱信号分开发 强弱信号之间间隔时间SIGNAL_INTERVAL_SEND*0.25ms  强/弱信号间隔时间SIGNAL_INTERVAL_SEND*0.25ms+data_length*4ms
#define data_length         16            //信号位数
#define long_level          12           //长电平时间long_level*0.25ms
#define short_level         4           //短电平时间short_level*0.25ms
#ifdef  SIGNAL_INTERVAL_SEND
#define sinal_interval      140        //信号间隔 sinal_interval*0.25ms
#else
#define sinal_interval      200        //信号间隔 sinal_interval*0.25ms
#endif

IR_CODE ir_decode[5][2];
int ir_send_gpio[5][2]={
      IR_TX_BACK_LEFT_STRONG,IR_TX_BACK_LEFT_WEAK,
      IR_TX_LEFT_STRONG,IR_TX_LEFT_WEAK,
      IR_TX_MIDDLE_STRONG,IR_TX_MIDDLE_WEAK,
      IR_TX_RIGHT_STRONG,IR_TX_RIGHT_WEAK,
      IR_TX_BACK_RIGHT_STRONG,IR_TX_BACK_RIGHT_WEAK,
};
void set_38k_off(void)  //38K信号关闭
{
  sys_set_pwm_out_duty(IR_SEND_PWM_CHANNEL,0);
}
void set_38k_on(void)   //38K信号打开
{
  sys_set_pwm_out_duty(IR_SEND_PWM_CHANNEL,135);
}
/****************************************************************
*Function   :  ir_send_code
*Description:  发射信号 
*Input      :  IR_SEND_POSITION instance:发射头ID ；u16 ir_code 发射码值  
IR_SEND_STRENGTH strength:发射强度
*Output     :  无
*Return     :  false 未发射完成  true 发射完成
*Others     :  
******************************************************************/
bool ir_send_code(IR_SEND_POSITION instance,u16 ir_code,IR_SEND_STRENGTH strength)  
{
  IR_CODE *code_p;
  u16 ir_code_state;
  code_p=&ir_decode[instance][strength];
  code_p->timer++;  //发送信号周期计数
  if(code_p->timer<=(data_length*(long_level+short_level)-1))  //在一个信号内用count计数
  code_p->count++; //高低电平的计数
  ir_code_state =ir_code;  //发送编码
  //先取出高位并且根据电路取反
  if(!(ir_code_state&(1<<(data_length-1-code_p->data_bit)))&&(code_p->timer<=(data_length*(long_level+short_level)-1)))
  {
    if(code_p->count<=short_level)   //短电平
    {

      gpio_set_value(ir_send_gpio[instance][strength],1);
    }
    else if(code_p->count>short_level&&code_p->count<=(long_level+short_level)) //长电平
    {
      gpio_set_value(ir_send_gpio[instance][strength],0);
      if(code_p->count>=(long_level+short_level)) 
      {
          code_p->data_bit++;   //记录已经发送的位数
          code_p->count=0;
      }
    } 
  }
  else if((ir_code_state&(1<<(data_length-1-code_p->data_bit)))&&(code_p->timer<=(data_length*(long_level+short_level)-1)))
  {
    if(code_p->count<=long_level)
    {
     gpio_set_value(ir_send_gpio[instance][strength],1);
    }
    else if(code_p->count>long_level&&code_p->count<=(long_level+short_level))
    {
      gpio_set_value(ir_send_gpio[instance][strength],0);
      if(code_p->count>=(long_level+short_level)) 
      {
          code_p->data_bit++;
          code_p->count=0;
      }
    }
  }
  if(code_p->timer>(data_length*(long_level+short_level)-1))  //信号周期以外管脚拉低
  {
    gpio_set_value(ir_send_gpio[instance][strength],0);
    
  }
  
  if(code_p->timer>=(sinal_interval+data_length*(long_level+short_level)-1)) //信号时间+信号延时=信号周期
  {
    code_p->timer=0;
    code_p->count=0;
    code_p->data_bit=0;
    return true;
  }
  else 
    return false;
}

U8 IR_WEAK=1;
U8 IR_STRONG=1;
void SET_IR_STATE(U8 state)
{
    if(state==1)
    {
      IR_WEAK=1;
      IR_STRONG=0;
    }
    else if(state==2)
    {
      IR_WEAK=0;
      IR_STRONG=1;
    }
    else if(state==3)
    {
      IR_WEAK=1;
      IR_STRONG=1;
    }
}
U8 IR_SEND_MODE=0;
void ir_send_on_off(U8 state)
{
    IR_SEND_MODE=state;
}
void ir_send_both(void)
{
    U8 signal_finsh=0;
    if(IR_STRONG==1)
    {
        signal_finsh=ir_send_code(AM_SEND_BACK_LEFT,BACK_LEFT_STRONG,IR_TX_STRONG);
        ir_send_code(AM_SEND_BACK_RIGHT,BACK_RIGHT_STRONG,IR_TX_STRONG);
        ir_send_code(AM_SEND_LEFT,LEFT_STRONG,IR_TX_STRONG);
        ir_send_code(AM_SEND_RIGHT,RIGHT_STRONG,IR_TX_STRONG);
        ir_send_code(AM_SEND_MIDDLE,MIDDLE_STRONG,IR_TX_STRONG);
#ifdef  SIGNAL_INTERVAL_SEND
        if(signal_finsh)
        {
          SET_IR_STATE(1);
        }
#endif
    }
  
    else  if(IR_WEAK==1)
    {
        signal_finsh=ir_send_code(AM_SEND_BACK_LEFT,BACK_LEFT_WEAK,IR_TX_WEAK);
        ir_send_code(AM_SEND_BACK_RIGHT,BACK_RIGHT_WEAK,IR_TX_WEAK);
        ir_send_code(AM_SEND_LEFT,LEFT_WEAK,IR_TX_WEAK);
        ir_send_code(AM_SEND_RIGHT,RIGHT_WEAK,IR_TX_WEAK);
        ir_send_code(AM_SEND_MIDDLE,MIDDLE_WEAK,IR_TX_WEAK);
#ifdef  SIGNAL_INTERVAL_SEND
        if(signal_finsh)
        {
          SET_IR_STATE(2);
        }
#endif
    }
    (void)signal_finsh; 
}
void ir_send_weak_only(void)
{
        ir_send_code(AM_SEND_BACK_LEFT,BACK_LEFT_WEAK,IR_TX_WEAK);
        ir_send_code(AM_SEND_BACK_RIGHT,BACK_RIGHT_WEAK,IR_TX_WEAK);
        ir_send_code(AM_SEND_LEFT,LEFT_WEAK,IR_TX_WEAK);
        ir_send_code(AM_SEND_RIGHT,RIGHT_WEAK,IR_TX_WEAK);
        ir_send_code(AM_SEND_MIDDLE,MIDDLE_WEAK,IR_TX_WEAK);
}
void ir_send_strong_only(void)
{
        ir_send_code(AM_SEND_BACK_LEFT,BACK_LEFT_STRONG,IR_TX_STRONG);
        ir_send_code(AM_SEND_BACK_RIGHT,BACK_RIGHT_STRONG,IR_TX_STRONG);
        ir_send_code(AM_SEND_LEFT,LEFT_STRONG,IR_TX_STRONG);
        ir_send_code(AM_SEND_RIGHT,RIGHT_STRONG,IR_TX_STRONG);
        ir_send_code(AM_SEND_MIDDLE,MIDDLE_STRONG,IR_TX_STRONG);
}
void ir_tx_send_4khz(void)
{
    if(IR_SEND_MODE==0)
    ir_send_both();
    
    if(IR_SEND_MODE==1)
    ir_send_weak_only();
    
    if(IR_SEND_MODE==2)
    ir_send_strong_only();
    
    if(IR_SEND_MODE==3)
      ;
}