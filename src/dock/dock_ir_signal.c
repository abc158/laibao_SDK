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
#define data_length         16            //�ź�λ��
#define long_level          12           //����ƽʱ��long_level*0.25ms
#define short_level         4           //�̵�ƽʱ��short_level*0.25ms
#define sinal_interval      200        //�źż�� sinal_interval*0.25ms

void set_38k_off(void)  //38K�źŹر�
{
  sys_set_pwm_out_duty(IR_SEND_PWM_CHANNEL,0);
}
void set_38k_on(void)   //38K�źŴ�
{
  sys_set_pwm_out_duty(IR_SEND_PWM_CHANNEL,135);
}
/****************************************************************
*Function   :  ir_send_code
*Description:  �����ź� 
*Input      :  IR_SEND_POSITION instance:����ͷID ��u16 ir_code ������ֵ  
IR_SEND_STRENGTH strength:����ǿ��
*Output     :  ��
*Return     :  false δ�������  true �������
*Others     :  
******************************************************************/
bool ir_send_code(IR_SEND_POSITION instance,u16 ir_code,IR_SEND_STRENGTH strength)  
{
  IR_CODE *code_p;
  u16 ir_code_state;
  code_p=&ir_decode[instance][strength];
  code_p->timer++;  //�����ź����ڼ���
  if(code_p->timer<=(data_length*(long_level+short_level)-1))  //��һ���ź�����count����
  code_p->count++; //�ߵ͵�ƽ�ļ���
  ir_code_state =ir_code;  //���ͱ���
  //��ȡ����λ���Ҹ��ݵ�·ȡ��
  if(!(ir_code_state&(1<<(data_length-1-code_p->data_bit)))&&(code_p->timer<=(data_length*(long_level+short_level)-1)))
  {
    if(code_p->count<=short_level)   //�̵�ƽ
    {

      gpio_set_value(ir_send_gpio[instance][strength],1);
    }
    else if(code_p->count>short_level&&code_p->count<=(long_level+short_level)) //����ƽ
    {
      gpio_set_value(ir_send_gpio[instance][strength],0);
      if(code_p->count>=(long_level+short_level)) 
      {
          code_p->data_bit++;   //��¼�Ѿ����͵�λ��
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
  if(code_p->timer>(data_length*(long_level+short_level)-1))  //�ź���������ܽ�����
  {
    gpio_set_value(ir_send_gpio[instance][strength],0);
    
  }
  
  if(code_p->timer>=(sinal_interval+data_length*(long_level+short_level)-1)) //�ź�ʱ��+�ź���ʱ=�ź�����
  {
    code_p->timer=0;
    code_p->count=0;
    code_p->data_bit=0;
  }
  if(code_p->data_bit>=data_length)  //�������
  {
    code_p->data_bit=0;
    return true;
  }
  else 
    return false;
}
U16 send_count=0;
U8 IR_WEAK=0;
U8 IR_STRONG=0;
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
}
void ir_tx_send_4khz(void)
{

    {
          ir_send_code(IR_TX_BACK_LEFT,BACK_LEFT_WEAK,IR_TX_WEAK);
          ir_send_code(IR_TX_BACK_RIGHT,BACK_RIGHT_WEAK,IR_TX_WEAK);
          ir_send_code(IR_TX_LEFT,LEFT_WEAK,IR_TX_WEAK);
          ir_send_code(IR_TX_RIGHT,RIGHT_WEAK,IR_TX_WEAK);
          ir_send_code(IR_TX_MIDDLE,MIDDLE_WEAK,IR_TX_WEAK);
    
    }

    {
        ir_send_code(IR_TX_BACK_LEFT,BACK_LEFT_STRONG,IR_TX_STRONG);
        ir_send_code(IR_TX_BACK_RIGHT,BACK_RIGHT_STRONG,IR_TX_STRONG);
        ir_send_code(IR_TX_LEFT,LEFT_STRONG,IR_TX_STRONG);
        ir_send_code(IR_TX_RIGHT,RIGHT_STRONG,IR_TX_STRONG);
        ir_send_code(IR_TX_MIDDLE,MIDDLE_STRONG,IR_TX_STRONG);
    
    }
    

}