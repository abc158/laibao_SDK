//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "ui-config.h"
#include "local_key_check.h"
#include "remote.h"
#include "ui-commands.h"
#include "am_config_macro.h"
#include "act.h"
#include "syscall_api.h"
#include "docking-new.h"
#include "ui-ir.h"
static U16 ir_press_on[TOTAL_IR_NUM];  /*ң�ذ������µĴ���*/
static U16 ir_press_off[TOTAL_IR_NUM]; /*ң�ذ����ͷŵĴ���*/

/*ң�ذ�������*/
static ir_state_t ir_press_state[TOTAL_IR_NUM]=
{	
  {KEY_NULL, (U32) IR_CLEAN,       KEY_CLEAN},
  {KEY_NULL, (U32) IR_DOCK,        KEY_DOCK},
  {KEY_NULL, (U32) IR_SPOT,        KEY_SPOT},
  {KEY_NULL, (U32) IR_POWER,       KEY_POWER},
  {KEY_NULL, (U32) IR_LEFT,        KEY_LEFT},
  {KEY_NULL, (U32) IR_RIGHT,       KEY_RIGHT},
  {KEY_NULL, (U32) IR_FORWARD,     KEY_FORWARD},
  {KEY_NULL, (U32) IR_SCHEDULE,    KEY_PA},
  {KEY_NULL, (U32) IR_DIRT,        KEY_DIRT},
  {KEY_NULL, (U32) IR_DRIVERSTOP,  KEY_STOP}, 
  {KEY_NULL, (U32) IR_BACKWARD,    KEY_WIFI} ,
  {KEY_NULL, (U32) IR_ADJUST,      KEY_ADJUST} ,  
  {KEY_NULL, (U32) IR_WALLFOLLOW,  KEY_WALLFLOW} ,
  {KEY_NULL, (U32) IR_GRID,        KEY_GRID} , 
  {KEY_NULL, (U32) IR_SOUND,       KEY_SOS} ,     
};

extern void ir_rx_init(void);
extern uint8_t get_ir_local(void);

/****************************************************************
*Function   :  _remote_init
*Description:  ң�ذ�����ʼ��  
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
******************************************************************/
void _remote_init(void)
{
  U8 i;
  
  for(i = 0; i < TOTAL_IR_NUM; i++)
  {
    ir_press_state[i].state = KEY_NULL;
    ir_press_on[i]  = 0;
    ir_press_off[i] = 0;
  } 
  ir_rx_init();
}

/****************************************************************
*Function   :  _remote_exit
*Description:  ң�ذ����˳� 
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
******************************************************************/
void _remote_exit(void)
{
  ;
}

/****************************************************************
*Function   :  remote_routine
*Description:  ң�ذ����Ľ���ʹ���  
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
******************************************************************/
void remote_routine(void)
{
  ir_rx_decode();
  handle_remote();
}


/****************************************************************
*Function   :  handle_remote
*Description:  ң�ذ�����⣬��¼������Ϣ 
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
******************************************************************/
static void handle_remote( void )
{
  U16 ir_val = KEY_NULL;  
  U8 i = 0;
  
  ir_val = get_ir_local();
  for(i=0; i<TOTAL_IR_NUM; i++)
  {
    if (ir_val == ir_press_state[i].ir_code)
    {
	  printf("re_ir:%x\r\n",ir_val);
      ir_press_on[i]++; 
      ir_press_off[i] = 0;    
      if (ir_press_on[i] >= (REMOTE_DATA_KEEP_COUNT+1) && ir_press_on[i] < 200)
      {
        ir_press_state[i].state = KEY_DOWN;
      }
      else if (ir_press_on[i] >= 200 && ir_press_on[i] < 400)
      {
        ir_press_state[i].state = KEY_LONG;
      }
      else if (ir_press_on[i] >= 400)
      {
        ir_press_state[i].state = KEY_HOLD;
        ir_press_on[i] = 400;
      }
    }
    else
    {
      ir_press_off[i]++;
      if (ir_press_off[i] >= 15)
      {
        if (ir_press_on[i] >= (REMOTE_DATA_KEEP_COUNT+1) && ir_press_on[i] < 200)
        {
          ir_press_state[i].state = KEY_SHORT_UP;          
        } 
        else if (ir_press_on[i] >= 200 && ir_press_on[i] < 400)
        {
          ir_press_state[i].state = KEY_LONG_UP; 
        }
        else if (ir_press_on[i] >= 400)
        {
          ir_press_state[i].state = KEY_HOLD_UP; 
        }
        ir_press_on[i] = 0;         
      }
    }
  }
} 

/****************************************************************
*Function   :  get_ir_state
*Description:  ����ң�ذ�����ŵĵ�ַ
*Input      :  ��
*Output     :  ң�ذ�����Ϣ��ŵĵ�ַ
*Return     :  ��
*Others     :  
******************************************************************/
ir_state_t *get_ir_state(void)
{
  return &ir_press_state[0];
}




