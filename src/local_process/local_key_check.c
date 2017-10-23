//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "local_key_check.h"
#include "am_key.h"
#include "ui-commands.h"
#include "am_date_base.h"
#include "act.h"
#include "syscall_api.h"

static U16 key_press_on[TOTAL_KEY_NUM];  /*�������µĴ���*/
static U8 key_press_off[TOTAL_KEY_NUM];  /*����̧��Ĵ���*/

/*����������*/
static key_state_t key_press_state[TOTAL_KEY_NUM]=
{	
  {PRESS_NULL, AM_IO_MC_KEY_WAKE_ON, 1, KEY_CLEAN}	,
  {PRESS_NULL, AM_IO_BUTTON_SPOT, 0, KEY_NULL },
  {PRESS_NULL, AM_IO_BUTTON_DOCK, 0, KEY_NULL },
  {PRESS_NULL, AM_IO_BUTTON_CLOCK, 0, KEY_NULL},
  {PRESS_NULL, AM_IO_BUTTON_CLEAN, 0, KEY_NULL}
};

/****************************************************************
*Function   :  key_single_init
*Description:  ����������ʹ�õ�IO  
*Input      :  ��
*Output     :  ����
*Return     :  ��
*Others     :  
******************************************************************/
U8 key_single_init()
{
  int ret = -1;
  for( int i=0; i<TOTAL_KEY_NUM; i++ )
  {
    ret = gpio_request_one(key_press_state[i].am_io, GPIO_F_DIR_IN);
    
    if(ret != -1)
    {
      ;
    }
    else
    {
      printf("key io not ok\r\n");
    }
  }  
  return 0;
}

/****************************************************************
*Function   :  key_single_init
*Description:  �ͷ�������ʹ�õ�IO  
*Input      :  ��
*Output     :  ����
*Return     :  ��
*Others     :  
******************************************************************/
U8 key_single_exit()
{
  for( int i=0; i<TOTAL_KEY_NUM; i++ )
  {
    gpio_free(key_press_state[i].am_io);
  }     
  return 0;
}

/****************************************************************
*Function   :  _key_init
*Description:  ��������ʼ��  
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
******************************************************************/
void _key_init(void)
{  
  U8 i;

  key_single_init();
  for(i=0; i<TOTAL_KEY_NUM; i++)
  {
    key_press_state[i].state = PRESS_NULL;
    key_press_on[i]          = 0;
    key_press_off[i]         = 0;
  }
}

/****************************************************************
*Function   :  _key_exit
*Description:  �����˳�  
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
******************************************************************/
void _key_exit(void)
{
  key_single_exit();
}


/****************************************************************
*Function   :  key_routine
*Description:  ������⣬��¼������Ϣ  
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
******************************************************************/
void key_routine(void)
{ 
	u8 i;
	u8 gpio_state;
	
  //read all gpio
  for(i=0; i<TOTAL_KEY_NUM; i++)
  {
    if(key_press_state[i].keyval==KEY_NULL)
    {
      continue;
    }
    else
    {
      gpio_state = gpio_get_value(key_press_state[i].am_io);
      if (gpio_state == key_press_state[i].input_gpio_state)
      {
        key_press_on[i]++; 
        key_press_off[i] = 0;    
        if (key_press_on[i] >= 4 && key_press_on[i] < 200)
        {
          key_press_state[i].state = KEY_DOWN;
        }
        else if (key_press_on[i] >= 200 && key_press_on[i] < 400)
        {
          key_press_state[i].state = KEY_LONG;
        }
        else if (key_press_on[i] >= 400)
        {
          key_press_state[i].state = KEY_HOLD;
          key_press_on[i] = 400;
        }
      } 
      else
      {
        key_press_off[i]++;
        if (key_press_off[i] >= 4)
        {
          if (key_press_on[i] >= 4 && key_press_on[i] < 200)
          {
            key_press_state[i].state = KEY_SHORT_UP;          
          } 
          else if (key_press_on[i] >= 200 && key_press_on[i] < 400)
          {
            key_press_state[i].state = KEY_LONG_UP; 
          }
          else if (key_press_on[i] >= 400)
          {
            key_press_state[i].state = KEY_HOLD_UP; 
          }
          key_press_on[i] = 0;         
        }
      }                     
    }
  }  
}

/****************************************************************
*Function   :  get_key_state
*Description:  ������������ŵĵ�ַ
*Input      :  ��
*Output     :  ��������Ϣ��ŵĵ�ַ
*Return     :  ��
*Others     :  
******************************************************************/
key_state_t *get_key_state(void)
{
  return &key_press_state[0];
}

/****************************************************************
*Function   :  any_key
*Description:  �Ƿ������������� 
*Input      :  ��
*Output     :  1���а��� 0���ް���
*Return     :  ��
*Others     :  
******************************************************************/
u8 any_key(void)
{
  u8 i = 0;
  u8 gpio_state;
  for(i=0; i<TOTAL_KEY_NUM; i++)
  {
    gpio_state = gpio_get_value(key_press_state[i].am_io);
    if ((gpio_state == key_press_state[i].input_gpio_state)&&(key_press_state[i].keyval!=KEY_NULL))
    {
      //key_press_state[i].state = PRESS_NULL;
      return 1;
    }
  }
  return 0;
}
