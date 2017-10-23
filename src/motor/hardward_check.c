//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include <am_type.h>
#include <syscall_api.h>

/****************************************************************
*Function   :  wheel_motors_fault_check
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ������ӵ�������Ƿ�fault�쳣
*CallBy     :  
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void wheel_motors_fault_check(void)
{
  if(get_ui_config()->funtion_info.clean_fun.bits.motor_driver == 1)
  {
    ;//do nothing
  }
  else
  {
    static u8 fault_times = 0;
    if(!gpio_get_value(AM_IO_MOTOR_WHEEL_FAULT))
    {
      fault_times++;
      if(fault_times >= 5) //����50ms��⵽
      {
        int delay = 1000;  
        fault_times = 0;
        gpio_set_value( AM_IO_MOTOR_WHEEL_RESET , 0) ;
        while(delay--);
        gpio_set_value( AM_IO_MOTOR_WHEEL_RESET , 1) ;
        printf("motor reset\r\n");
      }
    }
    else
    {
      fault_times = 0;
    }
  }
}
