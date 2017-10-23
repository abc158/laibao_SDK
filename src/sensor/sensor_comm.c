//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*edit by lyy*/
#include "sensor/sensor.h"
#include "syscall_api.h"

static u8 g_sensor_start_gather;
extern volatile u16 time_4khz_counter_touch;
u8 cliff_touch_turn_switch =0;

u16 mid_filter(s16* data_array, u16 new_data)
{
  u16 temp;
  u16 temp_array[3];
  
  temp_array[0]=*data_array;
  temp_array[1]=*(data_array+1);
  temp_array[2]=new_data;
  
  if(temp_array[0]>temp_array[1])
  {
    temp=temp_array[1];
    temp_array[1]=temp_array[0];
    temp_array[0]=temp;
  }
  if(temp_array[1]>temp_array[2])
  {
    temp=temp_array[2];
    temp_array[2]=temp_array[1];
    temp_array[1]=temp;
  }
  if(temp_array[0]>temp_array[1])
  {
    temp=temp_array[1];
    temp_array[1]=temp_array[0];
    temp_array[0]=temp;
  }
  
  *data_array=*(data_array+1);
  *(data_array+1)=new_data;
  return temp_array[1]; 
}


/****************************************************************
*Function   :  robot_sensor_gather_start
*Author     :  lyy
*Date       :  2017.4.20
*Description:  控制采样使能
*CallBy     :  
*Input      :  参数
*              en: 1： 打开   0：关闭
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_sensor_gather_start(u8 en)
{
  g_sensor_start_gather = en;
  if(en == 1)
  {
     set_lighttouch_enable(0);
  }
}

/****************************************************************
*Function   :  robot_sensor_handler
*Author     :  lyy
*Date       :  2017.4.20
*Description:  4k的中断采样和处理过程
*CallBy     :  
*Input      :  参数
*              无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
long robot_sensor_handler(void)
{
  static u8 led_close = 0;
  static u8 lt_cnt = 0;
  if(g_sensor_start_gather)
  {
    led_close = 0;
	if(cliff_touch_turn_switch)
	{
		sensor_gather_cliff();
		sensor_handle_cliff();	
		hal_isr();
		cliff_touch_turn_switch =0;
	}
	else
	{
		lt_cnt++;
		if (lt_cnt >= 1)
		{
		  lt_cnt = 0;
		  sensor_gather_touch();
		  sensor_handle_touch();
		  time_4khz_counter_touch = (time_4khz_counter_touch + 1) & 0xff;
		  cliff_touch_turn_switch =1;
		} 		
	}
  }
  else
  {
    if(led_close == 0)
    {
      led_close = 1;
      robot_sensor_init();
      robot_close_sensor_led();
    #ifdef USE_LT_AUTO_ADJUST
      if (is_lt_offset_adjust_enable()) {
          reset_lt_auto_offset();
      }
    #endif
    }
    lt_cnt = 0;
    time_4khz_counter_touch = 0;
  }
  return (1);
}

/****************************************************************
*Function   :  robot_ir_detection_init
*Author     :  lyy
*Date       :  2017.4.20
*Description:  初始化lt，cliff采样模块，申请一个4k的中断来进行采样。
                所以整个采样和处理过程都是在中断里
*CallBy     :  
*Input      :  参数
*              无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_ir_detection_init(void)
{
  robot_sensor_init();
  #ifdef USE_LT_AUTO_ADJUST
    //reset_ht_auto_offset();    
    printf("restore_lt_signal_offset!!!!\r\n");
    restore_lt_signal_offset();
  #endif
  sys_timer_register(HZ_4K,(long)robot_sensor_handler,0);
}