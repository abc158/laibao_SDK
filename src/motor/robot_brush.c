//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*edit by lyy*/
#include <am_type.h>
#include <am_tm_api.h>
#include <syscall_api.h>
#include "robot_brush.h"

static u8 sidebrush_off = 1;
static u8 midbrush_off = 1;
static u16 main_brush_voltage = 0;
static u16 side_brush_voltage = 0;
extern const IO_PIN_CFG io_table[];


/****************************************************************
*Function   :  update_side_duty
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ��ˢpwm  update
*CallBy     :  
*Input      :  ����
*              duty�� ռ�ձȡ�  ĿǰӲ����֧�������� ���֧�֣���
*              ͨ��duty�����������֣�  0<, ����   >0, ����
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void update_side_duty(int16_t duty)
{
  if(duty<0)
  {
    sys_set_pwm_out_duty(SIDE_BRUSH_PWM_CHANNEL,duty);
    gpio_set_value( AM_IO_SIDE_BRUSH_DIR , 1) ; 
  }
  else if(duty>0)
  {
    sys_set_pwm_out_duty(SIDE_BRUSH_PWM_CHANNEL,duty);
    gpio_set_value( AM_IO_SIDE_BRUSH_DIR , 0) ;
  }
  else
  {
    sys_set_pwm_out_duty(SIDE_BRUSH_PWM_CHANNEL,0);
    gpio_set_value( AM_IO_SIDE_BRUSH_DIR , 0);         
  }
}

/****************************************************************
*Function   :  update_main_duty
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ��ˢpwm  update
*CallBy     :  
*Input      :  ����
*              duty�� ռ�ձȡ�  ĿǰӲ����֧�������� ���֧�֣���
*              ͨ��duty�����������֣�  0<, ����   >0, ����
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void update_main_duty(int16_t duty)
{
  if(duty>0)
  {
    sys_set_pwm_out_duty(MAIN_BRUSH_PWM_CHANNEL,duty);
    gpio_set_value( AM_IO_MAIN_BRUSH_DIR , 0) ;
  }
  else if(duty<0)
  {
    sys_set_pwm_out_duty(MAIN_BRUSH_PWM_CHANNEL,duty);
    gpio_set_value(AM_IO_MAIN_BRUSH_DIR , 1) ;
  }
  else
  {
    sys_set_pwm_out_duty(MAIN_BRUSH_PWM_CHANNEL,0);
    gpio_set_value( AM_IO_MAIN_BRUSH_DIR , 0) ;      
  }
}

/****************************************************************
*Function   :  robot_brush_init
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ����ˢpwm��ʼ��
*CallBy     :  
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_brush_init(void)
{
  sidebrush_off = 1;
  midbrush_off = 1;
  robot_side_brush_adjust_set(SIDE_BRUSH_MAX_VOLTAGE);  
  robot_mid_brush_adjust_set(MAIN_BRUSH_MAX_VOLTAGE);
}

/****************************************************************
*Function   :  robot_sidebrush_vols_set
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ��ˢpwm����
*CallBy     :  
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_sidebrush_vols_set(u16 speed)
{
  if(speed > 0)
  {
    sidebrush_off = FALSE;
  }
  else
  {
    sidebrush_off = TRUE;
  }
}

/****************************************************************
*Function   :  robot_midbrush_vols_set
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ��ˢpwm����
*CallBy     :  
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_midbrush_vols_set(u16 speed)
{
  if(speed > 0)
  {
    midbrush_off = FALSE;
  }
  else
  {
    midbrush_off = TRUE;
  }
  if(!gpio_get_value(AM_I0_WATER_TANK_DETECT))
	   midbrush_off = TRUE;
}

/****************************************************************
*Function   :  robot_brush_update
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ����ˢ��pwm����
*CallBy     :  
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_brush_update(void)
{
    u16 cur_duty = 0;
    if(sidebrush_off == FALSE)
    {
        cur_duty = calc_cur_duty(side_brush_voltage);
        update_side_duty(cur_duty);
    }
    else
    {
        update_side_duty(0);
    }   
    
    if(midbrush_off == FALSE)
    {
        cur_duty = calc_cur_duty(main_brush_voltage);
        update_main_duty(cur_duty);
    }
    else
    {
        update_main_duty(0);
    }      
    
}
/****************************************************************
*Function   :  robot_side_brush_adjust_set
*Description:  ��ˢpwm����
*Input      :  ����
*Output     :  ��
*Return     :  ��
*Others     :  by ljh
******************************************************************/
void robot_side_brush_adjust_set(s16 speed)
{
    side_brush_voltage = speed;
}

/****************************************************************
*Function   :  robot_mid_brush_adjust_set
*Description:  ��ˢpwm����
*Input      :  ����
*Output     :  ��
*Return     :  ��
*Others     :  by ljh
******************************************************************/
void robot_mid_brush_adjust_set(u16 speed)
{
    main_brush_voltage = speed;
}

