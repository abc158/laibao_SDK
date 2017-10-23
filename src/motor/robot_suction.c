//----------------------------------------------------------------------------
//  Copyright (C) 2001-2011, Amicro, Inc.
//  permission of Amicro, Inc.  All rights reserved.
//----------------------------------------------------------------------------
/*edit by lyy*/
#include <am_type.h>
#include <charge/charge.h>
#include <am_tm_api.h>
#include <syscall_api.h>
#include "robot_suction.h"

#define ROBOT_HAS_VAC_POWER_CONTROL  //if  has vac power 

BOOLEAN suction_off;
s16   cur_duty;
s16   pwm_voltage; 


/****************************************************************
*Function   :  get_suction_current
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ��յ�����adcֵ��ȡ
*CallBy     :  
*Input      :  ����
*              ��
*Output     :  ��
*Return     :  ���� adcֵ
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
u16 get_suction_current(void)
{
  // need to do....
  return get_adc_chan_val(ADC_CHAN_VAC_CURENT);
}

/****************************************************************
*Function   :  update_suction_duty
*Author     :  lyy
*Date       :  2017.4.20
*Description:  pwmֵ���µ�timer
*CallBy     :  
*Input      :  ����
*              duty��ռ�ձ�
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void update_suction_duty(int16_t duty)
{
#ifdef USE_BLDC_PWM_CONTROL
static am_uint32 vac_zero_stp=0;
static am_boolean vac_zero_sig=FALSE;
  if(duty == 0 && ((vac_zero_sig == FALSE && timer_elapsed(vac_zero_stp)>1000) || (vac_zero_sig==TRUE)))
  {
    vac_zero_stp = timer_ms();
    vac_zero_sig = TRUE;
    sys_set_pwm_out_duty(VACUUM_PWM_CHANNEL,duty);
  }
  else
  {
    if(vac_zero_sig == TRUE && timer_elapsed(vac_zero_stp)<5000)
    {
      sys_set_pwm_out_duty(VACUUM_PWM_CHANNEL,1001);
    }
    else
    {
      vac_zero_sig = FALSE;
      sys_set_pwm_out_duty(VACUUM_PWM_CHANNEL,duty);
    }
  }
#else
    sys_set_pwm_out_duty(VACUUM_PWM_CHANNEL,duty);
  
#ifdef ROBOT_HAS_VAC_POWER_CONTROL
  // printf("update duty=%d\r\n",duty);
   gpio_set_value(AM_IO_VAC_POWER, ((duty!=0)?1:0));
#endif
   
#endif
}

/****************************************************************
*Function   :  robot_is_suction_on
*Author     :  lyy
*Date       :  2017.4.20
*Description:  �ж�����Ƿ���pwm���
*CallBy     :  
*Input      :  ����
*              ��
*Output     :  ��
*Return     :  ���� 1:��  0����
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
BOOLEAN robot_is_suction_on(void)
{
    return (cur_duty != 0);
}

/****************************************************************
*Function   :  robot_suction_ctrl
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ʹ�����pwm���
*CallBy     :  
*Input      :  ����
*              en:  1:ʹ��   0����ʹ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_suction_ctrl(BOOLEAN en)
{
    suction_off = en;
}

/****************************************************************
*Function   :  suction_is_maybe_stall
*Author     :  lyy
*Date       :  2017.4.20
*Description:  �ж���տ��ܱ���ס
*CallBy     :  
*Input      :  ����
*              current �� ��ס��������ֵ
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
u8 suction_is_maybe_stall(u16 current)
{  
    return ((get_suction_current() > current) && (suction_off == 0));
}

/****************************************************************
*Function   :  robot_suction_update
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ���pwm ��̬����
*CallBy     :  
*Input      :  ����
*              ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_suction_update(void)
{ 
    if(suction_off == FALSE)
    {
        cur_duty = calc_cur_duty(pwm_voltage);
        update_suction_duty(cur_duty);
    }
    else
    {
        update_suction_duty(0);
    }   
}


/****************************************************************
*Function   :  robot_is_maybe_not_bustbin
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ����ռ��������
*CallBy     :  
*Input      :  ����
*              ��
*Output     :  ��
*Return     :  ���� 1�������䱻�γ���  0��������״̬����
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
u8 robot_is_maybe_not_bustbin(void)
{
  return gpio_get_value(AM_I0_GARBAGE_BOX_DETECT);
}

/****************************************************************
*Function   :  robot_suction_init
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ��ձ����ȵȳ�ʼ��
*CallBy     :  
*Input      :  ����
*              ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_suction_init(void)
{
    suction_off = 1;
    cur_duty = 0;
    pwm_voltage  = 0;
   
    robot_suction_adjust_set(VACUUM_NORMAL_VOLTAGE);  

}

/****************************************************************
*Function   :  robot_suction_vols_set
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ���pwm����
*CallBy     :  
*Input      :  0���ر�pwm   >0:��pwm
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_suction_vols_set(u16 val)
{
    if(val > 0)
      suction_off = 0;
    else
      suction_off = 1;  
	if(!gpio_get_value(AM_I0_WATER_TANK_DETECT))
	  suction_off = 1;
}

/****************************************************************
*Function   :  robot_suction_adjust_set
*Description:  ���pwm����
*Input      :  ����
*              val��adcֵ�� �ѵ�ص�ѹΪ�ο��㣬
*             �����ص�ѹ������16.8 = 3268. 
*             ���Ҫ�ٷְ��������adc = 3628.
*Output     :  ��
*Return     :  ��
*Others     :  by ljh
******************************************************************/
void robot_suction_adjust_set(u16 val)
{
    pwm_voltage = val;
}