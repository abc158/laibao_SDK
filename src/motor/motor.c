//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*edit by lyy*/
#include <am_type.h>
#include <charge/charge.h>
#include <am_tm_api.h>
#include <syscall_api.h>
#include <motor/robot_brush.h>
#include <motor/robot_suction.h>

BOOLEAN initialized; 

/****************************************************************
*Function   :  calc_cur_duty
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ��adcֵת��Ҫ�����pwmռ�ձ�
*CallBy     :  
*Input      :  ����
*              voltage�� adcֵ�� �ѵ�ص�ѹΪ�ο��㣬
*             �����ص�ѹ������16.8 = 3268. 
*             ���Ҫ�ٷְ��������adc = 3628.
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
int calc_cur_duty(int16_t voltage)
{
  int percent = ((voltage*100)/CHARGING_MAXVOL );
  const u8 limit = 100;

  if(percent > limit)
    percent = limit;  
        
  int pwm = (percent * 10);
  
  return pwm;
}

/****************************************************************
*Function   :  robot_motor_update
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ��գ�����ˢ pwm update
*CallBy     :  
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_motor_update()
{
  robot_suction_update();
  robot_brush_update();
}

/****************************************************************
*Function   :  robot_motor_init
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ��գ�����ˢ ����init������һ����ʱ��ȥ��̬update pwm
*CallBy     :  
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_motor_init(void)
{
    if(!initialized)
    {
        timer_task_register(robot_motor_update, 100, TRUE);
        initialized = 1;
    }
    robot_suction_init();
    robot_brush_init();
}