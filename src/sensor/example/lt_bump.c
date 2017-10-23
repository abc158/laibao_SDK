#include "am_type.h"
#include "sensor/sensor.h"
#include "syscall_api.h"
#include "am_config_macro.h"

#define max(a,b) ((a)>(b)?(a):(b))

#define TOUCH_RIGHT_MOVE 2

#define TOUCH_BUMP_TRIGGER_TIME 120

#define LIGHT_BUMP_DELAY_COUNT   4//5
#define DOCK_LIGHT_TOUCH_THROD  30

BOOLEAN touch_bump_onoff=FALSE;
BOOLEAN tb_on_sig=FALSE;
static u32 front_time_stp=0;

/****************************************************************
*Function   :  turn_off_touch_bump
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  关闭light touch bump功能
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void turn_off_touch_bump()
{
  if(touch_bump_onoff==FALSE)
  {
  }
  touch_bump_onoff=TRUE;
}

/****************************************************************
*Function   :  turn_on_touch_bump
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  打开light touch bump功能
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void turn_on_touch_bump()
{
  if(touch_bump_onoff==TRUE)
  {
    front_time_stp=0;
  }
  touch_bump_onoff=FALSE;
}

//return value:
//0: no bump
//1: left bump
//2: right bump
//3: center bump
int touch_bump_run()
{
  u16 value_cl, value_cr,value_far_cl, value_far_cr;
  u16 value_fl, value_fr, value_far_fl, value_far_fr;
  u16 value_center, value_far_center;
  u32 time_now;
  static u16 value_max=0;
  static u8 decay_count=0;

  static u16 cl_value_max=0;
  static u8 cl_decay_count=0;
  static u32 cl_time_stp=0;

  static u16 cr_value_max=0;
  static u8 cr_decay_count=0;
  static u32 cr_time_stp=0;
  int TOTCH_BUMP_TIME_DELAY = 0;
  
  int lt_bump=0;

  if(touch_bump_onoff)
    return 0;
  
  value_fl = robot_rear_lt_distance(LT_FRONTLEFT)>>TOUCH_RIGHT_MOVE;
  value_fr = robot_rear_lt_distance(LT_FRONTRIGHT)>>TOUCH_RIGHT_MOVE;
  value_cl = robot_rear_lt_distance(LT_CENTERLEFT)>>TOUCH_RIGHT_MOVE;
  value_cr = robot_rear_lt_distance(LT_CENTERRIGHT)>>TOUCH_RIGHT_MOVE;

  value_far_fl = robot_signal_distance(LT_FRONTLEFT)>>TOUCH_RIGHT_MOVE;
  value_far_fr = robot_signal_distance(LT_FRONTRIGHT)>>TOUCH_RIGHT_MOVE;
  value_far_cl = robot_signal_distance(LT_CENTERLEFT)>>TOUCH_RIGHT_MOVE;
  value_far_cr = robot_signal_distance(LT_CENTERRIGHT)>>TOUCH_RIGHT_MOVE;

  TOTCH_BUMP_TIME_DELAY = 900;

  time_now = timer_ms();

  value_center = (value_fl + value_fr)>>1;
  value_far_center = (value_far_fl + value_far_fr)>>1;


  //process the front touch bump
  if((value_center >= max(UI_SET_LT_BUMP_THROD,50)) && (timer_elapsed(front_time_stp)>TOTCH_BUMP_TIME_DELAY))
  {
    if(value_max < value_center)
    {
      value_max = value_center;
      decay_count=0;
    }

    if(((value_max*7/8) > value_center) || (value_center>=UI_SET_LT_MAX_LIGHT_VALUE) || (value_center*2 > value_far_center))
    {
      decay_count++;
      if(decay_count== 2)
      {
        if(touch_bump_onoff==FALSE)
        {
        	lt_bump=3;
        }
        front_time_stp = timer_ms();   //this is to delete multiple trigger
      }

    }
  }
  else
  {
    value_max=value_center;
    decay_count=0;
    tb_on_sig=FALSE;
  }

  {
    time_now = timer_ms();

    if((time_now-front_time_stp)>TOTCH_BUMP_TIME_DELAY)   //when the front is in trigger, don't trigger the other touch
    {
        //center left
        if((value_cl >= max(UI_SET_LT_BUMP_LEFT_RIGHT_THROD,50)) && (timer_elapsed(cl_time_stp)>TOTCH_BUMP_TIME_DELAY))
        {
          if(cl_value_max < value_cl)
          {
            cl_value_max = value_cl;
            cl_decay_count=0;
          }

          if((cl_value_max*7/8 > value_cl) || (value_cl>=UI_SET_LT_MAX_LIGHT_VALUE) || (value_cl*2 > value_far_cl))
          {
            cl_decay_count++;
            if(cl_decay_count== 2)
            {
              if(touch_bump_onoff==FALSE)
              {
                lt_bump |= 1;
              }
              cl_time_stp = timer_ms(); 
            }
          }
          else
          {
            cl_value_max=value_cl;
            cl_decay_count=0;
            tb_on_sig=FALSE;
          }
        }
        else
        {
          cl_value_max=value_cl;
          cl_decay_count=0;
          tb_on_sig=FALSE;
        }
        //center right
        time_now = timer_ms();
        if((time_now-cl_time_stp)>TOTCH_BUMP_TIME_DELAY)   //to make sure there is only one touch bump is trigger
        {
          if((value_cr >= max(UI_SET_LT_BUMP_LEFT_RIGHT_THROD,50)) && (timer_elapsed(cr_time_stp)>TOTCH_BUMP_TIME_DELAY))
          {
            if(cr_value_max <= value_cr)
            {
              cr_value_max = value_cr;
              cr_decay_count=0;
            }

            if(((cr_value_max*7/8)>value_cr) || (value_cr >= UI_SET_LT_MAX_LIGHT_VALUE) || (value_cr*2>value_far_cr))
            {
              cr_decay_count++;
              if(cr_decay_count==2)//LIGHT_BUMP_DELAY_COUNT)
              {
                if(touch_bump_onoff==FALSE)
                {
                  lt_bump |= 2;
                }
                cr_time_stp = timer_ms();   //this is to delete multiple trigger
              }
            }
            else
            {
              cr_value_max=value_cr;
              cr_decay_count=0;
              tb_on_sig=FALSE;
            }
          }
          else
          {
            cr_value_max=value_cr;
            cr_decay_count=0;
            tb_on_sig=FALSE;
          }
        }
    }

  }
  return lt_bump;
}



