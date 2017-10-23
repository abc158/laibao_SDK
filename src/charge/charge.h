/************************************************************************
** file: charge.h
** ���������״̬����
** ���ߣ�lyy
** �汾��v1.0
** �汾��ʷ
*************************************************************************/
#ifndef CHARGE_H
#define CHARGE_H

#include "am_type.h"

#define CHARGE_ENABLE 0
#define CHARGE_DISABLE 1


typedef enum
{
  CHARGING_OFF = 0, 
  CHARGING_RECOVERY = 1,
  CHARGING_ON = 2,
  CHARGING_PURLING = 3,
  CHARGING_WAITING = 4,
  CHARGING_COMPLETE = 5
}CHARGING_STATE_E;


typedef enum
{
  CHARGING_NONE = 0, 
  CHARGING_NOBATTERY = 1,
  CHARGING_OVERCURRENT = 2,
  CHARGING_NOCURRENT = 3,
  CHARGING_OVERTEMP = 4
}CHARGING_STATE_ERROR_E;


//��ѹ�����ϵbat_voltage = Vadc/0.174  bat adc = bat_voltage*0.174*4096/3.3
// ���1.5v 6�� ����ѹ9v �ָ���ѹ6*0.9=5.4v �͵�6*1.08=6.48v 

#define CHARGING_MAXVOL          (3628)//16.8V
#define CHARGING_RECOVERYVOL      2591 //12v
#define CHARGING_AFTERFULL_MINVOL 3563 //16.5V
#define CHARGING_LOW_VOLTAGE      2916 //13.5V  //13.2V
#define CHARGING_CUTOFF_VOLTAGE   2634 //2591 //12.2v 
#define BATTERY_LOW_SLOWVECTORY   2268 //13.5v

//���ʱ���ֹ
#define CHARGING_TIME_ALLOW_MINUTES    (4*60) //7Сʱ
#define CHARGING_TIME_RECOVERY_MINUTES (4*60) //6Сʱ
#define CHARGING_TIME_COMPLETE_MINUTES (3*60) //3Сʱ
#define CHARGING_TIME_PURLING_MINUTES  (1*60) //30����
    
#define BATTERY_REF_OFFSET 10

extern void  charging_init(void);
extern void  process_charging(void);
extern void  quit_charging(void);
extern u16   charging_state_get(void);
U16 charging_jack(void);
U16 charging_dock(void);
s16 convert_to_voltage( s16 v_adc);
s16 convert_to_current( s16 c_adc);
S16 charge_current_get(void);
#endif