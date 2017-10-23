#ifndef __SYSCALL_DOCK_API_H__
#define __SYSCALL_DOCK_API_H__
//#include "stdio.h"
//#include "stdlib.h"
//#include "string.h"
//#include "ui-manager.h"
//#include "irq_syscall.h"
//#include "hal_amicro_gpio.h"
//#include "afio.h"
//#include "am_robot_type.h"
//#include "docking-core.h"
//#include "adc_chan.h"
//#include "ui-test_cmd_handle.h"
//#include "am_rebound.h"

#define EXPORT extern

/*!!!!!!!!!!!!!!下面的函数不能在中断上下文使用!!!!!!!!!!!!*/
/****************************************************************
*Function   :  dock_is_enable
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  判断dock 功能是否开启了
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT uint8_t dock_is_enable(void);

EXPORT void dock_core_enable(void);
EXPORT void dock_core_disable(void);
EXPORT void unregister_dock_function(U8 priorty);
EXPORT S8 register_dock_function(Dock_Data *dock_funtion);
EXPORT S8 register_dock_signals(get_dock_signals dock_signals);
EXPORT S8 register_random_conut(get_random_count dock_random);
EXPORT S8 last_dock_behavior(void);
EXPORT S8 current_dock_behavior(void);
EXPORT BOOLEAN register_debouncer(Debouncer_Data *debouncer_funtion);
EXPORT BOOLEAN unregister_debouncer(Debouncer_Data *debouncer_funtion);
EXPORT void clear_debouncer(void);
EXPORT U16 get_random(void);

EXPORT S16 robot_turn(BOOLEAN first, S16 rotation_angle, S16 l_speed,S16 r_speed, BOOLEAN care_bump, S8 *result);
EXPORT S16 robot_drive_go(BOOLEAN first, S16 distance, S16 speed, BOOLEAN care_bump, S8 *result);

#endif
