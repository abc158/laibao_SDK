//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "docking-new.h"
#include "docking-sensors.h"
#include "ui-ir.h"
#include "sensor/sensor.h"
#include "dock-avoid.h"
#include "virtual-wall.h"
#include "dock_ir_signal.h"
U16 dock_signals[IR_MAX_RECV];//need to get from core board
uint32_t dock_signal_get_time;

#define DOCK_SENSORS_DEBUG
#ifdef DOCK_SENSORS_DEBUG
enum {
	DEBUG_DOCK_SIGNAL = 0x1 << 0,
};

static U8 debug_mask = 0;
#define dprintf(level_mask, msg...)  if(debug_mask&level_mask)\
	printf(msg)
#else
#define dprintf(msg...) {}
#endif
BOOLEAN right_ir_strong(IR_REC_POSITION chan)
{
     return(((dock_signals[chan]&RIGHT_STRONG)==RIGHT_STRONG));
}
BOOLEAN right_ir_weak(IR_REC_POSITION chan)
{
    return(((dock_signals[chan]&RIGHT_WEAK)==RIGHT_WEAK));
}
BOOLEAN left_ir_strong(IR_REC_POSITION chan)
{
     return(((dock_signals[chan]&LEFT_STRONG)==LEFT_STRONG));
}
BOOLEAN left_ir_weak(IR_REC_POSITION chan)
{
   return(((dock_signals[chan]&LEFT_WEAK)==LEFT_WEAK));
}

BOOLEAN mid_ir_strong(IR_REC_POSITION chan)
{
     return(((dock_signals[chan]&MIDDLE_STRONG)==MIDDLE_STRONG));
}

BOOLEAN mid_ir_weak(IR_REC_POSITION chan)
{
     return(((dock_signals[chan]&MIDDLE_WEAK)==MIDDLE_WEAK));
}
BOOLEAN backleft_ir_strong(IR_REC_POSITION chan)
{
     return(((dock_signals[chan]&BACK_LEFT_STRONG)==BACK_LEFT_STRONG));
}

BOOLEAN backleft_ir_weak(IR_REC_POSITION chan)
{
     return(((dock_signals[chan]&BACK_LEFT_WEAK)==BACK_LEFT_WEAK));
}
BOOLEAN backright_ir_strong(IR_REC_POSITION chan)
{
     return(((dock_signals[chan]&BACK_RIGHT_STRONG)==BACK_RIGHT_STRONG));
}

BOOLEAN backright_ir_weak(IR_REC_POSITION chan)
{
     return(((dock_signals[chan]&BACK_RIGHT_WEAK)==BACK_RIGHT_WEAK));
}


////////////////////////right//////////////////////////////////////
BOOLEAN right_ir_strong_mid(void)
{
	return (right_ir_strong(AM_IR_REC_MIDDLE));
}
BOOLEAN right_ir_weak_mid(void)
{
        return (right_ir_weak(AM_IR_REC_MIDDLE));
}
BOOLEAN right_ir_strong_left(void)
{
	return (right_ir_strong(AM_IR_REC_LEFT));
}
BOOLEAN right_ir_weak_left(void)
{
        return (right_ir_weak(AM_IR_REC_LEFT));
}
BOOLEAN right_ir_strong_right(void)
{
	return (right_ir_strong(AM_IR_REC_RIGHT));
}
BOOLEAN right_ir_weak_right(void)
{
        return (right_ir_weak(AM_IR_REC_RIGHT));
}
BOOLEAN right_ir_weak_backleft(void)
{
        return (right_ir_weak(AM_IR_REC_BACK_LEFT));
}    
BOOLEAN right_ir_weak_backright(void)
{
        return (right_ir_weak(AM_IR_REC_BACK_RIGHT));
}

BOOLEAN right_ir_strong_backleft(void)
{
        return (right_ir_strong(AM_IR_REC_BACK_LEFT));
}  
BOOLEAN right_ir_strong_backright(void)
{
		return (right_ir_strong(AM_IR_REC_BACK_RIGHT));
}
/////////////////////////right end/////////////////////////
//////////////////////////////mid/////////////////////////////
BOOLEAN mid_ir_strong_mid(void)
{
	return (mid_ir_strong(AM_IR_REC_MIDDLE));
}
BOOLEAN mid_ir_weak_mid(void)
{
        return (mid_ir_weak(AM_IR_REC_MIDDLE));
}
BOOLEAN mid_ir_strong_left(void)
{
	return (mid_ir_strong(AM_IR_REC_LEFT));
}
BOOLEAN mid_ir_weak_left(void)
{
        return (mid_ir_weak(AM_IR_REC_LEFT));
}
BOOLEAN mid_ir_strong_right(void)
{
	return (mid_ir_strong(AM_IR_REC_RIGHT));
}
BOOLEAN mid_ir_weak_right(void)
{
        return (mid_ir_weak(AM_IR_REC_RIGHT));
}
BOOLEAN mid_ir_weak_backright(void)
{
        return (mid_ir_weak(AM_IR_REC_BACK_RIGHT));
}
BOOLEAN mid_ir_weak_backleft(void)
{
        return (mid_ir_weak(AM_IR_REC_BACK_LEFT));
}
BOOLEAN mid_ir_strong_backright(void)
{
        return (mid_ir_strong(AM_IR_REC_BACK_RIGHT));
}
BOOLEAN mid_ir_strong_backleft(void)
{
        return (mid_ir_strong(AM_IR_REC_BACK_LEFT));
}
/////////////////////////mid end/////////////////////////
/////////////////////////left////////////////////////////
BOOLEAN left_ir_weak_right(void)
{
        return (left_ir_weak(AM_IR_REC_RIGHT));
}
BOOLEAN left_ir_weak_left(void)
{
        return (left_ir_weak(AM_IR_REC_LEFT));
}
BOOLEAN left_ir_weak_mid(void)
{
        return (left_ir_weak(AM_IR_REC_MIDDLE));
}
BOOLEAN left_ir_weak_backleft(void)
{
        return (left_ir_weak(AM_IR_REC_BACK_LEFT));
}          

BOOLEAN left_ir_strong_right(void)
{
        return (left_ir_strong(AM_IR_REC_RIGHT));
}
BOOLEAN left_ir_strong_left(void)
{
        return (left_ir_strong(AM_IR_REC_LEFT));
}
BOOLEAN left_ir_strong_mid(void)
{
        return (left_ir_strong(AM_IR_REC_MIDDLE));
}
BOOLEAN left_ir_strong_backleft(void)
{
        return (left_ir_strong(AM_IR_REC_BACK_LEFT));
}   
///////////////////////left/////////////////////////////
///////////////////////backleft//////////////////////////
BOOLEAN backleft_ir_weak_right(void)
{
        return (backleft_ir_weak(AM_IR_REC_RIGHT));
}
BOOLEAN backleft_ir_weak_left(void)
{
        return (backleft_ir_weak(AM_IR_REC_LEFT));
}
BOOLEAN backleft_ir_weak_mid(void)
{
        return (backleft_ir_weak(AM_IR_REC_MIDDLE));
}
BOOLEAN backleft_ir_weak_backleft(void)
{
        return (backleft_ir_weak(AM_IR_REC_BACK_LEFT));
}          

BOOLEAN backleft_ir_strong_right(void)
{
        return (backleft_ir_strong(AM_IR_REC_RIGHT));
}
BOOLEAN backleft_ir_strong_left(void)
{
        return (backleft_ir_strong(AM_IR_REC_LEFT));
}
BOOLEAN backleft_ir_strong_mid(void)
{
        return (backleft_ir_strong(AM_IR_REC_MIDDLE));
}
BOOLEAN backleft_ir_strong_backleft(void)
{
        return (backleft_ir_strong(AM_IR_REC_BACK_LEFT));
}  

///////////////////////backleft end//////////////////////////
///////////////////////backright//////////////////////////
BOOLEAN backright_ir_weak_right(void)
{
        return (backright_ir_weak(AM_IR_REC_RIGHT));
}
BOOLEAN backright_ir_weak_left(void)
{
        return (backright_ir_weak(AM_IR_REC_LEFT));
}
BOOLEAN backright_ir_weak_mid(void)
{
        return (backright_ir_weak(AM_IR_REC_MIDDLE));
}
BOOLEAN backright_ir_weak_backleft(void)
{
        return (backright_ir_weak(AM_IR_REC_BACK_LEFT));
}          

BOOLEAN backright_ir_strong_right(void)
{
        return (backright_ir_strong(AM_IR_REC_RIGHT));
}
BOOLEAN backright_ir_strong_left(void)
{
        return (backright_ir_strong(AM_IR_REC_LEFT));
}
BOOLEAN backright_ir_strong_mid(void)
{
        return (backright_ir_strong(AM_IR_REC_MIDDLE));
}
BOOLEAN backright_ir_strong_backleft(void)
{
        return (backright_ir_strong(AM_IR_REC_BACK_LEFT));
}  

///////////////////////backright end//////////////////////////

/////////////////////////////Debouncer/////////////////////////////////
///////////////////////backright  //////////////////////
Debouncer_Data recently_backright_ir_strong_backleft = {
	.predicate = &backright_ir_strong_backleft,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_backright_ir_strong_left = {
	.predicate = &backright_ir_strong_left,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_backright_ir_strong_mid = {
	.predicate = &backright_ir_strong_mid,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_backright_ir_strong_right = {
	.predicate = &backright_ir_strong_right,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_backright_ir_weak_backleft = {
	.predicate = &backright_ir_weak_backleft,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_backright_ir_weak_left = {
	.predicate = &backright_ir_weak_left,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_backright_ir_weak_mid = {
	.predicate = &backright_ir_weak_mid,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_backright_ir_weak_right = {
	.predicate = &backright_ir_weak_right,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
///////////////////////backright end //////////////////////

///////////////////////back left  //////////////////////
Debouncer_Data recently_backleft_ir_strong_backleft = {
	.predicate = &backleft_ir_strong_backleft,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_backleft_ir_strong_left = {
	.predicate = &backleft_ir_strong_left,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_backleft_ir_strong_mid = {
	.predicate = &backleft_ir_strong_mid,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_backleft_ir_strong_right = {
	.predicate = &backleft_ir_strong_right,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_backleft_ir_weak_backleft = {
	.predicate = &backleft_ir_weak_backleft,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_backleft_ir_weak_left = {
	.predicate = &backleft_ir_weak_left,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_backleft_ir_weak_mid = {
	.predicate = &backleft_ir_weak_mid,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_backleft_ir_weak_right = {
	.predicate = &backleft_ir_weak_right,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
///////////////////////backleft end //////////////////////
///////////////////////left  //////////////////////
Debouncer_Data recently_left_ir_strong_backleft = {
	.predicate = &backright_ir_strong_backleft,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_left_ir_strong_left = {
	.predicate = &left_ir_strong_left,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_left_ir_strong_mid = {
	.predicate = &left_ir_strong_mid,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_left_ir_strong_right = {
	.predicate = &left_ir_strong_right,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_left_ir_weak_backleft = {
	.predicate = &left_ir_weak_backleft,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_left_ir_weak_left = {
	.predicate = &left_ir_weak_left,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_left_ir_weak_mid = {
	.predicate = &left_ir_weak_mid,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_left_ir_weak_right = {
	.predicate = &left_ir_weak_right,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
///////////////////////left end //////////////////////
//////////////////////////mid /////////////////////////
Debouncer_Data recently_mid_ir_strong_mid = {
	.predicate = &mid_ir_strong_mid,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_mid_ir_weak_mid = {
	.predicate = &mid_ir_weak_mid,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_mid_ir_strong_left = {
	.predicate = &mid_ir_strong_left,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_mid_ir_weak_left = {
	.predicate = &mid_ir_weak_left,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_mid_ir_strong_right = {
	.predicate = &mid_ir_strong_right,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_mid_ir_weak_right = {
	.predicate = &mid_ir_weak_right,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_mid_ir_weak_backright = {
	.predicate = &mid_ir_weak_backright,
	.trigger_on = 1,
	.trigger_off = 50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_mid_ir_weak_backleft = {
	.predicate = &mid_ir_weak_backleft,
	.trigger_on = 1,
	.trigger_off =50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_mid_ir_strong_backright = {
	.predicate = &mid_ir_strong_backright,
	.trigger_on = 1,
	.trigger_off = 50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_mid_ir_strong_backleft = {
	.predicate = &mid_ir_strong_backleft,
	.trigger_on = 1,
	.trigger_off = 50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
//////////////////////////mid end////////////////////////////
//////////////////////////right////////////////////////////////
Debouncer_Data recently_right_ir_strong_backleft = {
	.predicate = &right_ir_strong_backleft,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_right_ir_strong_left = {
	.predicate = &right_ir_strong_left,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_right_ir_strong_mid = {
	.predicate = &right_ir_strong_mid,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_right_ir_strong_right = {
	.predicate = &right_ir_strong_right,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_right_ir_weak_backleft = {
	.predicate = &right_ir_weak_backleft,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_right_ir_weak_backright = {
	.predicate = &right_ir_weak_backright,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_right_ir_strong_backright = {
	.predicate = &right_ir_strong_backright,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_right_ir_weak_left = {
	.predicate = &right_ir_weak_left,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_right_ir_weak_mid = {
	.predicate = &right_ir_weak_mid,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_right_ir_weak_right = {
	.predicate = &right_ir_weak_right,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
////////////////////////////////right////////////////////////////////
U8 decode_near=false;
extern void ir_send_on_off(U8 state);
U8 robot_get_dock_signals(U8 index)
{
  U8 signal=0;
#ifdef IR_WIRELESS
	dock_signals[index] = get_wireless_rx_code();
#else
	dock_signals[index] = remote_ir_get((IR_REMOT_POSITION_E)index);
#endif
//       if(dock_signals[index])
//       printf("%d REC %x  \r\n",index,dock_signals[index]);
	if (dock_signals[index] != 0)
	{
		dprintf(DEBUG_DOCK_SIGNAL, "IR%d : %x \r\n", index, dock_signals[index]);
                
    dock_signal_get_time=timer_ms();
    if((dock_signals[index]==MIDDLE_WEAK))   //避座处理
      signal=DOCK_CLOSE_BEACON;
    if((dock_signals[index]==LEFT_STRONG))   //地图信息记录
      signal=LEFT_BEACON_BYTE;
    if((dock_signals[index]==RIGHT_STRONG))
      signal=RIGHT_BEACON_BYTE;
		if(dock_signals[index]==0XF020)
    {
      decode_near=true;
      ir_send_on_off(1);  //靠近座子关闭强信号
    }
		//dock_avoid_get_signals(index, dock_signals[index]);
		//virtual_wall_get_signals(index, dock_signals[index]);
    dock_avoid_get_signals(index, signal);
    virtual_wall_get_signals(index, signal);
	}

	//return dock_signals[index];
        return signal;
}

#if 0
BOOLEAN force_field(IR_local_Index chan)
{
	return ((dock_signals[chan]&DOCK_CLOSE_BEACON) == DOCK_CLOSE_BEACON);
}

BOOLEAN buoy_left(IR_local_Index chan)
{
	return ((dock_signals[chan]&LEFT_BEACON_BYTE) == LEFT_BEACON_BYTE);
}

BOOLEAN buoy_right(IR_local_Index chan)
{
	return ((dock_signals[chan]&RIGHT_BEACON_BYTE) == RIGHT_BEACON_BYTE);
}

BOOLEAN check_near_dock(void)
{
	return (((force_field(IR_LOCAL_MID_RIGHT)) || (force_field(IR_LOCAL_MID_LEFT)) || \
		(force_field(IR_LOCAL_LEFT)) || (force_field(IR_LOCAL_RIGHT)) || \
		(force_field(IR_LOCAL_BACK_RIGHT)) || (force_field(IR_LOCAL_BACK_LEFT))) || \
		(recently_docking_go_forward_onlyright.current_state && recently_docking_go_forward_onlyleft.current_state));
}

BOOLEAN check_near_dock_1(void)
{
	return (((force_field(IR_LOCAL_MID_RIGHT)) || (force_field(IR_LOCAL_MID_LEFT)) || \
		(force_field(IR_LOCAL_LEFT)) || (force_field(IR_LOCAL_RIGHT))));
}

BOOLEAN check_left_near_dock(void)
{
	return ((buoy_left(IR_LOCAL_BACK_LEFT) || buoy_left(IR_LOCAL_LEFT)) || \
		(force_field(IR_LOCAL_BACK_LEFT) || force_field(IR_LOCAL_LEFT)) || \
		(buoy_right(IR_LOCAL_BACK_LEFT) || buoy_right(IR_LOCAL_LEFT)));
}

BOOLEAN check_right_near_dock(void)
{
	return ((buoy_left(IR_LOCAL_BACK_RIGHT) || buoy_left(IR_LOCAL_RIGHT)) || \
		(force_field(IR_LOCAL_BACK_RIGHT) || force_field(IR_LOCAL_RIGHT)) || \
		(buoy_right(IR_LOCAL_BACK_RIGHT) || buoy_right(IR_LOCAL_RIGHT)));
}

u8 check_center_left_focus_beacon(void)
{
    if (buoy_right(IR_LOCAL_MID_LEFT) || buoy_left(IR_LOCAL_MID_LEFT))
		return  1;
	else  if (force_field(IR_LOCAL_MID_LEFT))
		return  2;
	else
		return  0;
}

u8 check_center_right_focus_beacon(void)
{
    if (buoy_right(IR_LOCAL_MID_RIGHT) || buoy_left(IR_LOCAL_MID_RIGHT))
		return  1;
	else  if (force_field(IR_LOCAL_MID_RIGHT))
		return  2;
	else
		return  0;
}

u8 check_left_beacon(void)
{
	if (buoy_right(IR_LOCAL_LEFT) || buoy_left(IR_LOCAL_LEFT))
		return  1;
	else  if (force_field(IR_LOCAL_LEFT))
		return  2;
	else
		return  0;
}

u8 check_right_beacon(void)
{
	if (buoy_right(IR_LOCAL_RIGHT) || buoy_left(IR_LOCAL_RIGHT))
		return  1;
	else  if (force_field(IR_LOCAL_RIGHT))
		return  2;
	else
		return  0;
}

u8 check_back_left_beacon(void)
{
	if (buoy_right(IR_LOCAL_BACK_LEFT) || buoy_left(IR_LOCAL_BACK_LEFT))
		return  1;
	else  if (force_field(IR_LOCAL_BACK_LEFT))
		return  2;
	else
		return  0;
}

u8 check_back_right_beacon(void)
{
	if (buoy_right(IR_LOCAL_BACK_RIGHT) || buoy_left(IR_LOCAL_BACK_RIGHT))
		return  1;
	else  if (force_field(IR_LOCAL_BACK_RIGHT))
		return  2;
	else
		return  0;
}

BOOLEAN check_signal(void)
{
	if ((check_back_left_beacon()) || (check_back_right_beacon()) || \
		(check_left_beacon()) || (check_right_beacon()) || \
		(check_center_left_focus_beacon()) || (check_center_right_focus_beacon()))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_recently_left_left(void)
{
	return (buoy_left(IR_LOCAL_LEFT));
}

BOOLEAN check_recently_right_right(void)
{
	return (buoy_right(IR_LOCAL_RIGHT));
}

BOOLEAN check_recently_left_right(void)
{
	return (buoy_left(IR_LOCAL_RIGHT));
}

BOOLEAN check_recently_left_backleft(void)
{
	return (buoy_left(IR_LOCAL_BACK_LEFT));
}

BOOLEAN check_recently_right_backright(void)
{
	return (buoy_right(IR_LOCAL_BACK_RIGHT));
}

BOOLEAN check_recently_right_backleft(void)
{
	return (buoy_right(IR_LOCAL_BACK_LEFT));
}

BOOLEAN check_recently_left_backright(void)
{
	return (buoy_left(IR_LOCAL_BACK_RIGHT));
}

BOOLEAN check_recently_right_left(void)
{
	return (buoy_right(IR_LOCAL_LEFT));
}

BOOLEAN check_recently_force_field_middle(void)
{
	return (force_field(IR_LOCAL_MID_RIGHT) || force_field(IR_LOCAL_MID_LEFT));
}

BOOLEAN check_recently_force_field(void)
{
	return recently_near_dock_1.current_state;
}

BOOLEAN check_recently_follow_right_force_field(void)
{
	return ((force_field(IR_LOCAL_RIGHT)));
}

BOOLEAN check_recently_follow_left_force_field(void)
{
	return ((force_field(IR_LOCAL_LEFT)));
}

BOOLEAN check_recently_center_left_focus(void)
{
	return (1 == check_center_left_focus_beacon());
}

BOOLEAN check_recently_center_right_focus(void)
{
	return (1 == check_center_right_focus_beacon());
}

BOOLEAN check_docking_go_forward_right(void)
{
	if (buoy_right(IR_LOCAL_MID_RIGHT) && buoy_left(IR_LOCAL_MID_RIGHT))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_docking_go_forward_left(void)
{
	if (buoy_left(IR_LOCAL_MID_LEFT) && buoy_right(IR_LOCAL_MID_LEFT))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_docking_go_forward_onlyright(void)
{
	if (buoy_right(IR_LOCAL_MID_RIGHT) && !buoy_left(IR_LOCAL_MID_RIGHT))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_docking_go_forward_onlyleft(void)
{
	if (buoy_left(IR_LOCAL_MID_LEFT) && !buoy_right(IR_LOCAL_MID_LEFT))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_docking_go_forward(void)
{
	if ((recently_docking_go_forward_right.current_state && recently_docking_go_forward_left.current_state) || \
		(recently_docking_go_forward_onlyright.current_state && recently_docking_go_forward_onlyleft.current_state))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_docking_left(void)
{
	if (buoy_right(IR_LOCAL_MID_LEFT))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_docking_right(void)
{
	if (buoy_left(IR_LOCAL_MID_RIGHT))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_docking_bump(void)
{
	if ((get_cliff_state() != 0) || (get_bump_state() != 0))
		return TRUE;
	else
		return FALSE;
}

void set_near_dock_context(BOOLEAN value)
{
	if(value)
	{
		//set_slip_high_throd(6);
		set_stasis_high_throd();
		turn_off_touch_bump();
		for(int i=0;i<3;i++)
                {
	          set_cliff_threshold((SENSOR_E)i, 20);
                }
		set_lighttouch_enable(1);
	}
	else
	{
		//set_slip_normal();
		set_stasis_normal();
		turn_on_touch_bump();
		reset_cliff_threshold();
		set_lighttouch_enable(0);
	}
	return;
}

Debouncer_Data recently_signal = {
	.predicate = &check_signal,
	.trigger_on = 1,
	.trigger_off = 2000,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_near_dock = {
	.predicate = &check_near_dock,
	.trigger_on = 1,
	.trigger_off = 200,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = &set_near_dock_context
};

Debouncer_Data recently_near_dock_1 = {
	.predicate = &check_near_dock_1,
	.trigger_on = 1,
	.trigger_off = 10,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_left_near_dock = {
	.predicate = &check_left_near_dock,
	.trigger_on = 1,
	.trigger_off = 100,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_right_near_dock = {
	.predicate = &check_right_near_dock,
	.trigger_on = 1,
	.trigger_off = 100,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_center_left_focus = {
	.predicate = &check_recently_center_left_focus,
	.trigger_on = 1,
	.trigger_off = 50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_center_right_focus = {
	.predicate = &check_recently_center_right_focus,
	.trigger_on = 1,
	.trigger_off = 50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_go_forward_right = {
	.predicate = &check_docking_go_forward_right,
	.trigger_on = 1,
	.trigger_off = 5,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_go_forward_left = {
	.predicate = &check_docking_go_forward_left,
	.trigger_on = 1,
	.trigger_off = 5,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_go_forward_onlyright = {
	.predicate = &check_docking_go_forward_onlyright,
	.trigger_on = 1,
	.trigger_off = 10,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_go_forward_onlyleft = {
	.predicate = &check_docking_go_forward_onlyleft,
	.trigger_on = 1,
	.trigger_off = 10,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_go_forward = {
	.predicate = &check_docking_go_forward,
	.trigger_on = 1,
	.trigger_off = 15,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_left = {
	.predicate = &check_docking_left,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_right = {
	.predicate = &check_docking_right,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_left_left = {
	.predicate = &check_recently_left_left,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_right_right = {
	.predicate = &check_recently_right_right,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_left_backleft = {
	.predicate = &check_recently_left_backleft,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_right_backright = {
	.predicate = &check_recently_right_backright,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_right_backleft = {
	.predicate = &check_recently_right_backleft,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_left_backright = {
	.predicate = &check_recently_left_backright,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_left_right = {
	.predicate = &check_recently_left_right,
	.trigger_on = 1,
	.trigger_off = 50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_right_left = {
	.predicate = &check_recently_right_left,
	.trigger_on = 1,
	.trigger_off = 50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_force_field_middle = {
	.predicate = &check_recently_force_field_middle,
	.trigger_on = 1,
	.trigger_off = 10,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_force_field = {
	.predicate = &check_recently_force_field,
	.trigger_on = 1,
	.trigger_off = 1,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_no_force_field = {
	.predicate = &check_docking_go_forward,
	.trigger_on = 1,
	.trigger_off = 200,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_follow_right_force_field = {
	.predicate = &check_recently_follow_right_force_field,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_follow_left_force_field = {
	.predicate = &check_recently_follow_left_force_field,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_bump = {
	.predicate = &check_docking_bump,
	.trigger_on = 1,
	.trigger_off = 30,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
#endif

