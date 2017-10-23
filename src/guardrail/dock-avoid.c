//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------

#include "docking-new.h"
#include "sensor/sensor.h"
#include "dock-avoid.h"
#ifdef IR_WIRELESS
#include "dock_ir_signal.h"
#include "wireless/arf2496k.h"
#endif

#define DOCK_AVOID_SIGNAL_TIME_THRD 500
#define DOCK_AVOID_SIGNAL_INIT_TIME_THRD 10000
#define DOCK_AVOID_SIGNAL_ACTIVE(time) (timer_elapsed(time) < DOCK_AVOID_SIGNAL_TIME_THRD)
#define DOCK_AVOID_SIGNAL_INIT_ACTIVE(time) (timer_elapsed(time) < DOCK_AVOID_SIGNAL_INIT_TIME_THRD)

static dock_avoid_data_t dock_avoid_data;

void dock_avoid_get_signals(U8 index, U8 signal)
{
	dock_config_t *dock_config = NULL;

	dock_config = get_dock_config();

	if ((1 << index) & dock_config->dock_avoid_chan)
	{
		if((signal & DOCK_CLOSE_BEACON) == DOCK_CLOSE_BEACON)
		{
			dock_avoid_data.see_avoid_signal_time = timer_ms();
		}
	}

	if ((1 << index) & dock_config->binocular_see_avoid_chan)
	{
		if((signal & DOCK_CLOSE_BEACON == DOCK_CLOSE_BEACON))
		{
			dock_avoid_data.binocular_see_avoid_time = timer_ms();
		}
	}

	return;
}

static U32 get_avoid_signal_time(void)
{
	return dock_avoid_data.see_avoid_signal_time;
}

/*min:TRUE,����������ɨʱ��������Ϊ�Ƿ񴥷���
      FALSE,����������ɨʱ����ʼ��������Ϊʱ�������Ƿ񴥷����ϡ�*/
BOOLEAN dock_avoid_active(BOOLEAN min)
{
	if (min == TRUE)
	{
    	return DOCK_AVOID_SIGNAL_ACTIVE(get_avoid_signal_time());
	}
	else
	{
		return DOCK_AVOID_SIGNAL_INIT_ACTIVE(get_avoid_signal_time());
	}
}

/* Բ�ݽ��յ��ĳ����Բ����Ϣ��ǿ��
   ������Ϊȷ���Ƿ�����ɹ� */
U8 dock_avoid_signal_quality(void)
{
    U32 elapsed = timer_elapsed(get_avoid_signal_time());
    if (elapsed >= DOCK_AVOID_SIGNAL_TIME_THRD)
    {
        return 0;
    }
    return 255 - (U8)(elapsed*255/(DOCK_AVOID_SIGNAL_TIME_THRD*3));
}

/* ˫Ŀ���յ��ĳ����Բ����Ϣ��ǿ��
   ����ȷ�ϳ�����ڵ�ͼ�е�λ�ã�ͬʱ���ڱ�����Ϊȷ���Ƿ�����ɹ� */
U8 dock_binocular_avoid_signal_quality(void)
{
    U32 elapsed = timer_elapsed(dock_avoid_data.binocular_see_avoid_time);
    if (elapsed >= DOCK_AVOID_SIGNAL_TIME_THRD)
    {
        return 0;
    }
    return 255 - (U8)(elapsed*255/(DOCK_AVOID_SIGNAL_TIME_THRD*3));
}