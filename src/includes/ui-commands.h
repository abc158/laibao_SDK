#ifndef _COMMAND_H__
#define _COMMAND_H__

/*����״̬*/
typedef enum
{
    CMD_STOP  =  0x0,/*ֹͣ����*/
    CMD_RUN   =  0x1 /*ִ������*/
}CMD_STATE_E;

/*��������*/
typedef enum
{
    CONTROL_CMD  =  0x0,  /*��������*/
    CONFIG_CMD   =  0x1,  /*��������*/
    STATE_CMD    =  0x2,  /*״̬����*/	
    TEST_CMD     =  0x3,  /*��������*/
    DATA_CMD     =  0x4,  /*��������*/
    MP_TEST      =  0x5
}CMD_TYPE_E;

///////////////������������/////////////////
/*�������byteΪ0xf��ͷ��32λ,ע�����0xf�����������,���Բ��ܸ���,ÿһ�����������0xfxxxxxxx��ʽ*/
/*������������*/
typedef enum
{
    CMD_DOCK               = 0xf0000001,   /*�ҳ��������*/
    CMD_CLEAN              = 0xf0000002,   /*��ʼ��ɨ��ֹͣ��ɨ����*/
    CMD_SPOT               = 0xf0000004,   /*�ֲ�������ɨ����*/
    CMD_SCHDULE            = 0xf0000008,   /*����ԤԼ������*/ 
    CMD_CLOCK              = 0xf0000010,   /*����ϵͳʱ������*/
    CMD_POWER              = 0xf0000020,   /*�������ߵ�����*/   
    CMD_LEFT               = 0xf0000040,   /*����ת*/
    CMD_RIGHT              = 0xf0000080,   /*����ת*/
    CMD_FORWARD            = 0xf0000100,   /*��ǰ��*/
    CMD_BACK               = 0xf0000200,   /*�����*/
    CMD_SONG_ID            = 0xf0000400,   /*������������IC���������*/
    CMD_WALL_FOLLOW        = 0xf0000800,   /*�ӱ���ɨ*/
    CMD_RANDOM             = 0xf0001000,   /*�����ɨ*/
    CMD_DIRT               = 0xf0002000,   /*�ص���ɨ*/
    CMD_SET_GPIO           = 0xf0004000,   /*��ûʵ��*/
    CMD_SHUTOFF            = 0xf0008000,   /*�ػ�����*/ 
    CMD_GRID1              = 0xf0008001,   /*����*/
    CMD_GRID2              = 0xf0008002,   /*�ֲ�������*/
    CMD_GRID3              = 0xf0008004,   /*����*/
    CMD_OPEN_PA            = 0xf0008008,   /*����PA*/
    CMD_SOUND_OUT          = 0xf0008010,   /*�Ƿ��������岥������*/
    CMD_UV_ENABLE          = 0xf0008020,   /*����UV��*/
    CMD_EXIT               = 0xf0040000,//α����ڲ��������Ʒ������ֹͣ
    CMD_MODE               = 0xf0040001, 
    CMD_SOS                = 0xf0040002,
    CMD_TESTDATA_DISPLAY   = 0xf0040003,//α����ڲ��������Ʒ������ֹͣ
    CMD_TEST               = 0xf0040004,//α����ڲ��ж�
    CMD_DISABLE_SENSOR     = 0xf0040005,
    
    CMD_ADJUST_MOTOR       = 0xf0040007,
    CMD_WIFI               = 0xf0040008,
    CMD_REST_WIFI          = 0xf0040009,
    CMD_POWER_ON           = 0xf004000a,
    CMD_POWER_DOWN         = 0xf004000b,
    CMD_MOP_CLEAN          = 0xf004000c,    /*�ϵ�ģʽClean*/
    CMD_GET_MAP            = 0xf004000d,    /*��ȡ��ͼ��Ϣ*/
    
    CMD_LONG_TEST          = 0xf0080001,    /*�ϻ���������*/
    CMD_PAUSE              = 0xf0080002,
    CMD_TIMING             = 0xf0080003,    /*��ʱ������*/
    CMD_FULL_GO            = 0xf0080004,    /*full go*/
    CMD_SET_ERR_NUM        = 0xf0008040,    /*���ó���ID*/
    CMD_SPEED_FAST         = 0xf004000e,    /*α����,����*/
    CMD_SPEED_LOW          = 0xf004000f,    /*α����,����*/
    CMD_REMOTE_STOP        = 0xf0040010,    /**/
    CMD_MAX                = 0xf8000000
}CONTROL_CMD_E;


typedef enum
{
    CMD_TEST_WAIT      = 0x00,
    CMD_BUMP_L         = 0x01, /*�������������ײ����Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���ɹ�*/
    CMD_BUMP_R         = 0x02, /*���������Ҳ���ײ����Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���ɹ�*/
    CMD_BUMP_B         = 0x03, /*����������������ͬʱ��ײ�ļ���Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���ɹ�*/   
    CMD_CLIFF_L        = 0x04, /*��������������¼���Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���ɹ�*/
    CMD_CLIFF_R        = 0x05, /*���������Ҳ����¼���Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���ɹ�*/
    CMD_CLIFF_FL       = 0x06, /*��������ǰ������¼���Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���� ��*/
    CMD_CLIFF_FR       = 0x07, /*��������ǰ�Ҳ����¼���Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���� ��*/
    CMD_CLIFF_RL       = 0x08, /*���������������¼���Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���� ��*/
    CMD_CLIFF_RR       = 0x09, /*����������Ҳ����¼���Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���� ��*/                   
    CMD_LT_L           = 0x0a, /*����������࿿ǽ����Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���ɹ�*/
    CMD_LT_R           = 0x0b, /*���������Ҳ࿿ǽ����Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���ɹ�*/
    CMD_LT_CL          = 0x0c, /*������������࿿ǽ����Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���ɹ�*/
    CMD_LT_CR          = 0x0d, /*�����������Ҳ࿿ǽ����Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���ɹ�*/
    CMD_LT_FL          = 0x0e, /*��������ǰ��࿿ǽ����Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���ɹ�*/
    CMD_LT_FR          = 0x0f, /*��������ǰ�ҿ�ǽ����Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���ɹ�*/
    CMD_WHEEL_DROP_L   = 0x10, /*������������̧�����Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1*/
    CMD_WHEEL_DROP_R   = 0x11, /*������������̧�����Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1:�ɹ�*/
    CMD_REMOTE         = 0x12, /*������������ǽ����Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���ɹ�*/     
    CMD_BUOY_L         = 0x13, /*����������Ŀ����Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���ɹ�*/
    CMD_BUOY_R         = 0x14, /*����������Ŀ����Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���ɹ�*/
    CMD_BAT            = 0x15, /*���������ص�ѹ�Ƿ�����������ظ������ݣ���ص�ѹ����λmv��*/
    CMD_TEMP           = 0x16, /*�����������¶��Ƿ�����������ظ������ݣ�����¶ȣ���λ���϶� */
    CMD_JACK_CHARGING  = 0x17, /*���������ѹ��������Ƿ�����������ظ������ݣ�0��ʧ�ܣ�  1���ɹ�*/
    CMD_JACK_CURRENT   = 0x18, /*���������ѹ���������Ƿ�����������ظ������ݣ�����������λma*/
    CMD_DOCK_CHARGING  = 0x19, /*������������������Ƿ�����������ظ������ݣ�0��ʧ�ܣ�1���ɹ�*/
    CMD_DOCK_CURRENT   = 0x1a, /*�������������������Ƿ�����������ظ������ݣ�����������λma*/    
    CMD_WHEEL_L_FORWARD= 0x1B, /*������������ת��ʱ�ĵ����Ƿ�����������ظ������ݣ���������λma*/    
    CMD_WHEEL_R_FORWARD= 0x1C, /*������������ת��ʱ�ĵ����Ƿ�����������ظ������ݣ���������λma*/    
    CMD_WHEEL_L_REV_SPD= 0x1D, /*������������ת�����ٶ��Ƿ�����������ظ������ݣ������趨�ٶȺ�ʵ���ٶȵĲ�ֵ������ֵ��*/ 
    CMD_WHEEL_R_REV_SPD= 0x1E, /*������������ת�����ٶ��Ƿ�����������ظ������ݣ������趨�ٶȺ�ʵ���ٶȵĲ�ֵ������ֵ��*/
    CMD_MAIN_FORWARD   = 0x1f, /*����������ˢ����ת��ʱ�ĵ����Ƿ�����������ظ������ݣ���������λma*/ 
    CMD_MAIN_REV       = 0x20, /*����������ˢ����ת��ʱ�ĵ����Ƿ�����������ظ������ݣ���������λma*/ 
    CMD_SIDE_FORWARD   = 0x21, /*�������帱ˢ����ת��ʱ�ĵ����Ƿ�����������ظ������ݣ���������λma*/ 
    CMD_SIDE_REV       = 0x22, /*�������帱ˢ����ת��ʱ�ĵ����Ƿ�����������ظ������ݣ���������λma*/    
    CMD_VACCUM         = 0x23, /*�������������ĵ����Ƿ�����������ظ������ݣ���������λma*/
    CMD_BINFULL        = 0x24, /*TURE OR FALSE*/
    CMD_DEBRIS         = 0x25, /*TURE OR FALSE*/
    CMD_OPEN_ALL_LEDS  = 0x26,
    CMD_UI_VERSION     = 0x27,
    CMD_WHEEL_SPEED    = 0x28,
    CMD_GYRO           = 0X29,
    CMD_FLASH_SONG     = 0x2A,
    CMD_UART           = 0x2B,
    CMD_MPLAYER        = 0x2C,
    CMD_MASTER_VERSION = 0x2D,
    CMD_TICK           = 0x2E,
    CMD_LONGTEST       = 0x2F,
    CMD_STASIS_TEST    = 0x30, /*�����֣�����ture or FALSE*/
    CMD_TEST_ALL       = 0x31,
    CMD_BUOY_MIDDLE    = 0x32,
    CMD_TEST_CLIFF_LIGHTTOUCH  = 0x33,
    CMD_TEST_BUMP_DROP         = 0x34,
    CMD_TEST_ALL_IR            = 0x35,
    CMD_TEST_CLIFF_DROP_STASIS = 0x36,
    CMD_TEST_LIGHTTOUCH_BUMP   = 0x37,
    CMD_TEST_WIRELESS          = 0x38,
    CMD_TEST_MOVE_REVERSE      = 0x39,
    CMD_TEST_ALL_CHARGE_MODE   = 0x40,
//    CMD_DISABLE_SENSOR = 0xf0000024,
    CMD_NULL           = 0xf000008F	
}TEST_CMD_E;

/*״̬��������*/
typedef enum
{
    CMD_TIME_INFO      = 0xf0000001, /*��ȡ�����rtcʱ�� tm*/
    CMD_SYS_STATE_INFO = 0xf0000002, /*��ȡ����ϵͳ״̬ sys_state_info*/
    CMD_SYS_SCHEDULE_INFO=0xf0000004, /*��ȡԤԼʱ��� schedule_time_info*/
    CMD_SYS_TEST_INFO    = 0xf0000008,
    CMD_SYS_ADC_INFO     =0xf0000010,  /*��ȡ�������adc��ֵ sys_adc_info*/    
    CMD_GPIO_INFO        = 0xf0000020
}STATE_CMD_E;



typedef enum
{
  UART_CMD_NULL = 0x0000,
  UART_START = 0x0001,
  UART_STOP  = 0x0002,
  UART_FORWARD= 0x0004,
  UART_LEFT   = 0x0009,
  UART_RIGHT   = 0x0011,
  UART_BACK   = 0x0012,
  UART_DOCK  = 0x0020,
  UART_SPOT  = 0x0040,
  UART_WALL_FOLLOW = 0x60,
  UART_MODE1 = 0x0081,
  UART_MODE2 = 0x0082,
  UART_GRID1 = 0x0083,
  UART_MODE4 = 0x0084,
  UART_MODE5 = 0x0085,
  UART_MODE6 = 0x0086,
  UART_VOL_ADD = 0x1001,
  UART_VOL_SUB = 0x1002,
  UART_POWER_ON   = 0x1003,
  UART_POWER_DOWN = 0x1004,
  UART_SOS_ON     = 0x1005,
  UART_SOS_OFF    = 0x1006,
  UART_MPLAY_ON   = 0x1007,
  UART_MPLAY_OFF = 0x1008,
  UART_UV_ON      = 0x1009,
  UART_UV_OFF     = 0x100a,
  UART_SCHEDULE   = 0x100b,
  UART_OEM        = 0x100c,
  UART_GET_PRODUCT_ID = 0x100d,
  UART_GET_SCHDULE_INFO = 0x100e,
  UART_PLAY_MUSIC = 0x100f,
  UART_STOP_MUSIC = 0x1010,
  UART_ENHANCE    = 0X1011,
  UART_QUIT   = 0X1012
}UART_CTR_CMD_E;

#define UART_CMD_MAX 14
#endif

