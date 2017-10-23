#ifndef __SYSCALL_SENSOR_API_H__
#define __SYSCALL_SENSOR_API_H__
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

/****************************************************************
*Function   :  robot_set_going_speed
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ������ɨ���ٶȣ���λmm/s��
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              speed :  Ҫ���õ��ٶ�
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void robot_set_going_speed(uint16_t speed);

/****************************************************************
*Function   :  robot_set_adc_adjust
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ����adc��У׼ֵ
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              val :  У׼ֵ
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void robot_set_adc_adjust(uint16_t val);

/****************************************************************
*Function   :  robot_set_action_funs
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��̬���ø�·������Ƿ����Ĺ���
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              funs :  �ο� �ṹ�� clean_motor_config_t
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void robot_set_action_funs(uint16_t funs);

/****************************************************************
*Function   :  set_motor_vels
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ������������
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              left :   ���ֵ��ٶ�  �� =0�� ֹͣ�� <0 ����  >0��ǰ��  ��λ��mm/s
*              right :  ���ֵ��ٶ�  �� =0�� ֹͣ�� <0 ����  >0��ǰ��  ��λ��mm/s
*              acc   ��  ���ٶ� ��  ��λmm/s
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void set_motor_vels( int16_t left,int16_t right, uint16_t acc);

/****************************************************************
*Function   :  get_motor_speeds
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡ����ʵ���ٶ�
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              left :   ���ֵ��ٶ�  ��λ��ticks/s
*              right :  ���ֵ��ٶ�  ��λ��ticks/s
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void get_motor_speeds( int16_t *left,int16_t *right);

/****************************************************************
*Function   :  get_commanded_speeds
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡ����Ԥ����ٶ�
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              left :   ���ֵ��ٶ�  ��λ��ticks/s
*              right :  ���ֵ��ٶ�  ��λ��ticks/s
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void get_commanded_speeds( int16_t *left,int16_t *right);

/****************************************************************
*Function   :  get_cliff_state
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡcliff��״̬
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ���� CliffState����
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT CliffState get_cliff_state(void);

/****************************************************************
*Function   :  get_bump_state
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡbump��״̬
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ���� BumpState ����
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT BumpState  get_bump_state(void);

/****************************************************************
*Function   :  set_slip_high_throd
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �ָ�slip ��ֵΪ����ֵ
*CallBy     :  �κεط����ж������ĳ���
*Input      :  val�� ��ֵ ��Խ��slip���Խ��������
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void set_slip_normal(void);

/****************************************************************
*Function   :  set_slip_high_throd
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ����slip���жϵ���ֵ
*CallBy     :  �κεط����ж������ĳ���
*Input      :  val�� ��ֵ ��Խ��slip���Խ��������
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void set_slip_high_throd(float val);

/****************************************************************
*Function   :  set_stasis_normal
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �ָ�stasis ��ֵΪ����ֵ����������ȸ�
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void set_stasis_normal(void);

/****************************************************************
*Function   :  set_stasis_high_throd
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ����stasis ��ֵΪ�ߣ����������ȵ�
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void set_stasis_high_throd(void);

/****************************************************************
*Function   :  get_gyro_angle
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡ�����ǽǶ���Ϣ
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ���ؾ��ǽǶȶ�����Ϣ����-90,90
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT int  get_gyro_angle(void);

/****************************************************************
*Function   :  get_gyro_radian
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡ�����ǻ�����Ϣ
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ���ؾ��ǻ�����Ϣ����-PI,PI
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT float get_gyro_radian(void);

/****************************************************************
*Function   :  set_charging_enable
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �Ƿ�ʹ�ܳ�繦��
*CallBy     :  �κεط����ж������ĳ���
*Input      :  on�� 1��ʹ��   0����ʹ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void  set_charging_enable(U16 on);

/****************************************************************
*Function   :  v_ref_init
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ����ʼ��
*CallBy     :  ��ʼʹ�ܳ��ǰ����
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void  v_ref_init(void);

/****************************************************************
*Function   :  set_charge_current
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡ��ǰʵ�ʵĳ����� (��λ��mA)
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*Output     :  ��
*Return     :  ���� ʵ�ʵĳ�����ֵ (mA)
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void  set_charge_current(u16 val);

/****************************************************************
*Function   :  set_adaptor_output_voltage
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ���ó���������ĵ�ѹֵ (��λ��V)
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              val��  ����22V�� 19V��24V
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void  set_adaptor_output_voltage(u16 val);

/****************************************************************
*Function   :  set_charge_voltage
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ���ó��ĵ�ѹֵ (��λ��adcֵ)
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              adc_val:  ��س�������ڷ���adcֵ������Ŀǰ16.8V ��Ӧʱ3628
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void  set_charge_voltage(u16 adc_val);

/****************************************************************
*Function   :  charge_maybe_full
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��س������жϣ����ܳ�����
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT U8    charge_maybe_full(void);
/****************************************************************
*Function   :  gyro_is_ok
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �ж��������Ƿ����
*CallBy     :  ����ģʽ��
*Input      :  ����
*              ��
*Output     :  ��
*Return     :  ���أ� 1��ok��  0��err
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT U8    gyro_is_ok(void);

/****************************************************************
*Function   :  robot_set_slowdown_div
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ���ü��ٷ�Ƶ�ȣ�lt ����ǰ���ϰ���ʱ���ٱȣ����ڵ�ǰ�ٶ�*div��
*CallBy     :  
*Input      :  ����
*              div : ��Ƶ��
*              ��
*Output     :  ��
*Return     :  ���أ� chipid
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void robot_set_slowdown_div(float div);

#endif
