#ifndef __SYSCALL_SYS_API_H__
#define __SYSCALL_SYS_API_H__

#define EXPORT extern

typedef void (*TaskFunc)(void);
typedef void (*power_func_t)(uint8_t handler_time);


/*������Ŀǰ��ʱû��ʹ��*/
typedef struct {
  power_func_t sleep;
  power_func_t wake;
}power_ctrl_t;

/*ö������˵���� ϵͳ��Ϣ���Ͷ���*/
typedef enum
{
  SYS_SCHEDULE_INFO,/*ϵͳԤԼ��Ϣ*/
  SYS_TIME_INFO,    /*ϵͳʱ����Ϣ*/
  SYS_STATE_INFO,   /*ϵͳ״̬��Ϣ*/
  SYS_SENSOR_INFO,  /*ϵͳcliff��light��adc��Ϣ��Ŀǰ����ֱ��ͨ��adc get�ӿڷ���*/
  SYS_CURRENT_INFO,  /*ϵͳ������Ϣ������bat������� ��ȡ����ֵ��ȥ������ƽ��ֵ�� ĿǰҲ����ͨ��adc get�ӿڷ���˲ʱֵ*/
  SYS_BITMAP_INFO   /*��ȡ��ͼ��Ϣ*/
}INFO_TYPE_E;


/*�궨��˵����  ��ʱ���ж�ע������*/
#define HZ_8K 4  /*ע���timer������ÿ8K Hz������һ��*/
#define HZ_4K 1  /*ע���timer������ÿ4K Hz������һ��*/
#define HZ_64 2  /*ע���timer������ÿ64 Hz������һ��*/
#define HZ_1K 3  /*ע���timer������ÿ1K Hz������һ��*/

/*!!!!!!!!!!!!!!����ĺ����������ж�������ʹ��!!!!!!!!!!!!*/
/****************************************************************
*Function   :  act_ctrl_cmd
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ϵͳ���������Ϊ������
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              pkt_p: ��һ���ṹ�������ο�am_date_base.h ���struct command��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void act_ctrl_cmd( uint8_t *pkt_p );

/****************************************************************
*Function   :  sys_info_get
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡϵͳ��Ϣ
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              type: ��ο� INFO_TYPE_E ע��
*              addr: ������Ǵ��ڻ�ȡ��������Ϣ�Ľṹ���ַ
                  struct sys_state_info
                  struct sys_current_info
                  struct ui_tm
                  struct sys_sensor_adc_info
                  struct schedule_time_info  
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void sys_info_get(INFO_TYPE_E type, long addr);

/****************************************************************
*Function   :  timer_ms
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡϵͳ����ʱ�䣬��λms����ϵͳ�ϵ翪ʼ����
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
EXPORT uint32_t timer_ms(void);

/****************************************************************
*Function   :  timer_elapsed
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ����ϵͳ��ȥ��ʱ��
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              milli_start�� ��ȥ�ο���ʱ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT uint32_t timer_elapsed(uint32_t milli_start);

EXPORT void set_power_callback(power_ctrl_t *func);

/****************************************************************
*Function   :  register_ui_config
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ϵͳע��ui_manager ����Ϣ
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              cfg �� ui_manager_t�ṹ�壬�ṩ��SDK�̼�ʹ�õĺ���
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void register_ui_config(const ui_manager_t *cfg);

/****************************************************************
*Function   :  sys_timer_register
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ϵͳע�� ����HZ �жϴ�����
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              time_hz �� HZ_8K��HZ_4K��HZ_1K��HZ_64, 
*              handler :  ע����жϴ�����
*              pri     :  ���ȼ�
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT int8_t sys_timer_register(u8 time_hz, long handler, u8 pri);

/****************************************************************
*Function   :  timer_task_register
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ϵͳע���жϴ���������С��λ��1ms��
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              task :  ע����жϴ�����
*              interval     :  ���ڼ��������10������10ms����һ�Σ��������
*              periodic     �� 0�� �������У�  1����������
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT int timer_task_register( TaskFunc task, uint32_t interval, uint8_t periodic);//interval 1ms

/****************************************************************
*Function   :  timed_tasks_unregister
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ж��ע���жϴ�����
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              task :  ע����жϴ�����
*Output     :  ��
*Return     :  0�� ok��  1: fail
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT int timed_tasks_unregister( TaskFunc task );
//systime 
/****************************************************************
*Function   :  get_clocktime_seconds
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ϵͳ����ʱ�䣺��ǰ����
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ���� �� ��(0~59)
*Others     :  ϵͳ˯��ʱ�����ʱ���ֹͣ��ʱ
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT s16 get_clocktime_seconds(void);

/****************************************************************
*Function   :  get_clocktime_minutes
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ϵͳ����ʱ�䣺��ǰ��
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ���� �� ��(0~59)
*Others     :  ϵͳ˯��ʱ�����ʱ���ֹͣ��ʱ
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT s16 get_clocktime_minutes(void);//ϵͳ��������ȥ�ķ�����

/****************************************************************
*Function   :  get_clocktime_hours
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ϵͳ����ʱ�䣺��ǰʱ
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ���� �� ʱ(0~23)
*Others     :  ϵͳ˯��ʱ�����ʱ���ֹͣ��ʱ
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT s16 get_clocktime_hours(void);  //ϵͳ��������ȥ��Сʱ��

/****************************************************************
*Function   :  get_total_seconds
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ϵͳ����ʱ�䣺�ۼƵ�������
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ���� �� ʱ(0~0xfffffff)
*Others     :  ϵͳ˯��ʱ�����ʱ���ֹͣ��ʱ
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT u32 get_total_seconds(void);    

/****************************************************************
*Function   :  get_clocktime_days
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ϵͳ����ʱ�䣺�ۼƵ�������
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ���� �� ʱ(0~0xffff)
*Others     :  ϵͳ˯��ʱ�����ʱ���ֹͣ��ʱ
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT s16 get_clocktime_days(void); 

/****************************************************************
*Function   :  get_sum_ticks
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ϵͳ�����ۼƵ���tick��
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ���� �� ʱ(0~0xfffffff)
*Others     :  ϵͳ˯��ʱ�����ʱ���ֹͣ����
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT u32 get_sum_ticks(void);

/****************************************************************
*Function   :  get_adc_chan_val
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡadcͨ����adcֵ
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              adc_chan �� �ο� adc_chan.h ADC_ID_E
*              ��
*Output     :  ��
*Return     :  ���أ�adcֵ
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT U16   get_adc_chan_val(ADC_ID_E adc_chan);

/****************************************************************
*Function   :  get_main_software_version
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡϵͳ�İ汾��Ϣ
*CallBy     :  ����ģʽ��
*Input      :  ����
*              ��
*Output     :  ��
*Return     :  ���أ��汾�ţ�����50
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT U32   get_main_software_version(void);

/****************************************************************
*Function   :  chipid_read
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡchipid ��Ϣ
*CallBy     :  ����ģʽ��
*Input      :  ����
*              ��
*Output     :  ��
*Return     :  ���أ� chipid
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT u32   chipid_read(void);

/****************************************************************
*Function   :  vram_read
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡvram������,vram�������Զ�32 word��
*CallBy     :  
*Input      :  ����
*              offset��ƫ�Ƶ�ַ,wordλ��λ
*              data  ������
*              len   ������ �� wordΪ��λ
*Output     :  ��
*Return     :  ���أ� 0
*Others     :  ����ӿڲ�����wordΪ��λ������vram_read(2,buf,10), ��˼���Ǵ�ƫ�Ƶ�2 ��word��ʼ����ȡ10��word�����ݡ�
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT BOOLEAN vram_read(uint16_t offset , uint32_t *data,uint16_t len);

/****************************************************************
*Function   :  vram_write
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  д���ݵ�vram����ram��������д32 word��
*CallBy     :  
*Input      :  ����
*              offset��ƫ�Ƶ�ַ��wordλ��λ��һ��wordΪ4byte
*              data  ������
*              len   �����ȣ�wordΪ��λ��һ��wordΪ4byte
*Output     :  ��
*Return     :  ���أ� 0
*Others     :  ����ӿڲ�����wordΪ��λ������vram_write(2,buf,10), ��˼���Ǵ�ƫ�Ƶ�2 ��word��ʼ��д��10��word�����ݡ�
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT BOOLEAN vram_write(uint16_t offset , uint32_t *data,uint16_t len);

/****************************************************************
*Function   :  wall_follow_is_running
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �Ƿ����ر�״̬
*CallBy     :  
*Input      :  ����
               ��
*Output     :  ��
*Return     :  ���أ� 0����   1����
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT BOOLEAN wall_follow_is_running(void);

/****************************************************************
*Function   :  forbit_sys_handle_rebound
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ֹ��������ϵͳ����rebound���̣�����bump��cliff�ȵ�,�ο�Rebound ���壩
*CallBy     :  
*Input      :  ����
               rebound �� Rebound����
                en      �� 1: forbit   0��allow
*Output     :  ��
*Return     :  ���أ� 0����   1����
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void forbit_sys_handle_rebound(Rebound rebound, u8 en);


/****************************************************************
*Function   :  robot_xy_dist
*Author     :  lyy    
*Date       :  2017.6.28
*Description:  �������������֮���ֱ�߾���
*CallBy     :  
*Input      :  ����
               AM_xy a�� a������ 
               AM_xy b:  b������
*Output     :  ��
*Return     :  ���أ� ����֮��ľ���
*Others     :  robot_pos_get��ȡ��ǰ�������ϵ
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT am_real robot_xy_dist(AM_xy *a, AM_xy *b);

/****************************************************************
*Function   :  motor_encode_get
*Author     :  lyy    
*Date       :  2017.6.28
*Description:  ��ȡ��������ת����tick��,������������������̱ȡ�
*CallBy     :  
*Input      :  ����
*Output     :  ��
*Return     :  ���أ� motor_encode_t ���ͽṹ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT motor_encode_t *motor_encode_get(void);

/****************************************************************
*Function   :  system_reboot
*Author     :  lyy    
*Date       :  2017.7.7
*Description:  ϵͳ��λ��
*CallBy     :  
*Input      :  ����
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.7.7       v1.0         build this function
******************************************************************/
EXPORT void system_reboot(void);

/****************************************************************
*Function   :  update_dirt_level
*Author     :  lyy    
*Date       :  2017.7.25
*Description:  �����������������ȼ�
*CallBy     :  
*Input      :  0�� ������ 1��������  2��������
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.7.25       v1.0         build this function
******************************************************************/
EXPORT void update_dirt_level(u8 level);

/****************************************************************
*Function   :  local_map_reset
*Author     :  lyy    
*Date       :  2017.8.231
*Description:  ��λ��ͼ��Ϣ
*CallBy     :  
*Input      :  resume = 0�� ����ֵ����
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.8.31       v1.0         build this function
******************************************************************/
EXPORT int local_map_reset(int resume);
#endif
