#ifndef IRQ_SYSCALL_H
#define IRQ_SYSCALL_H
#include "stdint.h"

/****************************************************************
*Function   :  gpio_request
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ϵͳ����һ��IO��
*CallBy     :  �κεط�
*Input      :  gpio: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*Output     :  ��
*Return     :  0���ɹ�  �� -1:ʧ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int32_t (*io_request) (uint32_t gpio);

/****************************************************************
*Function   :  gpio_free
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ϵͳ�ͷ�һ��IO��
*CallBy     :  �κεط�
*Input      :  gpio: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef void (*io_free) (uint32_t gpio); 

/****************************************************************
*Function   :  gpio_request_array
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ϵͳ�����������IO��
*CallBy     :  �κεط�
*Input      :  ����
*              start: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*              num  : ���� IO�ڵĸ���
*              flags: am_gpio.h �� GPIO_F_xxx�ĺ�
*Output     :  ��
*Return     :  0���ɹ�  �� -1:ʧ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int32_t  (*io_request_array) (int32_t start, int32_t num, int32_t flags);

/****************************************************************
*Function   :  gpio_free_array
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ϵͳ�ͷ��������IO��
*CallBy     :  �κεط�
*Input      :  ����
*              start: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*              num  : ���� IO�ڵĸ���
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef void (*io_free_array)(int32_t start, int32_t num) ;

/****************************************************************
*Function   :  gpio_set_value
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��GPIO���һ����ƽ
*CallBy     :  �κεط�
*Input      :  ����
*              start: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*              value: 0: �͵�ƽ���    1�� �ߵ�ƽ���
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef void (*io_set_value)(uint32_t gpio,int32_t value) ;

/****************************************************************
*Function   :  gpio_get_value
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡһ��GPIO�������ƽ
*CallBy     :  �κεط�
*Input      :  ����
*              gpio: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*Output     :  ��
*Return     :  0�� �͵�ƽ    1:�ߵ�ƽ
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int32_t (*io_get_value)(uint32_t gpio);

/****************************************************************
*Function   :  gpio_to_exti
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��һ��GPIO���ó��ⲿ�ж�
*CallBy     :  �κεط�
*Input      :  ����
*              gpio: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef void (*io_to_exti) (uint32_t gpio);

/****************************************************************
*Function   :  gpio_set_afio
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��һ��GPIO���ö๦�ܸ���ģʽ
*CallBy     :  �κεط�
*Input      :  ����
*              gpio: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*              afio: �ο�am_gpio.h  AFIO_MODE_Enum
*Output     :  ��
*Return     :  0�� �ɹ�    -1��ʧ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int32_t  (*set_afio) (uint32_t gpio, uint32_t afio);

/****************************************************************
*Function   :  gpio_request_one
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ϵͳ����һ��GPIO�ڣ�Ȼ�����ָ��flags
*CallBy     :  �κεط�
*Input      :  ����
*              gpio: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*              flags: �ο�am_gpio.h �� GPIO_F_xxx�ĺ�
*Output     :  ��
*Return     :  0�� �ɹ�    -1��ʧ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int32_t  (*io_request_one) (uint32_t gpio, unsigned long flags) ;

/****************************************************************
*Function   :  remote_val
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ����ʹ��
*CallBy     :  �κεط�
*Input      :  ����
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef void (*remote_val) (uint8_t index, uint8_t val);

/****************************************************************
*Function   :  get_adc
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡadc���������ĵ�ַ��adc�ܹ���32��32bit������������
*CallBy     :  �κεط�
*Input      :  ��
*Output     :  ��
*Return     :  adc����ͨ�����������׵�ַ
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef u32* (*adc_val) (void);

/****************************************************************
*Function   :  robot_lt_update
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ϵͳע��light touch�Ľ����Ϣ
*CallBy     :  �κεط�
*Input      :  ����
*              val: signal_result[]
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef void (*robot_lt_update_t) (uint8_t *val);

/****************************************************************
*Function   :  printk
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ӡ����
*CallBy     :  �κεط�
*Input      :  ����
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int  (*printf_t)(const char *fmt, ...);

/****************************************************************
*Function   :  write
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ���豸������д������
*CallBy     :  �κεط�
*Input      :  ����
*              fd : �豸�����
*              buf: Ҫд������ 
*              len: Ҫд�����ݵĳ���
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int  (*write_t) (int fd, uint8_t *buf, uint16_t len);

/****************************************************************
*Function   :  read
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ���豸�����ж�����
*CallBy     :  �κεط�
*Input      :  ����
*              fd : �豸�����
*              buf: ��Ŷ�������� 
*              len: Ҫ��������ݵĳ���
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int  (*read_t)  (int fd, uint8_t *buf, uint16_t len);

/****************************************************************
*Function   :  ioctl
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �豸���ƺ���
*CallBy     :  �κεط�
*Input      :  ����
*              fd : �豸�����
*              cmd: �����֣��ο�am_device.h �����豸������
*              arg: ��Ӧ�����ֵĲ���
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int  (*ioctl_t) (int fd, int cmd,void *arg);

/****************************************************************
*Function   :  runing_mode
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡϵͳ������״̬
*CallBy     :  �κεط�
*Input      :  ��
*Output     :  ��
*Return     :  ϵͳ��״̬,�ο� ROBOT_STATE_E 
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef U8   (*runing_mode_t) (void);

typedef void (*robot_pos_get_t)(AM_Pose *out);
  
/****************************************************************
*Function   :  open
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ���豸����
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              devName : �豸����
*              devId   : �豸id 
*Output     :  ��
*Return     :  0���ɹ�    -1��ʧ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int (*open_t)(int devName, int devId);


/****************************************************************
*Function   :  close
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �ر��豸
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*              fd : �豸�����
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int (*close_t)(int fd);


/*������������*/
typedef int  (*null_fun_t)(int);
/*���溯���������κ�ʱ��ʹ�ã��ṩ���º�����Ҫ����Ϊ�ж��ﲻ��ʹ��ϵͳ���ú��������ж�������Ҫ�������¹��ܵĺ���*/
typedef struct
{
  io_request        gpio_request;
  io_free           gpio_free;
  io_request_array  gpio_request_array;
  io_free_array     gpio_free_array;
  io_set_value      gpio_set_value;
  io_get_value      gpio_get_value;
  io_to_exti        gpio_to_exti;
  set_afio          gpio_set_afio;
  io_request_one    gpio_request_one;
  remote_val        set_remote_val;
  adc_val           get_adc;
  robot_lt_update_t robot_lt_update;
  printf_t          printk; 
  write_t           write;
  read_t            read;
  ioctl_t           ioctl; 
  runing_mode_t     runing_mode;
  robot_pos_get_t   robot_pos_get;
  open_t            open;/*������չ��*/
  close_t           close;/*������չ��*/
  null_fun_t        rev4;/*������չ��*/  
}syscall_fun;

#endif