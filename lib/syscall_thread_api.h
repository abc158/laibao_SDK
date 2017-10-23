#ifndef __SYSCALL_THREAD_API_H__
#define __SYSCALL_THREAD_API_H__
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

/*!!!!!!!!!!!!!!����ĺ����������ж�������ʹ��!!!!!!!!!!!!*/

/****************************************************************
*Function   :  lock_irq
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ���ж�
*CallBy     :  �κεط��������ж���������
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  һ�㲻���������ã���Ϊ��Ӱ��ϵͳ��������
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/ 
EXPORT uint16_t lock_irq(void);

/****************************************************************
*Function   :  unlock_irq
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ������ж�
*CallBy     :  �κεط��������ж���������
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  һ�㲻���������ã���Ϊ��Ӱ��ϵͳ��������
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/ 
EXPORT uint16_t unlock_irq(void);

/****************************************************************
*Function   :  pthread_create
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �����߳�
*CallBy     :  �κεط��������ж���������
*Input      :  ����
*             task��    �̺߳���
*             p_arg:    �̺߳�������ڲ���
*             ptos :    �߳�ջ��
*             prio ��   �߳����ȼ�
*             pbos :    �߳�ջ��
*             stk_size: �߳�ջ���
*Output     :  ��
*Return     :  0�� �ɹ���  ��0�� ʧ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT uint8_t pthread_create (void (*task)(void *p_arg),u32 *ptos,u8  prio,u32  *pbos, u32 stk_size);

/****************************************************************
*Function   :  pthread_delete
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ǿ�������߳�
*CallBy     :  �κεط��������ж���������
*Input      :  ����
*             prio ��   �߳����ȼ�
*Output     :  ��
*Return     :  0�� �ɹ���  ��0�� ʧ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT uint8_t pthread_delete(uint8_t prio);

/****************************************************************
*Function   :  pthread_change_prio
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �ı��̵߳����ȼ�
*CallBy     :  �κεط��������ж���������
*Input      :  ����
*             oldprio ��   �̵߳�ǰ�����ȼ�
*             newprio ��   �߳��µ����ȼ�
*Output     :  ��
*Return     :  0�� �ɹ���  ��0�� ʧ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT uint8_t pthread_change_prio (uint8_t  oldprio,uint8_t  newprio);

/****************************************************************
*Function   :  pthread_resume
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �ָ̻߳�
*CallBy     :  �κεط��������ж���������
*Input      :  ����
*             prio ��   �߳����ȼ�
*Output     :  ��
*Return     :  0�� �ɹ���  ��0�� ʧ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT uint8_t pthread_resume(uint8_t prio);

/****************************************************************
*Function   :  pthread_suspend
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �̹߳���
*CallBy     :  �κεط��������ж���������
*Input      :  ����
*             prio ��   �߳����ȼ�
*Output     :  ��
*Return     :  0�� �ɹ���  ��0�� ʧ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT uint8_t pthread_suspend(uint8_t prio);

/****************************************************************
*Function   :  sleep
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �߳�˯��
*CallBy     :  �κεط��������ж���������
*Input      :  ����
*             tick ��   �߳�˯�ߵ�tick����һ��tick 10ms
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void sleep (uint32_t ticks);

/****************************************************************
*Function   :  pthread_wakeup
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �߳�˯�߻��ѣ����һ���߳�sleep�ˣ�������������ѣ�
*CallBy     :  �κεط��������ж���������
*Input      :  ����
*             prio ��   �߳����ȼ�
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT uint8_t pthread_wakeup (uint8_t prio) ;

/****************************************************************
*Function   :  pthread_sched_lock
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ס�̵߳���
*CallBy     :  �κεط��������ж���������
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void pthread_sched_lock(void);

/****************************************************************
*Function   :  pthread_sched_unlock
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ���̵߳���
*CallBy     :  �κεط��������ж���������
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void pthread_sched_unlock(void);

#endif
