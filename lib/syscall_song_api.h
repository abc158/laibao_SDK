#ifndef __SYSCALL_SONG_API_H__
#define __SYSCALL_SONG_API_H__
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
*Function   :  voiceplayer_init
*Author     :  luenwan    
*Date       :  2017.4.20
*Description:  ��ʼ������������
*CallBy     :  �κεط��������ж���������
*Input      :  ��
*Output     :  ��
*Return     :  0���ɹ� -1��ʧ�� 
*Others     :  һ�㽨����ϵͳ��ʼ��ʱ����
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    luenwan            17.6.9       v1.0         build this function
******************************************************************/ 
EXPORT int voiceplayer_init(void);

/****************************************************************
*Function   :  voiceplayer_play_index
*Author     :  luenwan    
*Date       :  2017.4.20
*Description:  ʹ����������������������Ŀ
*CallBy     :  �κεط��������ж���������
*Input      :  
*             index:             ������Ŀ���
*             force_replace :    �Ƿ��滻��ǰ���ڲ��ŵ�����
*Output     :  ��
*Return     :  0���ɹ� -1��ʧ�� 
*Others     :  һ�㽨������������������״̬�µ��ã���������������
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    luenwan            17.6.9       v1.0         build this function
******************************************************************/ 
EXPORT int voiceplayer_play_index(uint16_t index, uint8_t force_replace);

/****************************************************************
*Function   :  voiceplayer_play_stream
*Author     :  luenwan    
*Date       :  2017.4.20
*Description:  ʹ����������������������Ŀ
*CallBy     :  �κεط��������ж���������
*Input      :  
*             stream:   ����������ROM�ռ�ĵ�ַ
*             size :    �������ݵĴ�С����byteΪ��λ��
*Output     :  ��
*Return     :  0���ɹ� -1��ʧ�� 
*Others     :  һ�㽨������������������״̬�µ��ã�����������������
*              �ò���ģʽĬ�����ȼ��ϸߣ����滻��ǰ���ڲ��ŵ�������
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    luenwan            17.7.25       v1.0         build this function
******************************************************************/ 
EXPORT int voiceplayer_play_stream(const uint8_t* stream, uint32_t size);

/****************************************************************
*Function   :  voiceplayer_quit_playing
*Author     :  luenwan    
*Date       :  2017.4.20
*Description:  ֹͣ��������
*CallBy     :  �κεط��������ж���������
*Input      :  ��
*Output     :  ��
*Return     :  0���ɹ� -1��ʧ�� 
*Others     :  һ�㽨������������������������ʱ����
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    luenwan            17.6.9       v1.0         build this function
******************************************************************/ 
EXPORT int voiceplayer_quit_playing(void);

/****************************************************************
*Function   :  voiceplayer_is_playing
*Author     :  luenwan    
*Date       :  2017.4.20
*Description:  �鿴��ǰ�Ƿ����ڲ�������
*CallBy     :  �κεط��������ж���������
*Input      :  ��
*Output     :  ��
*Return     :  1�����ڲ������� 0����ǰ�������������ڿ���״̬
*Others     :  ��
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    luenwan            17.6.9       v1.0         build this function
******************************************************************/ 
EXPORT int voiceplayer_is_playing(void);

/****************************************************************
*Function   :  voiceplayer_get_file_version
*Author     :  luenwan    
*Date       :  2017.4.20
*Description:  �鿴��ǰ�����ļ��İ汾��
*CallBy     :  �κεط��������ж���������
*Input      :  ��
*Output     :  ��
*Return     :  ����ֵ�������������ļ�ʱ��������һ��
*Others     :  һ�㽨�����������������绯֮�����
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    luenwan            17.6.9       v1.0         build this function
******************************************************************/ 
EXPORT int voiceplayer_get_file_version(void);

/****************************************************************
*Function   :  voiceplayer_get_file_count
*Author     :  luenwan    
*Date       :  2017.4.20
*Description:  �鿴��ǰ������Ŀ�ĸ���
*CallBy     :  �κεط��������ж���������
*Input      :  ��
*Output     :  ��
*Return     :  ����ֵ����0
*Others     :  һ�㽨�����������������绯֮�����
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    luenwan            17.6.9       v1.0         build this function
******************************************************************/ 
EXPORT int voiceplayer_get_file_count(void);

#endif
