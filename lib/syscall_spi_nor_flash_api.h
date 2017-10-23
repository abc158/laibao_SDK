#ifndef __SYSCALL_SPI_NOR_FLASH_API_H__
#define __SYSCALL_SPI_NOR_FLASH_API_H__
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
*Function   :  spi_nor_flash_read
*Author     :  luenwan    
*Date       :  2017.7.5
*Description:  ��ȡnor flash�������
*CallBy     :  �κεط��������ж���������
*Input      :  addr����ʼ��ַ(��24λ��Ч) buf�����ݻ����ַ len: ��ȡ���ݵĳ���
*Output     :  ��
*Return     :  > 0���ɹ� <= 0��ʧ�� 
*Others     :  ��
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    luenwan            17.7.5       v1.0         build this function
******************************************************************/ 
EXPORT int spi_nor_flash_read(uint32_t addr, uint8_t* buf, uint32_t len);

/****************************************************************
*Function   :  spi_nor_flash_write
*Author     :  luenwan    
*Date       :  2017.7.5
*Description:  ������д��nor flash
*CallBy     :  �κεط��������ж���������
*Input      :  addr����ʼĿ���ַ(��24λ��Ч) buf�����ݴ洢��ַ len: ���ݵĳ���
*Output     :  ��
*Return     :  > 0���ɹ� <= 0��ʧ�� 
*Others     :  ��
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    luenwan            17.7.5       v1.0         build this function
******************************************************************/ 
EXPORT int spi_nor_flash_write(uint32_t addr, uint8_t* buf, uint32_t len);

/****************************************************************
*Function   :  spi_nor_flash_erase
*Author     :  luenwan    
*Date       :  2017.7.5
*Description:  ����norflash��һ����������ݡ�
*CallBy     :  �κεط��������ж���������
*Input      :  addr����ʼĿ���ַ(��24λ��Ч) len: ���ݵĳ���
*Output     :  ��
*Return     :  >= 0���ɹ� < 0��ʧ�� 
*Others     :  ������С��λΪ4096��addr,len����Ϊ4096��������(��4096����������¿��ܻ����ǰ������4096byte�ռ�)
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    luenwan            17.7.5       v1.0         build this function
******************************************************************/ 
EXPORT int spi_nor_flash_erase(uint32_t addr, uint32_t len);


#endif
