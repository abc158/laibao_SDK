#ifndef _DEVICE_H_
#define _DEVICE_H_

//ϵͳ��Ӳ��ģ��,ͨ���±��豸������,�豸�ľ͵������ļ��豸���
typedef enum
{
  DEV_UART = 0, /*���� �豸��*/
  DEV_BFTM ,    /*base timer �豸��,Ŀǰ���޿��Ÿ�appʹ�ã���Ϊϵͳʹ������*/
  DEV_GPTM ,    /*ͨ�� timer �豸��*/
  DEV_IR   ,    /*IC �Դ���IR �豸��*/
  DEV_EXTI ,    /*�ⲿ�ж� �豸��*/
  DEV_RTC  ,    /*rtc �豸�ţ���ʱ��ͨ���豸���ʣ�ֱ��ͨ��api�ӿ�*/
  DEV_WDG  ,    /*watchdog �豸��*/
  DEV_BAKUP,    /*����ģ�� �豸�ţ�2�������32λ���ã��رյ�Դ���غ�������Ȼ��������������������һ����־�ȵ�*/
  DEV_SPI  ,    /*SPI �豸��*/
  DEV_I2C  ,    /*i2c �豸��*/
  DEV_IPC  ,    /*ipc �豸�ţ� ��ʱû��ʹ��*/
  DEV_MCTM ,    /*�๦�� timer �豸��*/
}DEVICE_E;

//IPCͨ�ŵ�����
typedef enum
{
  IPC_USER  =1,
  IPC_KERNEL=2,
}IPC_TYPE_E;

/* IC �ܹ����֧��4·�Ĵ��ڣ� ÿ������id��*/
typedef enum
{
        UART_NULL  = 0,
	USART_ID0 = 1,
	USART_ID1 = 2,
	UART_ID0  = 3,
	UART_ID1  = 4
}UART_ID_E;

//����ģ���������
#define SYS_TM_START    0x10000  
#define SYS_UART_START  0x20000
#define SYS_WDT_START   0x30000
#define SYS_SPI_START   0x40000
#define SYS_RTC_START   0x50000
#define SYS_BAKUP_START 0x60000
#define SYS_I2C_START   0x70000
#define SYS_IPC_START   0x80000
#define SYS_EXTI_START  0x90000
#define SYS_IR_START    0xa0000

//rtc mod
typedef enum
{
	RTC_INIT      = SYS_RTC_START + 0, 
	RTC_EXIT      = SYS_RTC_START + 1
}SYS_RTC_CMD_E;

//spi mod
typedef enum
{
	SPI_INIT      = SYS_SPI_START + 0,   /*��ʼ��spi��ioctl������Ϊinput�� Ϊ UserSpiInit_t ����*/
	SPI_EXIT      = SYS_SPI_START + 1,   /*�ر�spi�� ioctl ���� û��ʹ�ò���*/
	SPI_CS        = SYS_SPI_START + 2,   /*����spi cs�ߵĸߵͣ�ioctl������Ϊinput��ΪU8����*/
	SPI_FIFO_RX_STATE= SYS_SPI_START + 3,/*��ȡspi rx Ӳ�����ݻ������Ƿ������ݣ�ioctl������Ϊoutput��ΪU8����*/
        SPI_FIFO_TX_STATE= SYS_SPI_START + 9,/*��ȡspi tx Ӳ�����ݻ������Ƿ�����ioctl������Ϊoutput��ΪU8����*/
	SPI_BUSY      = SYS_SPI_START + 4,   /*��ȡspi�����Ƿ�busy״̬��ioctl������û��ʹ�ò���*/
	SPI_START     = SYS_SPI_START + 5,   /*����spi ����*/
	SPI_STOP      = SYS_SPI_START + 6,   /*ֹͣspi ����*/
        SPI_SET_GYRO_ID = SYS_SPI_START + 7, /*���������ǵ�SPI ID�ţ�ioctl������Ϊinput��Ϊint��*/
        SPI_SET_NOR_ID  = SYS_SPI_START + 8, /*����nor ��SPI ID�ţ�ioctl������Ϊinput��Ϊint��*/       
	
}SYS_SPI_CMD_E;

//wdg mod
typedef enum
{
	WDT_INIT      = SYS_WDT_START + 0,/*wdt,��ʼ����Ŀǰ��ʱ����Ҫ*/
	WDT_START     = SYS_WDT_START + 1,/*wdt, ʹ��watchdog���ܣ� ioctl������Ϊinput��Ϊu16���ͣ��ǵ�λ��ms*/
	WDT_STOP      = SYS_WDT_START + 2,/*ֹͣwdt�� ioctl ���� û��ʹ�ò���*/
	WDT_FEED      = SYS_WDT_START + 3,/*ι����    ioctl ���� û��ʹ�ò���*/
}SYS_WDT_CMD_E;

//uart mod
typedef enum
{
	UART_INIT      = SYS_UART_START + 0,  /*uart��ʼ����ioctl������Ϊinput��ΪUserUartInit_t����*/
	UART_EXIT      = SYS_UART_START + 1,  /*�ر�uart*/
	UART_FIFO_FULL = SYS_UART_START + 2,  /*uart �ж�Ӳ���������Ƿ�����ioctl������Ϊinput��ΪU8����*/
	UART_FIFO_EMPTY= SYS_UART_START + 3,  /*uart �ж�Ӳ���������Ƿ�գ�ioctl������Ϊinput��ΪU8����*/
        UART_TRIG_TX   = SYS_UART_START + 4,  /*uart ����һ��tx�ж�*/
        UART_SET_CONSOLE = SYS_UART_START + 5,/*uart ���ô�ӡ�ڿ���̨��uart id��*/
        UART_GET_CONSOLE = SYS_UART_START + 6,/*uart ��ȡ��ӡ�ڿ���̨��uart id��*/
}SYS_UART_CMD_E;

//tm mod
typedef enum
{
	GPTM_CHAN0_SET_DUTY  = SYS_TM_START + 1, /*ͨ��timer�� 0ͨ������pwmռ�ձȣ�ioctl����Ϊinput��Ϊuint16_t����*/
	GPTM_CHAN0_GET_DUTY  = SYS_TM_START + 2, /*ͨ��timer�� 0ͨ����ȡpwmռ�ձȣ�ioctl����Ϊinput��Ϊuint16_t����*/
	GPTM_CHAN1_SET_DUTY  = SYS_TM_START + 9, /*ͨ��timer�� 1ͨ������pwmռ�ձȣ�ioctl����Ϊinput��Ϊuint16_t����*/
	GPTM_CHAN1_GET_DUTY  = SYS_TM_START + 10,/*ͨ��timer�� 1ͨ����ȡpwmռ�ձȣ�ioctl����Ϊinput��Ϊuint16_t����*/	
	GPTM_CHAN2_SET_DUTY  = SYS_TM_START + 11,/*ͨ��timer�� 2ͨ������pwmռ�ձȣ�ioctl����Ϊinput��Ϊuint16_t����*/
	GPTM_CHAN2_GET_DUTY  = SYS_TM_START + 12,/*ͨ��timer�� 2ͨ����ȡpwmռ�ձȣ�ioctl����Ϊinput��Ϊuint16_t����*/
	GPTM_CHAN3_SET_DUTY  = SYS_TM_START + 13,/*ͨ��timer�� 3ͨ������pwmռ�ձȣ�ioctl����Ϊinput��Ϊuint16_t����*/
	GPTM_CHAN3_GET_DUTY  = SYS_TM_START + 14,/*ͨ��timer�� 3ͨ����ȡpwmռ�ձȣ�ioctl����Ϊinput��Ϊuint16_t����*/
        
	MCTM_CHAN0_SET_DUTY  = SYS_TM_START + 15,/*mc timer�� 0ͨ������pwmռ�ձȣ�ioctl����Ϊinput��Ϊuint16_t����*/
	MCTM_CHAN0_GET_DUTY  = SYS_TM_START + 16,/*mc timer�� 0ͨ����ȡpwmռ�ձȣ�ioctl����Ϊinput��Ϊuint16_t����*/
	MCTM_CHAN1_SET_DUTY  = SYS_TM_START + 17,/*mc timer�� 1ͨ������pwmռ�ձȣ�ioctl����Ϊinput��Ϊuint16_t����*/
	MCTM_CHAN1_GET_DUTY  = SYS_TM_START + 18,/*mc timer�� 1ͨ����ȡpwmռ�ձȣ�ioctl����Ϊinput��Ϊuint16_t����*/	
	MCTM_CHAN2_SET_DUTY  = SYS_TM_START + 19,/*mc timer�� 2ͨ������pwmռ�ձȣ�ioctl����Ϊinput��Ϊuint16_t����*/
	MCTM_CHAN2_GET_DUTY  = SYS_TM_START + 20,/*mc timer�� 2ͨ����ȡpwmռ�ձȣ�ioctl����Ϊinput��Ϊuint16_t����*/
	MCTM_CHAN3_SET_DUTY  = SYS_TM_START + 21,/*mc timer�� 3ͨ������pwmռ�ձȣ�ioctl����Ϊinput��Ϊuint16_t����*/
	MCTM_CHAN3_GET_DUTY  = SYS_TM_START + 22,/*mc timer�� 3ͨ����ȡpwmռ�ձȣ�ioctl����Ϊinput��Ϊuint16_t����*/        
        
	TM_BFTM_INIT = SYS_TM_START + 3,/*base Ŀǰ��ʱû�и�app����ʹ��*/
	TM_BFTM_EXIT = SYS_TM_START + 4,/*base Ŀǰ��ʱû�и�app����ʹ��*/
        
	TM_GPTM_BASE_INIT = SYS_TM_START + 5,     /*ͨ�� timer�� ��ʼ����ioctl����Ϊinput��Ϊ GPTM_TimeBaseInitTypeDef ����*/
	TM_GPTM_OUT_MODE_INIT = SYS_TM_START + 6, /*ͨ�� timer�� ��ʼ����ioctl����Ϊinput��Ϊ UserOutInit_t ����*/
	TM_GPTM_IN_MODE_INIT  = SYS_TM_START + 7, /*ͨ�� timer�� ��ʼ����ioctl����Ϊinput��Ϊ UserCapInit_t ����*/
        TM_GPTM_EXIT = SYS_TM_START + 8,          /*ͨ�� timer�� ��ʼ����ioctl������û��ʹ��*/    
        
	TM_MCTM_BASE_INIT = SYS_TM_START + 23,    /*mc timer�� ��ʼ����ioctl����Ϊinput��Ϊ MCTM_TimeBaseInitTypeDef ����*/
	TM_MCTM_OUT_MODE_INIT = SYS_TM_START + 25,/*mc timer�� ��ʼ����ioctl����Ϊinput��Ϊ MUserOutInit_t ����*/
	TM_MCTM_IN_MODE_INIT  = SYS_TM_START + 26,/*mc timer�� ��ʼ����ioctl����Ϊinput��Ϊ MUserCapInit_t ����*/
        TM_MCTM_EXIT = SYS_TM_START + 27,         /*mc timer�� ��ʼ����ioctl������û��ʹ��*/     
        
}SYS_TM_CMD_E;

typedef enum
{
  BAKUP_INIT   = SYS_BAKUP_START + 0,
  BAKUP_EXIT   = SYS_BAKUP_START + 1,
}SYS_BAKUP_CMD_E;

typedef enum
{
  EXTI_INIT   = SYS_EXTI_START + 0,/*�ⲿ�жϳ�ʼ��, ioctl ����Ϊinput��Ϊ exti_init_t ����*/
  EXTI_EXIT   = SYS_EXTI_START + 1,/*�ⲿ�ж�quit*/
}SYS_EXTI_CMD_E;

typedef enum
{
	IPC_INIT       = SYS_IPC_START + 0,
	IPC_EXIT       = SYS_IPC_START + 1,
	IPC_FIFO_EMPTY = SYS_IPC_START + 2,
}SYS_IPC_CMD_E;

#endif