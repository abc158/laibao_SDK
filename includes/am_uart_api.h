#ifndef _UART_INTERFACE_H_
#define _UART_INTERFACE_H_
#include <stdint.h>

//uart������ʼ�����ݽṹ
typedef struct
{
  uint32_t USART_BaudRate;
  uint16_t USART_WordLength;
  uint16_t USART_StopBits;
  uint16_t USART_Parity;
  uint16_t USART_Mode;
  uint16_t USART_LSB_MSB;
}USART_InitTypeDef_t;

//uart dma����ʱ�ĳ�ʼ���ṹ
typedef struct
{
   uint32_t DataAddr; //�ڴ��ַ
   uint32_t DataLen;  //��Ҫ���˵������ܳ���
	 uint32_t DataSize; //
	 uint32_t AdrMod;   //��ַģʽ
	 uint32_t Prio;     //���ȼ�
}USART_DMATypeDef;

//uart ����ģʽ��ʼ��
typedef struct
{
	uint8_t  RxTxMode;//�����жϻ���dmaģʽ	
	uint8_t  irqMode; //irq ģʽ
	uint8_t  DmaMode; //dma ģʽ
  uint8_t  RxTxEna; //����rx, tx en
}USART_ModeTypeDef;

//uart��ʼ���ṹ�塣
typedef struct
{
  USART_InitTypeDef_t    UartInit;
  USART_ModeTypeDef    UartMode;
}UserUartInit_t;


//uart ����ģʽ
typedef enum
{
  CPU_MODE = 0,
  DMA_MODE = 1,
  IRQ_MODE = 2,
}UART_WORD_MODE_E;

//UART DMA����ʱ,֧�ֵ�ģʽ
typedef enum
{
	DMA_RXTX_MODE= 0,
	DMA_RX_MODE  = 1,
	DMA_TX_MODE  = 2,
}DMA_TRAN_MODE_E;

//UART CPU ����ʱ,֧�ֵ�ģʽ
typedef enum
{
	UART_TX = 1,
	UART_RX = 2,
	UART_RXTX=3
}UART_TRAN_MODE_E;


//ע���жϺ�����ԭ��
typedef int (*uart_fun_ptr)(int,int);
typedef struct
{ 
   uart_fun_ptr fun_handle;
}uart_irq_handler_t;


/* USART Word Length ---------------------------------------------------------------------------------------*/
#define USART_WORDLENGTH_7B                    ((uint32_t)0x00000000)
#define USART_WORDLENGTH_8B                    ((uint32_t)0x00000001)
#define USART_WORDLENGTH_9B                    ((uint32_t)0x00000002)

/* USART Stop Bits -----------------------------------------------------------------------------------------*/

#define USART_STOPBITS_1                       ((uint32_t)0x00000000)
#define USART_STOPBITS_2                       ((uint32_t)0x00000004)

/* USART Parity--------------------------------------------------------------------------------------------*/
#define USART_PARITY_NO                        ((uint32_t)0x00000000)
#define USART_PARITY_EVEN                      ((uint32_t)0x00000018)
#define USART_PARITY_ODD                       ((uint32_t)0x00000008)
                                             
#define USART_MODE_NORMAL                      ((uint32_t)0x00000000)
#define USART_MODE_IRDA                        ((uint32_t)0x00000001)
#define USART_MODE_RS485                       ((uint32_t)0x00000002)
#define USART_MODE_SYNCHRONOUS                 ((uint32_t)0x00000003)


#define USART_LSB_FIRST                        ((uint32_t)0x00000000)
#define USART_MSB_FIRST                        ((uint32_t)0x00000004)


/* USART Rx FIFO Interrupt Trigger Level--------------------------------------------------------------------*/
#define USART_RFITL_01                     ((uint32_t)0x00000000)
#define USART_RFITL_04                     ((uint32_t)0x00000040)
#define USART_RFITL_08                     ((uint32_t)0x00000080)
#define USART_RFITL_14                     ((uint32_t)0x000000C0)
                                              
/* USART Tx FIFO Interrupt Trigger Level------------------------------------------------------------------*/
#define USART_TFITL_00                     ((uint32_t)0x00000000)
#define USART_TFITL_02                     ((uint32_t)0x00000010)
#define USART_TFITL_04                     ((uint32_t)0x00000020)
#define USART_TFITL_08                     ((uint32_t)0x00000030)

/* USART Interrupt definition ------------------------------------------------------------------------------*/
#define USART_IID_RLS                          ((uint8_t)0x06)
#define USART_IID_RDA                          ((uint8_t)0x04)
#define USART_IID_CTI                          ((uint8_t)0x0C)
#define USART_IID_THRE                         ((uint8_t)0x02)
#define USART_IID_MS                           ((uint8_t)0x00)
#define USART_IID_NON                          ((uint8_t)0x01)

#define USART_IER_MSIE                         ((uint32_t)0x00000008)
#define USART_IER_RLSIE                        ((uint32_t)0x00000004)
#define USART_IER_THREIE                       ((uint32_t)0x00000002)
#define USART_IER_RDAIE                        ((uint32_t)0x00000001)
#define USART_IER_ALL                          ((uint32_t)0x0000000F)
                                                 
/* USART Flags -------------------------------------------------------------------------------------------*/
#define USART_LSR_RFDR                         ((uint32_t)0x00000001)
#define USART_LSR_OEI                          ((uint32_t)0x00000002)
#define USART_LSR_PEI                          ((uint32_t)0x00000004)
#define USART_LSR_FEI                          ((uint32_t)0x00000008)
#define USART_LSR_BII                          ((uint32_t)0x00000010)
#define USART_LSR_THRE                         ((uint32_t)0x00000020)
#define USART_LSR_TE                           ((uint32_t)0x00000040)
#define USART_LSR_ERR                          ((uint32_t)0x00000080)
#define USART_LSR_RSADDEF                      ((uint32_t)0x00000100)


#define USART_FIFO_TX                                 ((uint32_t)0x00000004)
#define USART_FIFO_RX                                 ((uint32_t)0x00000002)
                                                        
#define USART_STICK_LOW                               ((uint32_t)0x00000010)
#define USART_STICK_HIGH                              ((uint32_t)0xFFFFFFEF)


extern void sys_uart_cfg(void);
extern void sys_uart_close(void);

#endif