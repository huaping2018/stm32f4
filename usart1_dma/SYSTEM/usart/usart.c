#include "sys.h"
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F4̽���߿�����
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/6/10
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	

#if (USART1_DMA_EN == 1)

uart_rx_t tUART1_Rx;

u8 Rx_Buf[RX_BUF_LEN];     
u8 Tx_Buf[TX_BUF_LEN];     

u8 UART1_Use_DMA_Tx_Flag = 0;


/* ��汾: V1.8.0
 * ����1��ʼ��
 * ʹ��PA9��PA10
 */
void UART1Init(void)     
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    USART_DeInit(USART1);
    /* 1.ʹ��PA��ʱ�� */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* 2.ʹ�ܴ���1ʱ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* 3.�ܽ�ӳ�� */
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);  /* GPIOA9����ΪUSART1 */  
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); /* GPIOA10����ΪUSART1 */ 

    /* 4.���ô���1��PA9��PA10�ܽ� */
    //GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;         
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;       
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;       
    GPIO_Init(GPIOA, &GPIO_InitStructure);  /* TXIO */  

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;                
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;       
    GPIO_Init(GPIOA, &GPIO_InitStructure);  /* RXIO */

    /* 5.���ô��ڹ���ģʽ */
    //USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate            = 115200;                             
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;                    
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;                           
    USART_InitStructure.USART_Parity              = USART_Parity_No ;                       
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx; 
    USART_Init(USART1, &USART_InitStructure);

    /* 6.�����ж� */
    //NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;      
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;       
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 7.���ô����ж� */

    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART1, USART_IT_TC,   DISABLE);
    USART_ITConfig(USART1, USART_IT_TXE,  DISABLE);  
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);/* ʹ��IDLE�ж� */

    USART_Cmd(USART1, ENABLE);

}

void DMA_Use_USART1_Tx_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;


    /* 1.ʹ��DMA2ʱ�� */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    /* 2.����ʹ��DMA�������� */
    DMA_DeInit(DMA2_Stream7); 

    DMA_InitStructure.DMA_Channel             = DMA_Channel_4;               /* ����DMAͨ�� */
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (u32)(&(USART1->DR));   /* Ŀ�� */
    DMA_InitStructure.DMA_Memory0BaseAddr     = (u32)Tx_Buf;             /* Դ */
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_MemoryToPeripheral;    /* ���� */
    DMA_InitStructure.DMA_BufferSize          = TX_BUF_LEN;                    /* ���� */                  
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;    /* �����ַ�Ƿ����� */
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;         /* �ڴ��ַ�Ƿ����� */
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_MemoryDataSize_Byte;      /* Ŀ�����ݴ��� */
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;      /* Դ���ݿ�� */
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Normal;              /* ���δ���ģʽ/ѭ������ģʽ */
    DMA_InitStructure.DMA_Priority            = DMA_Priority_High;             /* DMA���ȼ� */
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;          /* FIFOģʽ/ֱ��ģʽ */
    DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_HalfFull; /* FIFO��С */
    DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;       /* ���δ��� */
    DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single;

    /* 3. ����DMA */
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);

    /* 4.ʹ��DMA�ж� */
    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);

    /* 5.ʹ�ܴ��ڵ�DMA���ͽӿ� */
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    /* 6. ����DMA�ж����ȼ� */
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream7_IRQn;           
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;          
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1; 
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 7.��ʹ��DMA */                  
    DMA_Cmd(DMA2_Stream7, DISABLE);
}

void DMA_Use_USART1_Rx_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    tUART1_Rx.dwUART1RxLen = 0;
    /* 1.ʹ��DMA2ʱ�� */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    /* 2.����ʹ��DMA�������� */
    DMA_DeInit(DMA2_Stream2); 

#if 0
    DMA_InitStructure.DMA_Channel             = DMA_Channel_4;               /* ����DMAͨ�� */
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (u32)(&(USART1->DR));   /* Դ */
    DMA_InitStructure.DMA_Memory0BaseAddr     = (u32)Rx_Buf;             /* Ŀ�� */
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;    /* ���� */
    DMA_InitStructure.DMA_BufferSize          = RX_BUF_LEN;                    /* ���� */                  
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;    /* �����ַ�Ƿ����� */
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;         /* �ڴ��ַ�Ƿ����� */
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_MemoryDataSize_Byte;      /* Ŀ�����ݴ��� */
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;      /* Դ���ݿ�� */
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Normal;              /* ���δ���ģʽ/ѭ������ģʽ */
    DMA_InitStructure.DMA_Priority            = DMA_Priority_VeryHigh;        /* DMA���ȼ� */
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;          /* FIFOģʽ/ֱ��ģʽ */
    //DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_HalfFull; /* FIFO��С */
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;       /* ���δ��� */
    DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single;

    /* 3. ����DMA */
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);

    /* 4.���ڽ��ղ���ҪDMA�жϣ��ʲ�����DMA�ж� */

    /* 5.ʹ�ܴ��ڵ�DMA���� */
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);

    /* 6. ���ڽ��ղ���ҪDMA�жϣ��ʲ�������DMA�ж����ȼ� */

    /* 7.ʹ��DMA */ 
    DMA_Cmd(DMA2_Stream2,ENABLE);
#else
//DMA_InitTypeDef DMA_InitStructure;

DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOMode_Disable;
DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

DMA_InitStructure.DMA_Channel = DMA_Channel_4;
DMA_InitStructure.DMA_BufferSize = 10;
DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &Rx_Buf[0];
//DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)tUART1_Rx.UART1RxBuf;


DMA_Init(DMA2_Stream2, &DMA_InitStructure);
DMA_Cmd(DMA2_Stream2, ENABLE);

USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

#endif
}

static u8 deal_irq_rx_end(u8 *buf)  
{     
    u16 len = 0;  
    /* ��������ж� */
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  
    {  
        USART1->SR;  
        USART1->DR; /* ��USART_IT_IDLE��־ */
        /* �رս���DMA  */
        DMA_Cmd(DMA2_Stream2,DISABLE);  
        /* �����־λ */
        DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);  

        /* ��ý���֡֡�� */
        len = RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);  
        memcpy(buf,Rx_Buf,len);  

        /* ���ô������ݳ��� */
        DMA_SetCurrDataCounter(DMA2_Stream2,RX_BUF_LEN);  
        /* ��DMA */
        DMA_Cmd(DMA2_Stream2,ENABLE);  

        return len;  
    }   

    return 0;  
}  

static void deal_irq_tx_end(void)  
{  
    if(USART_GetITStatus(USART1, USART_IT_TXE) == RESET)  
    {  
        /* �رշ�������ж�  */ 
        USART_ITConfig(USART1,USART_IT_TC,DISABLE);  
        /* �������  */
        UART1_Use_DMA_Tx_Flag = 0;  
    }    
}

void USART1_IRQFuc(void)   
{  
    /* ��������жϴ��� */
    deal_irq_tx_end();   
    /* ��������жϴ��� */
    tUART1_Rx.dwUART1RxLen = deal_irq_rx_end(tUART1_Rx.UART1RxBuf); 
}

void Use_DMA_tx(u8 *data, u16 size)  
{  
    /* �ȴ����� */
    while (UART1_Use_DMA_Tx_Flag);  
    UART1_Use_DMA_Tx_Flag = 1;  
    /* �������� */
    memcpy(Tx_Buf,data,size);  
    /* ���ô������ݳ��� */  
    DMA_SetCurrDataCounter(DMA2_Stream7, size);  
    /* ��DMA,��ʼ���� */  
    DMA_Cmd(DMA2_Stream7,ENABLE);  
} 

void DMA2_Stream7_IRQFuc(void)
{
    if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7) != RESET)   
    {  
        /* �����־λ */
        DMA_ClearFlag(DMA2_Stream7,DMA_IT_TCIF7);  
        /* �ر�DMA */
        DMA_Cmd(DMA2_Stream7,DISABLE);
        /* �򿪷�������ж�,ȷ�����һ���ֽڷ��ͳɹ� */
        USART_ITConfig(USART1,USART_IT_TC,ENABLE);  
    }  
}

#else
//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif
	
}

void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}   		 
  } 
#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();  											 
#endif
} 

#endif

#endif	

