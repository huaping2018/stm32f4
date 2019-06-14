#include "sys.h"
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F4探索者开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/6/10
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

#if (USART1_DMA_EN == 1)

uart_rx_t tUART1_Rx;

u8 Rx_Buf[RX_BUF_LEN];     
u8 Tx_Buf[TX_BUF_LEN];     

u8 UART1_Use_DMA_Tx_Flag = 0;


/* 库版本: V1.8.0
 * 串口1初始化
 * 使用PA9和PA10
 */
void UART1Init(void)     
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    USART_DeInit(USART1);
    /* 1.使能PA口时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* 2.使能串口1时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* 3.管脚映射 */
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);  /* GPIOA9复用为USART1 */  
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); /* GPIOA10复用为USART1 */ 

    /* 4.配置串口1的PA9和PA10管脚 */
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

    /* 5.配置串口工作模式 */
    //USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate            = 115200;                             
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;                    
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;                           
    USART_InitStructure.USART_Parity              = USART_Parity_No ;                       
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx; 
    USART_Init(USART1, &USART_InitStructure);

    /* 6.设置中断 */
    //NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;      
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;       
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 7.配置串口中断 */

    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART1, USART_IT_TC,   DISABLE);
    USART_ITConfig(USART1, USART_IT_TXE,  DISABLE);  
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);/* 使能IDLE中断 */

    USART_Cmd(USART1, ENABLE);

}

void DMA_Use_USART1_Tx_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;


    /* 1.使能DMA2时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    /* 2.配置使用DMA发送数据 */
    DMA_DeInit(DMA2_Stream7); 

    DMA_InitStructure.DMA_Channel             = DMA_Channel_4;               /* 配置DMA通道 */
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (u32)(&(USART1->DR));   /* 目的 */
    DMA_InitStructure.DMA_Memory0BaseAddr     = (u32)Tx_Buf;             /* 源 */
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_MemoryToPeripheral;    /* 方向 */
    DMA_InitStructure.DMA_BufferSize          = TX_BUF_LEN;                    /* 长度 */                  
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;    /* 外设地址是否自增 */
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;         /* 内存地址是否自增 */
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_MemoryDataSize_Byte;      /* 目的数据带宽 */
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;      /* 源数据宽度 */
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Normal;              /* 单次传输模式/循环传输模式 */
    DMA_InitStructure.DMA_Priority            = DMA_Priority_High;             /* DMA优先级 */
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;          /* FIFO模式/直接模式 */
    DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_HalfFull; /* FIFO大小 */
    DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;       /* 单次传输 */
    DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single;

    /* 3. 配置DMA */
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);

    /* 4.使能DMA中断 */
    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);

    /* 5.使能串口的DMA发送接口 */
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    /* 6. 配置DMA中断优先级 */
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream7_IRQn;           
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;          
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1; 
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 7.不使能DMA */                  
    DMA_Cmd(DMA2_Stream7, DISABLE);
}

void DMA_Use_USART1_Rx_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    tUART1_Rx.dwUART1RxLen = 0;
    /* 1.使能DMA2时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    /* 2.配置使用DMA接收数据 */
    DMA_DeInit(DMA2_Stream2); 

#if 0
    DMA_InitStructure.DMA_Channel             = DMA_Channel_4;               /* 配置DMA通道 */
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (u32)(&(USART1->DR));   /* 源 */
    DMA_InitStructure.DMA_Memory0BaseAddr     = (u32)Rx_Buf;             /* 目的 */
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;    /* 方向 */
    DMA_InitStructure.DMA_BufferSize          = RX_BUF_LEN;                    /* 长度 */                  
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;    /* 外设地址是否自增 */
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;         /* 内存地址是否自增 */
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_MemoryDataSize_Byte;      /* 目的数据带宽 */
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;      /* 源数据宽度 */
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Normal;              /* 单次传输模式/循环传输模式 */
    DMA_InitStructure.DMA_Priority            = DMA_Priority_VeryHigh;        /* DMA优先级 */
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;          /* FIFO模式/直接模式 */
    //DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_HalfFull; /* FIFO大小 */
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;       /* 单次传输 */
    DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single;

    /* 3. 配置DMA */
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);

    /* 4.由于接收不需要DMA中断，故不设置DMA中断 */

    /* 5.使能串口的DMA接收 */
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);

    /* 6. 由于接收不需要DMA中断，故不能配置DMA中断优先级 */

    /* 7.使能DMA */ 
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
    /* 接收完成中断 */
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  
    {  
        USART1->SR;  
        USART1->DR; /* 清USART_IT_IDLE标志 */
        /* 关闭接收DMA  */
        DMA_Cmd(DMA2_Stream2,DISABLE);  
        /* 清除标志位 */
        DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);  

        /* 获得接收帧帧长 */
        len = RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);  
        memcpy(buf,Rx_Buf,len);  

        /* 设置传输数据长度 */
        DMA_SetCurrDataCounter(DMA2_Stream2,RX_BUF_LEN);  
        /* 打开DMA */
        DMA_Cmd(DMA2_Stream2,ENABLE);  

        return len;  
    }   

    return 0;  
}  

static void deal_irq_tx_end(void)  
{  
    if(USART_GetITStatus(USART1, USART_IT_TXE) == RESET)  
    {  
        /* 关闭发送完成中断  */ 
        USART_ITConfig(USART1,USART_IT_TC,DISABLE);  
        /* 发送完成  */
        UART1_Use_DMA_Tx_Flag = 0;  
    }    
}

void USART1_IRQFuc(void)   
{  
    /* 发送完成中断处理 */
    deal_irq_tx_end();   
    /* 接收完成中断处理 */
    tUART1_Rx.dwUART1RxLen = deal_irq_rx_end(tUART1_Rx.UART1RxBuf); 
}

void Use_DMA_tx(u8 *data, u16 size)  
{  
    /* 等待空闲 */
    while (UART1_Use_DMA_Tx_Flag);  
    UART1_Use_DMA_Tx_Flag = 1;  
    /* 复制数据 */
    memcpy(Tx_Buf,data,size);  
    /* 设置传输数据长度 */  
    DMA_SetCurrDataCounter(DMA2_Stream7, size);  
    /* 打开DMA,开始发送 */  
    DMA_Cmd(DMA2_Stream7,ENABLE);  
} 

void DMA2_Stream7_IRQFuc(void)
{
    if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7) != RESET)   
    {  
        /* 清除标志位 */
        DMA_ClearFlag(DMA2_Stream7,DMA_IT_TCIF7);  
        /* 关闭DMA */
        DMA_Cmd(DMA2_Stream7,DISABLE);
        /* 打开发送完成中断,确保最后一个字节发送成功 */
        USART_ITConfig(USART1,USART_IT_TC,ENABLE);  
    }  
}

#else
//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif
	
}

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
  } 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 

#endif

#endif	

