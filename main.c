#define HSE_VALUE ((uint32_t)8000000) 
#define BT_BAUD 38400
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_usart.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "usb_dcd_int.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "delay.h"

void init_USART1(uint32_t);
int i=0;
uint8_t ch;
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

int main(void) {

	SystemInit();
	SysTick_Init();
        init_USART1(BT_BAUD);
	USBD_Init(&USB_OTG_dev,
	            USB_OTG_FS_CORE_ID,
	            &USR_desc,
	            &USBD_CDC_cb,
	            &USR_cb);
      
	while (1) {
         if(VCP_get_char(&ch))
         USART1->DR=ch;
           delay_nms(1);

           
	}
}

void init_USART1(uint32_t baudrate){

 
  GPIO_InitTypeDef GPIO_InitStruct; 
  USART_InitTypeDef USART_InitStruct; 
  NVIC_InitTypeDef NVIC_InitStructure; 
 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) 
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 		
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;	
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;			
  GPIO_Init(GPIOB, &GPIO_InitStruct);					
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

  USART_InitStruct.USART_BaudRate = baudrate;				
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;		
  USART_InitStruct.USART_Parity = USART_Parity_No;		
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 
  USART_Init(USART1, &USART_InitStruct);					

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
  NVIC_Init(&NVIC_InitStructure);							
  USART_Cmd(USART1, ENABLE);
}
  

void assert_failed(u8* file, u32 line)
{ 
  printf( "\r\nassert_failed(). file: %s, line: %ld\r\n", file, line );
  while (1)
  {}
}
void SysTick_Handler(void) {
    TimeTick_Decrement();
}

void OTG_FS_IRQHandler(void)
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

void OTG_FS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line18);
}

void USART1_IRQHandler(void){


  if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
      USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    VCP_put_char(USART1->DR); 
  }
}
