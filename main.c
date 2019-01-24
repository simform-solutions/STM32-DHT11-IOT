#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "string.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "math.h"


//#ifndef DHT11_H
#define DHT11_H

#define DHT11_SUCCESS         1
#define DHT11_ERROR_CHECKSUM  2
#define DHT11_ERROR_TIMEOUT   3

int a = 1;
int b = 2;
int c = 3;
char buff[32];
typedef struct DHT11_Dev {
	uint8_t temparature;
	uint8_t humidity;
	GPIO_TypeDef* port;
	uint16_t pin;
} DHT11_Dev;

int DHT11_init(struct DHT11_Dev* dev, GPIO_TypeDef* port, uint16_t pin);
int DHT11_read(struct DHT11_Dev* dev);


struct DHT11_Dev dev;
int res;
int i;
/*struct __FILE { int handle; };
FILE __stdout;
FILE __stdin;*/
volatile uint32_t msTicks;                      /* counts 1ms timeTicks       */

/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;
}
//void delay(u32 nCount);
/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
/*void Delay (uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}*/

#define RX_BUF_SIZE 80
volatile char RX_FLAG_END_LINE = 0;
volatile char RXi;
volatile char RXc;
volatile char RX_BUF[RX_BUF_SIZE] = {'\0'};
char gelen;

void clear_RXBuffer(void) {
	for (RXi=0; RXi<RX_BUF_SIZE; RXi++)
		RX_BUF[RXi] = '\0';
	RXi = 0;
}



void delay_ms(u16 nms)
{

	 u32 temp;
	 SysTick->LOAD = 9000*nms;
	 SysTick->VAL=0X00;
	 SysTick->CTRL=0X01;
	 do
	 {
      temp = SysTick->CTRL;
	 }while((temp&0x01)&&(!(temp&(1<<16))));
	 SysTick->CTRL=0x00;
	 SysTick->VAL =0X00;
}


void usart_init(void)
{
	/* Enable USART1 and GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

	/* NVIC Configuration */
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configure the GPIOs */
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure the USART1 */
	USART_InitTypeDef USART_InitStructure;

	/* USART1 configuration ------------------------------------------------------*/
	/* USART1 configured as follow:
		- BaudRate = 9600 baud
		- Word Length = 8 Bits
		- One Stop Bit
		- No parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
		- USART Clock disabled
		- USART CPOL: Clock is active low
		- USART CPHA: Data is captured on the middle
		- USART LastBit: The clock pulse of the last data bit is not output to
			the SCLK pin
	 */
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);

	/* Enable USART1 */
	USART_Cmd(USART1, ENABLE);

	/* Enable the USART1 Receive interrupt: this interrupt is generated when the
		USART1 receive data register is not empty */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}
void USARTSend(char *pucBuffer)
{
    while (*pucBuffer)
    {
        USART_SendData(USART1, *pucBuffer++);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}volatile uint16_t ADCBuffer[] = {0xAAAA, 0xAAAA, 0xAAAA};

void printfloat(float f_num, int dplaces)
{
    int p = 1;
    int i;
    for (i = 0; i < dplaces; i++){
        p*=10;
    }
    printf("%d.%d \r\n",(int)floor(f_num),(int)round(p*(f_num-floor(f_num))));

}


void SetSysClockTo72(void)
{
	ErrorStatus HSEStartUpStatus;
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();

    /* Enable HSE */
    RCC_HSEConfig( RCC_HSE_ON);

    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
    	//FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);

        /* Flash 2 wait state */
        //FLASH_SetLatency( FLASH_Latency_2);

        /* HCLK = SYSCLK */
        RCC_HCLKConfig( RCC_SYSCLK_Div1);

        /* PCLK2 = HCLK */
        RCC_PCLK2Config( RCC_HCLK_Div1);

        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config( RCC_HCLK_Div2);

        /* PLLCLK = 8MHz * 9 = 72 MHz */
        RCC_PLLConfig(0x00010000, RCC_PLLMul_9);

        /* Enable PLL */
        RCC_PLLCmd( ENABLE);

        /* Wait till PLL is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);

        /* Wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { /* If HSE fails to start-up, the application will have wrong clock configuration.
     User can add here some code to deal with this error */

        /* Go to infinite loop */
        while (1)
        {
        }
    }
}

/*void delay(u32 nCount)
{
	for(;nCount!=0;nCount--);
}
*/
/*----------------------------------------------------------------------------
 Redirect printf
*----------------------------------------------------------------------------*/
/*int fputc(int ch, FILE *f) {
	ITM_SendChar(ch);
  return(ch);
}*/

/*----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/
int main (void) {
	char buffer[80] = {'\0'};
  SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  if (SysTick_Config(SystemCoreClock / 1000)) { /* SysTick 1 msec interrupts  */
    while (1);                                  /* Capture error              */
  }
  int c, d;
  /*char buff1[100];
  char buff2[100];
  char buff3[100];
  char buff4[100];*/
  /* Initialize LED which connected to PC13 */
  	GPIO_InitTypeDef  GPIO_InitStructure;
  	// Enable PORTC Clock
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  	/* Configure the GPIO_LED pin */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);

  	GPIO_ResetBits(GPIOC, GPIO_Pin_13); // Set C13 to Low level ("0")

  	// Initialize USART
      usart_init();
    // send at commands to gsm module for activate gprs service and send data to cloud
      USARTSend(" AT\r\n");
      	delay_ms(2400000);
      	USARTSend(" AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n");
          delay_ms(2400000);
      	USARTSend(" AT+SAPBR=3,1,\"APN\",\"wap.vodafone.co.uk\"\r\n");
      	delay_ms(2400000);
      	USARTSend(" AT+SAPBR=1,1\r\n");
      	delay_ms(2400000);
      	USARTSend(" AT+SAPBR=2,1\r\n");
      	delay_ms(2400000);
      	USARTSend(" AT+HTTPINIT\r\n");
      	delay_ms(2400000);
          USARTSend("AT+HTTPPARA=\"CID\",1\r\n");
      	delay_ms(2400000);
      	USARTSend("AT+HTTPPARA=\"URL\",\"http://demo.thingsboard.io:80/api/v1/7Xhzm313Me12TCy9FpT0/telemetry\"\r\n");

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  DHT11_init(&dev, GPIOB, GPIO_Pin_6);

while(1) {
	  res = DHT11_read(&dev);

if(res == b) {
	printf("DHT11_ERROR_CHECKSUM\n");
	 	 	 }
else if(res == a) {

	printf("dht11 success\n");
	printf("TEMPRATURE %d - HUMIDITY %d\n", dev.temparature, dev.humidity);

	char example[100];

	/* Copy the first string into the variable */
	char result[50];
	char resultt[50];

	sprintf(result, "%d", dev.temparature);
    sprintf(resultt, "%d", dev.humidity);
	/* Concatenate the following two strings to the end of the first one */
	strcpy(example, "{\"Temp\":");
	strcat(example, result);
	strcat(example, ",");
	strcat(example, "\"Hum\":");
	strcat(example, resultt);
    strcat(example, "}");
	//USARTSend(example);

	printf("data : %s", example);
	//sprintf(buff, "%s",json);

	//delay_ms(24000000000000);
	USARTSend(" AT+HTTPDATA=20,10000\r\n");
	delay_ms(24000000000000);
	USARTSend(example);

	delay_ms(240000);
	USARTSend(" AT+HTTPACTION=1\r\n");

	 }
	else {
	 printf("TIMEOUT \r \n");
	 	 }
	//delay 5 second
	int c, d;
	for (c = 1; c <= 5000; c++)
	for (d = 1; d <= 5000; ++d);

  }
}
int DHT11_init(struct DHT11_Dev* dev, GPIO_TypeDef* port, uint16_t pin) {
	TIM_TimeBaseInitTypeDef TIM_TimBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	dev->port = port;
	dev->pin = pin;

	//Initialise TIMER2
	TIM_TimBaseStructure.TIM_Period = 84000000 - 1;
	TIM_TimBaseStructure.TIM_Prescaler = 84;
	TIM_TimBaseStructure.TIM_ClockDivision = 0;
	TIM_TimBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimBaseStructure);
	TIM_Cmd(TIM2, ENABLE);

	//Initialise GPIO DHT11
	GPIO_InitStructure.GPIO_Pin = dev->pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(dev->port, &GPIO_InitStructure);

	return 0;
}

int DHT11_read(struct DHT11_Dev* dev) {

	//Initialisation
	uint8_t i, j, temp;
	uint8_t data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	GPIO_InitTypeDef GPIO_InitStructure;

	//Generate START condition
	//o
	GPIO_InitStructure.GPIO_Pin = dev->pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(dev->port, &GPIO_InitStructure);

	//dev->port->MODER |= GPIO_MODER_MODER6_0;

	//Put LOW for at least 18ms
	GPIO_ResetBits(dev->port, dev->pin);

	//wait 18ms
	TIM2->CNT = 0;
	while((TIM2->CNT) <= 18000);

	//Put HIGH for 20-40us
	GPIO_SetBits(dev->port, dev->pin);

	//wait 40us
	TIM2->CNT = 0;
	while((TIM2->CNT) <= 40);
	//End start condition

	//io();
	//Input mode to receive data
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(dev->port, &GPIO_InitStructure);

	//DHT11 ACK
	//should be LOW for at least 80us
	//while(!GPIO_ReadInputDataBit(dev->port, dev->pin));
	TIM2->CNT = 0;
	while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
		if(TIM2->CNT > 100)
			return DHT11_ERROR_TIMEOUT;
	}

	//should be HIGH for at least 80us
	//while(GPIO_ReadInputDataBit(dev->port, dev->pin));
	TIM2->CNT = 0;
	while(GPIO_ReadInputDataBit(dev->port, dev->pin)) {
		if(TIM2->CNT > 100)
			return DHT11_ERROR_TIMEOUT;
	}

	//Read 40 bits (8*5)
	for(j = 0; j < 5; ++j) {
		for(i = 0; i < 8; ++i) {

			//LOW for 50us
			while(!GPIO_ReadInputDataBit(dev->port, dev->pin));
			/*TIM2->CNT = 0;
			while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
				if(TIM2->CNT > 60)
					return DHT11_ERROR_TIMEOUT;
			}*/

			//Start counter
			TIM_SetCounter(TIM2, 0);

			//HIGH for 26-28us = 0 / 70us = 1
			while(GPIO_ReadInputDataBit(dev->port, dev->pin));
			/*while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
				if(TIM2->CNT > 100)
					return DHT11_ERROR_TIMEOUT;
			}*/

			//Calc amount of time passed
			temp = TIM_GetCounter(TIM2);

			//shift 0
			data[j] = data[j] << 1;

			//if > 30us it's 1
			if(temp > 40)
				data[j] = data[j]+1;
		}
	}

	//verify the Checksum
	if(data[4] != (data[0] + data[2]))
		return DHT11_ERROR_CHECKSUM;

	//set data
	dev->temparature = data[2];
	dev->humidity = data[0];

	return DHT11_SUCCESS;
}


