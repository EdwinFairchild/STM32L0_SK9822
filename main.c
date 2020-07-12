
/*

 Driver for the SK9822 Addressable RGB leds
 1/2020
 EdwinFairchild.com
 www.instagram.com/edwinfairchild

*/

//---------| Incluides |--------------
#include "stm32l031xx.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "cmsis_armcc.h"

#define GLOBAL 		0
#define RED 			3
#define GREEN			2
#define BLUE			1
#define LEDCOUNT 	144



//---------| Global variables |------
uint32_t msTICKS = 0;
bool dataReceived = false ;
uint8_t data_buffer;
bool SENDCOMPLETE = false;

uint8_t LEDS_ARRAY[LEDCOUNT][4];


	
//--------| Prototypes |--------------
void setUpDevice(void);
void setClockTo32Mhz(void);
void init_uart2(void);
void uart2_send(uint8_t);
void delayMs(uint32_t);
static void printMsg(char *msg, ...);
void spiSend(uint8_t data);
void initSPI(void);
	
void clearArray(void);
void skSetLed(uint8_t index , uint8_t brightness, uint8_t red, uint8_t green , uint8_t blue);
void skUpdate(void);
	
int main(void)
{	

	setUpDevice();
	
	//set up SPI
	initSPI();
	clearArray();
                                        

//	printMsg("SystemCoreClock: %d", SystemCoreClock);
 
	while(1)
	{

		int i;
		
		for(i = 0 ; i<LEDCOUNT; i++)
		{
			skSetLed(i,15,0,0,155);
			skUpdate();
			//delayMs(1);
			skSetLed(i,0,0,0,0);
			skUpdate();			
		}
	}
	
}

void skSetLed(uint8_t index , uint8_t brightness, uint8_t red, uint8_t green , uint8_t blue)
{	
	if(brightness == 0)
	{
		LEDS_ARRAY[index][GLOBAL] = 0xE0;
	}
	else
	{
		LEDS_ARRAY[index][GLOBAL] = 0xE0 | brightness; 
	}
	LEDS_ARRAY[index][RED] 		=  red; 
	LEDS_ARRAY[index][GREEN] 	=  green; 
	LEDS_ARRAY[index][BLUE] 	=  blue; 		
}

void skUpdate(void)
{
	uint8_t i,j;
	
	
	//start sequence
	for(i = 0 ; i < 4 ; i++)
		{
			spiSend(0x00);
		}
	
	//led frame
	for(i = 0 ; i < LEDCOUNT ; i++)
	{
		for(j = 0 ; j<4; j++)
		{
			spiSend(LEDS_ARRAY[i][j]); //brightness
		}
	}
				
		
	//end sequence 
	
	for(i = 0 ; i < 4; i++)
	{
		spiSend(0xFF);
	}


	
		
}
void clearArray(void)
{
	//simple clears the array so all leds are off
	uint8_t i , j;
	for( i = 0 ; i<LEDCOUNT; i++)
	{	
		LEDS_ARRAY[i][GLOBAL] = 0xE0;
		for(j = 1 ; j<4; j++)
		{
			LEDS_ARRAY[i][j] = 0x00;			
		}		
	}
	
}

void initSPI(void)
{
	//clock enable
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	
	//gpio set to alt function mode 
	GPIOA->MODER &=  ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE7) ;
	GPIOA->MODER |=  ( GPIO_MODER_MODE5_1 | GPIO_MODER_MODE7_1);
	
	/*gpio set which alt function
		nothing to set in the alternate function registers because 
		by default they are set to 00 and 00 is the value we need to
		set it to SPI*/
	
	
	SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_MSTR  | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA;
	SPI1->CR2 = SPI_CR2_SSOE ; 
	SPI1->CR1 |= SPI_CR1_SPE;

	
		
}

void spiSend(uint8_t data)
{

		/* Will inititiate 8-bit transmission if TXE */
		
	while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE) ; /* Test Tx empty */
	(SPI1->DR) = data;	
	
}

void setUpDevice(void)
{
	
	setClockTo32Mhz();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000);
		
	//UART used for debugging
	NVIC_EnableIRQ(USART2_IRQn);
	init_uart2();
	
}

void USART2_IRQHandler(void)
{
	// get status register and check what generated the interrupt
	volatile const uint32_t STATUS = USART2->ISR;

	if ((STATUS & USART_ISR_TC)) // transmit complete
	{
		//clear interrupt flag
		USART2->ICR |= USART_ICR_TCCF;
		SENDCOMPLETE = true ;

		if (1) //if there is more data to send do so
		{

		}

	}
	if ((STATUS & USART_ISR_RXNE)) //send received data to buffer
	{
		/*
		 * put the data received at the current count of
		 * variable bytes_received and the variable itself
		 */
		data_buffer = USART2->RDR;
		dataReceived = true; 	

	}

}
void uart2_send(uint8_t data)
{

	SENDCOMPLETE = false;
	USART2->TDR = data;
	while (SENDCOMPLETE == false)
		;

}
void delayMs(uint32_t ms)
{
	msTICKS = 0;
	while (msTICKS < ms)
		;
}
void SysTick_Handler(void)
{
	msTICKS++;
}
void init_uart2(void)
{
	//enable  clock  needed for uart gpio aswell as uart itself
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	// PA2 and PA15 to Alternate Function Mode
	GPIOA->MODER = ( GPIOA->MODER & ~(GPIO_MODER_MODE2_0))
			| (GPIO_MODER_MODE2_1);

	GPIOA->MODER = ( GPIOA->MODER & ~(GPIO_MODER_MODE15_0))
			| (GPIO_MODER_MODE15_1);

	//Select the specific Alternate function
	GPIOA->AFR[0] |= 4 << GPIO_AFRL_AFSEL2_Pos;
	GPIOA->AFR[1] |= 4 << GPIO_AFRH_AFSEL15_Pos;

	// Baudrate = clk_Frq / BRR ===>  32Mhz / 9600 = 0xD05
	USART2->BRR = 0xD05; //160000 / 96;
	// Enable RX_NE interrupt and TXE interrupt, enable UART, RECEIVE , TRANSMIT COMPLETE
	USART2->CR1 = USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_RE	| USART_CR1_TCIE;

}
void setClockTo32Mhz(void)
{

	//adjust flash latency
	FLASH->ACR |= FLASH_ACR_LATENCY;
	while ((FLASH->ACR & FLASH_ACR_LATENCY) == 0)
		; //wait for latency set flag

	//set voltage scaling to range 1
	PWR->CR |= PWR_CR_VOS_0;
	PWR->CR &= ~(PWR_CR_VOS_1);
	while (((PWR->CSR) & (PWR_CSR_VOSF)) == 1)
		; //wait for voltage to settle

	//turn on HSE external, HSE bypass and security
	RCC->CR |= RCC_CR_CSSHSEON | RCC_CR_HSEBYP | RCC_CR_HSEON;
	while (((RCC->CR) & RCC_CR_HSERDY) == 0)
		; //wait for the HSE to be ready

	//reset and configure pll mull and div settings, and PLL source
	RCC->CFGR = ((RCC->CFGR & ~(RCC_CFGR_PLLDIV | RCC_CFGR_PLLMUL))
			| RCC_CFGR_PLLDIV2 | RCC_CFGR_PLLMUL8 | RCC_CFGR_PLLSRC_HSE);
	while ((RCC->CR & RCC_CR_PLLRDY) == 1)
		;

	//turn on PLL , wait for ready
	RCC->CR |= RCC_CR_PLLON;
	while (((RCC->CR) & RCC_CR_PLLRDY) == 0)
		; // wait for pll to ready

	//set PLL as system clock
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while (((RCC->CFGR) & (RCC_CFGR_SWS_PLL)) != RCC_CFGR_SWS_PLL)
		;
}

static void printMsg(char *msg, ...)
{
	
	char buff[80];
	va_list args;
	va_start (args, msg);
	vsprintf(buff,msg, args);
	
	for(int i = 0 ; i < strlen(buff) ; i++)
	{
		SENDCOMPLETE = false;
		USART2->TDR = buff[i];
		while (SENDCOMPLETE == false);		
		
	}
	
	
	
	
} 
