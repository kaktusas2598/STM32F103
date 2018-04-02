#include "stm32f1xx.h"

#define LED_ON()  GPIOA->ODR |= GPIO_ODR_ODR0
#define LED_OFF() GPIOA->ODR &= ~GPIO_ODR_ODR0


int SendChar (int ch)  {
	while (!(USART1->SR & USART_SR_TXE));
	USART1->DR = (ch & 0xFF);
	return (ch);
}

int GetChar (void)  {
	while (!(USART1->SR & USART_SR_RXNE));
	return ((int)(USART1->DR & 0xFF));
}

#define I2C_DUTYCYCLE_2                 0x00000000U
#define I2C_DUTYCYCLE_16_9              I2C_CCR_DUTY

#define I2C_MIN_PCLK_FREQ(__PCLK__, __SPEED__)             (((__SPEED__) <= 100000U) ? ((__PCLK__) < I2C_MIN_PCLK_FREQ_STANDARD) : ((__PCLK__) < I2C_MIN_PCLK_FREQ_FAST))
#define I2C_CCR_CALCULATION(__PCLK__, __SPEED__, __COEFF__)     (((((__PCLK__) - 1U)/((__SPEED__) * (__COEFF__))) + 1U) & I2C_CCR_CCR)
#define I2C_FREQRANGE(__PCLK__)                            ((__PCLK__)/1000000U)
#define I2C_RISE_TIME(__FREQRANGE__, __SPEED__)            (((__SPEED__) <= 100000U) ? ((__FREQRANGE__) + 1U) : ((((__FREQRANGE__) * 300U) / 1000U) + 1U))
#define I2C_SPEED_STANDARD(__PCLK__, __SPEED__)            ((I2C_CCR_CALCULATION((__PCLK__), (__SPEED__), 2U) < 4U)? 4U:I2C_CCR_CALCULATION((__PCLK__), (__SPEED__), 2U))
#define I2C_SPEED_FAST(__PCLK__, __SPEED__, __DUTYCYCLE__) (((__DUTYCYCLE__) == I2C_DUTYCYCLE_2)? I2C_CCR_CALCULATION((__PCLK__), (__SPEED__), 3U) : (I2C_CCR_CALCULATION((__PCLK__), (__SPEED__), 25U) | I2C_DUTYCYCLE_16_9))
#define I2C_SPEED(__PCLK__, __SPEED__, __DUTYCYCLE__)      (((__SPEED__) <= 100000U)? (I2C_SPEED_STANDARD((__PCLK__), (__SPEED__))) : \
                                                                  ((I2C_SPEED_FAST((__PCLK__), (__SPEED__), (__DUTYCYCLE__)) & I2C_CCR_CCR) == 0U)? 1U : \
                                                                  ((I2C_SPEED_FAST((__PCLK__), (__SPEED__), (__DUTYCYCLE__))) | I2C_CCR_FS))

//Stolen from CMSIS and HAL
#define RCC_CFGR_PPRE1_Pos                   (8U)
const uint8_t APBPrescTable[8U] =  {0, 0, 0, 0, 1, 2, 3, 4};
uint32_t RCC_GetPCLK1Freq(void)
{
  /* Get HCLK source and Compute PCLK1 frequency ---------------------------*/
  return (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);
}

void USART1_IRQHandler(void)
{
	//Received data
	if (USART1->SR & USART_SR_RXNE) {
		//Try to send received data back to mirror and toggle led
		//REdirect data from PC to ESP
		int tmp = ((int)(USART1->DR & 0xFF));
		USART2->DR = (tmp & 0xFF);
		GPIOA->ODR ^= GPIO_ODR_ODR0;
	}
	//Implement other events :)
}

void USART2_IRQHandler(void)
{
	//Received data event
	if (USART2->SR & USART_SR_RXNE) {
		int tmp = ((int)(USART2->DR & 0xFF));
		//Send data received from ESP-01 to PC
		USART1->DR = (tmp & 0xFF);
	}
}

//Why are you broken?
void SysTick_Handler(void) {

	GPIOA->ODR ^= GPIO_ODR_ODR0;
}

void USART1_Setup();
void USART2_Setup();
void I2C2_Setup();
//Enable clocks and setup pins
void GPIO_Setup();

void I2C2_SendCommand(uint16_t DevAddress, uint16_t MemAddress, uint16_t Command);
void delayMs(volatile uint32_t delay);

int main(void)
{
	//Setup clocks to use PLL and output 20Mhz
	/*RCC->CFGR &= ~RCC_CFGR_PLLSRC; //Use HSI clock divided by 2 - 4Mhz*/
	/*RCC->CFGR |= (1 << 19) | (1 << 18);//Bits set for 5 multiplication factor*/
	//Divide APB1 clock to get 10MHz
	/*RCC->CFGR |= (1 << 10); //Divides PCLK1 in 2, so should be 10*/
	//Turn on PLL
	/*RCC->CR |= RCC_CR_PLLON;*/

	//Updates SystemCoreClock variables, so it holds correct value
	SystemCoreClockUpdate();

	//Testing SysTick handler
	/*NVIC_EnableIRQ(SysTick_IRQn);*/
	/*NVIC_SetPriority(SysTick_IRQn, 1);*/

	GPIO_Setup();

	USART1_Setup(); //STM to PC
	USART2_Setup();//STM to ESP-01 (ESP2866)

	I2C2_Setup();

	TIM3->CR1 = 0x0000;//Just in case
	//Set prescaler to 8 so 1MHz?
	TIM2->PSC = 8;

	TIM2->ARR = 0xFFFF;//max value

	//Enable counter
	TIM2->CR1 |= TIM_CR1_CEN;

	//Give some time for slave devices to init
	delayMs(350);
	/*LED_ON();*/

	/*I2C2_SendCommand(0x3C, 0x00, 0xA5);*/

	/*LED_OFF();*/

	//SEND 'AT' to ESP-01
	while (!(USART2->SR & USART_SR_TXE));
	USART2->DR = ('A' & 0xFF);
	while (!(USART2->SR & USART_SR_TXE));
	USART2->DR = ('T' & 0xFF);
	while (!(USART2->SR & USART_SR_TXE));
	USART2->DR = ('\r' & 0xFF);
	while (!(USART2->SR & USART_SR_TXE));
	USART2->DR = ('\n' & 0xFF);


	while (1)
	{
		/*SendChar('A');*/
		/*SendChar('T');*/

		GPIOC->ODR ^= GPIO_ODR_ODR13;
		delayMs(1000);
		GPIOC->ODR ^= GPIO_ODR_ODR15;
	}

	return 0;
}

void delayMs(volatile uint32_t delay) {

	for (delay; delay > 0; delay--) {
		TIM2->EGR |= TIM_EGR_UG;//Re-initialize counter

		//If timer is 1MHz, 1000 should be one milisecond
		while(TIM2->CNT < 1000);
	}
}

void GPIO_Setup() {

	//Enable the clock to PORT A, B and C
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

	//Enable Timer 2 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	//PC13 and PC15 50Mhz output pins
	GPIOC->CRH |= GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1 |
		GPIO_CRH_MODE15_0 | GPIO_CRH_MODE15_1;

	//PA0 also 50Mhz output
	GPIOA->CRL |= GPIO_CRL_MODE0_0 | GPIO_CRL_MODE0_1;
}

void USART1_Setup() {

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	//use AFIO_MAPR register to remap alternate functions to use USART 1 PB6(TX) and PB7(RX)
	//Ref 7.4.2 and 6.3.7 in st manual
	AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;

	//Configure USART1 GPIO
	//Set RX as floating input(reset state) and TX as alternative function output push-pull
	GPIOB->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_0;

	USART1->CR1 |= (USART_CR1_RE | USART_CR1_TE);  // RX, TX enable
	USART1->CR1 |= USART_CR1_UE; //Enable USART
	//Make sure to clear OVER8 bit(15 bit in USART_CR1) to oversample by 16, I dont know if I need this
	USART1->CR1 &= ~(1 << 15);
	//Configure Baud Rate
	/*USART_BRR = Fck/BAUDRATE*/
	//Set 115200 Baud, 8 MHz crystal
	USART1->BRR = SystemCoreClock/115200L;

	//Enable RXNE (Receive Data register not empty) interrupt
	USART1->CR1 |= USART_CR1_RXNEIE;

	//Enable USART1 Interrupt and set highest priority
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 0);
}

void USART2_Setup() {

	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	//Configure USART2 GPIO
	//Set RX as floating input(reset state) and TX as alternative function output push-pull
	GPIOA->CRL |= GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1 | GPIO_CRL_CNF3_0;

	USART2->CR1 |= (USART_CR1_RE | USART_CR1_TE);  // RX, TX enable
	USART2->CR1 |= USART_CR1_UE; //Enable USART
	//Make sure to clear OVER8 bit(15 bit in USART_CR1) to oversample by 16, I dont know if I need this
	/*USART2->CR1 &= ~(1 << 15);*/
	//Configure Baud Rate
	USART2->BRR = SystemCoreClock/115200L;

	//Enable RXNE (Receive Data register not empty) interrupt
	USART2->CR1 |= USART_CR1_RXNEIE;

	//Enable USART2 Interrupt and set highest priority
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn, 0);
}

//BAD :(
void I2C2_Setup() {

	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

	//Configure I2C GPIO, use alternative function open drain outputs
	//Use 10MHz GPIO speed to enable Fast mode, see ref 22.3.3.
	GPIOB->CRH |= GPIO_CRH_MODE10_0 | GPIO_CRH_MODE11_0 |
	//Use 2Mhz, lets try 100KHz I2C
	/*GPIOB->CRH |= GPIO_CRH_MODE10_1 | GPIO_CRH_MODE11_1 |*/
		GPIO_CRH_CNF10_1 | GPIO_CRH_CNF10_0 |
		GPIO_CRH_CNF11_1 | GPIO_CRH_CNF11_0;

	/*uint32_t freqrange = 0U;*/
	/*uint32_t pclk1 = 0U;*/
	/* Get PCLK1 frequency */
	/*pclk1 = RCC_GetPCLK1Freq();*/
	/* Calculate frequency range */
	/*freqrange = I2C_FREQRANGE(pclk1);*/
	/* Configure I2Cx: Frequency range */
	/*I2C2->CR2 = freqrange;*/
	/* Configure I2Cx: Rise Time */
	/*I2C2->TRISE = I2C_RISE_TIME(freqrange, 100000);*/
	/* Configure I2Cx: Speed */
	/*I2C2->CCR = I2C_SPEED(pclk1, 100000, I2C_DUTYCYCLE_2);*/

	//Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings, 10MHz
	/*I2C2->CR2 |= 0b001010;*/
	// TODO: Configure the rise time register
	/*I2C2->TRISE = 0x02;*/
	//Enable Fast Mode, Bit 15 in I2C_CCR register
	/*I2C2->CCR |= (1 << 15);*/
	//Set value for 400kHz
	/*I2C2->CCR = 0b00000000001;*/

	//Try setup for 100kHz, PCLK1 - 8MHz
	I2C2->CR2 |= 0b001000;//PCLK1 should be same as sysclock with no prescaler
	I2C2->TRISE = 0x09;
	I2C2->CCR = 0x28;

	I2C2->CR1 &= ~I2C_CR1_ENGC;//Disable general call
	I2C2->CR1 |= I2C_CR1_NOSTRETCH;//Disable clock stretching(slave mode)

	//Enable I2C buffer, event and error interrupts
	/*I2C2->CR2 |= I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;*/

	/*NVIC_EnableIRQ(I2C2_EV_IRQn);*/
	/*NVIC_EnableIRQ(I2C2_ER_IRQn);*/
	/*NVIC_SetPriority(I2C2_EV_IRQn, 0);*/
	/*NVIC_SetPriority(I2C2_ER_IRQn, 0);*/

	I2C2->CR1 |= I2C_CR1_PE;//Enable I2C
}

//TODO: BAD
void I2C2_SendCommand(uint16_t DevAddress, uint16_t MemAddress, uint16_t Command) {


    /*I2C2->CR1 &= ~I2C_CR1_POS;*/

	//Send start condition
	I2C2->CR1 |= I2C_CR1_START;
	//Wait for start bit to be cleared
	while(I2C1->CR1 & I2C_CR1_START);

	//Wait until SB flag is set - Does not get past this
	//while(!(I2C1->SR1 & I2C_SR1_SB));

	//Send address left shifted by 1, R/W is not set, in SSD1306 case, it is write
	/*I2C2->DR |= (0x3C << 1);*/
	I2C2->DR = (0x3C << 1);
	/*I2C2->DR = 0x78;*/

	return;
	while (!(I2C2->SR1 & I2C_SR1_ADDR));//Gets set after end of addr transmission in master mode


	/*I2C2_CLEAR_ADDRFLAG();*/
	uint32_t tmpreg = 0x00U;
	tmpreg = I2C2->SR1;
	tmpreg = I2C2->SR2;
	/*I2C2->SR1 &= ~I2C_SR1_ADDR; //Clear addr flag*/

	//Wait until data register is empty after transmission
	/*while (!(I2C2->SR1 & I2C_SR1_TXE));//Doesnt get past here, WHY?*/

	I2C2->DR |= 0x00;//Command byte, 0x40 would be data byte


	/*while (!(I2C2->SR1 & I2C_SR1_TXE));//Doesnt get past here, WHY?*/
	//Send data byte, A5 is Entire Display ON command on SSD1306
	I2C2->DR |= 0xAF;

	/*while (!(I2C2->SR1 & I2C_SR1_TXE));*/
	/*while (!(I2C2->SR1 & I2C_SR1_BTF));//Wait for stop request*/

	//Send stop condition
	I2C2->CR1 |= I2C_CR1_STOP;
	//Wait for start bit to be cleared
	while(I2C1->CR1 & I2C_CR1_STOP);
}
