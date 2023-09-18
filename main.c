



#include "RccConfig_F446.h"
#include "Delay_F446.h"

#include "Math.h"




#define AUDIO_BUFFER_SIZE 1024
#define SAMPLE_RATE 44100
#define PI 3.1416

uint16_t audioBuffer[AUDIO_BUFFER_SIZE];

void ADC_Init (void)
{
	
	
// Enable ADC and GPIO clock
	RCC->APB2ENR |= (1<<8);  // enable ADC1 clock
	RCC->AHB1ENR |= (1<<0);  // enable GPIOA clock
	
// Set the prescalar in the Common Control Register (CCR)	
	ADC->CCR |= 1<<16;  		 // PCLK2 divide by 4
	
// Set the Scan Mode and Resolution in the Control Register 1 (CR1)	
	ADC1->CR1 = (1<<8);    // SCAN mode enabled
	ADC1->CR1 &= ~(1<<24);   // 12 bit RES
	
// Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
	ADC1->CR2 = (1<<1);     // enable continuous conversion mode
	ADC1->CR2 |= (1<<10);    // EOC after each conversion
	ADC1->CR2 &= ~(1<<11);   // Data Alignment RIGHT
	
// Set the Sampling Time for the channels	
	//ADC1->SMPR2 &= ~((7<<3);  // Sampling time of 3 cycles for channel 1 and channel 4
		ADC1->SQR1 = 0; // Reset sequence register 1
    ADC1->SQR2 = 0; // Reset sequence register 2
    ADC1->SQR3 |= ADC_SQR3_SQ1_0; // Channel 0 as the first conversion in the sequence
    ADC1->SMPR2 |= ADC_SMPR2_SMP0_0 | ADC_SMPR2_SMP0_1; // Set sample time for channel 0 (adjust as needed)
// Set the Regular channel sequence length in ADC_SQR1
	
		ADC1->SQR1 |= (2<<20);   // SQR1_L =2 for 3 conversions
	
// Set the Respective GPIO PINs in the Analog Mode	
		GPIOA->MODER |= (3<<2);  // analog mode for PA 1
	//GPIOA->MODER |= (3<<8);  // analog mode for PA 4
	
	
	
	// Sampling Freq for Temp Sensor 
	//ADC1->SMPR1 |= (7<<24);  // Sampling time of 21 us
	
	// Set the TSVREFE Bit to wake the sensor
	//ADC->CCR |= (1<<23);
	
	// Enable DMA for ADC
	ADC1->CR2 |= (1<<8);
	
	// Enable Continuous Request
	ADC1->CR2 |= (1<<9);
	
	// Channel Sequence
	ADC1->SQR3 |= (1<<0);  // SEQ1 for Channel 1
	//ADC1->SQR3 |= (4<<5);  // SEQ2 for CHannel 4
	//ADC1->SQR3 |= (18<<10);  // SEQ3 for CHannel 18
}

void ADC_Enable (void)
{
	ADC1->CR2 |= 1<<0;   // ADON =1 enable ADC1
	
	uint32_t delay = 10000; // Wait for ADC to stabilize (approx 10us)
	while (delay--);
}

void ADC_Start (void)
{
	ADC1->SR = 0;        // Clear the status register
	
	ADC1->CR2 |= (1<<30);  // Start the conversion by Setting the SWSTART bit in CR2
}


void DMA_Init (void)
{
	// Enable the DMA2 Clock
	RCC->AHB1ENR |= (1<<22);  // DMA2EN = 1
	
	// Channel 0 and Stream 0 is selected for ADC
	
	// Select the Data Direction
	DMA2_Stream0->CR &= ~(3<<6);  // Peripheral to memory
	
	// Select Circular mode
	DMA2_Stream0->CR |= (1<<8);  // CIRC = 1
	
	// Enable Memory Address Increment
	DMA2_Stream0->CR |= (1<<10);  // MINC = 1;

	// Set the size for data 
	DMA2_Stream0->CR |= (1<<11)|(1<<13);  // PSIZE = 01, MSIZE = 01, 16 bit data
	
	// Select channel for the stream
	DMA2_Stream0->CR &= ~(7<<25);  // Channel 0 selected
}


void DMA_Config (uint32_t srcAdd, uint32_t destAdd, uint16_t size)
{
	
	
	DMA2_Stream0->NDTR = size;   // Set the size of the transfer
	
	DMA2_Stream0->PAR = srcAdd;  // Source address is peripheral address
	
	DMA2_Stream0->M0AR = destAdd;  // Destination Address is memory address
	
	// Enable the DMA Stream
	DMA2_Stream0->CR |= (1<<0);  // EN =1
}

void DAC_Init(void) {
	RCC->AHB1ENR  |= ( RCC_AHB1ENR_GPIOAEN |
                     RCC_AHB1ENR_DMA1EN );
  RCC->APB1ENR  |= ( RCC_APB1ENR_DACEN |
                     RCC_APB1ENR_TIM6EN );
  // Pin A4 output type: Analog.
  GPIOA->MODER    &= ~( 0x3 << ( 4 * 2 ) );
  GPIOA->MODER    |=  ( 0x3 << ( 4 * 2 ) );
	
    // Enable the DAC clock
    //RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    // Configure PA4 (DAC_OUT1) as an analog output
    //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	
    //GPIOA->MODER |= GPIO_MODER_MODER4; // Set PA4 to analog mode
	
	TIM6->PSC  =  3;
  TIM6->ARR = 509;
	//TIM6->ARR  =  ( SystemCoreClock / ( 440 * SAMPLE_RATE ) );
  // Enable trigger output on timer update events.
  TIM6->CR2 &= ~( TIM_CR2_MMS );
  TIM6->CR2 |=  ( 0x2 << TIM_CR2_MMS_Pos );
  // Start the timer.
  TIM6->CR1 |=  ( TIM_CR1_CEN );
  // DAC configuration.
  // Set trigger sources to TIM6 TRGO.
  DAC1->CR  &= ~( DAC_CR_TSEL1 );
  // Enable DAC DMA requests.
  DAC1->CR  |=  ( DAC_CR_DMAEN1 );
  // Enable DAC Channels.
  DAC1->CR  |=  ( DAC_CR_EN1 );
  // Delay briefly to allow sampling to stabilize
  Delay_us( 1000 );  
  // Enable DAC channel trigger.
  DAC1->CR  |=  ( DAC_CR_TEN1 );
}



void DMA_Init_DAC(uint32_t srcAdd, uint32_t destAdd, uint16_t size) {
    // Enable the DMA1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // Configure the DMA stream for DAC
    DMA1_Stream5->CR &= ~DMA_SxCR_EN; // Disable DMA stream
    DMA1_Stream5->CR = 0; // Clear all previous settings
    DMA1_Stream5->PAR = srcAdd; // DAC data register address
    DMA1_Stream5->M0AR = destAdd; // Memory address of the data buffer
    DMA1_Stream5->NDTR = size; // Number of data to transfer
    DMA1_Stream5->CR |= DMA_SxCR_MINC; // Memory increment mode
    DMA1_Stream5->CR |= DMA_SxCR_DIR_0; // Memory-to-peripheral
    DMA1_Stream5->CR |= DMA_SxCR_CHSEL_0; // DAC channel 1 request
    DMA1_Stream5->CR |= DMA_SxCR_CIRC; // Circular mode
    DMA1_Stream5->CR |= DMA_SxCR_TCIE; // Transfer complete interrupt enable
    DMA1_Stream5->CR |= DMA_SxCR_HTIE; // Half-transfer interrupt enable
}


//uint16_t RxData[3];
//float Temperature;

void calSignal(){
	for(int i=0; i<AUDIO_BUFFER_SIZE; i++){
		audioBuffer[i] = ((sin(i*2*PI/100)+1)*(10000/2));
	}
}


int main ()
{
	SysClockConfig ();
	TIM6Config ();
	
	ADC_Init ();
	ADC_Enable ();
	DMA_Init ();
	
	//DMA_Config ((uint32_t ) &ADC1->DR, (uint32_t) audioBuffer, AUDIO_BUFFER_SIZE);
	calSignal();
	DMA_Init_DAC((uint32_t)(&DAC->DHR12R1),(uint32_t) audioBuffer, AUDIO_BUFFER_SIZE);
	DAC_Init();
	
	ADC_Start ();
	
	while (1)
	{
		
		//Temperature = (((float)(3.3*RxData[2]/(float)4095) - 0.76) / 0.0025) + 25;
		

		
	  Delay_ms (1000);
	}
	
}
