/*-----------------------------------------------------------------------------
 * Name:    DAJP_F303K8_Driver.h
 * Purpose: Low-level drivers for the demo LCR meter using an F303K8 Nucleo-32
 * Note(s): v0.0
 *-----------------------------------------------------------------------------
 * Supplied under the terms of the MIT Open-Source License:
 *
 * Copyright (c) 2024 David A.J. Pearce
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *----------------------------------------------------------------------------*/

#include <main.h>
#include "drivers.h"
#include <stdbool.h>  // Required for bool, true and false

////////////////////////////////////////////////////////
// First, some general purpose helper routines:

// LCR_MicroDelay attempts to delay by the requested number of microseconds
// The factors were determined experimentally for the Nucleo-G071RB board
// running at 64 MHz with no compiler optimisations.
enum eLCD_OP { READ_INSTRUCTION, WRITE_INSTRUCTION, READ_DATA, WRITE_DATA };
uint8_t LCD_shift = 0;
void LCR_MicroDelay (uint32_t delayInMicroSeconds) {
  float compensation = (float)SystemCoreClock / (float)16e6;
  volatile unsigned long x = (unsigned long)(compensation * (36 * delayInMicroSeconds >> 4));
  while (x-- > 0);
}

void LCR_Set_As_Output(int bit, GPIO_TypeDef* port) {
    // Configures one bit in the GPIO port as an output suitable
    // for driving LED outputs (or any other general purpose output).
    // This involves setting the corresponding bits in:
    // MODER set to "01" (general purpose output)
    // OTYPER set to "0" (push-pull output)
    // OSPEEDR set to "10" (high-speed - perhaps not required)
    // PUPDR set to "00" (no pull-up or pull-down)
	  // It also sets the output low.
	port->BSRR = 1UL < (16 + bit);
	unsigned long bitMask = ~(3UL << 2*bit);
	port->MODER = (port->MODER & bitMask) | (1UL << 2*bit);
	port->OTYPER &= ~(1UL << bit);
	port->OSPEEDR = (port->OSPEEDR & bitMask) | (2UL << 2*bit);
    port->PUPDR = (port->PUPDR & bitMask) | (0UL << 2*bit);
}

enum eTermType { PULLUP = 1, PULLDOWN = 2, PULLBOTH = 3, NOPULL = 0 };
void LCR_Set_As_Input(int bit, GPIO_TypeDef* port, enum eTermType eTT) {
  // Configures one bit in the GPIO port as an input suitable
	// for reading from.  Note, this includes a pull-up resistor
	// for compatibility with open-drain outputs.
	// This involves setting the corresponding bits in:
	// MODER set to "00" (general purpose input)
	// PUPDR should be set to NOPULL ("00") by default
	unsigned long bitMask = ~(3UL << 2*bit);
	port->MODER &= bitMask;
	port->PUPDR = (port->PUPDR & bitMask) | ((unsigned int)eTT << 2*bit);
}

//////////////////////////////////////////////////////
// PWM driver.  Sets up PB4 as a PWM output with the
// given frequency and duty cycle.  Uses TIM16.
void LCR_PWM_Init(int freq, int dutyCycle) {
	  GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER4) | 0x2 << GPIO_MODER_MODER4_Pos;
	  GPIOB->AFR[0] = (GPIOB->AFR[0] & 0xfff0ffff) | 0x00010000;
	  RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;

	  // Set-up TIM16 to time the ADC conversions:
	  uint32_t DivRatio = SystemCoreClock / freq;
	  uint32_t ARRvalue = DivRatio;
	  uint32_t PSCvalue = 0;
	  while (ARRvalue > 65535) {
	  	PSCvalue++;
	   	ARRvalue = DivRatio / (PSCvalue + 1);
	  }
	  TIM16->PSC = PSCvalue;
	  TIM16->ARR = ARRvalue - 1;
	  TIM16->CCR1 = (uint32_t)(ARRvalue * dutyCycle / 100);

	  TIM16->CCMR1 &= ~TIM_CCMR1_OC1M;  // Set PWM mode 1
	  TIM16->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	  TIM16->CCER |= TIM_CCER_CC1E;	   // Enable capture mode
	  TIM16->EGR |= TIM_EGR_UG;		   // Update registers
	  TIM16->BDTR |= TIM_BDTR_MOE;     // Main output enable on
	  TIM16->CR1 |= TIM_CR1_CEN;	   // Enable timer
}

//////////////////////////////////////////////////////
// Rotary encoder using timer set-up.  This assumes
// the quadrature outputs from the shaft encoder are
// on PA6 and PA7, and TIM3 is used as the counter.
int RotaryMin = 0;
int RotaryMax = 100;
int RotaryDir = 0;  // Clockwise if count increases
void LCR_Rotary_Init (int initValue, int minValue, int maxValue, int dir) {
	// Switch power on for general-purpose timer TIM3, and for GPIOA
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;

    // Configure PA6 and PA7 as TIM3 inputs.  On the F303, this is AF2 (it's AF1 on the G071)
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~0xff000000) | 0x22000000;
    GPIOA->MODER = (GPIOA->MODER & ~0xf000) | 0xa000;

    // Enable the timer first to allow configuration:
    TIM3->CR1 |= TIM_CR1_CEN;

    // Configure the timer in rotary encoder mode:
    TIM3->CCMR1 = (TIM3->CCMR1 & ~TIM_CCMR1_CC1S) | (0x1 << TIM_CCMR1_CC1S_Pos);
    TIM3->CCMR1 = (TIM3->CCMR1 & ~TIM_CCMR1_CC2S) | (0x1 << TIM_CCMR1_CC2S_Pos);
    TIM3->CCER = TIM3->CCER & ~TIM_CCER_CC1P;
    TIM3->CCER = TIM3->CCER & ~TIM_CCER_CC1NP;
    TIM3->CCER = TIM3->CCER & ~TIM_CCER_CC2P;
    TIM3->CCER = TIM3->CCER & ~TIM_CCER_CC2NP;

    // Use channel two edges only:
    TIM3->SMCR = (TIM3->SMCR & ~TIM_SMCR_SMS) | (0x1 << TIM_SMCR_SMS_Pos);

    // Depending on how the board is wired up, pulses will increment the counter
    // on clockwise or anti-clockwise rotations of the encoder.  The hardware can
    // only go one way, so I need to compensate for this in software, but noting
    // which way the encoder is wired at this stage:
    RotaryDir = dir;

    // Set initial value, and min and max values for correcting overruns.  (For
    // RotaryDir = 1, the count will be kept as negative values, and inverted
    // back on reading.
    if (dir) {
        RotaryMin = -1 * maxValue;
        RotaryMax = -1 * minValue;
        initValue *= -1;
    }
    else {
        RotaryMin = minValue;
        RotaryMax = maxValue;
    }
    TIM3->CNT = 0x8000 + initValue * 2;
    TIM3->ARR = 0xffff;
}
int32_t LCR_Rotary_Read (void) {
	if (TIM3->CNT < 0x8000 + RotaryMin) {
		TIM3->CNT = 0x8000 + RotaryMin;
	}
	if (TIM3->CNT > 0x8000 + RotaryMax) {
		TIM3->CNT = 0x8000 + RotaryMax;
	}

	int32_t newCount = ((int)TIM3->CNT - 0x8000) / 2;
	if (RotaryDir) return -1 * newCount;
	else return newCount;
}
void LCR_Rotary_Set(int val) {
	TIM3->CNT = 0x8000 + val * (RotaryDir ? -2 : 2);
}

//////////////////////////////////////////////////////
// LED Driver functions start here.
// LED zero (red) is on PA5, LED one (green) on PB3
// Note that PB3 is also the LED on the Nucleo board itself
GPIO_TypeDef* const LED_Ports[] = {GPIOA, GPIOB};
const uint32_t LED_Pins[] = {5, 3};

void LCR_LED_Init () {
	for (uint32_t u = 0; u < 2; u++) {
		LCR_Set_As_Output(LED_Pins[u], LED_Ports[u]);
	}
}
void LCR_LED_On (unsigned int num) {
	if (num < 2) LED_Ports[num]->BSRR = 1UL << LED_Pins[num];
}
void LCR_LED_Off (unsigned int num) {
	if (num < 2) LED_Ports[num]->BSRR = 1UL << (16 + LED_Pins[num]);
}
void LCR_LED_Toggle (unsigned int num) {
	if (num < 2) {
		unsigned int state = LCR_LED_GetState(num);
		if (state == 0) LCR_LED_On(num);
		else LCR_LED_Off(num);
	}
}
unsigned int LCR_LED_GetState (unsigned int num) {
	if (num < 2) {
		return (LED_Ports[num]->ODR & (1UL << LED_Pins[num])) ? 1 : 0;
	}
	return 0;
}

////////////////////////////////////////////////////////
// Switches and other input driver functions start here:
//
// SW0 is the main push button (on PA3)
// SW1 is the top DIP switch (on PA11)
// SW2 is the bottom DIP switch (on PB5)

void LCR_Init_Inputs(void) {
	// Sets up the switches and the analogue inputs for use by the
	// ADCs to digitise the waveforms:
	LCR_Switch_Init();

	// Set PA0 and PA1 as analogue inputs with no pulls:
	GPIOA->MODER |= 0x0f;
	GPIOA->PUPDR &= ~0x0f;

	// Enable the clock to the ADC:
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;

	// Set the ADC to do a single conversion from channel #4 (PA0)
	ADC1->CFGR = 0x00;  // Defaults are fine
	ADC1->SMPR1 = 0x04 << ADC_SMPR1_SMP1_Pos;   // Use 19.5 clock cycles for the sampling time
	ADC1->SQR1 = (0x04 << ADC_SQR1_SQ1_Pos) | 0x01;  // XXX Select channel #4 only
}
void LCR_Switch_Init(void) {
	// Sets up the IO associated with the controls as GPIO ports.
	// Enable clocks to GPIOA and GPIOB:
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

	// Set PA11, PB5 and PA3 as digital inputs with pull-up resistors:
	LCR_Set_As_Input(11, GPIOA, PULLUP);
	LCR_Set_As_Input(5, GPIOB, PULLUP);
	LCR_Set_As_Input(3, GPIOA, PULLUP);
}
unsigned int LCR_Switch_GetState (unsigned int which) {
	if (which == 0) return GPIOA->IDR & (1UL << 3) ? 1 : 0;
	else if (which == 1) return GPIOA->IDR & (1UL << 11) ? 1 : 0;
	else if (which == 2) return GPIOB->IDR & (1UL << 5) ? 1 : 0;
	else return 0;
}

///////////////////////////////////////////////////////////////////////////////
// LCD driver functions start here:
// I'll slow all the accesses down a bit, since I don't think the LCD
// controller will be able to keep up with the ARM-Cortex.  Experimentally,
// a delay of around XX us seems to be enough, and it makes writing to the
// LCD so fast that you can't see the individual characters appear.  It's
// also less likely to be interrupted by a reset in the middle of an operation,
// and this can upset it (I'm still not entirely clear why this happens, or
// how to kick it out of whatever random state it gets into at these times).
#define LCD_DELAY_CONST 500

void LCD_Set_Data(uint8_t data) {
	// This takes the lowest four bits in data and puts them on the
	// relevant pins of the LCD (LCD_D4 to LCD_D7) in four-bit mode.
	GPIOA->BSRR = (data & 0x1) ? 0x1 << 4 : 0x1 << 20;
	GPIOA->BSRR = (data & 0x2) ? 0x1 << 5 : 0x1 << 21;
	GPIOA->BSRR = (data & 0x4) ? 0x1 << 6 : 0x1 << 22;
	GPIOA->BSRR = (data & 0x8) ? 0x1 << 7 : 0x1 << 23;
}
void LCD_Set_RS(uint8_t data) {
	// Sets the RS control line to either high or low:
	GPIOA->BSRR = data ? 0x1 << 0 : 0x1 << 16;
}
void LCD_Set_RW(uint8_t data) {
	// Sets the RW control line to either high or low:
	GPIOF->BSRR = data ? 0x1 << 1 : 0x1 << 17;
}
void LCD_Set_E(uint8_t data) {
	// Sets the RW control line to either high or low:
	GPIOA->BSRR = data ? 0x1 << 1 : 0x1 << 17;
}
uint8_t LCR_LCD_IsBusy () {
  // For now, I'll just use the delay version of this.  Wait for a ms:
  LCR_MicroDelay(1000);
  return 0;
}
void LCR_LCD_Write (enum eLCD_OP op, uint8_t data) {
  // Writes a byte to the LCD.  This assumes four-bit mode.
  if (op == WRITE_DATA) LCD_Set_RS(1);
  else if (op == WRITE_INSTRUCTION) LCD_Set_RS(0);
  else return;

  unsigned int toWrite_High = (data >> 4) & 0x0f;
  unsigned int toWrite_Low = data & 0x0f;
  LCD_Set_Data(toWrite_High);
  LCR_MicroDelay(LCD_DELAY_CONST); LCD_Set_E(1);
  LCR_MicroDelay(LCD_DELAY_CONST); LCD_Set_E(0);
  LCD_Set_Data(toWrite_Low);
  LCR_MicroDelay(LCD_DELAY_CONST); LCD_Set_E(1);
  LCR_MicroDelay(LCD_DELAY_CONST); LCD_Set_E(0);
}
void LCR_LCD_Init (void) {
  // The LCD uses GPIOs A, B and F, so these clocks are required:
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOFEN;

  // Relevant control GPIO pins are PA8 (RS), PF1 (RW), PF0 (E)
  LCR_Set_As_Output(8, GPIOA);
  LCR_Set_As_Output(1, GPIOF);
  LCR_Set_As_Output(0, GPIOF);

  // And for data, GPIOB 1, 6, 7 and 0
  LCR_Set_As_Output(0, GPIOB);
  LCR_Set_As_Output(1, GPIOB);
  LCR_Set_As_Output(6, GPIOB);
  LCR_Set_As_Output(7, GPIOB);

  // The three writes in the following block are a workaround for
  // an interesting problem.  If this is a power on, then the LCD
  // will have powered up in the default eight-bit mode.  In this
  // case, all three writes will be treated as eight-bit writes,
  // confirming the eight-bit mode.  However if this was a reset
  // of the processor, then the LCD won't have been reset, and
  // will already be in four-bit mode.  So you can't just start
  // writing assuming that the LCD was in eight-bit mode.
  //
  // If it was in four-bit mode, then the first two of these
  // writes will be treated as one combination eight-bit write,
  // setting the LCD back into eight-bit mode.  The third write
  // is then an eight-bit write confirming this.  Once the LCD is
  // in the known eight-bit mode, it can be put into four-bit
  // mode unambiguously.
  //
  // You can't wait for the LCD to say it's ready yet, as the
  // instruction set has to be chosen first (see datasheet).
  // So these operations have to go slowly:

  LCR_MicroDelay(15000);
  LCD_Set_RS(0); // Set LCD_RS low
  LCD_Set_RW(0); // Set LCD_RW low
  LCD_Set_Data(3);  // Set the LCD_D4-D7 to 0b0011
  LCR_MicroDelay(5000); LCD_Set_E(1); // Set LCD_E high
  LCR_MicroDelay(5000); LCD_Set_E(0); // Set LCD_E low
  LCR_MicroDelay(5000); LCD_Set_E(1); // Set LCD_E high
  LCR_MicroDelay(5000); LCD_Set_E(0); // Set LCD_E low
  LCR_MicroDelay(5000); LCD_Set_E(1); // Set LCD_E high
  LCR_MicroDelay(5000); LCD_Set_E(0); // Set LCD_E low

  // Now LCD should be in eight-bit mode no matter where it started
  // from, so we can set the LCD to four-bit mode unambiguously with
  // one write cycle:
  LCD_Set_Data(2);  // Set the LCD_D4-D7 to 0b0010
  LCR_MicroDelay(5000); LCD_Set_E(1); // Set LCD_E high
  LCR_MicroDelay(5000); LCD_Set_E(0); // Set LCD_E low

  // Now can set the other control bits: two-line and 5*8 pixels
  while (LCR_LCD_IsBusy());
  LCR_LCD_Write(WRITE_INSTRUCTION, 0x28);

  // Display ON/OFF Control: ON, no cursor
  while (LCR_LCD_IsBusy());
  LCR_LCD_Write(WRITE_INSTRUCTION, 0x0c);

  // Clear the display
  while (LCR_LCD_IsBusy());
  LCR_LCD_Write(WRITE_INSTRUCTION, 0x01);

  // Entry Mode Set: increment address (move right)
  while (LCR_LCD_IsBusy());
  LCR_LCD_Write(WRITE_INSTRUCTION, 0x06);
}
void LCR_LCD_Clear (void) {
  while (LCR_LCD_IsBusy());
  LCR_LCD_Write(WRITE_INSTRUCTION, 0x01);
}
void LCR_LCD_GoToXY (int x, int y) {
  while (LCR_LCD_IsBusy());
  if( y == 0 ) {
    LCR_LCD_Write(WRITE_INSTRUCTION, 0x80 | (x & 0x3F));
  }
  else if( y == 1 ) {
    LCR_LCD_Write(WRITE_INSTRUCTION, 0xC0 | (x & 0x3F));
  }
}
void LCR_LCD_WriteChar (char ch) {
  // Write a character to the data register on the LCD:
  while (LCR_LCD_IsBusy());
  LCR_LCD_Write(WRITE_DATA, ch);
}
void LCR_LCD_WriteString (char *s, int maxLength) {
  while(*s && maxLength-- > 0) {
	// LCR_MicroDelay(10000); // This works, but
    while (LCR_LCD_IsBusy()){} // This does not
    LCR_LCD_Write(WRITE_DATA, *s++);
  }
}
void LCR_LCD_DefineChar (int ch, char *data) {
	// Defines the character (ch can be 0 - 7 inclusive) to have the
	// pattern defined in the first eight entries in the data array.
	if (ch < 0 || ch > 7) return;
	LCR_LCD_Write(WRITE_INSTRUCTION, 0x40 + ch * 8);
	for (int u = 0; u < 8; u++) {
		LCR_LCD_Write(WRITE_DATA, data[u]);
	}
	LCR_LCD_Write(WRITE_INSTRUCTION, 0x80);
}

////////////////////////////////////////////////////////////////////////////
// ADC Sampling Functions.  These allow the ADC to take a series of
// regularly spaced samples of the voltage on CH1 and CH2 and store
// them in an array.  Uses ADC1 and TIM2.
void LCR_ADC_Sample_Init() {
	// Sets up ADC1 on channel one to take a series of regularly-spaced samples
	// of the voltages on channels one and two for further processing.

	// Enable the clock to DMA1 and the ADC:
	RCC->AHBENR |= RCC_AHBENR_DMA1EN | RCC_AHBENR_ADC12EN;

	// Enable the clock to TIM2 and TIM15:
	// RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; // XXX
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	// First stop any new triggers coming in by disabling TIM2 (in case this
	// is not the first time this routine has been called).
	TIM2->CR1 &= ~TIM_CR1_CEN;

    // If the ADC is enabled, then disable it:
    if (ADC1->ISR & ADC_ISR_ADRDY) {
        ADC1->CR |= 0x02;
        while (ADC1->CR & 0x01) {
        	LCR_MicroDelay(100);
        }
    }

    // Make sure the voltage reference to the ADC is turned on:
    if ((ADC1->CR & 0x30000000) != 0x10000000) {
        ADC1->CR &= 0xCFFFFFFF; // It's a two stage process
        ADC1->CR |= 0x10000000; // Set field to 00, then 01
        LCR_MicroDelay(100);    // There's no ready flag to wait for
    }

    // Set overrun mode, 12-bit resolution, right-aligned output, and for the
    // ADC to trigger on a rising edge of the TIM2 TRGO event:
    ADC1->CFGR = 0x00001781; // For TIM15 TRGO on F303K8
    ADC1->CFGR = 0x000016C1; // For TIM2 TRGO on F303K8

    // Set up to take two samples on each trigger (channel one and channel two):
	ADC1->SQR1 = (0x1 << ADC_SQR1_SQ1_Pos) + (0x2 << ADC_SQR1_SQ2_Pos) + 1;

    // Set settling time to 19.5 clock cycles (305 ns), just OK for 1 MHz sampling rate?
    ADC1->SMPR1 = 0x0120; // This is 19.5 cycles;
    // ADC1->SMPR1 = 0x0090; // This is 4.5 cycles;
    // ADC1->SMPR1 = 0x0048; // This is 2.5 cycles;

    // Should now be able to enable the ADC:
    ADC1->CR |= ADC_CR_ADEN; // Enable the ADC...
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {   // ...and wait for it to be ready
        HAL_Delay(10); // Was 100 in the original
    }
}
void LCR_ADC_GetSamples(uint32_t *data, uint32_t length, uint32_t rate) {
	// Takes a set of length samples of the ADC readings at rate samples
	// per second, and puts them in the array pointed to by data.  Note that
	// two samples are returned for each trigger, so the data array needs to
	// have a size of at least length * 2.

    // Set up DMA1 channel one for the ADC capture (ADC is always on DMA Channel one):
    DMA1_Channel1->CCR &= ~0x01;    // Disable the DMA channel
    DMA1_Channel1->CPAR = (uint32_t) &ADC1->DR;  // Set peripheral address (source of data)
    DMA1_Channel1->CMAR = (uint32_t) data;       // Set memory address (destination of data)
    DMA1_Channel1->CNDTR = length * 2;           // Number of transfers
    DMA1_Channel1->CCR = 0x1A80;    // Set priority, increments, circular mode, bit-widths, etc
    DMA1_Channel1->CCR |= 0x01;     // Enable the DMA channel

    // Set-up TIM2 to time the ADC conversions:
    uint32_t DivRatio = SystemCoreClock / rate;
    uint32_t ARRvalue = DivRatio;
    uint32_t PSCvalue = 0;
    while (ARRvalue > 65535) {
    	PSCvalue++;
    	ARRvalue = DivRatio / (PSCvalue + 1);
    }
    TIM2->PSC = PSCvalue;
    TIM2->ARR = ARRvalue - 1;
    TIM2->CR2 |= TIM_CR2_MMS_1;  // Set TRGO on update event
    TIM2->EGR |= TIM_EGR_UG;     // Update the counter with new values

    ADC1->CR |= ADC_CR_ADSTART;  // Tell ADC to wait for triggers from timer
    TIM2->CR1 |= TIM_CR1_CEN;    // Set the timer going

    // Now wait for the DMA to complete before returning:
    while (DMA1_Channel1->CNDTR > 0) {
    	LCR_MicroDelay(1000);
    }
    TIM2->CR1 &= ~TIM_CR1_CEN;    // Stop the timer
}

////////////////////////////////////////////////////////////////////////////
// Function Generator Functions
static int funcGenDataLength = 0;
void LCR_FuncGen_Init(uint32_t *data, uint32_t length, uint32_t rate) {
	// Starts the DAC and outputs from DAC channel one a series of
	// samples in the data array (length samples long), output at a
	// sample rate of HCLK / rate.
	//
	// Note that this function requires the use of TIM6 and Channel 3 of
	// the DMA controller, as well as DAC channel one.

	// Enable the clock to GPIOA and the DMA controller:
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_DMA1EN;

	// Enable the clock to DAC1 and TIM6:
    RCC->APB1ENR |= RCC_APB1ENR_DAC1EN | RCC_APB1ENR_TIM6EN;

    // Set GPIO pin for PA4 to analogue mode:
    GPIOA->MODER |= GPIO_MODER_MODER4;

    // Set PUPDR pins for PA0, PA1 and PA4 to no pulls:
    GPIOA->PUPDR &= 0xFFFFFCF0;

    // For timer six, set pre-scale to x1 and ARR to rate desired:
    // First disable the timer:
    TIM6->CR1 &= ~ TIM_CR1_CEN;

    // Store the length of the data in case frequency needs to be changed:
    funcGenDataLength = length;

    // Now set-up TIM6 to time the DAC update rate, noting that
    // ARR is a 16-bit field, so large divisions will have to
    // use the pre-scaler PSC as well:
    uint32_t DivRatio = SystemCoreClock / rate / length;
    uint32_t ARRvalue = DivRatio;
    uint32_t PSCvalue = 0;
    while (ARRvalue > 65535) {
    	PSCvalue++;
    	ARRvalue = DivRatio / (PSCvalue + 1);
    }
    TIM6->PSC = PSCvalue;
    TIM6->ARR = ARRvalue - 1;
    // Enable trigger output to trigger the DAC:
    TIM6->CR2 &=  ~TIM_CR2_MMS;
    TIM6->CR2 |=  0x2 << TIM_CR2_MMS_Pos;

    // Remap the TIM6 up event to DMA1 Channel 3 in the SYSCFG block:
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP;

    // Setup DMA1 channel three for the DAC output:
    // First disable the DMA channel so it can be updated:
    DMA1_Channel3->CCR &= ~0x01;
    // Set destination address as DAC register DHR12R1:
    DMA1_Channel3->CPAR = (uint32_t) &DAC1->DHR12R1;
    // Set the source address as the sample buffer:
    DMA1_Channel3->CMAR = (uint32_t) &data[0];
    // Set the number of transfers:
    DMA1_Channel3->CNDTR = length;
    // Set bit widths to 32-bit, memory increments, direction and circular mode:
    DMA1_Channel3->CCR = 0x09b0;
    // Enable the DMA channel:
    DMA1_Channel3->CCR |= 0x01;

    // For channel one of the DAC:
    // Disable DAC channel one and two for now:
    DAC1->CR &= ~DAC_CR_EN2 & ~DAC_CR_EN1;
    // Enable DAC DMA requests so DAC asks for data from the DMA:
    DAC1->CR  |=  DAC_CR_DMAEN1;
    // Set the trigger source to TIM6 TRGO (trigger number 0):
    DAC1->CR  &= ~( DAC_CR_TSEL1);
    // Set output to buffered GPIO 'normal mode':
    DAC1->CR &= ~DAC_CR_BOFF1;
    // Finally enable the DAC channel one:
    DAC1->CR  |=  DAC_CR_EN1;

    // Now enable TIM6 and the DAC channel trigger:
    TIM6->CR1 |= TIM_CR1_CEN;
    DAC1->CR |= DAC_CR_TEN1;
}
void LCR_FuncGen_Off(){
    // This disables the trigger input to the DAC (allowing it to be directly
	// written to), and disables the timer, leaving the rest of the DAC and
	// the DMA alone.  This effectively stops the function generator:
    TIM6->CR1 &= ~ TIM_CR1_CEN;
    DAC1->CR &= ~DAC_CR_TEN1;
}
void LCR_FuncGen_Update(uint32_t rate) {
    // This just modifies the timer, leaving the DAC and DMA alone.
    // First disable the timer:
    TIM6->CR1 &= ~ TIM_CR1_CEN;

    // Now set-up TIM6 to time the DAC update rate, noting that
    // ARR is a 16-bit field, so large divisions will have to
    // use the pre-scaler PSC as well:
    uint32_t DivRatio = SystemCoreClock / rate / funcGenDataLength;
    uint32_t ARRvalue = DivRatio;
    uint32_t PSCvalue = 0;
    while (ARRvalue > 65535) {
    	PSCvalue++;
    	ARRvalue = DivRatio / (PSCvalue + 1);
    }
    TIM6->PSC = PSCvalue;
    TIM6->ARR = ARRvalue - 1;

    // Enable trigger output to trigger the DAC:
    TIM6->CR2 &=  ~TIM_CR2_MMS;
    TIM6->CR2 |=  0x2 << TIM_CR2_MMS_Pos;

    // And re-enable TIM6:
    TIM6->CR1 |= TIM_CR1_CEN;
}
