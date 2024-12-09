/*----------------------------------------------------------------------------
 * Name:    DAJP_F303K8_Driver.h
 * Purpose: Low-level drivers for my LCR meter based on the F303K8 Nucleo-32
 * Note(s): v0.0
 *----------------------------------------------------------------------------
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

#ifndef INC_DAJP_F303K8_DRIVER_H_
#define INC_DAJP_F303K8_DRIVER_H_

#include <stdbool.h> // Required for bool, true and false
#include "stdint.h"  // Required for uint32_t etc.
#include "stm32f303x8.h"  // For GPIO typedef

// Provides a delay of approximately delay microseconds
extern void LCR_MicroDelay (uint32_t delay);

// LED Definitions
extern void LCR_LED_Init (void);
extern void LCR_LED_On (unsigned int num);
extern void LCR_LED_Off (unsigned int num);
extern void LCR_LED_Toggle (unsigned int num);
extern unsigned int LCR_LED_GetState (unsigned int num);

// PWM setup
extern void LCR_PWM_Init(int freq, int dutyCycle);

// Rotary Encoder Definitions
extern void LCR_Rotary_Init (int initValue, int minValue, int maxValue, int dir);
extern int32_t LCR_Rotary_Read (void);
extern void LCR_Rotary_Set(int val);

// Switch Definitions
extern void LCR_Switch_Init (void);
extern unsigned int LCR_Switch_GetState (unsigned int);

// ADC Definitions
unsigned int LCR_GetADC_Reading(int channel);

// Serial Definitions
// extern void LCR_Serial_Init(uint32_t baud);
// extern int32_t LCR_Serial_Send(char c);
// extern int32_t LCR_Serial_Poll(void);

// Function Generator Definitions
extern void LCR_FuncGen_Init(uint32_t *data, uint32_t length, uint32_t rate);
extern void LCR_FuncGen_Update(uint32_t rate);
extern void LCR_FuncGen_Off();

// ADC Sampling Definitions
extern void LCR_ADC_Sample_Init();
extern void LCR_ADC_GetSamples(uint32_t *data, uint32_t length, uint32_t rate);

// LCD Definitions
void LCD_Set_Data(uint8_t data);
void LCD_Set_RS(uint8_t data);
void LCD_Set_RW(uint8_t data);
void LCD_Set_E(uint8_t data);
void LCD_Update(void);

extern void LCR_LCD_Init(void);
extern void LCR_LCD_Clear (void);
extern void LCR_LCD_GoToXY (int x, int y);
extern void LCR_LCD_WriteChar (char ch);
extern void LCR_LCD_WriteString (char *ch, int maxLength);
extern void LCR_LCD_DefineChar (int ch, char *data);

// Setting up the GPIO for the buttons, switches and ADC
extern void LCR_Init_Inputs(void);

#endif /* INC_DAJP_F303K8_DRIVER_H_ */
