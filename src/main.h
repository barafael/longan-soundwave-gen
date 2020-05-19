/*!
    \file    main.h
    \brief   Soundwave Generator
*/

/*
    Copyright (c) 2020 Rafael Bachmann

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdbool.h>
#include <math.h>

#define I2C_BASE_ADDRESS (uint32_t)0x40

#define SET_SILENT_REG (uint32_t)0x33
#define SET_UNSILENT_REG (uint32_t)0x35
#define SET_PITCH_REG (uint32_t)0x55

#define TIMER5_PRESCALER (uint32_t)0x4
#define TIMER6_PRESCALER (uint32_t)0x4

#define DAC0_R8DH_ADDRESS (uint32_t)0x40007410
#define DAC1_R8DH_ADDRESS (uint32_t)0x4000741C

double sound_func(double x);
void sample(double array[], size_t n);
void sample_to_u8(const double input[], uint8_t output[], size_t n);

void    rcu_config(void);
void    gpio_config(void);
void    dma_config(uint32_t channel, uint32_t data_address, size_t n);
void    dac_config(void);
void    timer5_config(void);
void    timer6_config(void);
void    i2c_config(uint32_t address_bits);
