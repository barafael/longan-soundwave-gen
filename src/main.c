/*!
    \file    main.c
    \brief   Soundwave Generator
*/

/*
    Copyright (c) 2020 Rafael Bachmann

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
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

#include "gd32vf103.h"
#include "systick.h"

#include "main.h"

uint8_t i2c0_receiver[32];
uint8_t i2c1_receiver[32];

uint8_t muted = false;

double sound_func(double x) {
    return sin(x);
}

void sample(double array[], size_t n) {
    for (size_t i = 0; i < n; i++) {
        double factor = (double)i / (double)n;
        array[i] = sound_func((factor * (2.0 * M_PI) - M_PI));
    }
}

void sample_to_u8(const double input[], uint8_t output[], size_t n) {
    for (size_t index = 0; index < n; index++) {
        output[index] = 127 + (128.0 * input[index]);
    }
}

void resample(double array[], uint8_t output[], size_t n) {
    sample(array, n);
    sample_to_u8(array, output, n);
}

void    rcu_config(void);
void    gpio_config(void);
void    dma_config(uint32_t channel, uint32_t data_address, size_t n);
void    dac_config(void);
void    timer5_config(void);
void    timer6_config(void);
uint8_t get_i2c_address_bits(void);
void    i2c_config(uint32_t address_bits);

#define BUFFER_SIZE 2048
double  signal[BUFFER_SIZE];
uint8_t soundwave1[BUFFER_SIZE];
uint8_t soundwave2[BUFFER_SIZE];

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void) {

    sample(signal, BUFFER_SIZE);
    sample_to_u8(signal, soundwave1, BUFFER_SIZE);

    sample(signal, BUFFER_SIZE);
    sample_to_u8(signal, soundwave2, BUFFER_SIZE);

    rcu_config();
    gpio_config();
    delay_1ms(100);
    uint32_t bits = get_i2c_address_bits();
    i2c_config(bits);
    dma_config(DMA_CH2, (uint32_t)soundwave1, BUFFER_SIZE);
    dma_config(DMA_CH3, (uint32_t)soundwave2, BUFFER_SIZE);
    dac_config();
    timer5_config();
    timer6_config();

    /*while (1) {
        uint32_t bits = get_i2c_address_bits();
        //while(gpio_input_bit_get(GPIOB, GPIO_PIN_15) == RESET);
        for (size_t index = 0; index < bits; index++) {
            gpio_bit_reset(GPIOC, GPIO_PIN_13);
            delay_1ms(400);
            gpio_bit_set(GPIOC, GPIO_PIN_13);
            delay_1ms(400);
        }
        delay_1ms(1000);
    }*/

    while (1) {
        /* check if ADDSEND bit is set */
        if (i2c_flag_get(I2C0, I2C_FLAG_ADDSEND)) {
            /* clear ADDSEND bit */
            i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
            int i = 0;
            while (1) {
                if (i2c_flag_get(I2C0, I2C_FLAG_RBNE)) {
                    /* read a data byte from I2C_DATA */
                    i2c0_receiver[i] = i2c_data_receive(I2C0);
                    i++;
                } else if (i2c_flag_get(I2C0, I2C_FLAG_STPDET)) {
                    i2c_enable(I2C0);
                    /* stop detected, now handle command */
                    switch (i) {
                        case 1: {
                            if (i2c0_receiver[0] == TOGGLE_MUTED_REG) {
                                if (muted) {
                                    timer_update_event_enable(TIMER5);
                                } else {
                                    timer_update_event_disable(TIMER5);
                                }
                                muted = !muted;
                            }
                        } break;
                        case 3: {
                            if (i2c0_receiver[0] == SET_PITCH_REG) {
                                uint32_t freq = i2c0_receiver[1] | (uint16_t) i2c0_receiver[2] << 8;
                                if (freq < BUFFER_SIZE) {
                                    resample(signal, soundwave1, freq);
                                    dma_config(DMA_CH2, (uint32_t)soundwave1, freq);
                                }
                            }
                        } break;
                    }
                    i = 0;
                    break;
                }
            }
        }

        /* wait until ADDSEND bit is set */
        if (i2c_flag_get(I2C1, I2C_FLAG_ADDSEND)) {
            /* clear ADDSEND bit */
            i2c_flag_clear(I2C1, I2C_FLAG_ADDSEND);
            int j = 0;
            while (1) {
                if (i2c_flag_get(I2C1, I2C_FLAG_RBNE)) {
                    /* read a data byte from I2C_DATA */
                    i2c1_receiver[j] = i2c_data_receive(I2C1);
                    j++;
                } else if (i2c_flag_get(I2C1, I2C_FLAG_STPDET)) {
                    i2c_enable(I2C1);
                    /* stop detected, now handle command */
                    switch (j) {
                        case 1: {
                            if (i2c1_receiver[0] == TOGGLE_MUTED_REG) {
                                if (muted) {
                                    timer_update_event_enable(TIMER6);
                                } else {
                                    timer_update_event_disable(TIMER6);
                                }
                                muted = !muted;
                            }
                        } break;
                        case 3: {
                            if (i2c1_receiver[0] == SET_PITCH_REG) {
                                uint32_t freq = i2c1_receiver[1] | (uint16_t) i2c1_receiver[2] << 8;
                                if (freq < BUFFER_SIZE) {
                                    resample(signal, soundwave2, freq);
                                    dma_config(DMA_CH3, (uint32_t)soundwave2, freq);
                                }
                            }
                        } break;
                    }
                    j = 0;
                    break;
                }
            }
        }
    }
}

/*!
    \brief      configure the RCU of peripherals
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void) {
    /* enable the clock of peripherals */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_AF);

    rcu_periph_clock_enable(RCU_I2C0);
    rcu_periph_clock_enable(RCU_I2C1);
    rcu_periph_clock_enable(RCU_DMA1);
    rcu_periph_clock_enable(RCU_DAC);
    rcu_periph_clock_enable(RCU_TIMER5);
    rcu_periph_clock_enable(RCU_TIMER6);
}

/*!
    \brief      configure the related GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void) {
    /* once enabled the DAC, the corresponding GPIO pin is connected to the DAC converter automatically */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_5);

    /* PB12, PB13, PB14 as input pins */
    gpio_init(GPIOB, GPIO_MODE_IPD, GPIO_OSPEED_2MHZ, GPIO_PIN_12);
    gpio_init(GPIOB, GPIO_MODE_IPD, GPIO_OSPEED_2MHZ, GPIO_PIN_13);
    gpio_init(GPIOB, GPIO_MODE_IPD, GPIO_OSPEED_2MHZ, GPIO_PIN_14);

    /* User Button on PB15 */
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_15);

    /* connect PB6 to I2C0_SCL */
    /* connect PB7 to I2C0_SDA */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);

    /* connect PB10 to I2C1_SCL */
    /* connect PB11 to I2C1_SDA */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_10 | GPIO_PIN_11);

    /* LED pin PC13 */
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_bit_reset(GPIOC, GPIO_PIN_13);
}

/*!
    \brief      configure the I2C0 interfaces
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_config(uint32_t address_bits) {
    uint32_t i2c_address_0 = I2C_BASE_ADDRESS + address_bits * 2;
    uint32_t i2c_address_1 = I2C_BASE_ADDRESS + address_bits * 2 + 16;

    /* I2C clock configure */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
    i2c_clock_config(I2C1, 100000, I2C_DTCY_2);

    /* I2C address configure */
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, i2c_address_0);
    i2c_mode_addr_config(I2C1, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, i2c_address_1);

    /* enable I2C */
    i2c_enable(I2C0);
    i2c_enable(I2C1);

    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
    i2c_ack_config(I2C1, I2C_ACK_ENABLE);
}

/*!
    \brief      configure the DMA
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dma_config(uint32_t DMA_CHANNEL, uint32_t data_address, size_t n) {
    if (DMA_CHANNEL != DMA_CH2 && DMA_CHANNEL != DMA_CH3) {
        return;
    }

    /* clear all the interrupt flags */
    dma_flag_clear(DMA1, DMA_CHANNEL, DMA_INTF_GIF);
    dma_flag_clear(DMA1, DMA_CHANNEL, DMA_INTF_FTFIF);
    dma_flag_clear(DMA1, DMA_CHANNEL, DMA_INTF_HTFIF);
    dma_flag_clear(DMA1, DMA_CHANNEL, DMA_INTF_ERRIF);

    dma_deinit(DMA1, DMA_CHANNEL);

    static dma_parameter_struct dma_config_struct = {
        .periph_addr  = 0,
        .periph_width = DMA_PERIPHERAL_WIDTH_8BIT,
        .memory_addr  = 0,
        .memory_width = DMA_MEMORY_WIDTH_8BIT,
        .number       = 0,
        .priority     = DMA_PRIORITY_ULTRA_HIGH,
        .periph_inc   = DMA_PERIPH_INCREASE_DISABLE,
        .memory_inc   = DMA_MEMORY_INCREASE_ENABLE,
        .direction    = DMA_MEMORY_TO_PERIPHERAL,
    };

    dma_config_struct.periph_addr = DMA_CHANNEL == DMA_CH2 ? DAC0_R8DH_ADDRESS : DAC1_R8DH_ADDRESS;
    dma_config_struct.memory_addr = data_address;
    dma_config_struct.number = n;

    /* configure the DMA */
    dma_init(DMA1, DMA_CHANNEL, &dma_config_struct);

    dma_circulation_enable(DMA1, DMA_CHANNEL);
    dma_channel_enable(DMA1, DMA_CHANNEL);
}

/*!
    \brief      configure the DAC
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dac_config(void) {
    dac_deinit();
    /* configure the DAC0 */
    dac_trigger_source_config(DAC0, DAC_TRIGGER_T5_TRGO);
    dac_trigger_enable(DAC0);
    dac_wave_mode_config(DAC0, DAC_WAVE_DISABLE);
    dac_output_buffer_disable(DAC0);

    /* enable DAC0 and DMA for DAC0 */
    dac_enable(DAC0);
    dac_dma_enable(DAC0);

    /* configure the DAC1 */
    dac_trigger_source_config(DAC1, DAC_TRIGGER_T6_TRGO);
    dac_trigger_enable(DAC1);
    dac_wave_mode_config(DAC1, DAC_WAVE_DISABLE);
    dac_output_buffer_disable(DAC1);

    /* enable DAC1 and DMA for DAC1 */
    dac_enable(DAC1);
    dac_dma_enable(DAC1);
}

/*!
    \brief      configure the TIMER5
    \param[in]  none
    \param[out] none
    \retval     none
*/
void timer5_config(void) {
    /* configure the TIMER5 */
    timer_prescaler_config(TIMER5, TIMER5_PRESCALER, TIMER_PSC_RELOAD_UPDATE);
    timer_autoreload_value_config(TIMER5, 0xFF);
    timer_master_output_trigger_source_select(TIMER5, TIMER_TRI_OUT_SRC_UPDATE);

    timer_enable(TIMER5);
}

/*!
    \brief      configure the TIMER6
    \param[in]  none
    \param[out] none
    \retval     none
*/
void timer6_config(void) {
    /* configure the TIMER6 */
    timer_prescaler_config(TIMER6, TIMER6_PRESCALER, TIMER_PSC_RELOAD_UPDATE);
    timer_autoreload_value_config(TIMER6, 0xFF);
    timer_master_output_trigger_source_select(TIMER6, TIMER_TRI_OUT_SRC_UPDATE);

    timer_enable(TIMER6);
}

uint8_t get_i2c_address_bits(void) {
    uint8_t address_bit_0 = gpio_input_bit_get(GPIOB, GPIO_PIN_12) == SET ? 1 : 0;
    uint8_t address_bit_1 = gpio_input_bit_get(GPIOB, GPIO_PIN_13) == SET ? 1 : 0;
    uint8_t address_bit_2 = gpio_input_bit_get(GPIOB, GPIO_PIN_14) == SET ? 1 : 0;
    uint8_t result        = address_bit_0;
    result |= address_bit_1 << 1;
    result |= address_bit_2 << 2;
    return result;
}
