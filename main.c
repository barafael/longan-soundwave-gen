/*!
    \file    main.c
    \brief   Soundwave Generator

    \version 2019-6-5, V1.0.0, firmware for GD32VF103
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32vf103.h"
#include "systick.h"

#define I2C0_OWN_ADDRESS7      0x82

#define TIMER5_PRESCALER 0xff

uint8_t i2c_receiver[128];

#define DAC0_R8DH_ADDRESS    (0x40007410)

#include "snd.h"

void rcu_config(void);
void gpio_config(void);
void dma_config(void);
void dac_config(void);
void timer5_config(void);
void i2c_config(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    rcu_config();
    gpio_config();
    i2c_config();
    dma_config();
    dac_config();
    timer5_config();
    while (1){
        /* wait until ADDSEND bit is set */
        while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
        /* clear ADDSEND bit */
        i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
        //printf("\r\n I2C slave receive data: \r\n\n");
        int i = 0;
        while(1) {
            if (i2c_flag_get(I2C0, I2C_FLAG_RBNE)) {
                /* read a data byte from I2C_DATA */
                i2c_receiver[i] = i2c_data_receive(I2C0);
                i++;
            } else if (i2c_flag_get(I2C0, I2C_FLAG_STPDET)) {
                i2c_enable(I2C0);
                if (i == 3) {
                    if (i2c_receiver[0] = 0x55) {
                        uint16_t freq = i2c_receiver[2] | (uint16_t)i2c_receiver[1] << 8;
                        uint16_t arr = (27000000 / (TIMER5_PRESCALER - 1)) / freq;
                        timer_autoreload_value_config(TIMER5, arr);
                    }
                }
                i = 0;
                break;
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
void rcu_config(void)
{
    /* enable the clock of peripherals */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_I2C0);
    rcu_periph_clock_enable(RCU_DMA1);
    rcu_periph_clock_enable(RCU_DAC);
    rcu_periph_clock_enable(RCU_TIMER5);
}

/*!
    \brief      configure the related GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
    /* once enabled the DAC, the corresponding GPIO pin is connected to the DAC converter automatically */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    /* connect PB6 to I2C0_SCL */
    /* connect PB7 to I2C0_SDA */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
}

/*!
    \brief      cofigure the I2C0 interfaces
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_config(void)
{
    /* I2C clock configure */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
    /* I2C address configure */
    /* enable I2C0 */
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_OWN_ADDRESS7);
    i2c_enable(I2C0);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}

/*!
    \brief      configure the DMA
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dma_config(void)
{
    dma_parameter_struct dma_struct;
    /* clear all the interrupt flags */
    dma_flag_clear(DMA1, DMA_CH2, DMA_INTF_GIF);
    dma_flag_clear(DMA1, DMA_CH2, DMA_INTF_FTFIF);
    dma_flag_clear(DMA1, DMA_CH2, DMA_INTF_HTFIF);
    dma_flag_clear(DMA1, DMA_CH2, DMA_INTF_ERRIF);
    
    /* configure the DMA1 channel 2 */
    dma_struct.periph_addr  = DAC0_R8DH_ADDRESS;
    dma_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_struct.memory_addr  = (uint32_t)sound;
    dma_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_struct.number       = SIZE;
    dma_struct.priority     = DMA_PRIORITY_ULTRA_HIGH;
    dma_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init(DMA1, DMA_CH2, &dma_struct);

    dma_circulation_enable(DMA1, DMA_CH2);
    dma_channel_enable(DMA1, DMA_CH2);
}

/*!
    \brief      configure the DAC
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dac_config(void)
{
    dac_deinit();
    /* configure the DAC0 */
    dac_trigger_source_config(DAC0, DAC_TRIGGER_T5_TRGO);
    dac_trigger_enable(DAC0);
    dac_wave_mode_config(DAC0, DAC_WAVE_DISABLE);
    dac_output_buffer_disable(DAC0);
    
    /* enable DAC0 and DMA for DAC0 */
    dac_enable(DAC0);
    dac_dma_enable(DAC0);
}

/*!
    \brief      configure the TIMER5
    \param[in]  none
    \param[out] none
    \retval     none
*/
void timer5_config(void)
{
    /* configure the TIMER5 */
    timer_prescaler_config(TIMER5, TIMER5_PRESCALER, TIMER_PSC_RELOAD_UPDATE);
    timer_autoreload_value_config(TIMER5, 0xFF);
    timer_master_output_trigger_source_select(TIMER5, TIMER_TRI_OUT_SRC_UPDATE);
    
    timer_enable(TIMER5);
}
