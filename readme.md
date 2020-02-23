This firmware for the GD32VF103 generates an audio signal via DAC (triggered by a timer and a DMA).
The frequency can be set over I2C (slave address 0x41, register 0x55).

This code depends on the GD32VF103\_Firmware\_Library. It should be placed next to the 'Template' directory in GD32VF103\_Firmware\_Library (https://github.com/riscv-mcu/GD32VF103_Firmware_Library)
