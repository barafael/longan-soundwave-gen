This firmware for the GD32VF103 generates an audio signal via DAC (triggered by a timer and a DMA).
The frequency can be set over I2C (slave address selected by B12, B13, B14.

For the flashing process via RVLink/Platformio, on Arch, these dependencies are needed:

* ncurses5-compat-libs
* hidapi
* libftdi-compat