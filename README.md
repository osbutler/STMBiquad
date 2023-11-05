# STMBiquad
Biquad implementation in STM32 with external ADC and DAC (I2S)

# Hardware

The STM32 used is the STM32F446ZE eval board. It is chosen for its great peripherial capacity including 3xI2S and 2xSAI (Serial Audio Interface) that can be configured in I2S an synchronised internally. In this project, only the two SAIs are used. Each SAI has two ports, configured as I2S here.

One AK5720VT ADC from AK and two ES9023 DAC from ESS are used. The SAI1 communicates with the ADC and SAI2 with the DACs. All peripherals are synchronized, SAI1 providing master clock, bit clock and L/R clock.

![Download](https://raw.githubusercontent.com/osbutler/STMBiquad/main/img/pinouts.png)


# Biquad structure

The biquad coefficients are calculated thanks to this useful link _https://webaudio.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html_.

The implemented structure is the direct form 2 transposed (DF2T). 

A Matlab implementation of biquad coefficient generation and real-time block processing filtering using DF2T can be found here _https://github.com/osbutler/MBiquad_.
