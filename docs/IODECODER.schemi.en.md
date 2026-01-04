# IO_DECODER

## Index

- [**Features**](#Features)  
- [**Pinout**](#Pinout)  
- [**Usage**](#Usage)  
- [**Where to buy**](#Where-to-buy)  
- [**Authors**](#Authors)  

[BACK TO README.en](README.en.md)

---

## Features

The system consists of a main board that carries the microcontroller with the USB connection and the peripherals provided by the installed firmware. It also provides the buses for connecting input and output expansion boards. These expansion boards are the same for inputs and outputs and can be freely swapped between the two buses. The PCB of the main board indicates the bus connector for the INPUT side and the OUTPUT side. The Pinout and Usage sections contain diagrams illustrating installation and wiring.

[back to index](#Index) 
## Pinout

### io_decoder board
#### Features dependent on the USB board hardware and firmware
  - **Firmware 101**
    - Quadrature encoders: 4 @5Vdc
    - DAC: 2 @8bit 5Vdc
    - ADC: 3 @10bit 5Vdc  
![schemi_io_decoder_1_master](git_USB_pannello_IO_4.png)  

### io_decoder board used as input
  - **Digital inputs**: 8–128 (16 expansions) freely configurable with expansion boards of 8 pins each. Only clean contacts between the common pin and the digital input are accepted as valid inputs.  
![schemi_io_decoder_input_side](git_input_pannello_espansione_4.png)  

### io_decoder board used as output
  - **Digital outputs**: 8–128 (16 expansions) freely configurable with expansion boards of 8 pins each. Each output can drive a load of 50mA@30Vdc with a maximum of 300mA per expansion board.  
![schemi_io_decoder_output_side](git_output_pannello_espansione_4.png)  

[back to index](#Index) 
## Usage

![schemi_io_decoder_panoramica](git_assieme_usb_4.png)

[back to index](#Index) 
## Where to buy
The system is still in prototype phase.

[back to index](#Index) 
## Authors

Roberto "bobwolf" Sassoli

[BACK TO README.en](README.en.md)  
[BACK TO README](../README.md)


Copyright (c) 2025 [bobwolf]
