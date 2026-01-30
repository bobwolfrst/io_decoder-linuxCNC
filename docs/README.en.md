---
layout: default
---
# io_decoder
<a id="contents"></a>
## Contents

- [io\_decoder](#io_decoder)
  - [Contents](#contents)
  - [Description](#description)
  - [Main features](#main-features)
    - [HAL component](#hal-component)
    - [USB Hardware](#usb-hardware)
      - [expansion\_8](#expansion_8)
      - [Features dependent on USB board hardware and firmware](#features-dependent-on-usb-board-hardware-and-firmware)
      - [Dimensions and Wiring](#dimensions-and-wiring)
  - [Installation](#installation)
    - [Synopsis](#synopsis)
    - [Functions](#functions)
    - [Pins](#pins)
    - [Requirements](#requirements)
    - [Installation commands](#installation-commands)
  - [Usage](#usage)
    - [Configuration example](#configuration-example)
    - [Available features](#available-features)
    - [Keyboard](#keyboard)
  - [Dependencies](#dependencies)
  - [Authors](#authors)
  - [License](#license)

[📖 README](../README.md) | [🏠 Project Home]({{ '/index/' | relative_url }})  
[🚀 Demo/Eval mode in English]({{ '/demo_mode.en' | relative_url }))

<a id="description"></a>
## Description
io_decoder is a HAL component for LinuxCNC. It allows controlling, via external hardware with a USB connection, the inputs and outputs required to manage an operator panel of a CNC machine.
It is a real-time component but its USB communication portion is handled by a non-realtime thread.
It is optimized to manage non-critical inputs and outputs such as buttons, switches and indicator lights for manual interaction with the control panel and distributed control boxes located on the machine.  

[back to contents](#contents)  
<a id="main-features"></a>
## Main features
### HAL component
- **Digital inputs**: 8–128 freely configurable from a .hal file; this value must match the installed hardware.
  - software debounce of 20 ms.
  - additional pin with toggle of the value on the rising edge of the input.  
  - simulated keyboard functionality.
  - in case of USB disconnection, the HAL pin value is set to 0.  

- **Digital outputs**: 8–128 freely configurable from a .hal file; this value must match the installed hardware. 
  - Function to make outputs blink. Frequency and duty cycle can be set individually for each output.
  - in case of USB disconnection, the physical output is turned off by the Arduino watchdog.  

- **Quadrature encoders (jog-wheel)**: the number can vary depending on the USB board firmware.
  - encoder output for the count value
  - invert input for reversing the count.
  - up and down outputs to directly drive HAL components like UPDOWN or MULTISWITCH.

- **DAC**: the number can vary depending on the USB board firmware.
  - dac input for the output value.
  - scale input to scale the output value.
  - invert input for inversion of the output value.

- **ADC**: the number can vary depending on the USB board firmware.
  - adc output with the acquired, filtered and converted value.
  - raw output with the acquired value without conversions
  - scale input to scale the acquired value.
  - invert input to invert the acquired value.
  - up and down outputs to directly drive HAL components like UPDOWN or MULTISWITCH.  
  - output with value derived from the up/down count (joystick)

- **Diagnostic pins**:
  - output to signal the component state
  - output for the total percentage of errors in the USB stream
  - output for the percentage of parsing errors per minute
  - output to signal the USB connection status  
  - output for the base-thread jitter value  
  - output for the servo-thread jitter value
  - output for the internal usb-thread jitter value
  - output for the USB transmission jitter value   
  
<a id="usb-hardware"></a>
### USB Hardware
  
#### expansion_8  
- **Digital inputs**: 8–128 freely configurable with expansion boards of 8 pins each. Only clean contacts between the common pin and the digital input are accepted as valid inputs.  
- **Digital outputs**: 8–128 freely configurable with expansion boards of 8 pins each. Each output can drive a load of 50mA@30Vdc with a maximum of 300mA per expansion board.

#### Features dependent on USB board hardware and firmware
- **Firmware 101**
  - Quadrature encoders: 4 @5Vdc
  - DAC: 2 @8bit 5Vdc
  - ADC: 3 @10bit 5Vdc
  - The HAL<=>USB communication cadence is 20 ms (50Hz)  
  
- **Firmware 255**
  - This is a special evaluation build designed to test io_decoder features using a standard Arduino UNO R3.
  - 4 digital input
  - 4 digital output
  - Quadrature encoders: 1 @5Vdc
  - DAC (PWM): 1 @8bit 5Vdc
  - ADC: 1 @10bit 5Vdc  

#### Dimensions and Wiring
- [**io_decoder USB board pinout**]({{ '/IODECODER.schemi.en' | relative_url }})

[back to contents](#contents)  
<a id="installation"></a>
  
## Installation
### Synopsis
- **loadrt io_decoder** [input=*num*] [output=*num*] [usb_port_name=*"string"*] [firmware=*num*] [verbose=*num*] [keymap_file=*"string"*] [uinput_chmod_cmd=*"string"*]  
   - **input**: this value must match the installed hardware. The number must be a multiple of 8 (min 8 max 128) otherwise an error occurs at startup. Default value = 8
   - **output**: this value must match the installed hardware. The number must be a multiple of 8 (min 8 max 128) otherwise an error occurs at startup. Default value = 8
   - **usb_port_name**: to give the port a custom fixed name; see section [**defining USB port**](#installation-commands). Default value "/dev/io_decoder"
   - **firmware**: parameter to configure the component according to the USB board features. It is printed on the USB board PCB. Default value = 101
   - **verbose**: to enable the level of error messages on the GUI. the number enables the indicated message type and those of lower value. default 1.  
     - 0 = none.
     - 1 = component. Sends a message in case of USB disconnection or restart of USB communication and reports keyboard-functionality messages if it is not activated for any reason.
     - 2 = minimal. Parsing error percentage messages.
     - 3 = all.
   - **keymap_file**: text file to set the mappings [**input => keyboard simulation**](#keyboard). Default value "io_decoder-keymap.cfg"
   - **uinput_chmod_cmd**: string parameter to give write permissions on UINPUT for the [**simulated keyboard functionality**](#keyboard). If you want to be sure not to give permissions the parameter must be "" (uinput_chmod_cmd="" with nothing inside the quotes). Default value "chmod 0666 /dev/uinput".  

### Functions
```bash
addf io_decoder.update	servo-thread
```  

### Pins
- **Digital IO**
-  Input
	- **io_decoder.in.*MM*-*N*** (bit out): pin to read the state of digital inputs. *MM* = two-digit number indicating the expansion board position. *N* = one-digit number indicating the input on the expansion board. Each pin has a software debounce of 20 ms. The pin is not created if the input is configured for keyboard functionality. default 0.    
    - **io_decoder.in.*MM*-*N*.toggle** (bit out): this pin toggles its value from 0 to 1 and from 1 to 0 on the rising edge of its digital input. The pin is not created if the input is configured for keyboard functionality. default 0.  
	- **io_decoder.in.*MM*-*N-keyboard*** (bit out): pin used to send signals to the simulated keyboard. Created if the input has keyboard functionality; it can still be used as a HAL pin and has the same characteristics as the ordinary pin. [**to configure pins**](#keyboard). default 0.   
 - Output
	- **io_decoder.out.*MM*-*N*** (bit in): pin to set the state of digital outputs. *MM* = two-digit number indicating the expansion board position. *N* = one-digit number indicating the output on the expansion board. default 0.     
   - **io_decoder.out.*MM*-*N*.blink-en** (bit in): enable that activates blinking of the output. If the corresponding HAL output pin is 0 the output is off regardless of enable state. If the HAL output is 1 and enable is 0, the output is steadily on. If HAL output is 1 and enable is 1, the output blinks at the configured frequency. default 0.  
   - **io_decoder.out.*MM*-*N*.blink-freq** (float in): blinking frequency (Hz) of the output. minimum 0.25Hz maximum 16Hz. Outside these values the frequency is set to the minimum or maximum. The ability to set the frequency individually per output is meant to avoid simultaneous blinking of all outputs. A difference of 0.01–0.02 Hz between outputs is enough to create a chaotic, not visually synchronized, signalling effect. default 1Hz.  
   - **io_decoder.out.*MM*-*N*.blink-width** (float in): varies the portion of the period the pulse stays high. values 0–1. default 0.5 (50%).  

- Encoder
	- **io_decoder.enc.*N*** (S32 out): for the count value. default 0.  
	- **io_decoder.enc.*N*.invert** (bit in): to invert the count. default 0.  
	- **io_decoder.enc.*N*.up** (bit out): to directly drive HAL components like UPDOWN or MULTISWITCH.
	- **io_decoder.enc.*N*.down** (bit out): to directly drive HAL components like UPDOWN or MULTISWITCH.

- ADC
	- **io_decoder.adc.*N*** (float out): acquired, filtered and converted value. default 0.  
	- **io_decoder.adc.*N*.raw** (S32 out): acquired value without conversions. default 0.  
	- **io_decoder.adc.*N*.invert** (bit in): to invert the acquired value. default 0.  
	- **io_decoder.adc.*N*.scale** (float in): to scale the acquired value. default 0.  
  - joystick section
    - **io_decoder.adc.*N*.joy.center** (float in): to center the joystick. value -1/+1. default 0.
    - **io_decoder.adc.*N*.joy.deadband** (float in): for the center point with output disabled. value 0/+1. default 0.
    - **io_decoder.adc.*N*.joy.factor** (float in): sensitivity factor. value 0/+1. default 0.
    - **io_decoder.adc.*N*.joy.pulse.up** (bit out): to directly drive HAL components like UPDOWN or MULTISWITCH.
    - **io_decoder.adc.*N*.joy.pulse.down** (bit out): to directly drive HAL components like UPDOWN or MULTISWITCH.
    - **io_decoder.adc.*N*.joy.count** (S32 out): direct value obtained from the up/down count.  

 - DAC
	- **io_decoder.dac.*N*** (float in): output value for the DAC. default 0.  
	- **io_decoder.dac.*N*.scale** (float in): to scale the output value. default 0.  
	- **io_decoder.dac.*N*.invert** (bit in): to invert the output value. default 0.  

  - Diagnostics
    - **io_decoder.diag.comm-state** (S32 out): output that signals the communication state of the component.  
      - HANDSHAKE = 0  
      - HANDSHAKE_WAIT_RESPONSE = 1  
      - COMMUNICATING = 2  
      - COMMUNICATING_WAIT_RESPONSE = 3  
      - ERROR = 4
    - **io_decoder.diag.error-count** (float out): total percentage of errors during USB communication since the component became active. Updated every minute.
    - **io_decoder.diag.parse-error** (float out): Percentage of error in the last minute. If it exceeds 5% it is possible that the PC/LinuxCNC system is slow and produces high thread jitter values. If the USB cable is of poor quality or installed in positions that pick up a lot of electrical/electromagnetic interference the value often rises above 1%. This value should always remain under 1%. 
    - **io_decoder.diag.usb-connected** (bit out): Signals that USB communication is active = 1 or disconnected/inactive = 0. 
    - **io_decoder.diag.servo-thread-time** (S32 in): To connect to servo-thread.time pin to measure jitter. 
    - **io_decoder.diag.base-thread-time** (S32 in): To connect to base-thread.time pin to measure jitter. 
    - **io_decoder.diag.servo-thread-jitter** (S32 out): Servo-thread jitter value over the last minute. 
    - **io_decoder.diag.base-thread-jitter** (S32 out): Base-thread jitter value over the last minute. 
    - **io_decoder.diag.usb-thread-jitter** (S32 out): Jitter value of the usb-thread created internally in the component over the last minute. This value should be as low as possible, considering the thread is called with a 1 ms cadence. Very high values indicate the PC/system is not performant and induces excessive delays in realtime.
    - **io_decoder.diag.usb-communication-jitter** (S32 out): Jitter value of the execution times of the USB retransmission alone over the last minute. This value should be as low as possible, given that the whole process of receiving and transmitting on USB takes about 8 ms at most.  

<a id="requirements"></a>

### Requirements
- **Hardware**
  - [io_decoder USB board pinout](IODECODER.schemi.en.md)

- **Software**
  - LinuxCNC 2.8 and later
  - the component was developed on a system created from a released image of LinuxCNC 2.8, but it might also work with earlier versions.

<a id="installation-commands"></a>

### Installation commands
- compile the component:  
  In a terminal open the folder where the component .c file is saved and type:   
  ```bash
  sudo halcompile --install io_decoder.c
  ```

- define USB port:
1. Find the USB device information  
    Connect the device and run this command to find its characteristics:
    ```bash
    lsusb
    ```  
    In the command output, with the board connected to USB, a line like this should appear:  
    **Bus 001 Device 008: ID 2341:8036 Arduino SA Leonardo (CDC ACM, HID)**    

2. Create a udev rule  
    Open a terminal and go to the rules folder with:  
    ```bash
    cd /etc/udev/rules.d/
    ```  
    Create a rule for a USB serial device:  
    ```bash
    sudo nano /etc/udev/rules.d/99-io_decoder.rules
    ```  
    Once executed, the terminal will open the nano text editor. The number 99 is used to tell the system to read this rule last among all the rules it has. The file name 99-io_decoder.rules is the one I chose; but this can be any name.  
    Paste the following rule inside the file:  
    ```bash
    SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="8036", ATTRS{product}=="io_decoder", SYMLINK+="io_decoder", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"  
    ```

    Press Ctrl + O to save (it will ask you to confirm the file name and if it doesn't exist it will be created, press Enter).  
    Press Ctrl + X to exit the editor.  
    This way your device will be accessible as /dev/io_decoder regardless of the physical port.  
    If you want to change the name of the USB symbolic link, modify the value written after SYMLINK+=  
    >**WARNING**
    With MODE="0666" read/write permissions on the USB are opened to anyone using the system. If you want to keep the system more secure you must create, using prompt commands, a group that can contain the communication users with appropriate permissions and set it in the string with GROUP="your_group_name".  

4. Reload udev rules
    ```bash
    sudo udevadm control --reload-rules    
    sudo udevadm trigger
    ``` 

5. Verify  
    Unplug and replug the device, then check with:  
    ```bash
    ls -l /dev/io_decoder
    ```  
    With these settings the default value declared at component initialization is satisfied.
    In any case everything is freely configurable.  

[back to contents](#contents)  
<a id="usage"></a>
  
## Usage
Designed to build/manage an operator panel for CNC machines.
For using this system to control motors and various machine sensors, and because of the nature of LinuxCNC, USB communication is not reliable and is highly discouraged for this purpose. Even though this component's system has been tested in various ways, for safety the emergency stop button should be connected directly to the physical pins provided by the LinuxCNC hardware.  
In case of software disconnection of the USB or slowdowns by the operating system due to old or underperforming PCs that produce high thread jitter values, the system is stable and reacts to various situations, keeping the component always active.  
  
<a id="configuration-examples"></a>

### Configuration example
```bash
   loadrt io_decoder output=24 input=24 usb_port_name="/dev/io_decoder" verbose=3 firmware=101 keymap_file="io_decoder-keymap.cfg" uinput_chmod_cmd="chmod 0666 /dev/uinput"
```  
<a id="available-features"></a>

### Available features
- The UP and DOWN pins of the encoder and the ADC allow, via HAL components like UPDOWN or MULTISWITCH, selecting values such as axis selection or increment (feed advance), with an incremental rotary encoder or an analog joystick.  
- In addition to HAL pins, the component generates error messages visible on the LinuxCNC GUI, if enabled during component initialization in the machine .hal file, in case of:  
  - communication timeout  
  - USB disconnection  
  - too many communication errors in case of damaged cable or electrical/electromagnetic interference  
  - USB communication restoration (not an error but reported for convenience, on basic configurations like mine)
- If the USB connection is lost the values HAL receives from the USB board are frozen and HAL input pins are forced to 0; at the same time the physical outputs on the USB board are disabled.  

<a id="keyboard"></a>

### Keyboard
Functionality that allows using physical inputs to send simulated keyboard commands that can be used to print on screen or send commands as if typed on the system keyboard.  
If the file [**"io_decoder-keymap.cfg"**](IOdecoder-keymap.cfg.md) (or the one specified by **keymap_file**) is present in the machine configuration folder, the functionality is enabled. If it is not present or is empty or contains only comments, the functionality is not enabled.  
  
**Configure input=>keyboard mappings:**  
The functionality uses UINPUT to send commands, so each numeric code indicates the position on the keyboard; it is not an absolute number for each keyboard layout (US, UK, IT, FR, etc.) but a code indicating the position of the key on the keyboard installed in the system. If code 27 is sent and a US keyboard is installed the result on screen will be ] (right square bracket); on an IT keyboard it will be a + (plus).  
You can also send a single or composite code to create combinations such as:  
 - shift+a = A 
 - ctrl+x = cut
 - alt_gr+à = @ (on the Italian keyboard)
 - alt+f = open dropdown menu where applicable
 - shift as a single key
 - and all other possible combinations
  
Verify the actual result for each installation for the code sent.  
    
**Format**: in.*MM*-*N* <Keysym_string> #comment  
I recommend ordering configuration lines with progressive numbers to have an overall ordered view of functions, but this is not mandatory.
  - in   = prefix
  - *MM* = two-digit number indicating the expansion board position.
  - *N*  = one-digit number indicating the input on the expansion board.
  - <Keysym_string> = code to send to the virtual keyboard.
  - *#* Everything written after is a comment and is ignored.
  
    The code to send can be either a number or a prefix + number. The prefix is used to send the modifier together with the key value:  
    - s = shift  
    - c = control  
    - a = alt  
    - g = alt graph (AltGr)      
  - key map codes:    
    - 30  -> a
    - s30 -> A
    - c44 -> control+z (cut)
    - 42  -> left shift
    - s40 -> @ on UK keyboard
    - g39 -> @ on IT keyboard
  - example configuration lines:
    - in.00-3 s20 #T	   -> input 00-3 is associated with the code of T (uppercase t)
    - in.01-5 c46 #ctrl+c  -> input 01-5 is associated with the copy action
    - in.00-1 18 #e	       -> input 00-1 is associated with the code for e (lowercase e)

  **Keycodes**  
  The [keyboard map]({{ '/ISO_kb_(105)_QWERTY_UK_IT.png' | relative_url }}) and the [keycodes]({{ '/keycodes_list' | relative_url }}) were derived from various sources and tests; they are certainly not 100% accurate. The map helps to understand what value the key position on your keyboard sends to the system, which will then display the symbol according to the keyboard layout. The keyboard functionality sends the key position value, not the associated symbol.  

  >## Attention!!!  
  >**This functionality, by default the parameter is uinput_chmod_cmd="chmod 0666 /dev/uinput", expects read and write permissions on UINPUT to be given to all users, with the chmod command called automatically by the component itself. If you want to set the system with your custom permissions you must configure permissions using prompt commands. If the parameter uinput_chmod_cmd is passed in the machine .hal configuration file with value uinput_chmod_cmd="" the component's chmod command is not executed and the system must already be configured with the correct permissions to use the keyboard functionality. With uinput_chmod_cmd you can provide a custom command to grant permissions suitable for your needs.**
  **If the file referenced by the keymap_file parameter does not exist, is empty or does not contain valid pins, and therefore the simulated keyboard functionality is not used, UINPUT is not initialized and no permissions are set for this functionality.**
  **Default permissions are temporary for the period during which the component is running.**  

[back to contents](#contents)  
<a id="dependencies"></a>
  
## Dependencies
Developed on LinuxCNC 2.8 and there should be no additional dependencies.

[back to contents](#contents)  
<a id="authors"></a>
  
## Authors
Roberto "bobwolf" Sassoli and his virtual twin.

[back to contents](#contents)  
<a id="license"></a>
  
## License

This software is distributed under the GNU General Public License, version 2 (GPLv2).  
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.  

**Disclaimer:**  
This software is provided "as is", without any warranty.  
The author is not responsible for damages resulting from the use of the program.  

Copyright (c) 2026 [bobwolf]

---
[🔝 back to contents](#contents) | [📖 README](../README.md) | [🏠 Project Home]({{ '/index/' | relative_url }})  
[🚀 Demo/Eval mode in English]({{ '/demo_mode.en' | relative_url }])
