
# ATTINY85 Servo Control

[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://en.wikipedia.org/wiki/MIT_License)

An RC servo control using an [AVR ATTINY85 microcontroller](https://www.microchip.com/wwwproducts/en/ATtiny85). This implementation uses **hardware PWM** for high-precision servo control with ~1 µs resolution. The system generates a 50 Hz PWM signal (20 ms period) with pulse widths ranging from 1-2 ms, providing smooth and jitter-free servo operation.

## Features
 - **High Resolution**: ~1000 discrete servo positions (vs. previous 256 steps)
 - **Hardware PWM**: Zero interrupt overhead and jitter-free operation using Timer1's OC1B output
 - **Precision**: ~1 µs timing resolution with 0.09° per step (improved from 0.35°/step)
 - **Standard RC Servo Compatible**: 50 Hz PWM with 1-2 ms pulse width range
 - **10-bit ADC Control**: Potentiometer input provides full 0-1023 range mapped to servo positions
 - **LED Indicator**: Shows when servo is near center position (±25 µs window)
 - **Low CPU Load**: Hardware timer handles all PWM generation automatically
 - **Robust Design**: Separate power considerations and noise immunity for reliable operation

## Technical Specifications
 - **Timer Configuration**: Timer1 Fast PWM mode with OCR1C as TOP
 - **Clock Frequency**: 8 MHz CPU with prescaler/8 → 1 MHz timer clock
 - **PWM Period**: 20,000 µs (50 Hz frame rate)
 - **Pulse Range**: 1000-2000 µs (1-2 ms)
 - **Resolution**: 1 µs per timer tick
 - **Pin Assignment**:
   - PB4 (OC1B): Servo signal output
   - PB3 (ADC3): Potentiometer input (10kΩ recommended)
   - PB0: LED indicator output (use 200Ω resistor)

## Example Breadboard Configuration

Obviously, before use, the code **main.c** needs to be programmed on the device via any ISP programmer.

![](images/Fritzing_Layout_1.png)

## Credits
 - The idea was found in the [AVR Freaks Forum](https://www.avrfreaks.net/comment/810846#comment-810846) posted by user **JimK**

## Prerequisites
 - For the code to work the CPU needs to run at 8 MHz (this is NOT the factory default)
 - The fuse **CKDIV8** has to be unset, otherwise the CPU would run at 1 MHz
 - As an alternative the **CKDIV8** could be left as is (factory default), but then the prescaler for the *8-bit Timer/Counter1* has to be set to **4** (using the CS1x bits in the *8-bit Timer/Counter1* control register TCCR1). This alternative is untested.

<!--- HTML code used in order to be able to resize image -->
<img src="images/Fuse_Settings.png" alt="drawing" width="600"/>

**Setting of fuses in ATMEL Studio 7 - CKDIV8 unchecked for 8 MHz clock speed**

## Additional information
 - Official MICROCHIP [ATTINY85 datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2586-AVR-8-bit-Microcontroller-ATtiny25-ATtiny45-ATtiny85_Datasheet.pdf)

<br></br>

Distributed under the MIT license. See [LICENSE](https://github.com/chiefenne/ATTINY85-Servo-Control/blob/master/LICENSE) for more information.


