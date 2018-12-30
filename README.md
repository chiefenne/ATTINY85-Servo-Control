
# ATTINY85 Servo Control

An RC servo control using an AVR ATTINY85 microcontroller. The control offers a full 8-bit resolution for a standard RC servo. The control is setup via timer-1 as it offers the the required prescaler to get a 250 kHz timer frequency from an 8 MHz clocked CPU. This allows for approximately 256 steps (8-bit) over a 1 millisecond period.

## Features
 - The servo can be moved to 256 positions within its 1ms-2ms duty range
 - A standard RC servo normally operates a total angle of approximately 90° (+/- 45°)
 - This is done via a 50 Hz PWM signal with a duty cycle of 5% (minimum position) to 10% (maximum position)
 - This means a 20 millisecond PWM signal with a duty cycle of 1 ms to 2 ms
 - To get the full resolution possible by the 8-bit microcontroller, timer-1 is setup so that it takes the 8-bit range (256) for the complete duty cycle variation (i.e. 1 millisecond)
 - With 8-bit, i.e. 256 steps, a resolution of 0.35° per step is possible
 - An LED indicates the servo center position
 - Servo and LED pins can be configured by the corresponding macros in the code

## Credits
 - The idea was found in the [AVR Freaks Forum](https://www.avrfreaks.net/comment/810846#comment-810846) posted by user **JimK**

## Prerequisites
 - For the code to work the CPU needs to run at 8 MHz (this is NOT the factory default)
 - The fuse **CKDIV8** has to be unset, otherwise the CPU would run at 1 MHz
 - As an alternative the **CKDIV8** could be left as is (factory default), but then the prescaler for timer-1 has to be set to **4** (using the CS1x bits in the timer-1 control register TCCR1). This alternative is untested.


Distributed under the MIT license.


Andreas Ennemoser – andreas.ennemoser@aon.at
