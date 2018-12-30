
# ATTINY85 servo control

An RC servo control using an AVR ATTINY85 microcontroller. The control offers a full 8-bit resolution for a standard RC servo. The control is setup via timer 1 as it offers the the required prescaler to get a 250kHz timer frequency from the 8MHz clocked CPU. This allows for approximately 256 steps (8-bit) over a 1 millisecond period.

## Features
 - The servo can be moved to 256 positions within its 1ms-2ms duty range
 - The servo angle normally is approximately 90°
 - With 8-bit, i.e. 256 steps, a resolution of 0.35° per step is possible

Andreas Ennemoser – andreas.ennemoser@aon.at

## Credits
 - The idea was found in the [AVR Freaks Forum](https://www.avrfreaks.net/comment/810846#comment-810846) posted by **JimK**
 - Coding done by myself

## Prerequisites
 - The CPU needs to run at 8MHz (this is NOT the factory default)
 - The fuse **CKDIV8** has to be unset, otherwise the CPU would run at 1MHz
 - As an alternative the **CKDIV8** could be left as is from factor default, but then the prescaler for timer 1 has to be set to **4**

Distributed under the MIT license.