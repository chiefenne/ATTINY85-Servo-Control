
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

## Key Improvements from Previous Implementation

This updated implementation represents a significant improvement over the earlier interrupt-driven approach:

### Previous Implementation (8-bit ISR-based)

- **Resolution**: 256 steps (8-bit) over 1 ms range → 0.35° per step
- **Method**: Interrupt Service Routine (ISR) with Timer1 overflow and compare interrupts
- **Timer Frequency**: 250 kHz (4 µs ticks) using CK/32 prescaler
- **CPU Load**: High interrupt overhead with potential jitter
- **Complexity**: Required careful ISR timing management

### New Implementation (Hardware PWM)

- **Resolution**: ~1000 steps over 1 ms range → 0.09° per step (nearly 4× improvement)
- **Method**: Pure hardware PWM using Timer1's Fast PWM mode with OCR1C as TOP
- **Timer Frequency**: 1 MHz (1 µs ticks) using CK/8 prescaler
- **CPU Load**: Minimal - hardware handles all PWM generation
- **Reliability**: Zero jitter, deterministic timing, immune to interrupt latency

### Benefits of the Hardware PWM Approach

- **Smoother Motion**: Higher resolution provides much smoother servo movement
- **Better Performance**: No CPU cycles wasted on interrupt handling
- **More Reliable**: Hardware-generated timing eliminates software-induced jitter
- **Simpler Code**: Less complex than ISR-based timing management
- **Power Efficient**: Lower CPU utilization allows for power-saving modes

## Electrical Considerations

### Power Supply Requirements

- **MCU Power**: 3.3V - 5V (ensure voltage matches your programmer/system)
- **Servo Power**: Typically 5-6V (check your specific servo specifications)
- **Shared Ground**: **Critical** - MCU and servo must share a common ground reference
- **Separate Power Rails**: Use separate power supplies for servo and MCU to avoid noise and voltage drops
- **Bulk Capacitance**: Place 100-1000µF capacitor near servo connector to handle current spikes

### Signal Integrity

- **Short Signal Leads**: Keep servo signal wire as short as practical
- **Ground Reference**: Route signal wire with a solid ground reference to minimize noise
- **EMI Considerations**: Servo motors generate electrical noise - proper grounding and decoupling is essential

### LED Indicator

- **Current Limiting**: Use 200Ω resistor in series with LED
- **Voltage Drop**: Account for ~2V LED forward voltage in your calculations

## Example Breadboard Configuration

Obviously, before use, the code **main.c** needs to be programmed on the device via any ISP programmer.

![ATtiny85 Servo Control Breadboard Layout](images/Fritzing_Layout_1.png)

## Credits

- The original idea was found in the [AVR Freaks Forum](https://www.avrfreaks.net/comment/810846#comment-810846) posted by user **JimK**
- Hardware PWM implementation developed by Andreas Ennemoser (2025)

## Prerequisites

- For the code to work the CPU needs to run at 8 MHz (this is NOT the factory default)
- The fuse **CKDIV8** has to be unset, otherwise the CPU would run at 1 MHz
- As an alternative the **CKDIV8** could be left as is (factory default), but then the prescaler for the *8-bit Timer/Counter1* has to be set to **4** (using the CS1x bits in the *8-bit Timer/Counter1* control register TCCR1). This alternative is untested.

![ATMEL Studio 7 Fuse Settings - CKDIV8 unchecked for 8 MHz clock speed](images/Fuse_Settings.png)

### Fuse Configuration

Setting of fuses in ATMEL Studio 7 - CKDIV8 unchecked for 8 MHz clock speed

## Additional Information

- Official MICROCHIP [ATTINY85 datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2586-AVR-8-bit-Microcontroller-ATtiny25-ATtiny45-ATtiny85_Datasheet.pdf)

---

Distributed under the MIT license. See [LICENSE](https://github.com/chiefenne/ATTINY85-Servo-Control/blob/master/LICENSE) for more information.


