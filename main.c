/*
 * main.c
 *
 * Author  : Andreas Ennemoser
 * Created: 2018-12-28 22:30:22
 * Updated: 2025-10-12 09:15:00 --> changed to hardware PWM for better resolution
 *
 *
 * Description:
 *   ATtiny85 servo control using Timer1 *hardware PWM* with ~1 µs resolution.
 *   The servo signal is generated fully in hardware (no ISRs), providing a
 *   50 Hz frame (20 ms period) with a 1–2 ms pulse-width range mapped from a
 *   10-bit ADC reading (0–1023).
 *
 *   Resolution:
 *     - Timer clock: 8 MHz / 8 → 1 MHz → 1 tick = 1 µs
 *     - Pulse range: 1000–2000 µs → ~1000 discrete steps
 *     - Typical servo travel: 90° → ~0.09° per step (vs. former 0.35°/step)
 *
 *   Timing configuration (Timer1, Fast PWM with OCR1C as TOP):
 *     - Prescaler:       8
 *     - TOP (OCR1C):     20000  → 20,000 µs = 20 ms (50 Hz)
 *     - Duty (OCR1B):    1000–2000 µs (position command)
 *     - Output mode:     Non-inverting on OC1B
 *
 *   I/O usage (default build):
 *     - PB4 / OC1B : Servo signal output (connect to servo control line)
 *     - PB3 / ADC3 : Potentiometer input (position command, 0–1023)
 *     - PB0        : LED indicator (on near center position)
 *     - VCC / GND  : Power; ensure servo supply shares GND with MCU
 *
 *   Clock & fuses:
 *     - CPU clock nominally 8 MHz.
 *     - CKDIV8 **must be unprogrammed** (disabled) so CPU runs at 8 MHz.
 *     - Optional: CKOUT fuse can expose system clock on PB4 for measurement,
 *       but note PB4 is also the OC1B servo output (choose one).
 *
 *   Electrical notes:
 *     - Servos draw pulsed currents; use a separate 5–6 V rail with bulk
 *       capacitance near the servo connector. Always share ground with the MCU.
 *     - Keep the signal lead short and routed with a solid ground reference to
 *       minimize noise and jitter.
 *
 *   Changes vs. earlier 8-bit ISR design:
 *     - Previous: 8-bit effective steps across 1–2 ms (≈256 positions,
 *       ≈0.35°/step) using overflow + compare ISR at 4 µs tick.
 *     - Now: Hardware PWM at 1 µs tick, ≈1000 positions (≈0.09°/step),
 *       zero ISR jitter and near-zero CPU load.
 *
 * Credits (for earlier design):
 *   - JimK (AVR Freaks) for foundational ideas:
 *     https://www.avrfreaks.net/comment/810846#comment-810846
 */

/* --------------------------------------------------------------------------
 * ATtiny85 Pin Reference (see datasheet page 2)
 *
 *                  -------
 *  RESET / PB5 ---| o     |--- VCC
 *   ADC3 / PB3 ---|       |--- PB2 / SCK / USCK / SCL
 *  OC1B  / PB4 ---|       |--- PB1 / MISO / OC1A
 *          GND ---|       |--- PB0 / MOSI / SDA / OC0A
 *                  -------
 *
 * Notes:
 *  - OC1B (PB4) is the hardware PWM output used for the servo signal.
 *  - ADC3 (PB3) is used for reading the potentiometer (0–1023).
 *  - If I²C (USI/TWI) is added later, SDA = PB0 and SCL = PB2 (with pull-ups).
 * -------------------------------------------------------------------------- */


#include <avr/io.h>
#include <util/delay.h>

// -----------------------------------------------------------------------------
// Pin configuration
// -----------------------------------------------------------------------------
#define SERVO_PIN PB4 // OC1B output
#define LED_PIN PB0   // LED indicator
#define POT_PIN PB3   // ADC3 input (PB3)

// Convenience macros
#define LED_ON (PORTB |= (1 << LED_PIN))
#define LED_OFF (PORTB &= ~(1 << LED_PIN))

// -----------------------------------------------------------------------------
// ADC initialization and read
// -----------------------------------------------------------------------------
void Init_ADC(void)
{
    // Select ADC3 (PB3) as input
    ADMUX = (1 << MUX1) | (1 << MUX0);

    // Enable ADC, prescaler = 64 → 8 MHz / 64 = 125 kHz ADC clock (OK)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

uint16_t Read_ADC(void)
{
    ADCSRA |= (1 << ADSC); // start conversion
    while (ADCSRA & (1 << ADSC))
        ;       // wait until finished
    return ADC; // 10-bit result (0–1023)
}

// -----------------------------------------------------------------------------
// Timer1 setup for hardware PWM (Fast PWM, OCR1C as TOP)
// -----------------------------------------------------------------------------
void Init_TIMER1_HWPWM(void)
{
    // Set PB4 (OC1B) as output
    DDRB |= (1 << SERVO_PIN);

    // Fast PWM mode, OCR1C as TOP
    // Clear OC1B on compare match, set at BOTTOM (non-inverting)
    // Prescaler = 8 → 8 MHz / 8 = 1 MHz → 1 µs per tick
    TCCR1 = (1 << PWM1B) | (1 << COM1B1) | (1 << CS11) | (1 << CS10);

    // Set PWM period = 20 000 µs (50 Hz)
    OCR1C = 20000;

    // Initial pulse = 1500 µs (center)
    OCR1B = 1500;
}

// -----------------------------------------------------------------------------
// LED helper
// -----------------------------------------------------------------------------
void LED_Control(uint16_t pulse_us)
{
    // Turn LED on near center (±25 µs window)
    if (pulse_us > 1475 && pulse_us < 1525)
        LED_ON;
    else
        LED_OFF;
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(void)
{
    // Init peripherals
    DDRB |= (1 << LED_PIN); // LED output
    Init_ADC();
    Init_TIMER1_HWPWM();

    while (1)
    {
        // Read potentiometer
        uint16_t adc_value = Read_ADC(); // 0–1023

        // Map 0–1023 → 1000–2000 µs
        uint16_t pulse_us = 1000 + ((uint32_t)adc_value * 1000) / 1023;

        // Update PWM duty directly in hardware
        OCR1B = pulse_us;

        // LED feedback
        LED_Control(pulse_us);

        _delay_ms(10); // small debounce / update smoothing
    }
}
