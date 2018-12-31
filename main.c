/*
 * main.c
 *
 * Created: 28.12.2018 22:30:22
 * Author : Andreas Ennemoser
 * Description: ATTINY85 servo control with 8-bit resolution
 *              The servo can be moved to 256 positions within its 1ms-2ms duty range
 *              The angle normally is -45° / +45° --> 90° / 256 steps = 0.35° resolution
 */ 

// the clock speed on ATTINY85 is approx. 8MHz
// if fuse CKDIV8 is set (factory default), a prescaler of 8 is used which results in a 1MHz clock
// for this code CKDIV8 needs to be unset as the code relies on 8MHz CPU speed
// the actual frequency can be measured at PB4 if the CKOUT fuse is set


//             ATTINY85 PIN Configuration (for more details see datasheet page 2)
//                  -------
//  RESET / PB5 ---|       |--- VCC
//   ADC3 / PB3 ---|       |--- PB2 / SCK
//          PB4 ---|       |--- PB1 / MISO
//          GND ---|       |--- PB0 / MOSI
//                  -------

// includes
#include <avr/io.h>
#include <avr/interrupt.h>

// macros
#define SERVO_PORT PB0
#define SERVO_PWM_ON PORTB |= (1 << SERVO_PORT)
#define SERVO_PWM_OFF PORTB &= ~(1 << SERVO_PORT)

#define LED_PORT PB4
#define LED_ON PORTB |= (1 << LED_PORT)
#define LED_OFF PORTB &= ~(1 << LED_PORT)

// variables
float servo_pwm_duty;                        // ADC reading (10-bit) mapped to servo duty cycle (8-bit )
volatile uint8_t servo_pwm_duty_int;         // 
volatile uint16_t adc_value;                 // result from 10-bit ADC reading (0-1023)
volatile uint8_t overflows = 0;              // number of timer1 overflows (0-255)
volatile uint8_t flag = 0;                   // flag that indicates that initial 1ms of servo pulse is over

void Init_ADC(void){
    
    // this function initializes the ADC

    // do NOT left shift result (ADLAR) when reading ADC (see ADC_Read)!!!
    ADMUX |= (1 << MUX1) | (1 << MUX0);      // use ADC3 at PB3 for ADC input (datasheet page 135)
    
    // input clock frequency should be between 50kHz and 200kHz to get maximum resolution (datasheet page 125)
    // 8MHz CPU with ADC prescaler of 64:  8000000 / 64 = 125000 --> 125 kHz
    ADCSRA |= (1 << ADEN);                   // Enable ADC (datasheet page 136)
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1);   // set ADC prescaler to 64 (datasheet page 136)
}

uint16_t Read_ADC(void){
    
    // this function reads analog values through ADC

    uint16_t adc_result;

    ADCSRA |= (1 << ADSC);                   // ADC Start Conversion (datasheet page 136)
    while (ADCSRA & (1 << ADSC) );           // wait till conversion complete (ADSC will read as one as long as a conversion is in progress. When the conversion is complete, it returns to zero. page 136)
    
    adc_result = ADC;                        // if not reading ADCL and ADCH do NOT left shift result in ADC_Init (i.e. keep ADLAR bit unset) !!!!! (the 16bit ADC is correctly populated by the AVRgcc toolchain)
    
    return adc_result;

}

float Map_ADC(float adc_value, float adc_min, float adc_max, float map_min, float map_max) {
    
    // function that maps the ADC reading to a target range
    // ADC reading with 10-bit resolution results in values from 0 to 1023
    // the mapping of these values results in values from map_min to map_max
    
    float result;
    
    result = map_min + adc_value * (map_max - map_min) / (adc_max - adc_min);
    
    return result;
    
}

void Init_PORT(void) {
    
    // this function initializes the pins which are used as output

    DDRB |= (1 << LED_PORT);                 // set led port as output
    DDRB |= (1 << SERVO_PORT);               // set servo port as output

}

void Init_TIMER(){
    
    // this function initializes the timer
    // the timer is set up in order to last approx. 1ms per overflow
    // the CPU runs at 8MHz (requires that CKDIV8 fuse is NOT set)
    // timer 1 is used as it allows for more prescalers; here 32 is used to come from (MHz to 250kHz
    // solution derived from this idea: "https://www.avrfreaks.net/comment/810846#comment-810846"

    TCCR1 |= (1 << CS12) | (1 << CS11);      // set prescaler of timer1 to 32 --> 250kHz timer @8MHz CPU

}

void Init_INTERRUPTS(){
    
    // this function initializes the interrupts
    
    TIMSK |= (1 << TOIE1);                   // enable overflow interrupt of timer1 (page 92)
    
    // did not work with OCIE1A???
    TIMSK |= (1 << OCIE1B);                  // enable compare match B interrupt of timer1 (page 92)
    
    sei();                                   // enable interrupts (MANDATORY)

}

void LED_Control(){
    
    // this function switches the LED on if servo center position is reached
    
    if ( (servo_pwm_duty_int > 125) && (servo_pwm_duty_int < 129) ) {
        LED_ON;
    }
    else {
        LED_OFF;
    }
}

int main(void)
{
       
    Init_PORT();                             // initialize PORTB (output pins only)
    Init_ADC();                              // initialize ADC converter
    Init_TIMER();                            // initialize timer
    Init_INTERRUPTS();                       // initialize interrupts
    
    while(1)
    {

        adc_value = Read_ADC();              // get 10-bit ADC result (0-1023)

        servo_pwm_duty = Map_ADC(adc_value, 0, 1023, 1, 254);
        servo_pwm_duty_int = (int)servo_pwm_duty;
        
        LED_Control();
                
    }        
   
}

ISR(TIMER1_OVF_vect){

    // interrupt service routine for timer1 overflow
    // 1 interrupt is approx. 1ms

    overflows++;
    
    if (overflows == 20){                    // corresponds to 20ms servo frequency (50Hz)
        SERVO_PWM_ON;
        overflows = 0;
    }

    if (overflows == 1){                     // corresponds to 1ms servo pulse minimum
        flag = 1;
        // adds in the compare B match interrupt ISR the remaining part of servo pulse on top of this minimum 1ms pulse
        // can be anything between 0ms and 1ms (or 0-255 steps according to timer1 setup)
        OCR1B = servo_pwm_duty_int;          // set Timer/Counter1 Output Compare RegisterB (datasheet page 91)
        TCNT1 = 0;                           // reset counter for the upcoming match with OCR1B
    }

}

ISR(TIMER1_COMPB_vect){

    // interrupt service routine for timer1 compare match based on OCR1B value
    // this match happens anywhere between 1ms and 2ms after SERVO_PWM_ON
    // the match is varying depending on the ADC value read from the potentiometer which is mapped to servo_pwm_duty_int (256 steps)
    
    if (flag){
        SERVO_PWM_OFF;
        flag = 0;
    }    
    
}
