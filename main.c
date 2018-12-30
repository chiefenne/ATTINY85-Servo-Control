/*
 * Servo_Tester_ADC_from_POTI_and_Timer_PWM_3.c
 *
 * Created: 28.12.2018 22:30:22
 * Author : Andreas Ennemoser
 */ 

// default clock speed on ATTINY85 is approx. 8MHz
// if fuse CKDIV8 is set (factory default), a prescaler of 8 is used --> would result in 1MHz clock
// actual frequency can be measured @PB4 with CKOUT fuse set

//                  -------
//  RESET / PB5 ---|       |--- VCC
//   ADC3 / PB3 ---|       |--- PB2 / SCK
//          PB4 ---|       |--- PB1 / MISO
//          GND ---|       |--- PB0 / MOSI
//                  -------

// includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// macros
#define SERVO_PORT PB0
#define SERVO_PWM_ON PORTB |= (1 << SERVO_PORT)
#define SERVO_PWM_OFF PORTB &= ~(1 << SERVO_PORT)
#define LED_PORT PB4
#define LED_ON PORTB |= (1 << LED_PORT)
#define LED_OFF PORTB &= ~(1 << LED_PORT)

// variables
float servo_pwm_duty;
volatile uint8_t servo_pwm_duty_int;
volatile uint16_t adc_value;
volatile uint8_t overflows = 0;              // count number of timer1 overflows
volatile uint8_t flag = 0;

void Init_ADC(){

    /* this function initializes the ADC
    https://www.marcelpost.com/wiki/index.php/ATtiny85_ADC  
    */

    ADMUX =
              (0 << ADLAR) |                 // do NOT left shift result when reading ADC (see ADC_Read)!!!
              (0 << REFS1) |                 // Sets ref. voltage to VCC, bit 1
              (0 << REFS0) |                 // Sets ref. voltage to VCC, bit 0
              (0 << MUX3)  |                 // use ADC3 for input (PB3), MUX bit 3
              (0 << MUX2)  |                 // use ADC3 for input (PB3), MUX bit 2
              (1 << MUX1)  |                 // use ADC3 for input (PB3), MUX bit 1
              (1 << MUX0);                   // use ADC3 for input (PB3), MUX bit 0

    /*
    By default, the successive approximation circuitry requires an input clock frequency between 50 kHz and 200 kHz
    to get maximum resolution.
    ATtiny25/45/85 [DATASHEET] --> page 125
    
    8MHz CPU with ADC prescaler of 64:  8000000 / 64 = 125000 --> 125 kHz
    */

    ADCSRA =                                 // ADC Control and Status Register A (data sheet page 136)
              (1 << ADEN)  |                 // Enable ADC 
              (1 << ADPS2) |                 // set ADC prescaler to 64, bit 2 
              (1 << ADPS1) |                 // set ADC prescaler to 64, bit 1 
              (0 << ADPS0);                  // set ADC prescaler to 64, bit 0  
}

uint16_t Read_ADC(void){
    
    // this function reads analog values through ADC

    uint16_t adc_result;

    ADCSRA |= (1 << ADSC);                   // start ADC measurement
    while (ADCSRA & (1 << ADSC) );           // wait till conversion complete
    
    adc_result = ADC;                        // if not reading ADCL and ADCH do NOT left shift result in ADC_Init!!!!! (the 16bit ADC is correctly populated by the AVRgcc toolchain)
    
    return adc_result;

}

float Map_ADC(float adc_value, float adc_min, float adc_max, float map_min, float map_max) {
    
    // function that maps the ADC reading
    
    float result;
    
    result = map_min + adc_value * (map_max - map_min) / (adc_max - adc_min);
    
    return result;
    
}

void Init_PORT() {

    DDRB |= (1 << LED_PORT);                 // set led port as output
    DDRB |= (1 << SERVO_PORT);               // set servo port as output

}

void Init_TIMER(){
    // solution derived from this idea: "https://www.avrfreaks.net/comment/810846#comment-810846"
    TCCR1 |= (1 << CS12) | (1 << CS11);      // set prescaler of timer1 to 32 --> 250kHz

}

void Init_INTERRUPTS(){
    
    TIMSK |= (1 << TOIE1);                   // enable overflow interrupt of timer1 (page 92)
    
    // did not work with A!!!!!
    TIMSK |= (1 << OCIE1B);                  // enable compare match B interrupt of timer1 (page 92)
    
    sei(); // ist hier zwingend, sonst geht der interrupt nicht

}

void LED_Control(){
    if (servo_pwm_duty_int > 127) {
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
    Init_INTERRUPTS();
    
    while(1)
    {

        adc_value = Read_ADC();              // get 10-bit ADC result (0-1023)
        // adc_value = 400;

        servo_pwm_duty = Map_ADC(adc_value, 0, 1023, 1, 254);
        servo_pwm_duty_int = (int)servo_pwm_duty;
        
        LED_Control();
                
    }        
   
}

ISR(TIMER1_OVF_vect){
    // solution derived from this idea: "https://www.avrfreaks.net/comment/810846#comment-810846"

    // 1 interrupt = 1,02ms

    overflows++;
    
    if (overflows == 20){
        SERVO_PWM_ON;
        overflows = 0;
    }

    if (overflows == 1){
        flag = 1;
        OCR1B = servo_pwm_duty_int;
        TCNT1 = 0;
    }

}

ISR(TIMER1_COMPB_vect){
    
    if (flag){
        SERVO_PWM_OFF;
        flag = 0;
    }    
    
}
