/**********************************************
 
    Firmware for the DrinkBot Device
                2/27/2016
 
        Ted Myers & Noah Johnson
 
**********************************************/

// Currently untested code


// Basic idea: code runs an Atmega328p that is connected to:
// -Two buttons w/integrated LEDs
// -One pushbutton rotary encoder
// -Display LEDs?
// -Decorative lighting LEDs
// -Buzzer/speaker?
// -Electromechanical relay, turns on/off pump

// Pours a drink for a user-defined length of time

// To do:
// -Decide and solder all devices, update pins in code
// -Decide about lighting and visual indication
// -Write code for everything
// -



#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h> // for boolean values

#define RELAY_PORT PORTB    // port relay connected to
#define RELAY_DDR DDRB      // port data direction register
#define RELAY_PIN PB1       // relay: controls drink flow (on/off) - can be P1?

#define SW_PORT 0   // port switches are on
#define SW_DDR 0    // port data direction register
#define SW_GREEN 0       // Switch 1 (green?)
#define SW_RED 0       // Switch 2 (red?)
#define SW_RED_LED 0   // LED for switch 1
#define SW_GREEN_LED 0   // LED for switch 2

#define ENC_PORT 0  // port encoders are on
#define ENC_DDR 0   // port data direction register
#define ENCA 0      // Encoder pin A
#define ENCB 0      // Encoder pin B
#define ENC_SW 0    // Switch for encoder

#define DISP_PORT 0 // port for segmented/led display
#define DISP_DDR 0  // port data direction register
// for shift register, need clock, latch, and data; define as necessary later


#define POUR_ON true //boolean values for input to pourDrink function
#define POUR_OFF false
volatile bool pour;

volatile int pourTime; // time pump is on (in milliseconds)

void initIO(void)
{
    
    RELAY_DDR |= (1<<RELAY_PIN);                    // Set relay pin (PB1) as output (check that it is correct pin)
    SW_DDR &= ~( (1<<SW_GREEN) || (1<<SW_RED) );    // initialize switch 1 and 2 as inputs
    SW_DDR |= ( (1<<SW_GREEN_LED) || (1<<SW_RED_LED) ); // initialize leds in buttons as outputs
    ENC_DDR &= ~( (1<<ENCA) || (1<<ENCB) );         // initialize encoder pins as inputs
    
    //DDRB |= (1<<PB5); // Set PortB Pin5 as an output (why? was in old code)
}

void moodLight(int state)
{
    // light LEDs (choose and attach)
}

void pourDrink(void) // uses volatile global variables to pour drink
{
    //pours for pourTime milliseconds while pour is true
    
    // while ( pourTimer && pour )
    // {
    //      // LED fade on/off, like a heartbeat
    // |
    
}


int main(void)
{
	initIO();
    
    moodLight(1); // light decorative LEDs
    
    // check encoder movement (ISR)
    // define a volatile encoder value, map to time in milliseconds (pourTime)
    // light LED indicator with the value
    
    // check if "pour" button has been pressed (ISR?)
    // light button while being pressed
    // if so, pour for pourTime milliseconds
    // light LEDs in interesting manner while pouring
    
    // check if "stop" button has been pressed (ISR?)
    // stop any pouring happening at that time; turn off lights too?
    // light button while pressed
    

    // for now, just toggle the relay pin (for HW debugging)
    while(1)
    {
        _delay_ms(1000); // wait a second
        
        //PORTB ^= (1<<PORTB5); // toggle PORTB5
        RELAY_PORT ^= _BV(RELAY_PIN); // use xor to toggle pin
        
    }
    
    return 0;   // never reached
}