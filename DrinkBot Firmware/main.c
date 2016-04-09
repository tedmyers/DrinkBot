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

// Relay: controls drink pouring
#define RELAY_PORT  PORTB   // port relay connected to
#define RELAY_DDR   DDRB    // port data direction register
#define RELAY_PIN   PB1     // relay: controls drink flow (on/off) - can be P1?

// Switches: Red and Green, both with an integrated LED
#define SW_PORT         0   // port switches are connected to
#define SW_DDR          0   // port data direction register
#define SW_PIN          0   // pin for button port
#define SW_GREEN        0   // Switch 1
#define SW_RED          0   // Switch 2
#define SW_RED_LED      0   // LED for switch 1
#define SW_GREEN_LED    0   // LED for switch 2
#define SW_INT_VECT     0   // Interrupt vector that corresponds to SW_port pins    (Ex: PCINT1_vect)
#define SW_INT_FLAG     0   // Interrupt flag to set                                (Ex: PCIE1)

// Analog Potentiometer
#define POT_PORT 0
#define POT_PIN  0

// pouring
#define POUR_ON true //boolean values for input to pourDrink function
#define POUR_OFF false

volatile bool pour_drink = false;
volatile int pourTime; // time pump is on (in milliseconds)

void initIO(void)
{
    // Set Pins as Inputs or Outputs
    RELAY_DDR   |= _BV(RELAY_PIN);                    // Set relay pin (PB1) as output (check that it is correct pin)
    SW_DDR      &= ~( _BV(SW_GREEN) | _BV(SW_RED) );    // initialize switch 1 and 2 as inputs
    SW_DDR      |= ( _BV(SW_GREEN_LED) | _BV(SW_RED_LED) ); // initialize leds in buttons as outputs

    // Enable internal pull-up resistors on switches
    SW_PORT     |= _BV(SW_RED) | _BV(SW_GREEN);
    
    // Make sure relay is off, just in case
    RELAY_PORT  &= ~_BV(RELAY_PIN);
    
    //Enable interrupt
    PCICR       |= _BV(SW_INT_FLAG);
    PCMSK1      |= _BV(SW_RED) | _BV(SW_GREEN);
    
}

/*
 
 ISR(PCINT1_vect){
 //Turn on all of port D
 PORTD |= (1 << PD5);
 }
 
 void initIO(void) {
 //Define PC3 as input with pullup-resistor
 DDRC &= ~(1 << button);
 PORTC |= (1 << button);
 
 //Enable interrupt for PCINT11
 PCICR |= _BV(PCIE1);
 PCMSK1 |= (_BV(button_interrupt));
 
 */

//pours for pourTime milliseconds while (volatile) pour is true
void pourDrink(void) // uses volatile global variables to pour drink
{

    // read analog value
    
    int pourCounter = pourTime;
    
    // pour drink, check every ms
    while ( pour_drink && pourCounter )
    {
        RELAY_PORT |= _BV(RELAY_PIN);
        _delay_ms(1);
        pourCounter--;
    }
    
    RELAY_PORT &= ~_BV(RELAY_PIN); // stop pouring
    
}

ISR(SW_INT_VECT) // put in vector
{
    switch ( SW_PIN )
    {
        case 1<<SW_GREEN:                   // green button pressed
            
        case 1<<SW_RED:                     // red switch pressed
            
        case _BV(SW_GREEN) | _BV(SW_RED):   // both switched pressed
            // Only green switch pressed
    }
    
    
    // ISR: if either "pour" button has been pressed
    // light correct button while being pressed
    // If green button, pour while it is pressed
    // If red button, set pourTime to equivalent POT_PIN value
    // and set bool pour = true
    
    // GR
    // 00           All off
    // 01           Pour based on time
    // 10           Pour while pressed
    // 11           Pour while pressed, then based on time
    // 01 -> 11     Initially: pour based on time; then stop
    // 10 -> 11     Initially: pour while pressed; then stop
    // 11 -> 10/01  keep pouring
}


int main(void)
{
	initIO();
    
    if (pour_drink)
        pourDrink();
    
    
    // for now, just toggle the relay pin (for HW debugging)
    while(1)
    {
        _delay_ms(1000); // wait a second
        
        RELAY_PORT ^= _BV(RELAY_PIN); // use xor to toggle pin
        
    }
    
    return 0;   // never reached
}