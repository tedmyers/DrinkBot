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
#define RELAY_PIN   PB0     // relay: controls drink flow (on/off) - can be P1?

// Switches: Red and Green, both with an integrated LED
#define SW_PORT         PORTC   // port switches are connected to
#define SW_DDR          DDRC    // port data direction register
#define SW_PIN          PINC    // pin for button port
#define SW_GREEN        PC3     // Switch 1
#define SW_RED          PC0     // Switch 2
#define SW_RED_LED      PC1     // LED for switch 1
#define SW_GREEN_LED    PC2     // LED for switch 2
#define SW_INT_VECT     PCINT1_vect       // Interrupt vector that corresponds to SW_port pins    (Ex: PCINT1_vect)
#define SW_INT_FLAG     0   // Interrupt flag to set                                (Ex: PCIE1)

// Analog Potentiometer
#define POT_PORT PORTC
#define POT_DDR  DDRC
//#define POT_PIN  PINC
#define POT      PC4

//boolean values for input to pourDrink function
#define POUR_ON     true
#define POUR_OFF    false

volatile bool pour_drink = false;
volatile int pourTime; // time pump is on (in milliseconds)

void initIO(void)
{
    // set relay pin as output
    RELAY_DDR   |= _BV(RELAY_PIN);
    
    // initialize switches as inputs
    SW_DDR      &= ~( _BV(SW_GREEN) | _BV(SW_RED) );
    
    // initialize leds in buttons as outputs
    SW_DDR      |= ( _BV(SW_GREEN_LED) | _BV(SW_RED_LED) );
    
    // initialize potentiometer as input
    POT_DDR     |= _BV(POT);
    
    // Enable internal pull-up resistors on switches
    SW_PORT     |= _BV(SW_RED) | _BV(SW_GREEN);
    
    // Make sure relay is off, just in case
    RELAY_PORT  &= ~_BV(RELAY_PIN);
    
    //Enable interrupts
    PCICR       |= _BV(SW_INT_FLAG);
    PCMSK1      |= _BV(SW_RED) | _BV(SW_GREEN);
}

//pours for pourTime milliseconds while (volatile) pour is true
void pourDrink_timed(void) // uses volatile global variables to pour drink
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

#if 0

ISR(SW_INT_VECT) // put in vector
{
    switch ( SW_PIN )
    {
        case _BV(SW_GREEN):                   // green button pressed
            
        case _BV(SW_RED):                     // red button pressed
            
        case ( _BV(SW_GREEN) | _BV(SW_RED) ):   // both switched pressed
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

#endif


int main(void)
{
    initIO();
    
#if 0
    
    // enable interrupts
    sei();
    
    if (pour_drink)
        pourDrink();
    
    // loops forever
    for (;;)
    {
        if ( SW_PIN & _BV(SW_GREEN) == 0 )  // switch pressed (brought low)
        {
            RELAY_PORT  |=  _BV(RELAY_PIN);     // pour drink
            SW_PORT     |=  _BV(SW_GREEN_LED);  // turn light on
        }
        else
        {
            RELAY_PORT  &= ~_BV(RELAY_PIN);     // stop pouring
            SW_PORT     &= ~_BV(SW_GREEN_LED);  // turn off LED
        }
    }
    
#endif
    
    
    // for now, just toggle the relay pin (for HW debugging)
    while(1)
    {
        _delay_ms(500); // wait a second
        
        RELAY_PORT ^= _BV(RELAY_PIN); // use xor to toggle pin
        
    }
    
    return 0;   // never reached
}

/*********************************
 
 ----------
 PC6    -| 1     28 |-   PC5
 PD0    -| 2     27 |-   PC4
 PD1    -| 3     26 |-   PC3
 PD2    -| 4     25 |-   PC2
 PD3    -| 5     24 |-   PC1
 PD4    -| 6     23 |-   PC0
 VCC    -| 7     22 |-   GND
 GND    -| 8     21 |-   AREF
 PB6    -| 9     20 |-   AVCC
 PB7    -| 10    19 |-   PB5
 PD5    -| 11    18 |-   PB4
 PD6    -| 12    17 |-   PB3
 PD7    -| 13    16 |-   PB2
 PB0    -| 14    15 |-   PB1
 ----------
 
 
 PIN CONNECTIONS
 
 Device             Pin     Wire Color
 ------             ---     ----------
 Red Switch LED     PC1     Grey wire
 Red Switch         PC0     White wire
 Green Switch LED   PC2     Purple wire
 Green Switch       PC3     Blue wire
 Relay              PB0     N/A (green on board)
 Potentiometer      PC4     Tan wire (GND = brown, +5 = Red)
 
 
 
 *******************************/

