/*
 * Implement the following routines to set GPIO pins to input or output,
 * and to read (input) and write (output) them.
 *
 * DO NOT USE loads and stores directly: only use GET32 and PUT32 
 * to read and write memory.  Use the minimal number of such calls.
 *
 * See rpi.h in this directory for the definitions.
 */
#include "rpi.h"

// see broadcomm documents for magic addresses.
#define GPIO_BASE 0x20200000
static const unsigned gpio_set0 = (GPIO_BASE + 0x1C);
static const unsigned gpio_clr0 = (GPIO_BASE + 0x28);
static const unsigned gpio_lev0 = (GPIO_BASE + 0x34);

unsigned get_gpio_fsel0(unsigned pin)
{
    // Which GPIO Function Select pin lives in e.g. pin 20 is in GPFSEL2
    unsigned fs_n = pin / 10;
    // Return FSEL0 of the GPFSELn
    return GPIO_BASE + (fs_n * sizeof(unsigned));
}

//
// Part 1 implement gpio_set_on, gpio_set_off, gpio_set_output
//

// set <pin> to be an output pin.
//
// note: fsel0, fsel1, fsel2 are contiguous in memory, so you
// can (and should) use array calculations!
void gpio_set_output(unsigned pin)
{
    // implement this
    // use <gpio_fsel0>
    unsigned gpio_fsel0 = get_gpio_fsel0(pin);
    unsigned pin_bits = (pin % 10) * 3; // e.g. pin 21: 3 i.e. FSEL1
    // assume: we want to set the bits 7,8 in <x> to v and
    // leave everything else undisturbed.
    //  x &=  ~(0b11 << 7);   // clear the bits 7, 8  in x
    //  x |=   (v << 7);   // or in the new bits
    unsigned value = GET32(gpio_fsel0);
    value &= ~(0b111 << pin_bits); // clear bits e.g. 0, 1, 2
    value |= (0b001 << pin_bits);  // or in the new bits
    PUT32(gpio_fsel0, value);
}

// set GPIO <pin> on.
void gpio_set_on(unsigned pin)
{
    // implement this
    // use <gpio_set0>
    PUT32(gpio_set0, (1 << pin));
}

// set GPIO <pin> off
void gpio_set_off(unsigned pin)
{
    // implement this
    // use <gpio_clr0>
    PUT32(gpio_clr0, (1 << pin));
}

// set <pin> to <v> (v \in {0,1})
void gpio_write(unsigned pin, unsigned v)
{
    if (v)
        gpio_set_on(pin);
    else
        gpio_set_off(pin);
}

//
// Part 2: implement gpio_set_input and gpio_read
//

// set <pin> to input.
void gpio_set_input(unsigned pin)
{
    // implement.
    unsigned gpio_fsel0 = get_gpio_fsel0(pin);
    unsigned pin_bits = (pin % 10) * 3; // e.g. pin 21: 3 i.e. FSEL1
    // assume: we want to set the bits 7,8 in <x> to v and
    // leave everything else undisturbed.
    //  x &=  ~(0b11 << 7);   // clear the bits 7, 8  in x
    //  x |=   (v << 7);   // or in the new bits
    unsigned value = GET32(gpio_fsel0);
    value &= ~(0b111 << pin_bits); // clear bits e.g. 0, 1, 2
    PUT32(gpio_fsel0, value);
}

// return the value of <pin>
int gpio_read(unsigned pin)
{
    unsigned v = 0;
    // the gpio pin level registers return the actual value of the pin
    // implement.
    unsigned mask = (1 << pin);
    unsigned value = (GET32(gpio_lev0) & mask) >> pin;
    return value;
}
