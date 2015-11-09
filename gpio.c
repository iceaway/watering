#include "gpio.h"

const struct dpin pinmap[] = {
  { 2,  "Relay", PD2, &PORTD, &DDRD, &PIND },
  { 9,  "Motor ctrl", PB1, &PORTB, &DDRB, &PINB },
  { 13, "LED", PB5, &PORTB, &DDRB, &PINB }
};

