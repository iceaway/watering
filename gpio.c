#include <stdint.h>
#include <avr/io.h>
#include <stdlib.h>
#include "gpio.h"

struct dpin {
  int pin;
  char *name;
  uint8_t bitno;
  volatile uint8_t *port;
  volatile uint8_t *ddr;
  volatile uint8_t *input;
};

const struct dpin pinmap[] = {
  { 2,  "Relay", PD2, &PORTD, &DDRD, &PIND },
  { 9,  "Motor ctrl", PB1, &PORTB, &DDRB, &PINB },
  { 13, "LED", PB5, &PORTB, &DDRB, &PINB },
  { 0, NULL, 0, NULL, NULL, NULL }
};

const struct dpin *find_pin(int pin)
{
  int i = 0;
  while (pinmap[i].name != NULL) {
    if (pinmap[i].pin == pin)
      return &pinmap[i];
    ++i;
  }

  return NULL;

}

int gpio_ddr(int pin, enum gpio_ddr ddr)
{
  const struct dpin *p = find_pin(pin);
  if (p) {
    if (ddr == GPIO_OUTPUT)
      *p->ddr |= (1 << p->bitno);
    else
      *p->ddr &= ~(1 << p->bitno);

    return 0;
  } else {
    return -1;
  }
}

int gpio_mode(int pin, enum gpio_mode mode)
{
  const struct dpin *p = find_pin(pin);
  if (p) {
    if (mode == GPIO_HIGH)
      *p->port |= (1 << p->bitno);
    else
      *p->port &= ~(1 << p->bitno);

    return 0;
  } else {
    return -1;
  }
}

int gpio_pullup(int pin, enum gpio_pullup pup)
{
  const struct dpin *p = find_pin(pin);
  if (p) {
    if (pup == GPIO_PULLUP)
      *p->port |= (1 << p->bitno);
    else
      *p->port &= ~(1 << p->bitno);

    return 0;
  } else {
    return -1;
  }
}

int gpio_read(int pin, enum gpio_pullup *value)
{
  const struct dpin *p = find_pin(pin);
  if (p) {
    if (*p->input & (1 << p->bitno ))
      *value = 1;
    else
      *value = 0;

    return 0;
  } else {
    return -1;
  }
}

