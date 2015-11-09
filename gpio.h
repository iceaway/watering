#ifndef __GPIO_H_
#define __GPIO_H_

struct dpin {
  int pin;
  char *name;
  uint8_t bitno;
  volatile uint8_t *port;
  volatile uint8_t *ddr;
  volatile uint8_t *input;
};

#endif
