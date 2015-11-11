#ifndef __GPIO_H_
#define __GPIO_H_

enum gpio_ddr {
  GPIO_INPUT,
  GPIO_OUTPUT
};

enum gpio_mode {
  GPIO_HIGH,
  GPIO_LOW
};

enum gpio_pullup {
  GPIO_PULLUP,
  GPIO_NOPULLUP
};


int gpio_ddr(int pin, enum gpio_ddr ddr);
int gpio_mode(int pin, enum gpio_mode mode);
int gpio_pullup(int pin, enum gpio_pullup pup);
int gpio_read(int pin, enum gpio_pullup *value);

#endif
