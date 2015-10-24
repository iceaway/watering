#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "rbuf.h"

/* PIN 13 = PB5     LED
 * PIN 2  = PD2     Relay
 */

#define PRINTS_BUFSIZE  128
#define MAX_ARGC        8 
#define ENV_SIZE        128

struct cmd {
  char const * const cmd;
  char const * const help;
  int (*callback_fn)(int argc, char *argv[]);
};

static int prints(const char *fmt, ...);

int cmd_help(int argc, char *argv[]);
int cmd_set(int argc, char *argv[]);
int cmd_echo(int argc, char *argv[]);
int cmd_env(int argc, char *argv[]);
int cmd_pin(int argc, char *argv[]);

static int g_echo = 1;
static struct rbuf g_rxbuf;
static struct rbuf g_txbuf;
static char env[ENV_SIZE] = { 0 };

const struct cmd commands[] = {
  { "help", "print help", cmd_help },
  { "set",  "Set environment variables", cmd_set },
  { "echo", "Print environment variables", cmd_echo },
  { "env",  "Environment commands", cmd_env },
  { "pin",  "set pin no high/low", cmd_pin },
  { NULL, NULL, NULL }
};

static void led_on(void)
{
  PORTB |= (1 << PB5);
}

static void led_off(void)
{
  PORTB &= ~(1 << PB5);
}

static void transmit(void)
{
  /* Enable the TX interrupt */
  UCSR0B |= (1 << UDRIE0);
  /* Trigger interrupt for first transmission */
  UCSR0A |= (1 << UDRE0);
}

static void print_char(char data)
{
  rbuf_push(&g_txbuf, data);
  transmit();
}

ISR(TIMER1_OVF_vect)
{
  static int on = 0;
  if (on) {
    led_off();
    on = 0;
  } else {
    led_on();
    on = 1;
  }
}

ISR(USART_UDRE_vect)
{
  char tmp;
  if (rbuf_pop(&g_txbuf, &tmp)) {
    UDR0 = tmp;
  } else {
    /* Nothing more to send, disable interrupt */
    UCSR0B &= ~(1 << UDRIE0);
  }
}

ISR(USART_RX_vect)
{
  uint8_t tmp;

  tmp = UDR0;
  rbuf_push(&g_rxbuf, tmp);
}

static int print_string(char *string)
{
  while (*string) {
    while (rbuf_push(&g_txbuf, *string) != 1);
    string++;
  } 
  transmit();
  return 0;
}

static int prints(const char *fmt, ...)
{
  va_list ap;
  char buf[PRINTS_BUFSIZE];
  int size;

  va_start(ap, fmt);
  size = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  print_string(buf);
  return size;
}

#define MIN(a,b)  ((a) < (b) ? (a) : (b))

static int env_get(char const *var, char *value, size_t size)
{
  char *val = NULL;
  char *valp = NULL;
  char *startp = NULL;
  char *endp = NULL;
  int n;

  val = env;
  while ((val = strstr(val, var)) != NULL) {
    valp = val;
    ++val;
  }

  if (valp) {
    startp = strchr(valp, '=');
    ++startp;
    endp = strchr(valp, ';');
    if (!endp || !startp) {
      prints("Error in environment!\r\n");
      return 0;
    } else {
      n = MIN(endp - startp, (int)size - 1);
      strncpy(value,
          startp,
          n);

      value[n] = '\0';
      return strlen(value);
    }
  } else {
    return 0;
  }
}

static int env_set(char const *var, char const *val)
{
  unsigned int varlen = strlen(var);
  unsigned int vallen = strlen(val);

  if ((vallen + varlen) > (ENV_SIZE - strlen(env) - 2)) {
    prints("Environment is full\r\n");
    return 0;
  } else {
    return snprintf(env+strlen(env), ENV_SIZE - strlen(env), "%s=%s;", var, val);
  }
}

static int env_clear(void)
{
  memset(env, 0, sizeof(env));
  return 0;
}

int cmd_pin(int argc, char *argv[])
{
  int pin;
  enum {
    HIGH,
    LOW
  } mode;

  if (argc >= 3) {
    pin = atoi(argv[1]);
    
    if ((pin < 0) | (pin > 53)) {
      prints("\nInvalid pin number: %d\r\n", pin);
      return 0;
    }

    if ((strcmp(argv[2], "high") == 0) || 
        (strcmp(argv[2], "h") == 0)) {
      mode = HIGH;
    } else if ((strcmp(argv[2], "low") == 0) || 
               (strcmp(argv[2], "l") == 0)) {
      mode = LOW;
    } else {
      prints("\nInvalid mode: '%s'\r\n", argv[2]);
      return 0;
    }

    switch (pin) {
    case 13: /* PB5 */
      if (mode == HIGH) {
        PORTB |= (1 << PB5);
      } else if (mode == LOW) {
        PORTB &= ~(1 << PB5);
      } else {
          prints("\nError! Invalid mode\r\n");
      }
      break;

    case 2: /* PD2 */
      if (mode == HIGH) {
        PORTD |= (1 << PD2);
      } else if (mode == LOW) {
        PORTD &= ~(1 << PD2);
      } else {
          prints("\nError! Invalid mode\r\n");
      }
      break;
    default:
      prints("\nUnsupported pin: %d\r\n", pin);

    }
  } else {
    prints("\nNot enough arguments\r\n");
  }

  return 0;
}

int cmd_set(int argc, char *argv[])
{
  char *eq;
  char *var;
  char *val;

  if (argc < 2) {
    prints("Not enough arguments\r\n");
  } else {
    if ((eq = strstr(argv[1], "=")) != NULL) {
      *eq = '\0';
      var = argv[1];
      val = eq+1;
      env_set(var, val);
      *eq = '=';
    }
  }

  return 0;

}

int cmd_env(int argc, char *argv[])
{
  if (argc >= 2) {
    if (strcmp(argv[1], "dump") == 0) {
      prints(env);
      prints("\r\n");
    } else if (strcmp(argv[1], "clear") == 0) {
      env_clear();
    } else {
      prints("Unknown argument.\r\n");
    }
  } else {
    prints("Not enough arguments\r\n");
  }
  return 0;
}

int cmd_echo(int argc, char *argv[])
{
  char *var = NULL;
  char value[32] = { 0 };
  int len;

  if (argc < 2) {
    prints("Not enough arguments\r\n");
  } else {
    if (argv[1][0] == '$') {
      var = argv[1]+1;
      if ((len = env_get(var, value, sizeof(value))))
        prints("\r\n%s (%d)\r\n", value, len);
      else
        prints("Variable does not exist\r\n");

    } else {
      prints(argv[1]);
    }
  }

  return 0;
}

int cmd_help(int argc, char *argv[])
{
  const struct cmd *p;
  (void)argc;
  (void)argv;
  p = &commands[0];
  prints("Available commands:\r\n");
  while (p->cmd) {
    prints("%-15s - %s\r\n", p->cmd, p->help);
    ++p;
  } 
  return 0;
}

static int parse_args(char *buf, char *argv[])
{
  char *p;
  int argc = 0;
  p = buf;

  argv[argc++] = p;

  while (*p) {
    if ((*p == ' ') && (*(p+1) != '\0')) {
      *p = '\0';
      argv[argc++] = p+1;
    }

    if (argc >= MAX_ARGC) {
      argv[argc] = 0;
      break;
    }
    ++p;
  }

  return argc;
}

static int parse_cmd(char data)
{
  static char buf[RBUF_SIZE];
  static int idx = 0;
  char *argv[MAX_ARGC+1];
  int argc = 0;
  const struct cmd *p;
  int len;
  int ret = 0;
  int process = 0;

  if (data != '\r') {
    buf[idx++] = data;
    if (idx >= (RBUF_SIZE-1)) {
      buf[idx] = '\0';
      len = idx;
      idx = 0;
      process = 1;
    }
  } else {
    process = 1;
    buf[idx] = '\0';
    len = idx;
    idx = 0;
  }
  
  if (process) {
    if (len != 0) {
      argc = parse_args(buf, argv);
      
      p = &commands[0];
      while (p->cmd) {
        if (strncmp(p->cmd, buf, sizeof(buf)) == 0) {
          p->callback_fn(argc, argv);
          break;
        }
        ++p;
      }

      if (!p->cmd)
        prints("\nUnknown command\r\n");

    }
    ret = 1;
    memset(buf, 0, sizeof(buf));
  }

  return ret; 
}

static void echo(char data)
{
  if (g_echo) {
    print_char(data);
  }
}

static void init_wd(void)
{
  /* Enable timer overflow interrupt */
  TIMSK1 = (1 << TOIE1);
  
  /* Prescaler = clk / 1024 */
  TCCR1B = (1 << CS11) | (1 << CS10);
}

static void init_gpio(void)
{
  /* Set PB5 as output for LED */
  DDRB |= (1 << DDB5);

  /* Set PD2 as output for relay */
  DDRD |= (1 << DDD2);


  /* Start with pin low */
  //PORTB &= ~(1 << PB5);

  /* Disable pull-ups */
  //PORTD &= ~((1 << PD2) | (1 << PD1) | (1 << PD0));
#if 0
  /* Trigger on falling edge for D0 and D1 */
  EICRA |= (1 << ISC01) | (1 << ISC11);

  /* Any level change for CardLoad */
  EICRA |= (1 << ISC20);
  
  /* Clear any outstanding interrupts */
  EIFR = (1 << INT0) | (1 << INT1) | (1 << INT2);

  /* Interrupt enable mask */
  EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2);

  /* For blinking the diode */
  DDRB |= _BV(DDB7);
#endif

}

static void init_usart(void)
{
  /* Set baud rate to 9600 */
  UBRR0H = 0;
  UBRR0L = 103; /* for 9600, required for USART0 (USB) */

  /* Enable TX and RX */
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  /* Set 8N1 frame format */
  UCSR0C = (1 << USBS0) | (3 << UCSZ00);

  /* Enable RX interrupt */
  UCSR0B |= (1 << RXCIE0);

}

int main(void)
{
  char tmp;

  rbuf_init(&g_rxbuf);
  rbuf_init(&g_txbuf);

  init_wd();
  init_gpio();

  init_usart();

  /* Enable interrupts globally */
  sei(); 
  prints("\r\nWelcome to PellShell\r\n");
  prints(">> ");
  while(1) {
    if (rbuf_pop(&g_rxbuf, &tmp)) {
      echo(tmp);
      if (parse_cmd(tmp))
        prints("\n>> ");
    }
  }
}