#ifndef __SCHED_H_
#define __SCHED_H_

#include <stdint.h>

typedef void (*task_fn)(void);

#define MAX_TASK_NAME 16

enum task_mode {
  PERIODIC,
  ONE_SHOT,
  BACKGROUND
};

void sched_loop(uint32_t now);
void sched_update(uint32_t now);
void sched_init(void);
int sched_add(task_fn task, enum task_mode mode, uint32_t period, uint8_t prio);
int sched_remove(int task_id);


#endif
