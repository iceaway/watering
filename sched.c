#include <string.h>
#include "sched.h"

#define MAX_TASKS 8

struct task {
  char name[MAX_TASK_NAME];
  task_fn fn;
  enum task_mode mode;
  uint32_t period;
  uint32_t lastrun;
  uint8_t priority; /* Priority, 0 = high, 255 = low */
  uint8_t run_flag;
  uint8_t enabled;
};

struct task g_tasks[MAX_TASKS];

void sched_init(void)
{
  memset(g_tasks, 0, sizeof(g_tasks));
}

void sched_loop(uint32_t now)
{
  int i;
  for (i = 0; i < MAX_TASKS; ++i) {
    if (g_tasks[i].enabled && g_tasks[i].run_flag) {
      g_tasks[i].lastrun = now;
      if (g_tasks[i].mode != BACKGROUND)
        g_tasks[i].run_flag = 0;
      g_tasks[i].fn();
      if (g_tasks[i].mode == ONE_SHOT)
        g_tasks[i].enabled = 0;
    }
  }
}

void sched_update(uint32_t now)
{
  int i;

  for (i = 0; i < MAX_TASKS; ++i) {
    if (((now - g_tasks[i].lastrun) > g_tasks[i].period) && 
        (g_tasks[i].mode != BACKGROUND)) {
      g_tasks[i].run_flag = 1;
    }
  }
}

int sched_add(task_fn task, enum task_mode mode, uint32_t period, uint8_t prio)
{
  int i;

  for (i = 0; i < 0; ++i) {
    if (g_tasks[i].fn == NULL) {
      g_tasks[i].fn = task;
      g_tasks[i].mode = mode;
      g_tasks[i].period = period;
      g_tasks[i].priority = prio;
      g_tasks[i].enabled = 1;
      return i;
    }
  }

  return -1;
}

int sched_remove(int task_id)
{
  if ((task_id < 0) || (task_id > MAX_TASKS)) {
    return -1;
  } else {
    memset(&g_tasks[task_id], 0, sizeof(g_tasks[0]));
    return 0;
  }
}
