// scheduler.cpp

#include "scheduler.h"

namespace Core {

Scheduler::Scheduler() = default;

int Scheduler::registerTask(TaskFunction fn, uint32_t period_ticks) {
    if (task_count_ >= kMaxTasks) return -1;

    int idx = task_count_++;
    tasks_[idx].fn = fn;
    tasks_[idx].period_ticks = period_ticks;
    tasks_[idx].next_due_tick = period_ticks; // first run after one full period
    tasks_[idx].due = false;
    tasks_[idx].registered = true;
    return idx;
}

void Scheduler::onTimerTick() {
    tick_count_++;

    for (int i = 0; i < task_count_; ++i) {
        Task& t = tasks_[i];
        if (!t.registered) continue;

        if (tick_count_ >= t.next_due_tick) {
            t.due = true;
            // Schedule next occurrence relative to when it was due, not
            // when it actually ran, to avoid slow drift accumulating
            // across a long flight.
            t.next_due_tick += t.period_ticks;
        }
    }
}

void Scheduler::run() {
    for (int i = 0; i < task_count_; ++i) {
        Task& t = tasks_[i];
        if (t.due) {
            t.due = false;
            if (t.fn) t.fn();
        }
    }
}

} // namespace Core
