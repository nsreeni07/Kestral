// scheduler.h
// Fixed-rate cooperative task scheduler driven by a single hardware
// timer tick (no RTOS). Chosen over FreeRTOS for a first-flight
// computer: fewer moving parts to audit for determinism, no priority
// inversion risk across only 5 periodic tasks, and simpler worst-case
// timing analysis. Revisit if/when GPS + telemetry radio tasks are
// added and task count/variability grows enough to justify preemption.
//
// Design: a single hardware timer (e.g. TIM6) fires an interrupt at the
// fastest required rate (1000 Hz, matching Config::Rates::IMU_HZ). The
// ISR increments a tick counter and sets flags for any task whose
// period has elapsed. The main loop polls those flags and runs the
// corresponding task function to completion (cooperative — no task
// preempts another; each task must be fast enough to finish within its
// period, which is the discipline this architecture requires from every
// module).

#pragma once

#include <cstdint>

namespace Core {

using TaskFunction = void (*)();

class Scheduler {
public:
    Scheduler();

    // Registers a task to run every `period_ticks` base ticks (base
    // tick = 1ms, i.e. 1000Hz, matching the fastest required rate).
    // Returns a task handle (index) or -1 if the task table is full.
    int registerTask(TaskFunction fn, uint32_t period_ticks);

    // Call this from the base timer ISR (TIM6_IRQHandler or similar).
    // Increments the tick counter and sets due-flags for elapsed tasks.
    void onTimerTick();

    // Call this continuously from main(). Runs any due tasks in
    // registration order, then clears their due-flags. Should be the
    // entire body of the superloop.
    void run();

    // Monotonic millisecond counter, incremented once per base tick.
    // Used by every module needing wall-clock timestamps (state
    // machine, logger, LED blink timing).
    uint32_t nowMs() const { return tick_count_; }

private:
    static constexpr int kMaxTasks = 8;

    struct Task {
        TaskFunction fn = nullptr;
        uint32_t period_ticks = 0;
        uint32_t next_due_tick = 0;
        volatile bool due = false;
        bool registered = false;
    };

    Task tasks_[kMaxTasks];
    int task_count_ = 0;
    volatile uint32_t tick_count_ = 0;
};

} // namespace Core
