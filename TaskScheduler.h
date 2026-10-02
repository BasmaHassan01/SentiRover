/**
 * @file TaskScheduler.h
 * @brief Deterministic Cooperative Non-Blocking Task Scheduler
 * @author Senior Embedded Systems Engineer
 *
 * Implements a zero-heap-allocation cooperative scheduler for resource-constrained
 * 8-bit AVR microcontrollers. Replaces blocking delay() loops with a time-triggered
 * tick architecture.
 */

#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include <Arduino.h>

typedef void (*TaskCallback)();

struct Task {
    TaskCallback callback;      // Pointer to task function
    unsigned long periodMs;     // Execution period in milliseconds
    unsigned long lastRunMs;    // Timestamp of last execution
    bool enabled;               // Active state flag
};

class TaskScheduler {
public:
    static const uint8_t MAX_TASKS = 8;

    TaskScheduler();

    /**
     * @brief Registers a periodic task in the schedule allocation table.
     * @param callback Function pointer to execution block.
     * @param periodMs Interval between task runs in milliseconds.
     * @return int Task ID (0 to MAX_TASKS-1), or -1 if allocator pool full.
     */
    int addTask(TaskCallback callback, unsigned long periodMs, bool enabled = true);

    /**
     * @brief Enables or disables a registered task.
     */
    void setTaskEnabled(int taskId, bool enabled);

    /**
     * @brief Modifies a registered task execution period dynamically.
     */
    void setTaskPeriod(int taskId, unsigned long periodMs);

    /**
     * @brief Dispatches ready tasks based on system millisecond timer ticks.
     * Must be called continuously in loop().
     */
    void tick();

private:
    Task _tasks[MAX_TASKS];
    uint8_t _taskCount;
};

#endif // TASK_SCHEDULER_H
