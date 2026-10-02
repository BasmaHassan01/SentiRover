/**
 * @file TaskScheduler.cpp
 * @brief Implementation of Deterministic Cooperative Task Scheduler
 * @author Senior Embedded Systems Engineer
 */

#include "TaskScheduler.h"

TaskScheduler::TaskScheduler() : _taskCount(0) {
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        _tasks[i].callback = nullptr;
        _tasks[i].periodMs = 0;
        _tasks[i].lastRunMs = 0;
        _tasks[i].enabled = false;
    }
}

int TaskScheduler::addTask(TaskCallback callback, unsigned long periodMs, bool enabled) {
    if (_taskCount >= MAX_TASKS || callback == nullptr) {
        return -1; // Pool exhausted or invalid callback
    }

    uint8_t id = _taskCount;
    _tasks[id].callback = callback;
    _tasks[id].periodMs = periodMs;
    _tasks[id].lastRunMs = millis();
    _tasks[id].enabled = enabled;
    _taskCount++;

    return id;
}

void TaskScheduler::setTaskEnabled(int taskId, bool enabled) {
    if (taskId >= 0 && taskId < _taskCount) {
        _tasks[taskId].enabled = enabled;
    }
}

void TaskScheduler::setTaskPeriod(int taskId, unsigned long periodMs) {
    if (taskId >= 0 && taskId < _taskCount) {
        _tasks[taskId].periodMs = periodMs;
    }
}

void TaskScheduler::tick() {
    unsigned long now = millis();

    for (uint8_t i = 0; i < _taskCount; i++) {
        if (!_tasks[i].enabled || _tasks[i].callback == nullptr) {
            continue;
        }

        // Handle unsigned long millis overflow protection
        if ((now - _tasks[i].lastRunMs) >= _tasks[i].periodMs) {
            _tasks[i].lastRunMs = now;
            _tasks[i].callback(); // Execute task callback
        }
    }
}
