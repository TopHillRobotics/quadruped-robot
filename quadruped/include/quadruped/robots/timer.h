/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: the time tic-toc
* Author: Zhu Yijie
* Create: 2021-11-08
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*         add mit timer. @ Zhu Yijie 2022.04.01
*/

#ifndef ASCEND_QUADRUPED_CPP_TIMER_TIMER_H
#define ASCEND_QUADRUPED_CPP_TIMER_TIMER_H
#include <assert.h>
#include <stdint.h>
#include <time.h>
#include <ros/ros.h>

class Timer {
public:
    Timer()
    {
        start = clock();
        startTime = (double)start;
    }

    virtual ~Timer() = default;

    virtual double GetTimeSinceReset()
    {
        finish = clock();
        double timeSinceReset = (double)(finish - startTime) / CLOCKS_PER_SEC; // second(s)
        return timeSinceReset;
    }

    virtual double ResetStartTime()
    {
        start = clock();
        startTime = (double)start;
        return startTime;
    }

private:
    clock_t start;
    clock_t finish;
    double startTime;

};

class RosTimer : public Timer {
public:
    RosTimer()
    {
        startRos = ros::Time::now().toSec();
    }

    virtual ~RosTimer() = default;

    virtual double GetTimeSinceReset()
    {
        double timeSinceReset = ros::Time::now().toSec() - startRos; // second(s)
        return timeSinceReset;
    }

    virtual double ResetStartTime()
    {
        startRos = ros::Time::now().toSec();
        return startRos;
    }

private:
    double startRos;
};


class TimerInterface {
public:
    TimerInterface(bool useRosTimeIn=false)
    : useRosTime(useRosTimeIn), timerPtr(nullptr)
    {
        
        if (useRosTime) {
            timerPtr = new RosTimer();
        } else {
            timerPtr = new Timer();
        }

        startTime = 0;
        timeSinceReset = 0;
    }

    ~TimerInterface()
    {
        delete timerPtr;
    }

    double GetTimeSinceReset()
    {
        timeSinceReset = timerPtr->GetTimeSinceReset();
        return timeSinceReset;
    }

    void ResetStartTime()
    {
        startTime = timerPtr->ResetStartTime();
    }

private:
    bool useRosTime;
    Timer* timerPtr;
    double startTime;
    double timeSinceReset;
};

/*!
 * Timer for measuring time elapsed with clock_monotonic
 */
class MITTimer {
 public:

  /*!
   * Construct and start timer
   */
  explicit MITTimer() { start(); }

  /*!
   * Start the timer
   */
  void start() { clock_gettime(CLOCK_MONOTONIC, &_startTime); }

  /*!
   * Get milliseconds elapsed
   */
  double getMs() { return (double)getNs() / 1.e6; }

  /*!
   * Get nanoseconds elapsed
   */
  int64_t getNs() {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)(now.tv_nsec - _startTime.tv_nsec) +
           1000000000 * (now.tv_sec - _startTime.tv_sec);
  }

  /*!
   * Get seconds elapsed
   */
  double getSeconds() { return (double)getNs() / 1.e9; }

  struct timespec _startTime;
};


/*
#include <stdlib.h>
#include <sys/time.h>

class Timer {
public:
    Timer()
    {
        gettimeofday(&start, NULL);
        startTime = 1000000 * start.tv_sec + start.tv_usec;
    }

    inline unsigned long GetTimeSinceResetUs()
    {
        gettimeofday(&finish, NULL);
        timeSinceResetUs = (1000000 * finish.tv_sec + finish.tv_usec) - (1000000 * start.tv_sec + start.tv_usec);
        return timeSinceResetUs;
    }
    
    inline double GetTimeSinceReset()
    {
        return GetTimeSinceResetUs() / 1000000.f;
    }

    inline unsigned long micros()
    {
        return GetTimeSinceResetUs();
    }

    inline void ResetStartTime()
    {
        gettimeofday(&start, NULL);
        startTime = 1000000 * start.tv_sec + start.tv_usec;
    }

private:
    struct timeval start;
    struct timeval finish;
    unsigned long startTime;
    unsigned long timeSinceResetUs;
};
*/
#endif // ASCEND_QUADRUPED_CPP_TIMER_TIMER_H