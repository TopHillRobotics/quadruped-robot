#ifndef QR_TIMER_H
#define QR_TIMER_H
#include <time.h>

class Timer {
public:
    Timer()
    {
        start = clock();
        startTime = (double)start;
    }

    inline double GetTimeSinceReset()
    {
        finish = clock();
        double timeSinceReset = (double)(finish - startTime) / CLOCKS_PER_SEC; // second(s)
        return timeSinceReset;
    }

    inline void ResetStartTime()
    {
        start = clock();
        startTime = (double)start;
    }

private:
    clock_t start;
    clock_t finish;
    double startTime;
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
#endif // QR_TIMER_H
