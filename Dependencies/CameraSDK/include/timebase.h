//==================================================================================-----
//== NaturalPoint 2012
//==================================================================================-----
#ifndef __TIMEBASE_H__
#define __TIMEBASE_H__

#include "cameracommonglobals.h"

class CLAPI cPrecisionTimeBase
{
public:
    cPrecisionTimeBase();
    ~cPrecisionTimeBase();

    void  CatchUp(void);       // reset elapsed time
#ifdef __PLATFORM__LINUX__
    float Elapsed(void);       // returns the elapsed time in seconds
#elif defined WIN32
    double  Elapsed();              //== Return Elapsed Milliseconds =============--------
#endif

private:
#ifdef __PLATFORM__LINUX__
    long  GetRawTime(void);
    long  start;
#elif defined WIN32
    double  Ticks();
    __int64	mStart;
    __int64	mFrequency;
#endif
};

#endif
