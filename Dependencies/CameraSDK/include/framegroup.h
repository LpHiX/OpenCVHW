
//======================================================================================================-----
//== Copyright NaturalPoint
//======================================================================================================-----

#ifndef __CAMERALIBRARY__FRAMEGROUP_H__
#define __CAMERALIBRARY__FRAMEGROUP_H__

//== INCLUDES ===========================================================================================----

#include <stdio.h>
#include <vector>

#include "cameralibraryglobals.h"
#include "synchronizer.h"
#include "frame.h"
#include "lock.h"

//== GLOBAL DEFINITIONS AND SETTINGS ====================================================================----

namespace CameraLibrary
{
    //== dropped frame info is reported as part of the FrameGroup object for applications seeking to 
    //== determine when one or more cameras has dropped a frame

    class CLAPI DroppedFrameInfo
    {
    public:
        DroppedFrameInfo( int serial, int userData ) : mSerial( serial ), mUserData( userData ) {}
        DroppedFrameInfo() : mSerial( 0 ), mUserData( 0 ) {}
        
        int Serial()   const { return mSerial; }
        int UserData() const { return mUserData; }

    private:

        int mSerial;
        int mUserData;
    };

    //== FrameGroup object contains a synchronized group of frames from multiple cameras when the 
    //== cModuleSync is utilized to synchronize frame data.

    class CLAPI FrameGroup
    {
        // To access LegacyAddRef and LegacyRelease functions.
        friend class cModuleSyncBase;
        friend class cModuleSync;

    public:
        FrameGroup();
        ~FrameGroup() {};

        enum Modes
        {
            None=0,
            Software,
            Hardware,
            ModeCount
        };

        int     GetFrameUserData(int Index) const;
        Frame * GetFrame(int Index) const;
        int     Count() const;

#ifndef CAMERALIBRARY_STATICLIB
        void    AddRef();
        void    Release();
        int     RefCount() const;
#endif
        
        void    AddFrame(int UserData, Frame* frame);
        void    Clear();
        
        void    SetMode(Modes Mode);
        Modes   Mode() const;

        void    SetTimeStamp (double timeStamp);
        void    SetTimeSpread(double timeSpread);
        void    SetEarliestTimeStamp(double Stamp);
        void    SetLatestTimeStamp(double Stamp);

        double  TimeSpread() const;
        double  TimeStamp() const;
        double  EarliestTimeStamp() const;
        double  LatestTimeStamp() const;

        int     FrameID() const;

        double  TimeSpreadDeviation(int Index) const;
        
        int     DroppedFrameCount() const;
        
        const DroppedFrameInfo & DroppedFrame( int index ) const;

    };
}

#endif
