
//======================================================================================================-----
//== Copyright NaturalPoint
//======================================================================================================-----

#pragma once

//== INCLUDES ===========================================================================================----

#include "Core/Serializer.h"
#include "Core/Quaternion.h"

//== GLOBAL DEFINITIONS AND SETTINGS ====================================================================----

namespace Core
{
    class cIMU
    {
    public:
        cIMU();
        
        cIMU( const Core::cQuaternionf & orientation, unsigned int tagID,
            unsigned int frameIDTimeStamp, int timeStampOffset );

        ~cIMU();

        void SetOrientation( const Core::cQuaternionf & orientation );
        void SetTagID( unsigned int tagID );
        void SetTiming( unsigned int frameIDTimeStamp, int timeStampOffset );


        const Core::cQuaternionf & Orientation() const;

        unsigned int TagID() const;
        int          FrameIDTimeStamp() const;
        int          TimeStampOffset() const;

        void            Save( Core::cIWriter *Stream ) const;
        bool            Load( Core::cIReader *Stream );

    private:

        Core::cQuaternionf mOrientation;

        unsigned int mTagID; 
        unsigned int mFrameIDTimeStamp;
        int          mTimeStampOffset;

    };
}


