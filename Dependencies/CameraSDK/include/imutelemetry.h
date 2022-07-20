
//======================================================================================================-----
//== Copyright NaturalPoint
//======================================================================================================-----

#ifndef __IMUTELEMETRY_H__
#define __IMUTELEMETRY_H__

//== INCLUDES ===========================================================================================----

#include "Core/Serializer.h"
#include "Core/Quaternion.h"

//== GLOBAL DEFINITIONS AND SETTINGS ====================================================================----

namespace CameraLibrary
{

    class cIMUTelemetry
    {
    public:
        cIMUTelemetry();
        
        cIMUTelemetry( const Core::cQuaternionf & orientation, unsigned int tagID,
            unsigned int frameIDTimeStamp, int timeStampOffset, double softTimeStamp );

        ~cIMUTelemetry();

        //== Intended Public Command Interface ==============================================----

        void SetOrientation( const Core::cQuaternionf & orientation );
        void SetTagID( unsigned int tagID );
        void SetTiming( unsigned int frameIDTimeStamp, int timeStampOffset );


        const Core::cQuaternionf & Orientation() const;

        unsigned int TagID() const;
        int          FrameIDTimeStamp() const;
        int          TimeStampOffset() const;

        //== Serialization / Deserialization =================================================---

        void            Save( Core::cIWriter *Stream ) const;
        bool            Load( Core::cIReader *Stream );

    };
}

#endif

