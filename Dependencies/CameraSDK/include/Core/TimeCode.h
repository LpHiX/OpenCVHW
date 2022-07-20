//======================================================================================================
// Copyright 2014, NaturalPoint Inc.
//======================================================================================================
#pragma once

#include <string>

// Local includes
#include "Core/BuildConfig.h"

namespace Core
{
    class CORE_API cTimeCode
    {
    public:
        cTimeCode() : mHours( 0 ), mMinutes( 0 ), mSeconds( 0 ), mFrames( 0 ), mTimeCodeSubFrame( 0 ), mTimeCodeDropFrame( false ),
            mValid( false ) { }
        cTimeCode( unsigned int compositeTimecode );
        cTimeCode( unsigned int totalFrames, double frameRate );

        unsigned int    mHours;
        unsigned int    mMinutes;
        unsigned int    mSeconds;
        unsigned int    mFrames;
        unsigned int    mTimeCodeSubFrame;
        bool            mTimeCodeDropFrame;
        bool            mValid;

        int             Hours() const { return mHours; }
        int             Minutes() const { return mMinutes; }
        int             Seconds() const { return mSeconds; }
        int             Frame() const { return mFrames; }
        int             SubFrame() const { return mTimeCodeSubFrame; }
        bool            IsDropFrame() const { return mTimeCodeDropFrame; }
        bool            Valid() const { return mValid; }

        /// <summary>Composite, compact timecode value (hours/mins/secs/frames).</summary>
        unsigned int    TimeCode() const;

        std::string     Stringify( bool includeSubframes = true ) const;

    private:
        void            Stringify( char *buffer, int bufferSize, bool includeSubframes = true ) const;
    };
}


