//======================================================================================================
// Copyright 2013, NaturalPoint Inc.
//======================================================================================================
#pragma once

#include "Core/BuildConfig.h"
#include "Core/Vector3.h"

namespace Core
{
    struct CORE_API sHSVColor
    {
    public:
        double H;
        double S;
        double V;

        sHSVColor( double h, double s, double v ) : H( h ), S( s ), V( v ) { }

        /// <summary>Initialize an HSV color with RGB values (in 0-255 range).</summary>
        sHSVColor( int r, int g, int b );
    };

    /// <summary>A collection of platform-neutral color management and conversion routines.</summary>
    class CORE_API cColorHelpers
    {
    public:
        /// <summary>
        /// Convert from a packed int representation to an RGB vector representation. Alpha values are lost
        /// in this conversion.
        /// </summary>
        static cVector3f PackedIntToVector3( unsigned int color );

        /// <summary>Convert floating point 3-vector (with color ranges in [0,1]) to a packed int.</summary> 
        static unsigned int Vector3ToPackedInt( const cVector3f& color );
    };
}


