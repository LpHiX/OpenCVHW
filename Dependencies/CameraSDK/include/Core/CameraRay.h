//======================================================================================================-----
// Copyright 2014, NaturalPoint Inc.
//======================================================================================================-----
#pragma once

#include <iostream>

// Local includes
#include "Core/BuildConfig.h"
#include "Core/Vector2.h"
#include "Core/Vector3.h"
#include "Core/UID.h"

#pragma warning( push )
#pragma warning( disable: 4251 ) // Warning about "needs to have dll-interface" on STL types

namespace Core
{
    /// <summary>
    /// A class representing a ray that emanates from a camera, and may be assigned to a 3D marker reconstruction.
    /// </summary>
    class CORE_API cCameraRay
    {
    public:
        cCameraRay();

        cCameraRay( int id, int cameraSerial, const cUID& markerID, const cVector2f& imagePosition,
            float imageArea, const cVector3f& origin, const cVector3f& direction, float length, bool observed, bool tracked );
        virtual ~cCameraRay();

        /// <summary>The (unique among rays) ID for this ray.</summary>
        int             ID() const;

        /// <summary>Retrieve the serial number of the camera from which this ray originated.</summary>
        int             CameraSerial() const;

        /// <summary>Set the assigned marker ID.</summary>
        void            SetMarkerID( const cUID& id );

        /// <summary>The ID of the marker that this ray is assigned to, or cUID::kInvalid if none.</summary>
        const cUID&     MarkerID() const;

        /// <summary>Retrieve the 2D image position in the camera image.</summary>
        const cVector2f& ImagePosition() const;

        /// <summary>Retrieves the (pixel) base area in the camera image.</summary>
        float           ImageArea() const;

        /// <summary>3D ray origin.</summary>
        const cVector3f& Origin() const;

        /// <summary>3D ray direction.</summary>
        const cVector3f& Direction() const;

        /// <summary>Set the ray length.</summary>
        void            SetLength( float len );

        /// <summary>Ray length.</summary>
        float           Length() const;

        /// <summary>Returns true if this ray instance is identical to the given instance.</summary>
        bool            Equals( const cCameraRay& ray ) const;

        /// <summary>Comparison operators. Used mostly for sorting.</summary>
        bool            operator<( const cCameraRay& other ) const;
        bool            operator<( const cUID& id ) const;
        friend bool     operator<( const cUID& id, const cCameraRay& other );

        /// <summary>Observed ray.  When true, this means ray originated from an actual camera centroid and not a predicted ray or anything else.
        bool            IsObserved() const;

        /// <summary>Tracked ray. This signifies that the ray intersects one or more other rays that has resulted in a 3D marker reconstruction.
        bool            IsTracked() const;

    private:
        // An ID that is unique within the context of camera rays.
        int             mID;

        // The serial number of the camera that originated this ray.
        int             mCameraSerial;

        // The ID of the reconstruction this ray is assigned to, or kInvalid if the ray is unassigned.
        cUID            mMarkerID;

        // The (pixel) area of the ray origin in the camera image.
        float           mImageArea;

        // The 2D coordinates of the ray origin in the camera image.
        cVector2f       mImagePosition;

        // The 3D ray origin.
        cVector3f       mOrigin;

        // The normalized 3D ray direction
        cVector3f       mDirection;

        // The length of the ray.
        float           mLength;

        // Observed ray.  This means this ray originated from an actual camera centroid and not a predicted ray or anything else.
        bool            mObserved;

        // Tracked. This signifies that the ray intersects one or more other rays that has resulted in a 3D marker reconstruction.
        bool            mTracked;
    };

#if !defined(__PLATFORM__LINUX__) // gcc has no clue about stream operators.
    std::ostream& operator<<( std::ostream& os, const cCameraRay& ray );
    std::istream& operator>>( std::istream& is, cCameraRay& ray );
#endif
}

#pragma warning( pop )

