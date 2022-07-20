
//======================================================================================================-----
//== Copyright 2018 NaturalPoint, All Rights Reserved
//======================================================================================================-----

//== This is the >>> PrimeX 41/41W Camera <<<

#ifndef __CAMERALIBRARY__CAMERAREV49_H__
#define __CAMERALIBRARY__CAMERAREV49_H__

//== INCLUDES ===========================================================================================----

#include "camerarev29.h"

//== GLOBAL DEFINITIONS AND SETTINGS ====================================================================----

namespace CameraLibrary
{
    class CameraRev49 : public CameraRev29
    {
    public:
        CameraRev49( bool is41W = false );

        virtual int    MaximumFrameRateValue() override; //== Returns the maximum frame rate ====---
        virtual int    MaximumMJPEGRateValue() override; //== Returns the maximum MJPEG rate ====---

        virtual bool IsFilterSwitchAvailable() override;

    };
}

#endif

