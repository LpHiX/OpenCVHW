
//======================================================================================================-----
//== Copyright 2018 NaturalPoint, All Rights Reserved
//======================================================================================================-----

//== This is the >>> PrimeX 22 Camera <<<

#ifndef __CAMERALIBRARY__CAMERAREV50_H__
#define __CAMERALIBRARY__CAMERAREV50_H__

//== INCLUDES ===========================================================================================----

#include "camerarev49.h"

//== GLOBAL DEFINITIONS AND SETTINGS ====================================================================----

namespace CameraLibrary
{
    class CameraRev50 : public CameraRev49
    {
    public:
        CameraRev50();

        virtual int    MaximumFrameRateValue() override;          //== Returns the maximum frame rate ====---
        virtual int    MaximumMJPEGRateValue() override;          //== Returns the maximum MJPEG rate ====---
        virtual int    MaximumFullImageFrameRateValue() override; //== Returns the maximum full image frame rate ====---

        virtual double ImagerWidth() override;           //== Physical imager width (in mm) =====---
        virtual double ImagerHeight() override;          //== Physical imager height (in mm) ====---

        virtual void GetDistortionModel(Core::DistortionModel &Model) override;    //== Distortion Model -

    };
}

#endif

