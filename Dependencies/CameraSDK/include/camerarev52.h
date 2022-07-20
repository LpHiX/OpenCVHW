
//======================================================================================================-----
//== Copyright NaturalPoint, All Rights Reserved
//======================================================================================================-----

//== This is the >>> PrimeX 13W Camera <<<

#ifndef __CAMERALIBRARY__CAMERAREV52_H__
#define __CAMERALIBRARY__CAMERAREV52_H__

//== INCLUDES ===========================================================================================----

#include "camerarev51.h"

//== GLOBAL DEFINITIONS AND SETTINGS ====================================================================----

namespace CameraLibrary
{
	class CameraRev52 : public CameraRev51
    {
    public:
        CameraRev52();

        //== Camera Information =========================================================================----

        virtual void   GetDistortionModel(Core::DistortionModel &Model);    //== Distortion Model ========---

    };
}

#endif
