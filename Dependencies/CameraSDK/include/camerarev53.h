
//======================================================================================================-----
//== Copyright NaturalPoint, All Rights Reserved
//======================================================================================================-----

//== This is the >>> SlimX 13E Camera <<<

#ifndef __CAMERALIBRARY__CAMERAREV53_H__
#define __CAMERALIBRARY__CAMERAREV53_H__

//== INCLUDES ===========================================================================================----

#include "camerarev51.h"

//== GLOBAL DEFINITIONS AND SETTINGS ====================================================================----

namespace CameraLibrary
{
	class CameraRev53 : public CameraRev51
    {
    public:
        CameraRev53();

        //== Status Ring Lights =========================================================================----

        virtual int    StatusRingLightCount();            //== Number of status ring LEDs ===============----
        virtual void   SetStatusRingLights(int Count, sStatusLightColor *LightColors);
        virtual bool   IsIRIlluminationAvailable();       //== IR Illumination ring presence ============----

    };
}

#endif
