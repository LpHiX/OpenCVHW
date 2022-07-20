
//======================================================================================================-----
//== Copyright NaturalPoint, All Rights Reserved
//======================================================================================================-----

//== This is the >>> Slim 13E Camera <<<

#ifndef __CAMERALIBRARY__CAMERAREV33_H__
#define __CAMERALIBRARY__CAMERAREV33_H__

//== INCLUDES ===========================================================================================----

#include "camerarev31.h"

//== GLOBAL DEFINITIONS AND SETTINGS ====================================================================----

class cInputBase;

namespace CameraLibrary
{
	class CameraRev33 : public CameraRev31
    {
    public:
        CameraRev33();

        //== Status Ring Lights =========================================================================----

        virtual int    StatusRingLightCount();            //== Number of status ring LEDs ===============----
        virtual void   SetStatusRingLights(int Count, sStatusLightColor *LightColors);
        virtual bool   IsIRIlluminationAvailable();       //== IR Illumination ring presence ============----

    };
}




#endif
