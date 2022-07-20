
//======================================================================================================-----
//== NaturalPoint 2012
//======================================================================================================-----

//== This is the >>> Slim 3U <<<

#ifndef __CAMERALIBRARY__CAMERAREV14_H__
#define __CAMERALIBRARY__CAMERAREV14_H__

//== INCLUDES ===========================================================================================----

#include "camerarev12.h"
#include "camerarevisions.h"

//== GLOBAL DEFINITIONS AND SETTINGS ====================================================================----

class cInputBase;

namespace CameraLibrary
{
    class CameraRev14 : public CameraRev12
    {
    public:
        CameraRev14();
        ~CameraRev14();
 
        virtual int    MaximumFrameRateValue();         //== Returns the maximum frame rate ==============---
        int    HardwareFrameRate() { return 120; }

        virtual bool IsFilterSwitchAvailable(); 
        virtual bool IsIRIlluminationAvailable();       //== IR Illumination ring presence ==----

    };
}



#endif
