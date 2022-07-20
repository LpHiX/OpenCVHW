
//======================================================================================================-----
//== NaturalPoint 2018
//======================================================================================================-----

//== This is the >>> OptiTrack Active Base Station <<<

#ifndef __CAMERALIBRARY__CAMERAREV38_H__
#define __CAMERALIBRARY__CAMERAREV38_H__

//== INCLUDES ===========================================================================================----

#include <queue>

#include "camerarev11.h"

//== GLOBAL DEFINITIONS AND SETTINGS ====================================================================----

class cInputBase;

namespace CameraLibrary
{

	class CameraRev38 : public CameraRev11
    {
    public:
        CameraRev38();

        virtual bool IsCamera() override            { return false; }
        virtual bool IsBaseStation()                { return true;  }
        virtual bool IsIRIlluminationAvailable()    { return false; }

        //== Hardware Time Stamp ============================================================----

        virtual void   QueryHardwareTimeStampValue( int UserData );

    };
}

#endif

