
//======================================================================================================-----
//== Copyright NaturalPoint, All Rights Reserved
//======================================================================================================-----


#ifndef __CAMERALIBRARY__CAMERAREV37_H__
#define __CAMERALIBRARY__CAMERAREV37_H__

//== INCLUDES ===========================================================================================----

#include "camerarev36.h"

namespace CameraLibrary
{
    class CameraRev37 : public CameraRev36
    {
    public:
        CameraRev37();

        //== Camera Information =========================================================================----

        virtual int               MaximumFrameRateValue() override;          //== Returns the maximum frame rate
        virtual int               MaximumFullImageFrameRateValue() override; //== Returns the maximum full image frame rate ====---

        virtual int				  CameraResolutionCount();
        virtual int				  CameraResolutionID();
        virtual sCameraResolution CameraResolution(int index);
        virtual void			  SetCameraResolution(int ResolutionID);

        virtual bool              IsFilterSwitchAvailable();

        virtual bool              IsVideoTypeSupported( Core::eVideoMode Mode ) override;
        
        virtual bool              IsIRIlluminationAvailable();

    };
}
#endif

