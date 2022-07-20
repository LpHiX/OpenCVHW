
//======================================================================================================-----
//== Copyright NaturalPoint, All Rights Reserved
//======================================================================================================-----

#ifndef __CAMERALIBRARY__MODULEACTIVELABEL_H__
#define __CAMERALIBRARY__MODULEACTIVELABEL_H__

//== INCLUDES ===========================================================================================----

#include "cameramodulebase.h"
#include "trajectorizer2d.h"

//== GLOBAL DEFINITIONS AND SETTINGS ====================================================================----

namespace CameraLibrary
{
    class CLAPI cModuleActiveLabel : public cCameraModule
    {
    protected:
        cModuleActiveLabel();
        ~cModuleActiveLabel();
    public:

        static cModuleActiveLabel * Create();
        static void                 Destroy( cModuleActiveLabel * object );

        void SetEnabled(bool Enabled);
        bool Enabled();

        void SetPatternDepth( int PatternDepth );
        int  PatternDepth() const;

        unsigned int FullPattern() const;

        void PrePostFrame(Camera *Camera, Frame *Frame);

        double TelemetryRunningAverageProcessingTime() const;

    };
}

#endif
