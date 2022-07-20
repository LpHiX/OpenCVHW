
//======================================================================================================-----
//== Copyright NaturalPoint, All Rights Reserved
//======================================================================================================-----


#ifndef __CAMERALIBRARY__CAMERAREV36_H__
#define __CAMERALIBRARY__CAMERAREV36_H__

//== INCLUDES ===========================================================================================----

#include "camerarev30.h"

//== GLOBAL DEFINITIONS AND SETTINGS ====================================================================----

class cInputBase;

namespace CameraLibrary
{
    class CameraRev36 : public CameraRev30
    {
    public:
        CameraRev36();

        //== Camera Information =========================================================================----

        virtual void GetDistortionModel( Core::DistortionModel &Model );      //== Distortion Model =====----

        virtual int  MaximumFrameRateValue();         //== Returns the maximum frame rate ===============----

        //== Direct H.264 Access (when camera is in Video Mode) =========================================----

        //== SetColorCompression Parameters: ==--
    
        //== Mode: 0 = Variable Bit Rate, 1 = Constant Bit Rate. Default: CBR ==--
        //== Quality: Value affects compression, resulting image quality, and data rate when the camera
        //==    is in VBR Mode.  The range is 0.0 to 1.0 and adjusts image compression.  The default 
        //==    value for this setting is 0.2.  This setting is specific to VBR Mode.
        //== BitRate: Value affects compression, resulting image quality, and data rate when the
        //==    camera is in CBR Mode.  The range is 0.0 to 1.0 as an approximate percentage of 100 MB/s.
        //==    The default value of 0.5 will yield approximately 50 MB/s of H.264 compressed data from
        //==    the camera.  By comparison setting a value of 0.2 would yield approximately 20 MB/s of
        //==    H.264 compressed data from the camera.  This value is specific to CBR Mode.

        virtual void         SetColorCompression( int Mode, float Quality, float BitRate ) override;

        virtual bool         IsColor() override;
        virtual void         SetColorMatrix( sColorMatrix Matrix ) override;
        virtual void         SetColorGamma( float Gamma ) override;
        virtual void         SetColorPrescalar( float R, float G1, float G2, float B ) override;
        virtual void         SetColorEnhancement( float LNoiseThreshold,
            float LEdgeStrength, float LHaloSuppress, float RNoiseThreshold,
            float REdgeStrength, float RHaloSuppress ) override;

        virtual sColorMatrix    ColorMatrix() override;
        virtual float           ColorGamma() override;
        virtual int             ColorMode() override;
        virtual float           ColorCompression() override;
        virtual sColorPrescalar ColorPrescalar();
        virtual float           ColorBitRate() override;
        virtual void            ColorEnhancement( float &LNoiseThreshold,
            float &LEdgeStrength, float &LHaloSuppress, float &RNoiseThreshold,
            float &REdgeStrength, float &RHaloSuppress ) override;

        virtual int  CompressedImageSize( Frame *frame );
        virtual int  CompressedImage( Frame *frame, unsigned char *Buffer, int BufferSize );

        virtual bool IsMJPEGAvailable() override { return false; }
        virtual bool IsVideoTypeSupported( Core::eVideoMode Mode ) override;

        virtual bool IsIRIlluminationAvailable();

    };
}




#endif
