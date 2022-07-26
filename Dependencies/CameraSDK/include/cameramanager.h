//======================================================================================================-----
// Copyright 2010, NaturalPoint Inc.
//======================================================================================================-----
#pragma once

//== INCLUDES ===========================================================================================----

#include <vector>
#include <set>
#include <map>

#include "Core/UID.h"
#include "cameralibraryglobals.h"
#include "singleton.h"
#include "timebase.h"
#include "threading.h"
#include "cameratypes.h"
#include "inputmanagerbase.h"
#include "lock.h"

//========================================================================================================---

class InputManager;

namespace Core
{
    class cIReader;
    class cIWriter;
}

namespace CameraLibrary
{
    class Camera;
    class CameraList;    
    class HardwareKeyList;
    class HardwareDeviceList;
    class HubList;
    class CameraEntry;
    class CameraManagerListener;

    struct sSyncSettings;

    typedef Camera HardwareKey;
    typedef Camera Hub;
    typedef Camera cDevice;

    //== Synchronization System ===============================================================----

    class CLAPI cSyncFeatures
    {
    public:
        cSyncFeatures();
        ~cSyncFeatures();
    
        enum  eSyncSystemTypes
        {
            EthernetSyncSystem,
            USBSyncSystem,
            UnknownSyncSystem
        };

        eSyncSystemTypes SyncSystemType;

        bool  CustomSyncSupport;
        bool  ShutterGoggleSupportByManufacturer;
        bool  ShutterGoggleSupportBySlider;
        bool  RequireGogglesVideoFrameRate;

        int   SyncOutputCount;
        bool  SyncVesaStereoOut;

        float DefaultInternalSyncFrequency;

        bool  SyncOffsetIsGlobal;

        bool  GlassesInputSourceSupport [SyncInputSourceCount];
        bool  SyncInputSourceSupport    [SyncInputSourceCount];
        bool  TriggerSourceSupport      [SyncInputSourceCount];
        bool  SyncInputTriggerSupport   [SyncInputSourceCount];
        bool  SyncOutput[CameraLibrary::SyncOutputCount];
        bool  SyncOutputPolarity;
        bool  WiredSyncSupport;
        bool  SyncInternalFreq;
        bool  RecordTriggering;
        bool  InputMonitoring;
        bool  SyncInDividerBySlider;
        bool  SyncInputMultiplier;

    };

    struct CLAPI sSyncSettings
    {
        sSyncSettings();
       

        eSyncMode   Mode;
        eSyncType   SyncType;

        //== Shutter Goggles =================----

        int   GoggleType;       //== 1 = Stereographics, 2 = NuVision 60Gx, 3= NuVision APG6000
        int   VideoFrameRate;   //== Video Frame Rate
        float CustomOffs;       //== Custom Offset Slider Value (0-100)

        //== Shutter Goggles Slider Approach =----

        long   GogglesSliderFrameRate; //== Shutter Frame Rate 
        double GogglesSliderOffset;    //== Shutter Slider Offset (0-100)

        //== Custom Sync Settings ============----

        int   CameraExposure;   //== Camera exposure (all cameras are locked to a single exposure)
        int   ImagerScanRate;   //== Camera Frame Rate

        eSyncCameraSync           CameraSync;
        eSyncInputSource          SyncInputSource;
        eSyncInputSource          SyncInputTrigger;
        int                       SyncInputDivider;
        int                       SyncInputMultiplier;
        float                     InternalSyncGeneratorFrequency;
        float                     GlobalSyncOffset;
        float                     ExternalTriggerFreq;

        //=== First Sync Output ==============----

        eSyncOutputPhase          SyncOutput1Phase;
        eSyncOutputPulseDuration  SyncOutput1PulseDuration;
        eSyncOutputPolarity       SyncOutput1Polarity;


        //=== Second Sync Output ============----

        eSyncOutputPhase          SyncOutput2Phase;
        eSyncOutputPulseDuration  SyncOutput2PulseDuration;
        eSyncOutputPolarity       SyncOutput2Polarity;

        //=== Third Sync Output =============----

        eSyncOutputPhase          SyncOutput3Phase;
        eSyncOutputPulseDuration  SyncOutput3PulseDuration;
        eSyncOutputPolarity       SyncOutput3Polarity;

        //=== Fourth Sync Output ============----

        eSyncOutputPhase          SyncOutput4Phase;
        eSyncOutputPulseDuration  SyncOutput4PulseDuration;
        eSyncOutputPolarity       SyncOutput4Polarity;

        //== VESA Stereo Out =================----

        eSyncOutputPhase          SyncOutputVesaPhase;
        eSyncOutputPulseDuration  SyncOutputVesaPulseDuration;
        eSyncOutputPolarity       SyncOutputVesaPolarity;

        //=== Record Triggering ==============----

        eSyncInputSource          SyncRecordTrigger;
        eSyncInputSource          SyncRecordEdge;

        eUSBSyncInControl         USBSyncInControl;
        bool                      RecordActive;

        //== Helpers ==--

        eSyncOutputPhase         * SyncOutputPhase(int SyncOutputIndex);
        eSyncOutputPulseDuration * SyncOutputPulseDuration(int SyncOutputIndex);
        eSyncOutputPolarity      * SyncOutputPolarity(int SyncOutputIndex);

        eSyncOutputPhase         * VesaSyncOutputPhase();
        eSyncOutputPulseDuration * VesaSyncOutputPulseDuration();
        eSyncOutputPolarity      * VesaSyncOutputPolarity();
        

        bool    ExternalSignalSetsFrameRate() const;
    };

    //== Camera Manager ==================================================================================---

    class CLAPI CameraManager : public cInputManagerListener 
    {
    protected:
        CameraManager();                            //== Constructors / Destructors Protected ============---
        ~CameraManager();                           //== Access CameraManager with: CameraManager::X().   ---

    public:
        static   CameraManager & X();               //== Access to CameraManager Singleton ===============---

        bool     WaitForInitialization();           //== Optional execution stall until cameras are init'd --
        bool     AreCamerasInitialized();           //== Check and see if all attached cameras are init'd ---
        bool     AreCamerasShutdown();              //== Check and see if all cameras are shutdown =======---
        void     Shutdown();                        //== Shutdown Camera Library =========================---
        Camera * GetCameraBySerial(int Serial);     //== Get a camera by camera serial number ============---
        Camera * GetCamera(const Core::cUID& UID);  //== Get a camera by UID (UIDs come from CameraList) =---
        Camera * GetCamera();                       //== Get an attached & initialized camera ============---
        void     GetCameraList(CameraList &List);   //== Used by CameraList to self-populate =============---

        HardwareKey*  GetHardwareKey();             //== Get an attached & initialized hardware key ======---

        cDevice *GetDevice(const Core::cUID& UID) const; //== Get device by UID (UIDs from HardwareDeviceList) =--

        void     PrepareForSuspend();               //== Power Management to prepare for system suspend ==---
        void     ResumeFromSuspend();               //== Power Management to resume after system suspend =---

        double   TimeStamp();                       //== Fetch system timestamp (in seconds) =============---
        double   TimeStampFrequency();              //== Fetch system timestamp seconds per second (1) ===---
        void     ResetTimeStamp();                  //== Reset global camera library timestamp ===========---

        void     RegisterListener(CameraManagerListener*);      //== Register Camera Manager listener ====---
        void     UnregisterListener(CameraManagerListener*);    //== Unregister Camera Manager listener ==---

        static Camera* CameraFactory(int Revision, bool Init=true, int Serial = 0
                      , bool startCameraEngineThread = true, int subModel = 0 );    //= Virtual Cameras ==---

        void     AddCamera(Camera *Cam);            //== Add virtual cameras to camera manager list ======---
        void     RemoveCamera(Camera *Cam);         //== Remove cameras from camera manager list =========---
        void     RemoveVirtualCameras();            //== Remove cameras without a physical counterpart ===---
    
        void     ScanForCameras();                  //== Camera Manager manages scanning for cameras =====---

        void      ApplySyncSettings(sSyncSettings  SyncSettings); //== Apply Synchronization Settings ====---
        void      GetSyncSettings  (sSyncSettings &SyncSettings); //== Get Current Sync Settings =========--- 
        const sSyncSettings & SyncSettings() const;               //== Get Current Sync Settings =========---
        void      SoftwareTrigger();                              //== Trigger to shutter cameras ========---
        eSyncMode SyncMode();                                     //== Returns the Current Sync Mode =====---
        void      UpdateRecordingBit(bool Recording);             //== Update real-time app info =========---
        cSyncFeatures GetSyncFeatures();                          //== Returns what sync features are ====---
                                                                  //== available given connected hardware ---
        const char*   SyncDeviceName();                           //== Returns the name of the synch =====---
                                                                  //== device name, or empty string if ===---
                                                                  //== none connected. ===================---

        bool     ShouldLockCameraExposures();       //== Should app force all camera exposures equal =====---
        bool     ShouldForceCameraRateControls();   //== Should app force all camera frame rates equal ===---
        bool     ShouldApplySyncOnExposureChange(); //== Should app reapply sync settings on exp change ==--- 

        void     SuggestCameraIDOrder(int *CameraIDList, int ListCount); //== Suggest CameraID order =====---

        //== CameraManager Singleton Methods =============================================================---

        static void DestroyInstance();              //== Destroy CameraManager Singleton =================---
        static bool IsActive();                     //== Is CameraManager Singleton instantated ==========---
        static CameraManager * Ptr();               //== Access to CameraManager Singleton ===============---

    };

    class CLAPI CameraEntry
    {
    protected:
        friend class CameraList;
        friend class HardwareKeyList;
        friend class HardwareDeviceList;
        friend class HubList;

        CameraEntry() {};                                       //== Constructors / Destructors Protected =--

        CameraEntry( const char* name, const char* serialString, int serial, const Core::cUID & id, int revision
            , eCameraState state, bool virtualDevice );
    public:
        Core::cUID UID() const;                                 //== Camera's universal unique ID ========---
        int   Serial() const;                                   //== Camera's serial number ==============---
        int   Revision() const;                                 //== Camera's revision ===================---
        const char*  Name() const;                              //== Camera's name =======================---
        eCameraState State() const;                             //== Camera's state ======================---
        bool  IsVirtual() const;
        const char* SerialString() { return ( const char* ) mSerialString; } //== Camera's alphanumeric serial ==--

    protected:
        friend class CameraManager;
        void  SetName( const char* Name );
        void  SetSerialString( const char* Name );
        void  SetUID( const Core::cUID& UID );
        void  SetSerial(int Value);
        void  SetState(eCameraState State);
        void  SetRevision(int Revision);
        void  SetVirtual(bool Virtual);

    private:
        char  mName[kCameraNameMaxLen];
        char  mSerialString[ kCameraNameMaxLen ];
        Core::cUID mUID;
        int   mSerial;
        int   mRevision;
        eCameraState  mState;
        bool  mVirtual;
    };

    class CLAPI CameraList
    {
    public:
        CameraList();                                       //== Create a CameraList to see what ======--
        ~CameraList();                                      //== cameras are available ================--

        CameraEntry& operator[](int index);                 //== Index the list by CameraList[index] ==--
        int           Count() const;                        //== Number of entries in the CameraList ==--
        void          Refresh();                            //== Repopulate the list ==================--

    protected:
        friend class CameraManager;
        void AddEntry( const CameraEntry & entry );

    private:
        std::vector<CameraEntry> mEntries;
        void ClearList();
    };

    class CLAPI HardwareKeyList
    {
    public:
        HardwareKeyList();                                  //== Create a HardwareKeyList to see what =--
        ~HardwareKeyList();                                 //== hardware keys are available ==========--

        CameraEntry& operator[](int index);                 //== Index the list by HardwareKeyList[index] ==--
        int           Count() const;                        //== Number of entries in the HardwareKeyList ==--

    protected:
        friend class CameraManager;
        void SetCount( int Value );

    private:
        int mCameraCount;
        CameraEntry   mCameraEntry[kMaxCameras];
    };

    class CLAPI HubList
    {
    public:
        HubList();                                          //== Create a HubList to see what =========--
        ~HubList();                                         //== OptiHubs are available ===============--

        CameraEntry& operator[](int index);                 //== Index the list by HubList[index] =====--
        int           Count() const;                        //== Number of entries in the HubList =====--

    protected:
        friend class CameraManager;
        void SetCount( int Value );

    private:
        int mCameraCount;
        CameraEntry   mCameraEntry[kMaxCameras];
    };

    class CLAPI HardwareDeviceList
    {
    public:
        HardwareDeviceList();                               //== Create a HubList to see what =========--
        ~HardwareDeviceList();                              //== hardware devices are available =======--

        CameraEntry& operator[](int index);                 //== Index by HardwareDeviceList[index] ===--
        int           Count() const;                        //== Number of entries in the List ========--

    protected:
        friend class CameraManager;
        void SetCount( int Value );

    private:
        int mCameraCount;
        CameraEntry   mCameraEntry[kMaxCameras];
    };

    class CLAPI CameraManagerListener
    {
    public:
        virtual ~CameraManagerListener() {}

        virtual void CameraConnected()          {}   //== A camera has been connected to the system ====---
        virtual void CameraRemoved()            {}   //== A camera has been removed from the system ====---
        virtual void SyncSettingsChanged()      {}   //== Global synchronization settings have changed =---
        virtual void CameraInitialized()        {}   //== A camera has completed initialization ========---
        virtual void SyncAuthorityInitialized() {}   //== A synchronization device has initialized =====---
        virtual void SyncAuthorityRemoved()     {}   //== Synchronization device removed from the system =-
        virtual void CameraMessage( int Type, int Value, int ID ) {}		   //== Internal Use =========---	
        virtual Camera* RequestUnknownDeviceImplementation(int Revision);  //== Internal Use =========---

        virtual bool ShouldConnectCamera( const char * NetworkInterface, const char * CameraSerial );

    };   

}

//========================================================================================================---

