#pragma once

#include <mutex>
#include <thread>

#include <wvr/wvr.h>
#include <wvr/wvr_ctrller_render_model.h>
#include <wvr/wvr_device.h>
#include <wvr/wvr_events.h>

#include "../shared/Matrices.h"
#include "../object/Mesh.h"
#include "../object/Texture.h"
#include "../object/Shader.h"

enum CtrlerCompEnum
{
    CtrlerComp_Body              = 0,
    CtrlerComp_TouchPad          = 1,
    CtrlerComp_AppButton         = 2,
    CtrlerComp_HomeButton        = 3, //SystemKey
    CtrlerComp_DigitalTriggerKey = 4,
    CtrlerComp_TriggerKey        = 5,
    CtrlerComp_VolumeKey         = 6,
    CtrlerComp_VolumeUpKey       = 7,
    CtrlerComp_VolumeDownKey     = 8,
    CtrlerComp_Grip              = 9,
    CtrlerComp_DPad_Left         = 10,
    CtrlerComp_DPad_Right        = 11,
    CtrlerComp_DPad_Up           = 12,
    CtrlerComp_DPad_Down         = 13,
    CtrlerComp_TouchPad_Touch    = 14,
    CtrlerComp_BeamOrigin        = 15,
    CtrlerComp_Emitter           = 16,
    CtrlerComp_Battery           = 17,
    CtrlerComp_BumperKey         = 18,
    CtrlerComp_Thumbstick        = 19,
    CtrlerComp_MaxCompNumber,
};

enum CtrlerBtnStateEnum
{
    CtrlerBtnState_None = 0,
    CtrlerBtnState_Tapped,
    CtrlerBtnState_Pressed,
};

enum CtrlerDrawModeEnum
{
    CtrlerDrawMode_General = 0,
    CtrlerDrawMode_Multiview,
    CtrlerDrawMode_MaxModeMumber
};

class Controller
{
public:
    static const std::string sControllerCompNames[CtrlerComp_MaxCompNumber];
public:
    explicit Controller(WVR_DeviceType iCtrlerType);
    ~Controller();
public:
    void loadControllerModelAsync();
    void render(CtrlerDrawModeEnum iMode, const Matrix4 iProjs[CtrlerDrawMode_MaxModeMumber], const Matrix4 iEyes[CtrlerDrawMode_MaxModeMumber], const Matrix4 &iView, const Matrix4 &iCtrlerPose);
    bool isThisCtrlerType(WVR_DeviceType iCtrlerType) const;
    void switchCtrlerType();
    void refreshButtonStatus(const WVR_Event_t &iEvent);
    void handleDisconnected();
    WVR_DeviceType getCtrlerType() const;
public:
    void setButtonEffectColor(float r, float g, float b, float a);
    void resetButtonEffects();
protected:
    void initializeGLComp();
    void releaseGLComp();
protected:
    void initializeCtrlerModelGLComp();//protected by mCachedDataMutex!!!
    void releaseCtrlerModelGLComp();
    uint32_t getCompIdxByName(const std::string &iName) const;
protected:
    void drawCtrlerBody(CtrlerDrawModeEnum iMode, const Matrix4 iMVPs[CtrlerDrawMode_MaxModeMumber]);
    void drawCtrlerBattery(CtrlerDrawModeEnum iMode, const Matrix4 iMVPs[CtrlerDrawMode_MaxModeMumber]);
    void drawCtrlerButtonEffect(CtrlerDrawModeEnum iMode, const Matrix4 iMVPs[CtrlerDrawMode_MaxModeMumber]);
    void drawCtrlerTouchPad(CtrlerDrawModeEnum iMode, const Matrix4 iMVPs[CtrlerDrawMode_MaxModeMumber]);
    void drawCtrlerRay(CtrlerDrawModeEnum iMode, const Matrix4 iMVPs[CtrlerDrawMode_MaxModeMumber]);
protected:
    void refreshBatteryStatus();
protected:
    float mBtnEffect[4];
    bool mIsShowBattery;
    int32_t mBatteryLevel;
    float mCalmDownTime;
protected:
    WVR_CtrlerModel_t *mCachedData;
    bool mIsDataReady;
    std::mutex mCachedDataMutex;
    std::mutex mLoadingThreadMutex; //**** IMPORTANT : only can used in lambda function in loadModelAsync
    bool mInitialized;
    WVR_DeviceType mCtrlerType;
    std::thread mLoadModelFuncThread;
protected: //component
    bool mCompExistFlags[CtrlerComp_MaxCompNumber];
    Mesh mCompMeshes[CtrlerComp_MaxCompNumber];
    int32_t mCompTexID[CtrlerComp_MaxCompNumber];
    Matrix4 mCompLocalMats[CtrlerComp_MaxCompNumber];
    CtrlerBtnStateEnum mCompStates[CtrlerComp_MaxCompNumber];
protected: //battery
    std::vector<Texture*> mBatLvTex;
    std::vector<int32_t> mBatMinLevels;
    std::vector<int32_t> mBatMaxLevels;
    std::chrono::system_clock::time_point mLastUpdateTime;
protected: //volume key.
    bool mIsOneVolumeKey;
protected: //touchpad plane matrix
    Matrix4 mTouchPadPlaneMat;
    float mFloatingDistance;
    float mRadius;
    Vector4 mTouchPadDotOffset;
    float mTouchpadSacleFactor;
    bool mIsNeedRevertInputY;
protected:
    Matrix4 mEmitterPose;
    Mesh mRayMesh;
protected: //shader
    std::vector<Texture*> mTextureTable;
    Shader *mTargetShader;
    std::shared_ptr<Shader> mShaders[CtrlerDrawMode_MaxModeMumber];
    int32_t mDiffTexLocations[CtrlerDrawMode_MaxModeMumber];
    int32_t mMatrixLocations[CtrlerDrawMode_MaxModeMumber];
    int32_t mUseEffectLocations[CtrlerDrawMode_MaxModeMumber];
    int32_t mEffectColorLocations[CtrlerDrawMode_MaxModeMumber];
    std::string mCurrentRenderModelName;
protected:
    Matrix4 mShift;
public:
    Matrix4 getEmitterPose();
};
