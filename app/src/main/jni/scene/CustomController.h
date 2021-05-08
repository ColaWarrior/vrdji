#pragma once

#include <mutex>
#include <thread>

#include "Controller.h"

class CustomController
{
public:
    explicit CustomController(WVR_DeviceType iCtrlerType);
    ~CustomController();
public:
    void loadControllerEmitterAsync();
    void render(CtrlerDrawModeEnum iMode, const Matrix4 iProjs[CtrlerDrawMode_MaxModeMumber], const Matrix4 iEyes[CtrlerDrawMode_MaxModeMumber], const Matrix4 &iView, const Matrix4 &iCtrlerPose);
    bool isThisCtrlerType(WVR_DeviceType iCtrlerType) const;
    void switchCtrlerType();
    WVR_DeviceType getCtrlerType() const;
protected:
    void drawCtrler(CtrlerDrawModeEnum iMode, const Matrix4 iMVPs[CtrlerDrawMode_MaxModeMumber]);
protected:
    void initializeGLComp();
    void releaseGLComp();
protected:
    bool mInitialized;
    WVR_DeviceType mCtrlerType;
    std::thread mGetEmitterFuncThread;
    std::mutex mLoadingThreadMutex; //**** IMPORTANT : only can used in lambda function in loadControllerEmitterAsync
protected:
    Mesh mCustomMesh;
protected:
    Matrix4 mEmitterPose;
    Mesh mRayMesh;
    Shader *mTargetShader;
    std::shared_ptr<Shader> mShaders[CtrlerDrawMode_MaxModeMumber];
    int32_t mMatrixLocations[CtrlerDrawMode_MaxModeMumber];
    int32_t mColorLocations[CtrlerDrawMode_MaxModeMumber];
};