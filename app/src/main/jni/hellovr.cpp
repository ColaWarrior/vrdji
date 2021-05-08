// "WaveVR SDK
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."

#include <log.h>
#include <string.h>
#include <unistd.h>
#include <limits>
#include <sys/types.h>
#include <sys/time.h>
#include <cstdlib>
#include <math.h>
#include <Texture.h>
#include <Picture.h>
#include <SkyBox.h>
#include <ControllerAxes.h>
#include <ControllerCube.h>
#include <Controller.h>
#include <ReticlePointer.h>
#include <FrameBufferObject.h>
#include <GLES2/gl2ext.h>
#include <GLES3/gl3.h>

#include <wvr/wvr.h>
#include <wvr/wvr_ctrller_render_model.h>
#include <wvr/wvr_render.h>
#include <wvr/wvr_device.h>
#include <wvr/wvr_projection.h>
#include <wvr/wvr_overlay.h>
#include <wvr/wvr_system.h>
#include <wvr/wvr_events.h>
#include <RaySphereIntersection.h>

#include "hellovr.h"

// Return micro second.  Should always positive because now is bigger.
#define timeval_subtract(now, last) \
    ((now.tv_sec - last.tv_sec) * 1000000LL + now.tv_usec - last.tv_usec)

#undef LOGENTRY
#define LOGENTRY(...)

bool gDebug = true;
bool gDebugOld = gDebug;
bool gMsaa = true;
bool gScene = false;
bool gSceneOld = gScene;
bool gUseScale = true;
float gScale = 1;

#define LOGDIF(args...) if (gDebug) LOGD(args)

#define VR_MAX_CLOCKS 200

// To demonstrate how to use heavy-effect foveated rendering
#define ENABLE_LOW_FOVEATED_RENDERING 0

// To demonstrate how to use WaveVR AdaptiveQuality
#define DISABLE_ADAPTIVE_QUALITY 0

static void dumpMatrix(const char * name, const Matrix4& mat) {
    const float * ptr = mat.get();
    LOGD("%s =\n"
        " ⎡%+6f  %+6f  %+6f  %+6f⎤\n"
        " ⎢%+6f  %+6f  %+6f  %+6f⎥\n"
        " ⎢%+6f  %+6f  %+6f  %+6f⎥\n"
        " ⎣%+6f  %+6f  %+6f  %+6f⎦\n",
        name,
        ptr[0], ptr[4], ptr[8],  ptr[12],
        ptr[1], ptr[5], ptr[9],  ptr[13],
        ptr[2], ptr[6], ptr[10], ptr[14],
        ptr[3], ptr[7], ptr[11], ptr[15]);
}

MainApplication::MainApplication()
        : mControllerCount(0)
        , mControllerCount_Last(-1)
        , mValidPoseCount(0)
        , mValidPoseCount_Last(-1)
        , mPoseClasses("")
        , m3DOF(true)
        , mMove(true)
        , mLight(true)
        , mTimeDiff(0.0f)
        , mDriveSpeed(0.0f)
        , mDriveAngle(0.0f)
        , mScene(-1)
        , mIndexLeft(0)
        , mIndexRight(0)
        , mLeftEyeQ(NULL)
        , mRightEyeQ(NULL)
        , mCurFocusController(WVR_DeviceType_HMD){
    // other initialization tasks are done in init
    memset(mDevClassChar, 0, sizeof(mDevClassChar));
    mSkyBox = NULL;
    mSphere=NULL;
    mFloor=NULL;
    mGridPicture = NULL;
    mReticlePointer = NULL;
#if defined(USE_CONTROLLER) || defined(USE_CUSTOM_CONTROLLER)
    mControllerObjs[0] = nullptr;
    mControllerObjs[1] = nullptr;
#else
    mControllerAxes = NULL;
    memset(mControllerCubeTableById, 0, sizeof(mControllerCubeTableById));
#endif
    LOGI("MainApplication::MainApplication()");
}

MainApplication::~MainApplication() {
    // work is done in Shutdown
    LOGI("Shutdown");
}

static void printGLString(const char *name, GLenum s) {
    const char *v = (const char *) glGetString(s);
    LOGI("GL %s = %s\n", name, v);
}

bool MainApplication::initVR() {
    LOGENTRY();
    LOGI("MainApplication::initVR()");
    // Loading the WVR Runtime
    WVR_InitError eError = WVR_InitError_None;
    LOGI("initVR():start call WVR_Init");
    eError = WVR_Init(WVR_AppType_VRContent);
    LOGI("initVR():start call WVR_Init end");
    if (eError != WVR_InitError_None) {
        LOGE("Unable to init VR runtime: %s", WVR_GetInitErrorString(eError));
        return false;
    }

    WVR_InputAttribute inputIdAndTypes[] = {
        {WVR_InputId_Alias1_Menu, WVR_InputType_Button, WVR_AnalogType_None},
        {WVR_InputId_Alias1_Touchpad, WVR_InputType_Button | WVR_InputType_Touch | WVR_InputType_Analog, WVR_AnalogType_2D},
        {WVR_InputId_Alias1_Trigger, WVR_InputType_Button , WVR_AnalogType_None},
        {WVR_InputId_Alias1_Bumper, WVR_InputType_Button , WVR_AnalogType_None}
    };
    WVR_SetInputRequest(WVR_DeviceType_HMD, inputIdAndTypes, sizeof(inputIdAndTypes) / sizeof(*inputIdAndTypes));
    WVR_SetInputRequest(WVR_DeviceType_Controller_Right, inputIdAndTypes, sizeof(inputIdAndTypes) / sizeof(*inputIdAndTypes));
    WVR_SetInputRequest(WVR_DeviceType_Controller_Left, inputIdAndTypes, sizeof(inputIdAndTypes) / sizeof(*inputIdAndTypes));

    WVR_SetArmModel(WVR_SimulationType_Auto); //add for use arm mode.

    // Must initialize render runtime before all OpenGL code.
    WVR_RenderInitParams_t param;
    param = { WVR_GraphicsApiType_OpenGL, WVR_RenderConfig_Default };

    WVR_RenderError pError = WVR_RenderInit(&param);
    if (pError != WVR_RenderError_None) {
        LOGE("Present init failed - Error[%d]", pError);
    }

    mInteractionMode = WVR_GetInteractionMode();
    mGazeTriggerType = WVR_GetGazeTriggerType();
    LOGI("initVR() mInteractionMode: %d, mGazeTriggerType: %d", mInteractionMode, mGazeTriggerType);
    return true;
}

bool MainApplication::initGL() {
    LOGENTRY();
    mNearClip = 0.1f;
    mFarClip = 30.0f;
    printGLString("Version", GL_VERSION);
    printGLString("Vendor", GL_VENDOR);
    printGLString("Renderer", GL_RENDERER);
    printGLString("Extensions", GL_EXTENSIONS);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(true);
    mLUV[0] = 0.0f;
    mLUV[1] = 0.0f;
    mUUV[0] = gScale;
    mUUV[1] = gScale;
    GLenum glerr = glGetError();
    if (glerr != GL_NO_ERROR) {
        LOGE("glGetError() before initGL: %d", glerr);
    }

#define OBJ_ERROR_CHECK(obj) if (obj->hasError() || obj->hasGLError()) return false


    mFloor = new Floor();
    OBJ_ERROR_CHECK(mFloor);

    oriSpherePos=Vector3(1,2,-4);
    mSphere = new Sphere(oriSpherePos);
    OBJ_ERROR_CHECK(mSphere);

    // Setup Scenes
    mSkyBox = new SkyBox(gDebug);
    OBJ_ERROR_CHECK(mSkyBox);
    mLightDir = mSkyBox->getLightDir();


    mGridPicture = new Picture();
    OBJ_ERROR_CHECK(mGridPicture);


#if defined(USE_CONTROLLER)
    mControllerObjs[0] = new Controller(WVR_DeviceType_Controller_Right);
    mControllerObjs[1] = new Controller(WVR_DeviceType_Controller_Left);
#elif defined(USE_CUSTOM_CONTROLLER)
    mControllerObjs[0] = new CustomController(WVR_DeviceType_Controller_Right);
    mControllerObjs[1] = new CustomController(WVR_DeviceType_Controller_Left);
#else
    mControllerAxes = new ControllerAxes();
    OBJ_ERROR_CHECK(mControllerAxes);
#endif

    mReticlePointer = new ReticlePointer();
    OBJ_ERROR_CHECK(mReticlePointer);

    setupCameras();

    // Setup stereo render targets
    WVR_GetRenderTargetSize(&mRenderWidth, &mRenderHeight);
    LOGD("Recommended size is %ux%u", mRenderWidth, mRenderHeight);
    if (mRenderWidth == 0 || mRenderHeight == 0) {
        LOGE("Please check server configure");
        return false;
    }

    mIndexLeft = 0;
    mIndexRight = 0;

    mLeftEyeQ = WVR_ObtainTextureQueue(WVR_TextureTarget_2D, WVR_TextureFormat_RGBA, WVR_TextureType_UnsignedByte, mRenderWidth, mRenderHeight, 0);
    for (int i = 0; i < WVR_GetTextureQueueLength(mLeftEyeQ); i++) {
        FrameBufferObject* fbo;

        fbo = new FrameBufferObject((int)(long)WVR_GetTexture(mLeftEyeQ, i).id, mRenderWidth, mRenderHeight, true);
        if (!fbo) return false;
        if (fbo->hasError())  {
            delete fbo;
            return false;
        }
        mLeftEyeFBOMSAA.push_back(fbo);

        fbo = new FrameBufferObject((int)(long)WVR_GetTexture(mLeftEyeQ, i).id, mRenderWidth, mRenderHeight);
        if (!fbo) return false;
        if (fbo->hasError())  {
            delete fbo;
            return false;
        }
        mLeftEyeFBO.push_back(fbo);
    }
    mRightEyeQ = WVR_ObtainTextureQueue(WVR_TextureTarget_2D, WVR_TextureFormat_RGBA, WVR_TextureType_UnsignedByte, mRenderWidth, mRenderHeight, 0);
    for (int i = 0; i < WVR_GetTextureQueueLength(mRightEyeQ); i++) {
        FrameBufferObject* fbo;

        fbo = new FrameBufferObject((int)(long)WVR_GetTexture(mRightEyeQ, i).id, mRenderWidth, mRenderHeight, true);
        if (!fbo) return false;
        if (fbo->hasError())  {
            delete fbo;
            return false;
        }
        mRightEyeFBOMSAA.push_back(fbo);

        fbo = new FrameBufferObject((int)(long)WVR_GetTexture(mRightEyeQ, i).id, mRenderWidth, mRenderHeight);
        if (!fbo) return false;
        if (fbo->hasError())  {
            delete fbo;
            return false;
        }
        mRightEyeFBO.push_back(fbo);
	}

#if defined(USE_CONTROLLER) || defined(USE_CUSTOM_CONTROLLER)
    setupControllers();
#else
    setupControllerCubes();
#endif
#undef OBJ_ERROR_CHECK

    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);

#if ENABLE_LOW_FOVEATED_RENDERING
    WVR_RenderFoveationMode(WVR_FoveationMode_Enable);
#endif

#if defined(DISABLE_ADAPTIVE_QUALITY) && DISABLE_ADAPTIVE_QUALITY
    // 1. WaveVR AQ is enabled with WVR_QualityStrategy_SendQualityEvent by default from WaveVR SDK 3.2.0
    // 2. WVR_EnableAdaptiveQuality must be invoked after WVR_RenderInit().
    // 3. If you don't want to use AQ, you need to disable it.
    WVR_EnableAdaptiveQuality(false);

    /* There are some strategies in WaveVR AQ can to choose.
     *
     * EX: Use auto foveated rendering to improve performance while it is not good enough.
     * WVR_EnableAdaptiveQuality(true, WVR_QualityStrategy_AutoFoveation);
     *
     * EX: Receive recommended quailty chagned event and auto foveated rendering to improve performance while
     *     it is not good enough.
     * WVR_EnableAdaptiveQuality(true, WVR_QualityStrategy_SendQualityEvent | WVR_QualityStrategy_AutoFoveation);
     */
#endif

    return true;
}

void MainApplication::shutdownGL() {
    LOGENTRY();


    if (mFloor != NULL )
        delete mFloor;
    mFloor = NULL;

    if (mSphere != NULL )
        delete mSphere;
    mSphere = NULL;

    if (mSkyBox != NULL)
        delete mSkyBox;
    mSkyBox = NULL;

    if (mGridPicture != NULL)
        delete mGridPicture;
    mGridPicture = NULL;

#if defined(USE_CONTROLLER) || defined(USE_CUSTOM_CONTROLLER)
    for (uint32_t cID = 0; cID < 2; ++cID) {
        if (mControllerObjs[cID] != nullptr) {
            delete mControllerObjs[cID];
        }
        mControllerObjs[cID] = nullptr;
    }
#else
    if (mControllerAxes != NULL)
        delete mControllerAxes;
    mControllerAxes = NULL;

    for (int i = 0; i < WVR_DEVICE_COUNT_LEVEL_1; i++) {
        if (mControllerCubeTableById[i] != NULL) {
            delete mControllerCubeTableById[i];
            mControllerCubeTableById[i] = NULL;
        }
    }
    mControllerCubes.clear();
#endif

    if (mReticlePointer != NULL)
        delete mReticlePointer;
    mReticlePointer = NULL;

    if (mLeftEyeQ != 0) {
        for (int i = 0; i < WVR_GetTextureQueueLength(mLeftEyeQ); i++) {
            delete mLeftEyeFBOMSAA.at(i);
            delete mLeftEyeFBO.at(i);
        }
        WVR_ReleaseTextureQueue(mLeftEyeQ);
    }

    if (mRightEyeQ != 0) {
        for (int i = 0; i < WVR_GetTextureQueueLength(mRightEyeQ); i++) {
            delete mRightEyeFBOMSAA.at(i);
            delete mRightEyeFBO.at(i);
        }
        WVR_ReleaseTextureQueue(mRightEyeQ);
    }
}

void MainApplication::shutdownVR() {
    WVR_Quit();
}



class ControllerState {
private:
    bool mLastTouchpadPressed;
    bool mTouchpadClicked;
    bool mMenuClicked;
    bool mLastMenuPressed;

public:

    ControllerState() : mLastTouchpadPressed(false), mTouchpadClicked(false), mLastMenuPressed(false), mMenuClicked(false) {
    }

    void updateState(WVR_DeviceType deviceType) {
        bool bTouchpadPressed = WVR_GetInputButtonState(deviceType, WVR_InputId_Alias1_Touchpad);
        if (mLastTouchpadPressed && (bTouchpadPressed) == 0) {
            mTouchpadClicked = true;
        } else {
            mTouchpadClicked = false;
        }
        mLastTouchpadPressed = bTouchpadPressed;

        bool bMenuPressed = WVR_GetInputButtonState(deviceType, WVR_InputId_Alias1_Menu);
        if (mLastMenuPressed && (bMenuPressed) == 0) {
            mMenuClicked = true;
        } else {
            mMenuClicked = false;
        }
        mLastMenuPressed = bMenuPressed;
    }

    inline bool touchpadClicked() const {
        return mTouchpadClicked;
    }

    inline bool menuClicked() const {
        return mMenuClicked;
    }
};

void MainApplication::moveSphereHandler() {
    Vector3 pos;

    if (!mSphere) return;
    if (mCurFocusController==WVR_DeviceType_Controller_Right) {
        if(mSphere->getCenter()==oriSpherePos){
            pos=Vector3(1,0,0)+mSphere->getCenter();
        }else{
            pos=oriSpherePos;
        }
        mSphere->setSpherePos(pos);
    } else if (mCurFocusController==WVR_DeviceType_Controller_Left) {
        if(mSphere->getCenter()==oriSpherePos){
            pos=Vector3(-1,0,0)+mSphere->getCenter();
        }else{
            pos=oriSpherePos;
        }
        mSphere->setSpherePos(pos);
    }
}

//-----------------------------------------------------------------------------
// Purpose: Poll events.  Quit application if return true.
//-----------------------------------------------------------------------------
bool MainApplication::handleInput() {
    LOGENTRY();
    static ControllerState states[WVR_DEVICE_COUNT_LEVEL_1];

    bool resolutionChange = false;

    if (gScene != gSceneOld) {
        gSceneOld = gScene;
    }

    if (gDebug != gDebugOld) {
        gDebugOld = gDebug;
        mSkyBox->setDebug(gDebug);
        mLightDir = mSkyBox->getLightDir();
        setupCameras();
    }

    // Process WVR events
    bool isCtrlerStatusChange = false;
    WVR_Event_t event;
    while(WVR_PollEventQueue(&event)) {
        if (event.common.type == WVR_EventType_Quit) {
            return true;
        }

        if (event.common.type == WVR_EventType_DeviceConnected ||
            event.common.type == WVR_EventType_DeviceDisconnected) {
            isCtrlerStatusChange = true;
            if (event.common.type == WVR_EventType_DeviceConnected) {
                LOGI("WVR_EventType_DeviceConnected");
            } else {
                LOGI("WVR_EventType_DeviceDisconnected");
            }
        }
        processVREvent(event);

#if defined(USE_CONTROLLER) || defined(USE_CUSTOM_CONTROLLER)
            handleControllerConnectEvent(isCtrlerStatusChange);
#endif

        if (event.common.type == WVR_EventType_ButtonPressed) {
            if (event.device.deviceType == WVR_DeviceType_Controller_Right && event.input.inputId == WVR_InputId_Alias1_Bumper
                && mCurFocusController==WVR_DeviceType_Controller_Right){
                moveSphereHandler();
            }
            if (event.device.deviceType == WVR_DeviceType_Controller_Left && event.input.inputId == WVR_InputId_Alias1_Bumper
                && mCurFocusController==WVR_DeviceType_Controller_Left){
                moveSphereHandler();
            }

            if (event.device.deviceType == WVR_DeviceType_Controller_Right && event.input.inputId == WVR_InputId_Alias1_Trigger
                && mCurFocusController==WVR_DeviceType_Controller_Right){
                moveSphereHandler();
            }
            if (event.device.deviceType == WVR_DeviceType_Controller_Left && event.input.inputId == WVR_InputId_Alias1_Trigger
                && mCurFocusController==WVR_DeviceType_Controller_Left){
                moveSphereHandler();
            }

            if (event.device.deviceType == WVR_DeviceType_Controller_Right && event.input.inputId == WVR_InputId_Alias1_Touchpad
                && mCurFocusController==WVR_DeviceType_Controller_Right){
                moveSphereHandler();
            }
 
            if (event.device.deviceType == WVR_DeviceType_Controller_Left && event.input.inputId == WVR_InputId_Alias1_Touchpad
                && mCurFocusController==WVR_DeviceType_Controller_Left){
                moveSphereHandler();
            }

        }
    }
    if (resolutionChange) {
        switchResolution();
    }
    return false;
}

void MainApplication::switchResolution() {
#if ENABLE_LOW_FOVEATED_RENDERING
    LOGI("menu key pressed");
#else
    if (gUseScale == true) {
            if (std::abs(gScale - 1.0) <= std::numeric_limits<float>::epsilon()) {
                gScale = 0.5;
            } else {
                gScale += 0.1;
            }
            mUUV[0] = gScale;
            mUUV[1] = gScale;

            FrameBufferObject* fbo=NULL;

            for (int i = 0; i < WVR_GetTextureQueueLength(mLeftEyeQ); i++) {
                fbo = gMsaa ? mLeftEyeFBOMSAA.at(i) : mLeftEyeFBO.at(i);
                fbo->resizeFrameBuffer(gScale);
            }

            for (int i = 0; i < WVR_GetTextureQueueLength(mRightEyeQ); i++) {
                fbo = gMsaa ? mRightEyeFBOMSAA.at(i) : mRightEyeFBO.at(i);
                fbo->resizeFrameBuffer(gScale);
            }
    }
#endif
}

void MainApplication::switchInteractionMode() {
    if (mInteractionMode == WVR_InteractionMode_SystemDefault) {
        mInteractionMode = WVR_InteractionMode_Gaze;
    } else if (mInteractionMode == WVR_InteractionMode_Gaze) {
        mInteractionMode = WVR_InteractionMode_Controller;
    } else if (mInteractionMode == WVR_InteractionMode_Controller) {
        mInteractionMode = WVR_InteractionMode_SystemDefault;
    }
    LOGD("switchInteractionMode mInteractionMode: %d", mInteractionMode);
    WVR_SetInteractionMode(mInteractionMode);
}

void MainApplication::switchGazeTriggerType() {
    if (mGazeTriggerType == WVR_GazeTriggerType_Timeout) {
        mGazeTriggerType = WVR_GazeTriggerType_Button;
    } else if (mGazeTriggerType == WVR_GazeTriggerType_Button) {
        mGazeTriggerType = WVR_GazeTriggerType_TimeoutButton;
    } else if (mGazeTriggerType == WVR_GazeTriggerType_TimeoutButton) {
        mGazeTriggerType = WVR_GazeTriggerType_Timeout;
    }
    LOGD("switchGazeTriggerType mGazeTriggerType: %d", mGazeTriggerType);
    WVR_SetGazeTriggerType(mGazeTriggerType);
}

//-----------------------------------------------------------------------------
// Purpose: Processes a single VR event
//-----------------------------------------------------------------------------
void MainApplication::processVREvent(const WVR_Event_t & event) {
    switch(event.common.type) {
    case WVR_EventType_DeviceConnected:
        {
#if defined(USE_CONTROLLER) || defined(USE_CUSTOM_CONTROLLER)
#else
            setupControllerCubeForDevice(event.device.deviceType);
            LOGD("Device %u attached. Setting up controller cube.\n", event.device.deviceType);
#endif
        }
        break;
    case WVR_EventType_ButtonPressed:
    case WVR_EventType_ButtonUnpressed:
    case WVR_EventType_TouchTapped:
    case WVR_EventType_TouchUntapped:
        {
#if defined(USE_CONTROLLER)
            WVR_DeviceType ctrlType = event.device.deviceType;
            LOGI("Device Btn Event:C(%d)EType(%d)", ctrlType, event.common.type);
            for (uint32_t cID = 0; cID < 2; ++cID) {
                if (mControllerObjs[cID] != nullptr) {
                    if (mControllerObjs[cID]->isThisCtrlerType(ctrlType) == true) {
                        mControllerObjs[cID]->refreshButtonStatus(event);
                    }
                }
            }
#else
#endif
        }
        break;
    case WVR_EventType_DeviceRoleChanged:
        {
#if defined(USE_CONTROLLER) || defined(USE_CUSTOM_CONTROLLER)
            LOGI("DeviceRoleChanged.");
            for (uint32_t cID = 0; cID < 2; ++cID) {
                if (mControllerObjs[cID] != nullptr) {
                    mControllerObjs[cID]->switchCtrlerType();
                }
            }
#else
            LOGD("Device %u role change.\n", event.device.deviceType);
#endif
        }
        break;
    case WVR_EventType_DeviceStatusUpdate:
        {
            LOGD("Device %u updated.\n", event.device.deviceType);
#if defined(USE_CONTROLLER) || defined(USE_CUSTOM_CONTROLLER)
#else
            // Delete if the controller cube of device id already exists and is not real.
            int index = event.device.deviceType - WVR_DeviceType_HMD;  // Shift the HMD base as index
            if (index > WVR_DEVICE_HMD && index < WVR_DEVICE_COUNT_LEVEL_1) {
                if (mControllerCubeTableById[index] != NULL) {
                    ControllerCube *pControllerCube = mControllerCubeTableById[index];
                    for (std::vector<ControllerCube *>::iterator iter = mControllerCubes.begin(); iter != mControllerCubes.end(); iter++) {
                        if ((*iter) == pControllerCube) {
                            LOGD("Find empty controller cube, delete it.");
                            mControllerCubes.erase(iter);
                            break;
                        }
                    }
                    LOGI("Clear the controller cube of device[%d]", index);
                    delete pControllerCube;
                    mControllerCubeTableById[index] = NULL;
                }
                setupControllerCubeForDevice(event.device.deviceType);
            }
#endif
        }
        break;
    case WVR_EventType_IpdChanged:
        {
            WVR_RenderProps_t props;
            bool ret = WVR_GetRenderProps(&props);
            float ipd = 0;
            if (ret) {
                ipd = props.ipdMeter;
            }
            LOGI("IPD is changed (%.4f) and renew the transform from eye to head.", ipd);
            mProjectionLeft = wvrmatrixConverter(
                WVR_GetProjection(WVR_Eye_Left, mNearClip, mFarClip));
            mProjectionRight = wvrmatrixConverter(
                WVR_GetProjection(WVR_Eye_Right, mNearClip, mFarClip));

            mEyePosLeft = wvrmatrixConverter(
                WVR_GetTransformFromEyeToHead(WVR_Eye_Left)).invert();
            mEyePosRight = wvrmatrixConverter(
                WVR_GetTransformFromEyeToHead(WVR_Eye_Right)).invert();

            dumpMatrix("ProjectionLeft", mProjectionLeft);
            dumpMatrix("ProjectionRight", mProjectionRight);
            dumpMatrix("EyePosLeft", mEyePosLeft);
            dumpMatrix("EyePosRight", mEyePosRight);
        }
        break;
    case WVR_EventType_SystemInteractionModeChanged:
        {
            mInteractionMode = WVR_GetInteractionMode();
            LOGI("Receive WVR_EventType_SystemInteractionModeChanged mode = %d", mInteractionMode);
        }
        break;
    case WVR_EventType_SystemGazeTriggerTypeChanged:
        {
            mGazeTriggerType = WVR_GetGazeTriggerType();
            LOGI("Receive WVR_EventType_SystemGazeTriggerTypeChanged type = %d", mGazeTriggerType);
        }
        break;
    // If WaveVR AQ have enabled with WVR_QualityStrategy_SendQualityEvent, you will receive
    // the wvr recommendedQuality_Lower/Higher event based on rendering performance.
    case WVR_EventType_RecommendedQuality_Lower:
        {
            /* Once you got this recommend event:
             * 1. If your conetent rendering quality has already in the worst quality, you can ignore this event.
             * 2. Or, you can adjust the rendering resolution lower gradually by re-create texture queue, disable MSAA, etc.
             */
            LOGI("[Sample] Get WVR_EventType_RecommendedQuality_Lower");
        }
        break;
    case WVR_EventType_RecommendedQuality_Higher:
        {
            /* 1. Once you got this recommend event:
             * 1. If your conetent rendering quality has already in the best quality, you can ignore this event.
             * 2. Or, you can adjust the rendering resolution higher ASAP by re-create texture queue, enable MSAA, etc.
             */
            LOGI("[Sample] Get WVR_EventType_RecommendedQuality_Higher");
        }
        break;
    default:
        break;
    }
}

#if defined(USE_CONTROLLER) || defined(USE_CUSTOM_CONTROLLER)
void MainApplication::handleControllerConnectEvent(bool iIsCtrlerStatusChanged)
{
    //1. check controller connection state, if controller is connected, we loading. Otherwise, we release.
    //*** reload empty controller won't memory leak because we will release resource before we initialize graphics res.
    //*** release empty controller won't cause crash because we will check validation resource is exist before we release.
    if (iIsCtrlerStatusChanged == true) {
        LOGI("[APCtrler] Trigger connect or disconnect.");
        for (uint32_t cID = 0; cID < 2; ++cID) {
            if (mControllerObjs[cID] != nullptr) {
                //2.1 get ctrler device type.
                WVR_DeviceType ctrlerType = mControllerObjs[cID]->getCtrlerType();
                bool ctrlerConStatus = WVR_IsDeviceConnected(ctrlerType);
                if (ctrlerConStatus == true) {// connect
#if defined(USE_CONTROLLER)
                    mControllerObjs[cID]->loadControllerModelAsync();
#else
                    mControllerObjs[cID]->loadControllerEmitterAsync();
#endif
                } else {
#if defined(USE_CONTROLLER)
                    mControllerObjs[cID]->handleDisconnected();
#endif
                }
            }
        }
    }
}
#endif

bool MainApplication::renderFrame() {
    LOGENTRY();

    unsigned int ext = WVR_SubmitExtend_Default;

	mIndexLeft = WVR_GetAvailableTextureIndex(mLeftEyeQ);
    mIndexRight = WVR_GetAvailableTextureIndex(mRightEyeQ);

    //LOGD("renderFrame start");
    // for now as fast as possible
    drawControllers();

    if (mInteractionMode == WVR_InteractionMode_Gaze) {
        drawReticlePointer();
    }
    renderStereoTargets();
    ext |= WVR_SubmitExtend_Default;
#if ENABLE_LOW_FOVEATED_RENDERING
#else
    if (gScale < 1 && gScale > 0)
        ext |= WVR_SubmitExtend_PartialTexture;
#endif

    WVR_TextureParams_t leftEyeTexture = WVR_GetTexture(mLeftEyeQ, mIndexLeft);
    WVR_SubmitError e;

    leftEyeTexture.layout.leftLowUVs.v[0] = 0;
    leftEyeTexture.layout.leftLowUVs.v[1] = 0;
    leftEyeTexture.layout.rightUpUVs.v[0] = 1;
    leftEyeTexture.layout.rightUpUVs.v[1] = 1;
#if ENABLE_LOW_FOVEATED_RENDERING
#else
        if (gScale < 1 && gScale > 0) {
            leftEyeTexture.layout.leftLowUVs.v[0] = mLUV[0];
            leftEyeTexture.layout.leftLowUVs.v[1] = mLUV[1];
            leftEyeTexture.layout.rightUpUVs.v[0] = mUUV[0];
            leftEyeTexture.layout.rightUpUVs.v[1] = mUUV[1];
        }
#endif
        e = WVR_SubmitFrame(WVR_Eye_Left, &leftEyeTexture, NULL, (WVR_SubmitExtend)ext);
        if (e != WVR_SubmitError_None) return true;

        // Right eye
        WVR_TextureParams_t rightEyeTexture = WVR_GetTexture(mRightEyeQ, mIndexRight);

        rightEyeTexture.layout.leftLowUVs.v[0] = 0;
        rightEyeTexture.layout.leftLowUVs.v[1] = 0;
        rightEyeTexture.layout.rightUpUVs.v[0] = 1;
        rightEyeTexture.layout.rightUpUVs.v[1] = 1;

#if ENABLE_LOW_FOVEATED_RENDERING
#else
        if (gScale < 1 && gScale > 0) {
            rightEyeTexture.layout.leftLowUVs.v[0] = mLUV[0];
            rightEyeTexture.layout.leftLowUVs.v[1] = mLUV[1];
            rightEyeTexture.layout.rightUpUVs.v[0] = mUUV[0];
            rightEyeTexture.layout.rightUpUVs.v[1] = mUUV[1];
        }
#endif
        e = WVR_SubmitFrame(WVR_Eye_Right, &rightEyeTexture, NULL, (WVR_SubmitExtend)ext);
        if (e != WVR_SubmitError_None) return true;

    updateTime();

    // Clear
    {
        // We want to make sure the glFinish waits for the entire present to complete, not just the submission
        // of the command. So, we do a clear here right here so the glFinish will wait fully for the swap.
        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    // Spew out the controller and pose count whenever they change.
    if (mControllerCount != mControllerCount_Last || mValidPoseCount != mValidPoseCount_Last) {
        mValidPoseCount_Last = mValidPoseCount;
        mControllerCount_Last = mControllerCount;

        LOGD("PoseCount:%d(%s) Controllers:%d\n", mValidPoseCount, mPoseClasses.c_str(), mControllerCount);
    }

    usleep(1);
    //LOGD("renderFrame end");

    return false;
}

//-----------------------------------------------------------------------------
// Purpose: Draw all of the controllers as X/Y/Z lines
//-----------------------------------------------------------------------------
void MainApplication::drawControllers() {
    // don't draw controllers if somebody else has input focus
//    LOGI("drawControllers(): start");
    if (WVR_IsInputFocusCapturedBySystem())
        return;

    if (mInteractionMode == WVR_InteractionMode_Gaze) {
        return;
    }

    std::vector<float> buffer;

    int vertCount = 0;
    mControllerCount = 0;
    WVR_DeviceType type;
    for (uint32_t id = WVR_DEVICE_HMD + 1; id < WVR_DEVICE_COUNT_LEVEL_1; ++id) {
        if ((mVRDevicePairs[id].type != WVR_DeviceType_Controller_Right) && (mVRDevicePairs[id].type != WVR_DeviceType_Controller_Left)){
//            LOGD("drawControllers(): not Controller : %d ", mVRDevicePairs[id].type);
            continue;
        }

        if (!WVR_IsDeviceConnected(mVRDevicePairs[id].type)){
//            LOGD("drawControllers(): DeviceType: %d pose is disconnected :", mVRDevicePairs[id].type);
            continue;
        }

        if (!mVRDevicePairs[id].pose.isValidPose) {
//            LOGD("drawControllers(): DeviceType: %d pose is invalid", mVRDevicePairs[id].type);
            continue;
        }

        type= mVRDevicePairs[id].type;
//        LOGD("drawControllers(): DeviceType: %d start ",type);


        Matrix4 mat;
        if (m3DOF) {
            // If the controller is 3DOF always put the model in the bottom of the view.
            mat = mDevicePoseArray[WVR_DEVICE_HMD];
            mat.invert();
            float angleY = atan2f(-mat[8], mat[10]);  // Yaw
            float angleX = asin(-mat[9]);             // Pitch
            float angleZ = atan2f(mat[1], mat[5]);    // Roll

            mat.identity().rotateY(-angleY / M_PI * 180.0f);
            mat.rotateX(angleX / M_PI * 180.0f);
            mat.rotateZ(angleZ / M_PI * 180.0f);
            mat *= mDevicePoseArray[id];

            int offset = mVRDevicePairs[id].type - WVR_DeviceType_Controller_Right;
            float x = (offset % 2) == 0 ? 0.1f : -0.1f;
            float z = -0.45f - (offset / 2) * 0.3f;
            mat.setColumn(3, Vector4(x,-0.12f,z,1));
        } else {
            mat = mDevicePoseArray[id];
        }
#if defined(USE_CONTROLLER) || defined(USE_CUSTOM_CONTROLLER)
#else
        vertCount += mControllerAxes->makeVertices(mat, buffer);
#endif
        mControllerCount += 1;

        /**
         * Add Raycaster to hit the sphere
         */
        Matrix4 WorldFromHead;
        WorldFromHead=mDevicePoseArray[WVR_DEVICE_HMD];
        Matrix4 WorldFromController_new=WorldFromHead*mat;

        Matrix4 mat4WorldRotation;
        mat4WorldRotation.rotate(mWorldRotation, 0, 1, 0); // if world is rotated , the ray of controller is also changed too.

#if defined(USE_CONTROLLER) || defined(USE_CUSTOM_CONTROLLER)
        uint32_t ctrlerRealID = 0;
        for (uint32_t cID = 0; cID < 2; ++cID) {
            if (mControllerObjs[cID] != nullptr) {
                if (mControllerObjs[cID]->getCtrlerType() == mVRDevicePairs[id].type) {
                    ctrlerRealID = cID;
                    break;
                }
            }
        }
        Matrix4 emitterPose = mControllerObjs[ctrlerRealID]->getEmitterPose();

        WorldFromController_new = mWorldTranslation * mat4WorldRotation * WorldFromController_new * emitterPose; //Because default World position that wee see doesn't base on (0,0,0) , so we need to use actual translation matrix to make ray coordinate as same as current coordinate of the world.
#else
        WorldFromController_new = mWorldTranslation * mat4WorldRotation * WorldFromController_new; //Because default World position that wee see doesn't base on (0,0,0) , so we need to use actual translation matrix to make ray coordinate as same as current coordinate of the world.
#endif
        Point3D origin = {WorldFromController_new[12], WorldFromController_new[13], WorldFromController_new[14]};

        Vector3 front(0.0f, 0.0f, -1.0f);
        Matrix3 mat3(WorldFromController_new[0], WorldFromController_new[1], WorldFromController_new[2], WorldFromController_new[4], WorldFromController_new[5], WorldFromController_new[6], WorldFromController_new[8], WorldFromController_new[9], WorldFromController_new[10]);
        Vector3 direction3 =(mat3 * front).normalize();

        Vector3D direction = {direction3.x, direction3.y, direction3.z};

        Vector3 c; //get real collision point
        Ray3D ray = {origin, direction,c};

        // Get center & radius of the sphere
        Vector3 sCenter = mSphere->getCenter();
        Point3D center = {sCenter.x, sCenter.y, sCenter.z};
        Sphere3D sphere = {center, mSphere->getRadius()};

//        LOGD("drawControllers(): mPointToSphere DeviceType start: %d ", mVRDevicePairs[id].type);
        // Check if the ray intersects the sphere

        mPointToSphere = intersection(ray, sphere);

        if(mVRDevicePairs[id].type==WVR_DeviceType_Controller_Right){

            if(mPointToSphere){
//                LOGD("drawControllers(): Right_Controller Hit");
                if(!mPointToSphere_R){
                    currColor=Sphere::Color ::red;
                    mPointToSphere_R=true;
                    mCurFocusController=WVR_DeviceType_Controller_Right;
                }
            }else{
                if (mPointToSphere_R){
                    mPointToSphere_R= false;
                    if(mPointToSphere_L){
                        currColor=Sphere::Color ::blue;
                        mCurFocusController=WVR_DeviceType_Controller_Left;
                    }else{
                        currColor=Sphere::Color ::green;
                        mCurFocusController=WVR_DeviceType_HMD;
                    }
                }

            }
        }else{
            if(mPointToSphere){
//                LOGD("drawControllers(): Left_Controller Hit");
                if(!mPointToSphere_L) {
                    currColor=Sphere::Color ::blue;
                    mPointToSphere_L=true;
                    mCurFocusController=WVR_DeviceType_Controller_Left;
                }
            }else{
                if (mPointToSphere_L){
                    mPointToSphere_L= false;
                    if(mPointToSphere_R){
                        currColor=Sphere::Color ::red;
                        mCurFocusController=WVR_DeviceType_Controller_Right;
                    }else{
                        currColor=Sphere::Color ::green;
                        mCurFocusController=WVR_DeviceType_HMD;
                    }
                }

            }
        }
    }
#if defined(USE_CONTROLLER) || defined(USE_CUSTOM_CONTROLLER)
#else
    mControllerAxes->setVertices(buffer, vertCount);
#endif
//    LOGI("drawControllers(): end");
}

//-----------------------------------------------------------------------------
// Purpose: Draw reticle pointer
//-----------------------------------------------------------------------------
void MainApplication::drawReticlePointer() {
    if (WVR_IsInputFocusCapturedBySystem())
        return;

        std::vector<float> buffer;

        int vertCount = 0;
        WVR_DeviceType type;
        if (!mVRDevicePairs[WVR_DEVICE_HMD].pose.isValidPose) {
            LOGD("drawReticle(): DeviceType: HMD pose is invalid");
            return;
        }
        type= WVR_DeviceType_HMD;

        Matrix4 mat;
        /**
        * Add Raycaster to hit the sphere
        */
        Matrix4 WorldFromHead;
        Matrix4 WorldFromReticlePointer_new;
        //WorldFromHead =mDevicePoseArray[WVR_DEVICE_HMD];
        WorldFromReticlePointer_new = mDevicePoseArray[WVR_DEVICE_HMD];

        Matrix4 mat4WorldRotation;
        mat4WorldRotation.rotate(mWorldRotation, 0, 1, 0); // if world is rotated , the reticle pointer is also changed too.
        WorldFromReticlePointer_new = mWorldTranslation*mat4WorldRotation * WorldFromReticlePointer_new; //Because default World position that wee see doesn't base on (0,0,0) , so we need to use actual translation matrix to make ray coordinate as same as current coordinate of the world.
        Point3D origin = {WorldFromReticlePointer_new[12], WorldFromReticlePointer_new[13], WorldFromReticlePointer_new[14]};

        Vector3 front(0.0f, 0.0f, -1.0f);
        Matrix3 mat3(WorldFromReticlePointer_new[0], WorldFromReticlePointer_new[1], WorldFromReticlePointer_new[2], WorldFromReticlePointer_new[4], WorldFromReticlePointer_new[5], WorldFromReticlePointer_new[6], WorldFromReticlePointer_new[8], WorldFromReticlePointer_new[9], WorldFromReticlePointer_new[10]);
        Vector3 direction3 = (mat3 * front).normalize();

        Vector3D direction = {direction3.x, direction3.y, direction3.z};
        Vector3 c; //get real collision point
        Ray3D ray = {origin, direction,c};

        // Get center & radius of the sphere
        Vector3 sCenter = mSphere->getCenter();
        Point3D center = {sCenter.x, sCenter.y, sCenter.z};

        Sphere3D sphere = {center, mSphere->getRadius()};

        //Check if the ray intersects the sphere
        mPointToSphere = intersection(ray, sphere);

        if(mPointToSphere){
            currColor= (WVR_GetDefaultControllerRole() == WVR_DeviceType_Controller_Right)
                ? Sphere::Color ::red : Sphere::Color ::blue;
            mCurFocusController = (WVR_GetDefaultControllerRole() == WVR_DeviceType_Controller_Right)
                ? WVR_DeviceType_Controller_Right : WVR_DeviceType_Controller_Left;
        }else{
            currColor=Sphere::Color ::green;
            mCurFocusController = WVR_DeviceType_HMD;
        }
    vertCount += mReticlePointer->makeVertices(WorldFromReticlePointer_new, buffer);
    mReticlePointer->setVertices(buffer, vertCount);
}

void MainApplication::setupCameras() {
    mProjectionLeft = wvrmatrixConverter(
        WVR_GetProjection(WVR_Eye_Left, mNearClip, mFarClip));
    mProjectionRight = wvrmatrixConverter(
        WVR_GetProjection(WVR_Eye_Right, mNearClip, mFarClip));

    mEyePosLeft = wvrmatrixConverter(
        WVR_GetTransformFromEyeToHead(WVR_Eye_Left)).invert();
    mEyePosRight = wvrmatrixConverter(
        WVR_GetTransformFromEyeToHead(WVR_Eye_Right)).invert();

    dumpMatrix("ProjectionLeft", mProjectionLeft);
    dumpMatrix("ProjectionRight", mProjectionRight);
    dumpMatrix("EyePosLeft", mEyePosLeft);
    dumpMatrix("EyePosRight", mEyePosRight);

    // Initial position need a little backward and upper to avoid been in a cube.
    mWorldTranslation.identity().setColumn(3, Vector4(1.0f, 1.5f, 2.0f, 1));
    mWorldRotation = 0;
    gettimeofday(&mRtcTime, NULL);
}

void MainApplication::renderStereoTargets() {
    LOGENTRY();
    glClearColor(0.30f, 0.30f, 0.37f, 1.0f); // nice background color, but not black
    FrameBufferObject * fbo = NULL;

    fbo = gMsaa ? mLeftEyeFBOMSAA.at(mIndexLeft) : mLeftEyeFBO.at(mIndexLeft);
    fbo->bindFrameBuffer();

    WVR_TextureParams_t leftEyeTexture = WVR_GetTexture(mLeftEyeQ, mIndexLeft);
#if ENABLE_LOW_FOVEATED_RENDERING
        WVR_RenderFoveationParams_t foveated;
        foveated.focalX = foveated.focalY = 0.0f;
        foveated.fovealFov = 30.0f;
        foveated.periQuality = WVR_PeripheralQuality_Low;
        fbo->glViewportFull();
        WVR_PreRenderEye(WVR_Eye_Left, &leftEyeTexture, &foveated);
#else
        if (gScale < 1 && gScale > 0)
            fbo->glViewportScale(mLUV, mUUV);
        else
            fbo->glViewportFull();
        WVR_PreRenderEye(WVR_Eye_Left, &leftEyeTexture);
#endif
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderScene(WVR_Eye_Left);
        fbo->unbindFrameBuffer();

        // Right Eye
        fbo = gMsaa ? mRightEyeFBOMSAA.at(mIndexRight) : mRightEyeFBO.at(mIndexRight);
        fbo->bindFrameBuffer();
        WVR_TextureParams_t rightEyeTexture = WVR_GetTexture(mRightEyeQ, mIndexRight);
#if ENABLE_LOW_FOVEATED_RENDERING
        foveated.focalX = foveated.focalY = 0.0f;
        foveated.fovealFov = 30.0f;
        foveated.periQuality = WVR_PeripheralQuality_Low;
        fbo->glViewportFull();
        WVR_PreRenderEye(WVR_Eye_Right, &rightEyeTexture, &foveated);
#else
        if (gScale < 1 && gScale > 0)
            fbo->glViewportScale(mLUV, mUUV);
        else
            fbo->glViewportFull();
        WVR_PreRenderEye(WVR_Eye_Right, &rightEyeTexture);
#endif
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderScene(WVR_Eye_Right);
        fbo->unbindFrameBuffer();
}


void MainApplication::renderScene(WVR_Eye nEye) {
    WVR_RenderMask(nEye);


    if (mGridPicture && mGridPicture->isEnabled()) {
        if (nEye == WVR_Eye_Left)
            mGridPicture->draw(mProjectionLeft, mEyePosLeft, mHMDPose, mLightDir);
        else if (nEye == WVR_Eye_Right)
            mGridPicture->draw(mProjectionRight, mEyePosRight, mHMDPose, mLightDir);
        return;
    }



    // Controller Axes
    bool isInputCapturedBySystem = WVR_IsInputFocusCapturedBySystem();
#if defined(USE_CONTROLLER) || defined(USE_CUSTOM_CONTROLLER)
    if (isInputCapturedBySystem == false) {
        for (uint32_t cID = 0; cID < 2; ++cID) {
            if (mControllerObjs[cID] != nullptr && mInteractionMode != WVR_InteractionMode_Gaze) {
                //uint32_t idx = mControllerObjs[cID]->getCtrlerType() - WVR_DeviceType_HMD;
                uint32_t ctrlerRealID = 0;
                for (uint32_t devID = 0; devID < WVR_DEVICE_COUNT_LEVEL_1; ++devID) {
                    if (mControllerObjs[cID]->getCtrlerType() == mVRDevicePairs[devID].type) {
                        ctrlerRealID = devID;
                        break;
                    }
                }
                if (mVRDevicePairs[ctrlerRealID].pose.isValidPose == true) {
                    Matrix4 ctrlerPose = mDevicePoseArray[ctrlerRealID];
                    Matrix4 projs[2], eyes[2];
                    Matrix4 view;
                    if(m3DOF){
                        view = mHMDPose; 
                    }
                    if (nEye == WVR_Eye_Left) {
                        projs[0] = mProjectionLeft;
                        eyes[0] = mEyePosLeft;
                        mControllerObjs[cID]->render(CtrlerDrawMode_General, projs, eyes, view, ctrlerPose);
                    } else if (nEye == WVR_Eye_Right) {
                        projs[0] = mProjectionRight;
                        eyes[0] = mEyePosRight;
                        mControllerObjs[cID]->render(CtrlerDrawMode_General, projs, eyes, view, ctrlerPose);
                    }
                } else {
#if defined(USE_CONTROLLER)
                    mControllerObjs[cID]->resetButtonEffects();
#endif
                }
            }
        }
    }
#else
    if (!isInputCapturedBySystem) {
        Matrix4 view;
        if (!m3DOF) {
            view = mHMDPose;
        }
        if (mControllerAxes) {
            if (mInteractionMode == WVR_InteractionMode_SystemDefault || mInteractionMode == WVR_InteractionMode_Controller){
                if (nEye == WVR_Eye_Left)
                    mControllerAxes->draw(mProjectionLeft, mEyePosLeft, view, mLightDir);
                else if (nEye == WVR_Eye_Right)
                    mControllerAxes->draw(mProjectionRight, mEyePosRight, view, mLightDir);
            }
        }
    }

    // Controller cube
    int localControllerIdx = -1;
    for (uint32_t id = WVR_DEVICE_HMD + 1;
            id < WVR_DEVICE_COUNT_LEVEL_1; id++) {
        if ((mVRDevicePairs[id].type != WVR_DeviceType_Controller_Right) && (mVRDevicePairs[id].type != WVR_DeviceType_Controller_Left))
            continue;

        if (!WVR_IsDeviceConnected(mVRDevicePairs[id].type))
            continue;

        const WVR_PoseState_t & pose = mVRDevicePairs[id].pose;
        if (!pose.isValidPose)
            continue;

        if (isInputCapturedBySystem)
            continue;

        // When device Connection is confirmed, keep its location even not draw it.
        localControllerIdx++;

        // Load the controller cube asynchronously.
        if (mControllerCubeTableById[id] == NULL) {
            setupControllerCubeForDevice((WVR_DeviceType)(id + WVR_DeviceType_HMD));
            continue;
        }

        // If button Grip is pressed, hide controller cube.
        if (!mShowDeviceArray[id])
            continue;

        ControllerCube * ControllerCube = mControllerCubeTableById[id];
        const Matrix4 & matDeviceToTracking = mDevicePoseArray[id];
        Matrix4 view;
        if (m3DOF) {
            // If the controller is 3DOF always put the model in the bottom of the view.
            Matrix4 mat = mDevicePoseArray[WVR_DEVICE_HMD];
            mat.invert();
            float angleY = atan2f(-mat[8], mat[10]);  // Yaw
            float angleX = asin(-mat[9]);             // Pitch
            float angleZ = atan2f(mat[1], mat[5]);    // Roll

            mat.identity().rotateY(-angleY / M_PI * 180.0f);
            mat.rotateX(angleX / M_PI * 180.0f);
            mat.rotateZ(angleZ / M_PI * 180.0f);
            mat *= matDeviceToTracking;
            int offset = mVRDevicePairs[id].type - WVR_DeviceType_Controller_Right;
            float x = (offset % 2) == 0 ? 0.1f : -0.1f;
            float z = -0.45 - (offset / 2) * 0.3f;
            mat.setColumn(3, Vector4(x,-0.12f,z,1));
            ControllerCube->getTransform().set(mat.get());
            ControllerCube->getNormalMatrix() = ControllerCube->makeNormalMatrix(matDeviceToTracking);
        } else {
            view = mHMDPose;
            ControllerCube->getTransform().set(matDeviceToTracking.get());
        }
        Vector4 light = mLightDir;
        if (!mLight)
            light = Vector4(0,0,0,1);

        if (mInteractionMode == WVR_InteractionMode_SystemDefault || mInteractionMode == WVR_InteractionMode_Controller){
            if (nEye == WVR_Eye_Left)
                ControllerCube->draw(mProjectionLeft, mEyePosLeft, view, light);
            else if (nEye == WVR_Eye_Right)
                ControllerCube->draw(mProjectionRight, mEyePosRight, view, light);
        }
    }
#endif
    // Reticle Pointer
    if(mReticlePointer){
        Vector4 light = mLightDir;
        if (!mLight)
        light = Vector4(0,0,0,1);
        Matrix4 view = mHMDPose;
        // Gaze mode, use reticle pointer as input module
        if (mInteractionMode == WVR_InteractionMode_Gaze){
            if (nEye == WVR_Eye_Left)
                mReticlePointer->draw(mProjectionLeft, mEyePosLeft, view, light);
            else if (nEye == WVR_Eye_Right)
                mReticlePointer->draw(mProjectionRight, mEyePosRight, view, light);
        }
    }

    // Sphere
    if (mSphere) {
        mSphere->setSphereColor(currColor);
        if (nEye == WVR_Eye_Left)
            mSphere->draw(mProjectionLeft, mEyePosLeft, mHMDPose, mLightDir);
        else if (nEye == WVR_Eye_Right)
            mSphere->draw(mProjectionRight, mEyePosRight, mHMDPose, mLightDir);
    }

    if (mFloor) {
        if (nEye == WVR_Eye_Left)
            mFloor->draw(mProjectionLeft, mEyePosLeft, mHMDPose, mLightDir);
        else if (nEye == WVR_Eye_Right)
            mFloor->draw(mProjectionRight, mEyePosRight, mHMDPose, mLightDir);
    }

    // SkyBox
    // minimize gpu loading by putting SkyBox in the end
    if (mSkyBox) {
            if (nEye == WVR_Eye_Left)
                mSkyBox->draw(mProjectionLeft, mEyePosLeft, mHMDPose, mLightDir);
            else if (nEye == WVR_Eye_Right)
                mSkyBox->draw(mProjectionRight, mEyePosRight, mHMDPose, mLightDir);
    }

    glUseProgram(0);
    GLenum glerr = glGetError();
    if (glerr != GL_NO_ERROR) {
        LOGW("glGetError(): %d", glerr);
    }
}

void MainApplication::updateTime() {
    // Process time variable.
    struct timeval now;
    gettimeofday(&now, NULL);

    mClockCount++;
    if (mRtcTime.tv_usec > now.tv_usec)
        mClockCount = 0;
    if (mClockCount >= VR_MAX_CLOCKS)
        mClockCount--;

    uint32_t timeDiff = timeval_subtract(now, mRtcTime);
    mTimeDiff = timeDiff / 1000000.0f;
    mTimeAccumulator2S += timeDiff;
    mRtcTime = now;
    mFrameCount++;
    if (mTimeAccumulator2S > 2000000) {
        mFPS = mFrameCount / (mTimeAccumulator2S / 1000000.0f);
        LOGI("HelloVR FPS %3.0f", mFPS);

        mFrameCount = 0;
        mTimeAccumulator2S = 0;
    }
}

void MainApplication::updateEyeToHeadMatrix(bool is6DoF) {
    if (is6DoF != mIs6DoFPose) {
        if(is6DoF) {
            mEyePosLeft = wvrmatrixConverter(
                WVR_GetTransformFromEyeToHead(WVR_Eye_Left, WVR_NumDoF_6DoF)).invert();
            mEyePosRight = wvrmatrixConverter(
                WVR_GetTransformFromEyeToHead(WVR_Eye_Right, WVR_NumDoF_6DoF)).invert();
        } else {
            mEyePosLeft = wvrmatrixConverter(
                WVR_GetTransformFromEyeToHead(WVR_Eye_Left, WVR_NumDoF_3DoF)).invert();
            mEyePosRight = wvrmatrixConverter(
                WVR_GetTransformFromEyeToHead(WVR_Eye_Right, WVR_NumDoF_3DoF)).invert();
        }

    }
    mIs6DoFPose = is6DoF;
}

void MainApplication::updateHMDMatrixPose() {
    LOGENTRY();

    WVR_GetSyncPose(WVR_PoseOriginModel_OriginOnHead, mVRDevicePairs, WVR_DEVICE_COUNT_LEVEL_1);
    mValidPoseCount = 0;
    mPoseClasses = "";
    for (int nDevice = 0; nDevice < WVR_DEVICE_COUNT_LEVEL_1; ++nDevice) {
        if (mVRDevicePairs[nDevice].pose.isValidPose) {
            mValidPoseCount++;
            mDevicePoseArray[nDevice] = wvrmatrixConverter(mVRDevicePairs[nDevice].pose.poseMatrix);

            if (mDevClassChar[nDevice]==0) {
                switch (WVR_DeviceType_HMD + nDevice) {
                case WVR_DeviceType_HMD:                       mDevClassChar[nDevice] = 'H'; break;
                case WVR_DeviceType_Controller_Right:          mDevClassChar[nDevice] = 'R'; break;
                case WVR_DeviceType_Controller_Left:           mDevClassChar[nDevice] = 'L'; break;
                default:                                       mDevClassChar[nDevice] = '?'; break;
                }
            }
            mPoseClasses += mDevClassChar[nDevice];
        }
    }

    if (mVRDevicePairs[WVR_DEVICE_HMD].pose.isValidPose) {
        updateEyeToHeadMatrix(mVRDevicePairs[WVR_DEVICE_HMD].pose.is6DoFPose);
        Matrix4 hmd = mDevicePoseArray[WVR_DEVICE_HMD];

        if (!mMove) {
            // The controller make the sample not simple.  If you don't have
            // a controller, we just need invert the hmd pose.

            // When the head turn left, acturally the object turn right.
            // When the head move left, acturally the object move right.
            // So we need invert the hmd matrix.
            mHMDPose = hmd.invert();
        } else {
            // In order to add translation and rotation to HMD. We need seperate
            // the translation and rotation into two matrix from HMD.

            Matrix4 hmdRotation = hmd;
            hmdRotation.setColumn(3, Vector4(0,0,0,1));

            Matrix4 hmdTranslation;
            hmdTranslation.setColumn(3, Vector4(hmd[12], hmd[13], hmd[14], 1));

            // Update world rotation.
            mWorldRotation += -mDriveAngle * mTimeDiff;
            Matrix4 mat4WorldRotation;
            mat4WorldRotation.rotate(mWorldRotation, 0, 1, 0);

            // Update WorldTranslation
            Vector4 direction = (mat4WorldRotation * hmdRotation) * Vector4(0, 0, 1, 0);  // Not apply the tranlsation of hmdpose
            direction *= -mDriveSpeed * mTimeDiff;
            direction.w = 1;

            // Move toward -z
            Matrix4 update;
            update.setColumn(3, direction);
            mWorldTranslation *= update;

            // Check world bound
            if (mWorldTranslation[12] >= mFarClip/2)
                mWorldTranslation[12] = mFarClip/2;
            if (mWorldTranslation[12] <= -mFarClip/2)
                mWorldTranslation[12] = -mFarClip/2;
            if (mWorldTranslation[13] >= mFarClip/2)
                mWorldTranslation[13] = mFarClip/2;
            if (mWorldTranslation[13] <= -mFarClip/2)
                mWorldTranslation[13] = -mFarClip/2;
            if (mWorldTranslation[14] >= mFarClip/2)
                mWorldTranslation[14] = mFarClip/2;
            if (mWorldTranslation[14] <= -mFarClip/2)
                mWorldTranslation[14] = -mFarClip/2;

            // DEFINE: The invert A^-1 is notated A'
            // The invert property: (AB)' = B'A'
            // "WT" means "world translation", "HR" means "hmd rotation", and so on.
            // We can get the model tranform matrix as (WT*HT*WR*HR)' = HR'*WR'*WT'*HT'
            // The tranlation matrix property: TA = AT , then: (TA)' = (AT)'
            // So we can put tranlsation matrix any where.
            // We apply WR' to vertex first, then do HR'.  If not, the world will be weired when look up or down.
            mHMDPose = (mWorldTranslation * hmdTranslation * mat4WorldRotation * hmdRotation).invert();
        }
    }
    if (gDebug) dumpMatrix("hmd", mHMDPose);
}

#if defined(USE_CONTROLLER) || defined(USE_CUSTOM_CONTROLLER)
void MainApplication::setupControllers()
{
    for (uint32_t cID = 0; cID < 2; ++cID) {
        if (mControllerObjs[cID] != nullptr) {
#if defined(USE_CONTROLLER)
            mControllerObjs[cID]->loadControllerModelAsync();
#else
            mControllerObjs[cID]->loadControllerEmitterAsync();
#endif
        } else {
            LOGW("[APCtrlerObj] CtrlerObj(%d) is nullptr", cID);
        }
    }
}
#else
//-----------------------------------------------------------------------------
// Purpose: Finds a controller cube we've already loaded or loads a new one
//-----------------------------------------------------------------------------
ControllerCube *MainApplication::findOrLoadControllerCube(WVR_DeviceType deviceType) {
    LOGDIF("LoadControllerCube %d", deviceType);

    ControllerCube *pControllerCube = NULL;
    for (std::vector<ControllerCube *>::iterator i = mControllerCubes.begin(); i != mControllerCubes.end(); i++) {
        if ((*i)->getDeviceType() == deviceType) {
            pControllerCube = *i;
            break;
        }
    }

    if (!pControllerCube) {
        pControllerCube = new ControllerCube(deviceType);
        if (m3DOF)
            pControllerCube->set3DOF(true);
        mControllerCubes.push_back(pControllerCube);
    }
    return pControllerCube;
}

//-----------------------------------------------------------------------------
// Purpose: Create/destroy GL a controller cube for a single device
//-----------------------------------------------------------------------------
void MainApplication::setupControllerCubeForDevice(WVR_DeviceType deviceType) {
    if (deviceType != WVR_DeviceType_Controller_Right && deviceType != WVR_DeviceType_Controller_Left) {
        LOGW("setupControllerCubeForDevice() Ignore unknown device type: %d", deviceType);
        return;
    }

    // try to find a model we've already set up
    ControllerCube *pControllerCube = findOrLoadControllerCube(deviceType);
    if (pControllerCube != NULL) {
        uint32_t index = deviceType - WVR_DeviceType_HMD;
        mControllerCubeTableById[index] = pControllerCube;
        mShowDeviceArray[index] = true;
    }
}

//-----------------------------------------------------------------------------
// Purpose: Create/destroy GL controller cubes
//-----------------------------------------------------------------------------
void MainApplication::setupControllerCubes() {
    for (uint32_t index = WVR_DEVICE_HMD + 1; index <= WVR_DEVICE_COUNT_LEVEL_1; index++) {
        WVR_DeviceType deviceType = (WVR_DeviceType)(index + WVR_DeviceType_HMD);
        if (!WVR_IsDeviceConnected(deviceType))
            continue;

        setupControllerCubeForDevice(deviceType);
    }
}
#endif
