#define LOG_TAG "APCustomCtrler"

#include <functional>

#include <log.h>

#include "../Context.h"
#include "CustomController.h"

CustomController::CustomController(WVR_DeviceType iCtrlerType)
: mInitialized(false)
, mCtrlerType(iCtrlerType)
, mMatrixLocations{-1, -1}
, mColorLocations{-1, -1}
{
    initializeGLComp();
}

CustomController::~CustomController()
{
    LOGI("(%d) dtor!!", mCtrlerType);
    mInitialized = false;
    releaseGLComp();
}

void CustomController::loadControllerEmitterAsync()
{
    std::function<void()> getEmitterFunc = [this](){
        //1. Clear status and cached data(if it exist).
        mLoadingThreadMutex.lock();
        mInitialized = false;
        float ep[16] = {0.0f};
        WVR_Result result = WVR_GetCurrentControllerEmitter(mCtrlerType, ep);
        if (result == WVR_Success) {
            mEmitterPose.set(ep);
        } else {
            mEmitterPose = Matrix4();
        }
        mLoadingThreadMutex.unlock();
    };

    LOGI("(%d): Trigger Getting Emitter Thread", mCtrlerType);
    if (mGetEmitterFuncThread.joinable() == true) {
        mGetEmitterFuncThread.detach();
    }
    mGetEmitterFuncThread = std::thread(getEmitterFunc);
}

void CustomController::initializeGLComp()
{
    const char *shaderNames[2] = {
        "CustomCtrlerShader",
        "CustomCtrlerMultiShader"
    };
    const char *vpaths[2] ={
        "shader/vertex/line_vertex.glsl",
        "shader/vertex/line_multview_vertex.glsl"
    };
    const char *fpaths[2] = {
        "shader/fragment/line_fragment.glsl",
        "shader/fragment/line_fragment.glsl"
    };
    //Initialize shader.
    for (uint32_t mode = CtrlerDrawMode_General; mode < CtrlerDrawMode_MaxModeMumber; ++mode) {
        mShaders[mode] = Shader::findShader(vpaths[mode], fpaths[mode]);
        if (mShaders[mode] != nullptr) {
            LOGI("(%d): Shader find!!!", mCtrlerType);
        } else {
            Context * context = Context::getInstance();
            EnvWrapper ew = context->getEnv();
            JNIEnv * env = ew.get();

            AssetFile vfile(context->getAssetManager(), vpaths[mode]);
            AssetFile ffile(context->getAssetManager(), fpaths[mode]);
            if (!vfile.open() || !ffile.open()) {
                LOGE("(%d): Unable to read shader files!!!", mCtrlerType);
                return;
            }

            char *vstr = vfile.toString();
            char *fstr = ffile.toString();

            //LogD(mName, "%s\n%s\n", vpath, vstr);
            //LogD(mName, "%s\n%s\n", fpath, fstr);

            mShaders[mode] = std::make_shared<Shader>(shaderNames[mode], vpaths[mode], vstr, fpaths[mode], fstr);
            bool ret = mShaders[mode]->compile();

            delete [] vstr;
            delete [] fstr;
            if (ret == false) {
                LOGE("(%d): Compile shader error!!!", mCtrlerType);
            } else {
                Shader::putShader(mShaders[mode]);
            }
        }
        //
        mMatrixLocations[mode] = mShaders[mode]->getUniformLocation("matrix");
        mColorLocations[mode] = mShaders[mode]->getUniformLocation("color");
        LOGI("(%d): Mode[%d]:matrix(%d) color(%d)", mCtrlerType, 
            mode, mMatrixLocations[mode], mColorLocations[mode]);
    }
    //Ray
    float s = 0.00125f;
    float dis = 1.0f;
    float rayVertices[15] = {
           s,    s, -0.003f,
          -s,    s, -0.003f,
          -s,   -s, -0.003f,
           s,   -s, -0.003f,
        0.0f, 0.0f, -dis
    };

    uint32_t rayIndices[18] = {
        0, 1, 2,
        0, 2, 3,
        0, 4, 1,
        0, 3, 4,
        2, 4, 3,
        1, 4, 2
    };

    mRayMesh.createVertexBufferData(VertexAttrib_Vertices, rayVertices, 15, 3);
    mRayMesh.createIndexBufferData(rayIndices, 18, 3);
    mRayMesh.createVAO();
    //Custom Model
    float cms = 0.025f;
    float modelVertices[15] = {
         cms,  cms,  0.10f,
        -cms,  cms,  0.10f,
        -cms, -cms,  0.10f,
         cms, -cms,  0.10f,
        0.0f, 0.0f, -0.05f
    };

    uint32_t modelIndices[18] = {
        0, 1, 2,
        0, 2, 3,
        0, 4, 1,
        0, 3, 4,
        2, 4, 3,
        1, 4, 2
    };
    
    mCustomMesh.createVertexBufferData(VertexAttrib_Vertices, modelVertices, 15, 3);
    mCustomMesh.createIndexBufferData(modelIndices, 18, 3);
    mCustomMesh.createVAO();
}

void CustomController::releaseGLComp()
{
    mCustomMesh.releaseGLComp();
    mRayMesh.releaseGLComp();
}

void CustomController::render(CtrlerDrawModeEnum iMode, const Matrix4 iProjs[CtrlerDrawMode_MaxModeMumber], const Matrix4 iEyes[CtrlerDrawMode_MaxModeMumber], const Matrix4 &iView, const Matrix4 &iCtrlerPose)
{
    if (mInitialized == false && WVR_IsDeviceConnected(mCtrlerType) == false) {
        return;
    }

    //1. cache depth and alpha setting.
    GLboolean oldDepth, oldAlpha;
    GLint oldDepthFunc;
    GLboolean lastPolygonOffsetFill;
    GLfloat lastFactor, lastUnits;
    oldDepth = glIsEnabled(GL_DEPTH_TEST);
    glGetIntegerv(GL_DEPTH_FUNC, &oldDepthFunc);
    oldAlpha = glIsEnabled(GL_BLEND);
    lastPolygonOffsetFill = glIsEnabled(GL_POLYGON_OFFSET_FILL);
    glGetFloatv(GL_POLYGON_OFFSET_FACTOR, &lastFactor);
    glGetFloatv(GL_POLYGON_OFFSET_UNITS, &lastUnits);
    //2. draw
    Matrix4 mvps[CtrlerDrawMode_MaxModeMumber];
    if (iMode == CtrlerDrawMode_General) {
        mvps[0] = iProjs[0] * iEyes[0] * iView * iCtrlerPose * mEmitterPose;
    } else {
        mvps[0] = iProjs[0] * iEyes[0] * iView * iCtrlerPose * mEmitterPose;
        mvps[1] = iProjs[1] * iEyes[1] * iView * iCtrlerPose * mEmitterPose;
    }

    drawCtrler(iMode, mvps);
    //3. status recovering.
    if (lastPolygonOffsetFill == GL_TRUE) {
        glEnable(GL_POLYGON_OFFSET_FILL);
    } else {
        glDisable(GL_POLYGON_OFFSET_FILL);
    }
    glPolygonOffset(lastFactor, lastUnits);

    if (oldDepth == GL_TRUE) {
        glEnable(GL_DEPTH_TEST);
    } else {
        glDisable(GL_DEPTH_TEST);
    }
    glDepthFunc(oldDepthFunc);

    if (oldAlpha == GL_TRUE) {
        glEnable(GL_BLEND);
    } else {
        glDisable(GL_BLEND);
    }
}

void CustomController::drawCtrler(CtrlerDrawModeEnum iMode, const Matrix4 iMVPs[CtrlerDrawMode_MaxModeMumber])
{
    glEnable(GL_DEPTH_TEST);
    Matrix4 finalMats[2];
    uint32_t matNumber = 1;
    std::vector<GLfloat> glMats;
    if (iMode == CtrlerDrawMode_General) {
        finalMats[0] = iMVPs[0];
        matNumber = 1;
        glMats.resize(16);
        memcpy(glMats.data(), finalMats[0].get(), 16 * sizeof(GLfloat));
    } else {
        finalMats[0] = iMVPs[0];
        finalMats[1] = iMVPs[1];
        matNumber = 2;
        glMats.resize(32);
        memcpy(glMats.data()     , finalMats[0].get(), 16 * sizeof(GLfloat));
        memcpy(glMats.data() + 16, finalMats[1].get(), 16 * sizeof(GLfloat));
    }
    
    mTargetShader = mShaders[iMode].get();

    if (mTargetShader != nullptr) {      
        mTargetShader->useProgram();
        glUniformMatrix4fv(mMatrixLocations[iMode], matNumber, false, glMats.data());
        glUniform4f(mColorLocations[iMode], 0.5f, 0.5f, 1.0f, 1.0f);
        //
        mCustomMesh.draw();
        mRayMesh.draw();
        mTargetShader->unuseProgram();
    }

    glDisable(GL_DEPTH_TEST);
}

bool CustomController::isThisCtrlerType(WVR_DeviceType iCtrlerType) const
{
    return (iCtrlerType == mCtrlerType);
}

WVR_DeviceType CustomController::getCtrlerType() const
{
    return mCtrlerType;
}


void CustomController::switchCtrlerType()
{
    if (mCtrlerType == WVR_DeviceType_Controller_Left) {
        mCtrlerType = WVR_DeviceType_Controller_Right;
    } else {
        mCtrlerType = WVR_DeviceType_Controller_Left;
    }
}