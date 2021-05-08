# Copyright (C) 2015 Sensics, Inc. and contributors
#
# Based on Android NDK samples, which are:
# Copyright (C) 2010 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
LOCAL_PATH := $(call my-dir)

VR_SDK_LIB := $(firstword  \
        $(realpath $(VR_SDK_ROOT)/jni/$(TARGET_ARCH_ABI))  \
        $(realpath $(VR_SDK_ROOT)/lib/$(TARGET_ARCH_ABI)) )

ifeq ($(OS),Windows_NT)
VR_SDK_LIB := $(VR_SDK_ROOT)/jni/$(TARGET_ARCH_ABI)
endif

include $(CLEAR_VARS)
LOCAL_MODULE := wvr_api
LOCAL_SRC_FILES := $(VR_SDK_LIB)/libwvr_api.so
include $(PREBUILT_SHARED_LIBRARY)

COMMON_INCLUDES := \
    $(LOCAL_PATH)/include \
    $(VR_SDK_ROOT)/include \
    $(LOCAL_PATH)/object \
    $(LOCAL_PATH)/scene \
    $(LOCAL_PATH)/shared \
    $(LOCAL_PATH)
    
COMMON_FILES := \
    hellovr.cpp \
    Context.cpp \
    shared/Matrices.cpp \
    object/Texture.cpp \
    object/VertexArrayObject.cpp \
    object/FrameBufferObject.cpp \
    object/Shader.cpp \
    object/Object.cpp \
    object/Mesh.cpp \
    scene/SkyBox.cpp \
    scene/ControllerAxes.cpp \
    scene/Picture.cpp \
    scene/ControllerCube.cpp \
    scene/Sphere.cpp \
    scene/Floor.cpp \
    scene/ReticlePointer.cpp \
    scene/Controller.cpp \
    scene/CustomController.cpp

#USE_CONTROLLER use device controller.
#USE_CUSTOM_CONTROLLER use device emitter.

include $(CLEAR_VARS)
LOCAL_MODULE    := hellovr_common
LOCAL_C_INCLUDES := $(COMMON_INCLUDES)
LOCAL_SRC_FILES := $(COMMON_FILES)
LOCAL_CFLAGS    := -DUSE_CONTROLLER -g
LOCAL_LDLIBS    := -llog -ljnigraphics -landroid -lEGL -lGLESv3
LOCAL_SHARED_LIBRARIES := wvr_api
include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE    := hellovr_jni
LOCAL_C_INCLUDES := $(COMMON_INCLUDES)
LOCAL_CFLAGS    := -DUSE_CONTROLLER -g
LOCAL_LDLIBS    := -llog -landroid
LOCAL_SRC_FILES := \
    jni.cpp
LOCAL_SHARED_LIBRARIES := hellovr_common wvr_api
include $(BUILD_SHARED_LIBRARY)
