# Copyright 2016 The Android Open Source Project

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_INCLUDES += $(LOCAL_PATH)

LOCAL_CFLAGS += \
    -DAPF_FRAME_HEADER_SIZE=14 \
    -std=c99 \
    -Werror

LOCAL_SRC_FILES += apf_interpreter.c

LOCAL_MODULE:= libapf

include $(BUILD_STATIC_LIBRARY)
