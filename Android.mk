LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= camtest.cpp

LOCAL_MODULE:= camtest
LOCAL_SHARED_LIBRARIES:= libutils libui liblog libbinder libcutils libhardware libnvmmodule libexpat libgui

LOCAL_C_INCLUDES := $(KERNEL_HEADERS)

LOCAL_CFLAGS := 

LOCAL_MODULE_TAGS := eng
include $(BUILD_EXECUTABLE)
