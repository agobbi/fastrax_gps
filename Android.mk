ifeq ($(strip $(BOARD_USES_FASTRAX_GPS)),true)
  LOCAL_PATH := $(call my-dir)
  include $(CLEAR_VARS)

  LOCAL_SRC_FILES := fastrax_gps.c
  LOCAL_MODULE := gps.boundary
  LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
  LOCAL_SHARED_LIBRARIES := \
    libutils \
    libcutils \
    libdl \
    libc
  LOCAL_PRELINK_MODULE := false
  LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
  LOCAL_MODULE_TAGS := eng
  include $(BUILD_SHARED_LIBRARY)

endif
