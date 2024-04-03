#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "ArduCopter V4.4.0"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_OFFICIAL

// ArduPilot version
#define FW_MAJOR 4
#define FW_MINOR 4
#define FW_PATCH 0
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL

// Align version 
#define MIDDLE_MAJOR 1
#define MIDDLE_MINOR 0
#define MIDDLE_PATCH 3
#define MIDDLE_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
