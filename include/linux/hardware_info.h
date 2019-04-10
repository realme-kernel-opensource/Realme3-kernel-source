/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __HARDWARE_INFO_H__
#define __HARDWARE_INFO_H__

#ifdef ODM_HQ_EDIT /* 2018/10/13,add hq hardware info */
#define EMMC_CMP_SIZE  18

typedef struct
{
        char  *cid;
        char  *name;
} EMMC_TABLE;

typedef struct
{
        unsigned int  adc_vol;
        char  *revision;
} BOARD_VERSION_TABLE;

enum hardware_id {
        HWID_NONE = 0x00,
        HWID_DDR = 0x10,
        HWID_EMMC,
        HWID_NAND,
        HWID_FLASH,

        HWID_LCM = 0x20,
        HWID_LCD_BIAS,
        HWID_BACKLIGHT,
        HWID_CTP_DRIVER,
        HWID_CTP_MODULE,
        HWID_CTP_FW_VER,
        HWID_CTP_FW_INFO,

        HWID_MAIN_CAM = 0x30,
        HWID_MAIN_CAM_2,
        HWID_SUB_CAM,
        HWID_SUB_CAM_2,
        HWID_MAIN_LENS,
        HWID_MAIN_LENS_2,
        HWID_SUB_LENS,
        HWID_SUB_LENS_2,
        HWID_MAIN_OTP,
        HWID_MAIN_OTP_2,
        HWID_SUB_OTP,
        HWID_SUB_OTP_2,
        HWID_FLASHLIGHT,
        HWID_FLaSHLIGHT_2,

        HWID_GSENSOR = 0x70,
        HWID_ALSPS,
        HWID_GYROSCOPE,
        HWID_MSENSOR,
        HWID_FINGERPRINT,
        HWID_SAR_SENSOR_1,
        HWID_SAR_SENSOR_2,
        HWID_IRDA,
        HWID_BAROMETER,
        HWID_PEDOMETER,
        HWID_HUMIDITY,
        HWID_NFC,
        HWID_TEE,

        HWID_BATERY_ID = 0xA0,
        HWID_CHARGER,

        HWID_USB_TYPE_C = 0xE0,

        HWID_SUMMARY = 0xF0,
        HWID_VER,
        HWID_END
};

//Add for camera otp information
struct global_otp_struct {
        char *sensor_name;
        int otp_valid;
        int vendor_id;
        int module_code;
        int module_ver;
        int sw_ver;
        int year;
        int month;
        int day;
        int vcm_vendorid;
        int vcm_moduleid;
};

typedef struct {
        const char *version;
        const char *lcm;
        const char *ctp_driver;
        const char *ctp_module;
        unsigned char ctp_fw_version[20];
        const char *ctp_fw_info;
        const char *main_camera;
        const char *sub_camera;
        const char *aux_camera;
        const char *alsps;
        const char *gsensor;
        const char *gyroscope;
        const char *msensor;
        const char *fingerprint;
        const char *sar_sensor_1;
        const char *sar_sensor_2;
        const char *bat_id;
        const unsigned int *flash;
        const char *nfc;
        //const struct hwinfo_cam_otp *main_otp;
        //const struct hwinfo_cam_otp *sub_otp;
} HARDWARE_INFO;


void get_hardware_info_data(enum hardware_id id, const void *data);
#endif /* ODM_HQ_EDIT */
#endif /* __HARDWARE_INFO_H__ */
