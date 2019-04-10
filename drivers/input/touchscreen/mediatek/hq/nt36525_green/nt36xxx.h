/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 *
 * Revision: 32206 $
 * Date: 2018-08-10 19:23:04 +0800
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef 	_LINUX_NVT_TOUCH_H
#define		_LINUX_NVT_TOUCH_H

#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <soc/oppo/device_info.h>


#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "nt36xxx_mem_map.h"

#ifdef CONFIG_MTK_SPI
/* Please copy mt_spi.h file under mtk spi driver folder */
#include "mt_spi.h"
#endif

#ifdef CONFIG_SPI_MT65XX
#include <linux/platform_data/spi-mt65xx.h>
#endif

#define NVT_DEBUG 1

//---GPIO number---
#define NVTTOUCH_RST_PIN 980
#define NVTTOUCH_INT_PIN 943

#define LCD_GPIO_1 490
#define LCD_GPIO_2 491
#define LCD_GPIO_3 492

/* chenyunrui@RM.BSP.TP.FUNCTION, 2018/11/8, Add start */
#define MAX_FW_NAME_LENGTH 60
#define MAX_DEVICE_VERSION_LENGTH 16
#define MAX_DEVICE_MANU_LENGTH 16
/* chenyunrui@RM.BSP.TP.FUNCTION, 2018/11/8, Add end */

//---INT trigger mode---
//#define IRQ_TYPE_EDGE_RISING 1
//#define IRQ_TYPE_EDGE_FALLING 2
#define INT_TRIGGER_TYPE IRQ_TYPE_EDGE_RISING


//---SPI driver info.---
#define NVT_SPI_NAME "NVT-ts"

#if NVT_DEBUG
#define NVT_LOG(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)
#else
#define NVT_LOG(fmt, args...)    pr_info("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)
#endif
#define NVT_ERR(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)

//---Input device info.---
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"


//---Touch info.---
#define TOUCH_DEFAULT_MAX_WIDTH 720
#define TOUCH_DEFAULT_MAX_HEIGHT 1520
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_KEY_NUM 0
#if TOUCH_KEY_NUM > 0
extern const uint16_t touch_key_array[TOUCH_KEY_NUM];
#endif
#define TOUCH_FORCE_NUM 1000

/* Enable only when module have tp reset pin and connected to host */
#define NVT_TOUCH_SUPPORT_HW_RST 1

//---Customerized func.---
#define NVT_TOUCH_PROC 1
#define NVT_TOUCH_EXT_PROC 1
#define NVT_TOUCH_MP 1
#define MT_PROTOCOL_B 1
#define WAKEUP_GESTURE 1
#if WAKEUP_GESTURE
//extern const uint16_t gesture_key_array[];
#endif
struct coordinate {
	uint16_t x;
	uint16_t y;
};
struct gesture_black {
	int flag;
	uint8_t gesture_backup;
	char *message;
};

struct gesture_info {
	uint8_t gesture_type;
	uint8_t clockwise;
	struct coordinate Point_start;
	struct coordinate Point_end;
	struct coordinate Point_1st;
	struct coordinate Point_2nd;
	struct coordinate Point_3rd;
	struct coordinate Point_4th;
};

struct oppo_debug_info {
	struct coordinate coordinate[10];
};

#define BOOT_UPDATE_FIRMWARE 1
#define BOOT_UPDATE_FIRMWARE_NAME "tp/18601/novatek_ts_fw.bin"
#define MP_UPDATE_FIRMWARE_NAME   "tp/18601/novatek_ts_mp.bin"
#define BOOT_UPDATE_FIRMWARE_NAME_SIGNED "tp/18601/novatek_ts_fw_signed.bin"

//---ESD Protect.---
#define NVT_TOUCH_ESD_PROTECT 1
#define NVT_TOUCH_ESD_CHECK_PERIOD 1500	/* ms */
#define NVT_TOUCH_WDT_RECOVERY 1

struct nvt_ts_data {
	struct spi_device *client;
	struct input_dev *input_dev;
	struct delayed_work nvt_fwu_work;
	struct work_struct nvt_resume_work;
	struct manufacture_info nvt_ts_info;
	bool nvt_fw_updating;
	uint16_t addr;
	int boot_mode;
	int8_t phys[32];
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	uint8_t fw_ver;
	uint8_t x_num;
	uint8_t y_num;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint32_t int_trigger_type;
	int32_t irq_gpio;
	uint32_t irq_flags;
	int32_t reset_gpio;
	uint32_t reset_flags;
	struct mutex lock;
	struct mutex bus_lock;
	const struct nvt_ts_mem_map *mmap;
	uint8_t carrier_system;
	uint8_t hw_crc;
	uint16_t nvt_pid;
	uint8_t rbuf[1025];
	bool irq_enabled;
	spinlock_t irq_lock;
	uint8_t debug_level;
	struct gesture_info gesture;
	uint8_t gesture_enable;
	uint8_t gesture_flag_back;
	uint8_t bTouchIsAwake;
	uint8_t bTouchSuspendReason;
	struct oppo_debug_info oppo_debug_info;
	struct gesture_black gesture_test;
	int g_gesture_bak;
	uint8_t sleep_flag;
#ifdef CONFIG_MTK_SPI
	struct mt_chip_conf spi_ctrl;
#endif
#ifdef CONFIG_SPI_MT65XX
    struct mtk_chip_config spi_ctrl;
#endif
	
};

#if NVT_TOUCH_PROC
struct nvt_flash_data{
	rwlock_t lock;
};
#endif

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,		// ReK baseline
	RESET_STATE_REK_FINISH,	// baseline is ready
	RESET_STATE_NORMAL_RUN,	// normal run
	RESET_STATE_MAX  = 0xAF
} RST_COMPLETE_STATE;

typedef enum {
    EVENT_MAP_HOST_CMD                      = 0x50,
    EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
    EVENT_MAP_RESET_COMPLETE                = 0x60,
    EVENT_MAP_FWINFO                        = 0x78,
    EVENT_MAP_PROJECTID                     = 0x9A,
} SPI_EVENT_MAP;

typedef enum {
	MODE_EDGE = 0,
	MODE_CHARGE,
	MODE_GAME,
	MODE_HEADSET
} NVT_CUSTOMIZED_MODE;

typedef enum {
    EDGE_REJECT_L = 0,
    EDGE_REJECT_H,
    PWR_FLAG,
	JITTER_FLAG = 6,
	HEADSET_FLAG = 7,
} CMD_OFFSET;

typedef enum {
	NVT_TP_SLEEP = 0,
    NVT_TP_WKG,
	NVT_TP_ACTIVE
} NVT_TP_STATUS;

//---customized command---
#define HOST_CMD_PWR_PLUG_IN          (0x53)
#define HOST_CMD_PWR_PLUG_OUT         (0x51)
#define HOST_CMD_HEADSET_PLUG_IN      (0x77)
#define HOST_CMD_HEADSET_PLUG_OUT     (0x78)
#define HOST_CMD_EDGE_LIMIT_VERTICAL  (0x7A)
#define HOST_CMD_EDGE_LIMIT_LEFT_UP   (0x7B)
#define HOST_CMD_EDGE_LIMIT_RIGHT_UP  (0x7C)
#define HOST_CMD_JITTER_ON            (0x7D)
#define HOST_CMD_JITTER_OFF           (0x7E)

//---SPI READ/WRITE---
#define SPI_WRITE_MASK(a)	(a | 0x80)
#define SPI_READ_MASK(a)	(a & 0x7F)

#define DUMMY_BYTES (1)
#define NVT_TANSFER_LEN		(64*1024)

typedef enum {
	NVTWRITE = 0,
	NVTREAD  = 1
} NVT_SPI_RW;

//---extern structures---
extern struct nvt_ts_data *ts;

//---extern functions---
int32_t CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len);
int32_t CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len);
void nvt_bootloader_reset(void);
void nvt_eng_reset(void);
void nvt_sw_reset(void);
void nvt_sw_reset_idle(void);
void nvt_boot_ready(void);
void nvt_bld_crc_enable(void);
void nvt_fw_crc_enable(void);
void nvt_irq_enable(bool enable);
void nvt_update_firmware(char *firmware_name);
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state);
int32_t nvt_get_fw_info(void);
int32_t nvt_clear_fw_status(void);
int32_t nvt_check_fw_status(void);
#if NVT_TOUCH_ESD_PROTECT
extern void nvt_esd_check_enable(uint8_t enable);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

int32_t nvt_set_page(uint32_t addr);
int32_t nvt_write_addr(uint32_t addr, uint8_t data);
int32_t nvt_mode_switch(NVT_CUSTOMIZED_MODE mode, uint8_t flag);
void nvt_mode_change_cmd(uint8_t cmd);

void nvt_read_mdata(uint32_t xdata_addr, uint32_t xdata_btn_addr);
void nvt_read_mdata_rss(uint32_t xdata_i_addr, uint32_t xdata_q_addr,
		uint32_t xdata_btn_i_addr, uint32_t xdata_btn_q_addr);
void resume_Update_Firmware(struct work_struct *work);
void force_update_firmware_release(void);

#endif /* _LINUX_NVT_TOUCH_H */
