/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** FILE: - config.c
** Description : This program is for ili9881 driver confic.c
** Version: 1.0
** Date : 2018/5/17
** Author: Zhonghua.Hu@ODM_WT.BSP.TP-FP.TP
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/

#include "../common.h"
#include "../platform.h"
#include "config.h"
#include "protocol.h"
#include "i2c.h"
#include "flash.h"
#include "gesture.h"
#include "finger_report.h"
#include "mp_test.h"
#include "firmware.h"


#define HARDWARE_MAX_ITEM_LONGTH		256

//extern char Ctp_name[HARDWARE_MAX_ITEM_LONGTH];
char Ctp_name[HARDWARE_MAX_ITEM_LONGTH];

extern int ili_ctpmodule;

extern struct ili_gesture_info * gesture_report_data;
#ifdef ILITEK_ESD_CHECK
extern void ilitek_cancel_esd_check(void);
#endif

char ili_version[20] = {"0"};


bool gesture_done = false;
extern bool psensor_close;

//extern void devinfo_info_tp_set(char *version, char *manufacture, char *fw_path);
void devinfo_info_tp_set(char *version, char *manufacture, char *fw_path){
	ipio_info("\n");
}


//extern int  ili_power_on(struct ilitek_platform_data *ts, bool on);
extern struct ilitek_platform_data *ipd;

extern int ili_ctpmodule;

/* the list of support chip */
uint32_t ipio_chip_list[] = {
	CHIP_TYPE_ILI7807,
	CHIP_TYPE_ILI9881,
};

uint8_t g_read_buf[128] = { 0 };

struct core_config_data *core_config = NULL;
static void read_flash_info(uint8_t cmd, int len)
{
	int i;
	uint16_t flash_id = 0, flash_mid = 0;
	uint8_t buf[4] = { 0 };

	/*
	 * This command is used to fix the bug of spi clk for 7807F-AB
	 * when operating with its flash.
	 */
	if (core_config->chip_id == CHIP_TYPE_ILI7807 && core_config->chip_type == ILI7807_TYPE_F_AB) {
		core_config_ice_mode_write(0x4100C, 0x01, 1);
		msleep(25);
	}

	core_config_ice_mode_write(0x41000, 0x0, 1);	/* CS high */
	core_config_ice_mode_write(0x41004, 0x66aa55, 3);	/* Key */

	core_config_ice_mode_write(0x41008, cmd, 1);
	for (i = 0; i < len; i++) {
		core_config_ice_mode_write(0x041008, 0xFF, 1);
		buf[i] = core_config_ice_mode_read(0x41010);
	}

	core_config_ice_mode_write(0x041000, 0x1, 1);	/* CS high */

	/* look up flash info and init its struct after obtained flash id. */
	flash_mid = buf[0];
	flash_id = buf[1] << 8 | buf[2];
	core_flash_init(flash_mid, flash_id);
}

/*
 * It checks chip id shifting sepcific bits based on chip's requirement.
 *
 * @pid_data: 4 bytes, reading from firmware.
 *
 */
static uint32_t check_chip_id(uint32_t pid_data)
{
	int i;
	uint32_t id = 0;
	uint32_t type = 0;

	id = pid_data >> 16;
	type = (pid_data & 0x0000FF00) >> 8;  //mfeng modify

	ipio_info("id = 0x%x, type = 0x%x\n", id, type);

	if(id == CHIP_TYPE_ILI9881) {
		for(i = ILI9881_TYPE_F; i <= ILI9881_TYPE_H; i++) {
			if (i == type) {
				core_config->chip_type = i;
				core_config->ic_reset_addr = 0x040050;
				return id;
			}
		}
	}

	if(id == CHIP_TYPE_ILI7807) {
		for(i = ILI7807_TYPE_F_AA; i <= ILI7807_TYPE_H; i++) {
			if (i == type) {
				core_config->chip_type = i;
				if (i == ILI7807_TYPE_F_AB)
					core_config->ic_reset_addr = 0x04004C;
				else if (i == ILI7807_TYPE_H)
					core_config->ic_reset_addr = 0x040050;

				return id;
			}
		}
	}

	return 0;
}

/*
 * Read & Write one byte in ICE Mode.
 */
uint32_t core_config_read_write_onebyte(uint32_t addr)
{
	int res = 0;
	uint32_t data = 0;
	uint8_t szOutBuf[64] = { 0 };

	szOutBuf[0] = 0x25;
	szOutBuf[1] = (char)((addr & 0x000000FF) >> 0);
	szOutBuf[2] = (char)((addr & 0x0000FF00) >> 8);
	szOutBuf[3] = (char)((addr & 0x00FF0000) >> 16);

	res = core_write(core_config->slave_i2c_addr, szOutBuf, 4);
	if (res < 0)
		goto out;

	mdelay(1);

	res = core_read(core_config->slave_i2c_addr, szOutBuf, 1);
	if (res < 0)
		goto out;

	data = (szOutBuf[0]);

	return data;

out:
	ipio_err("Failed to read/write data in ICE mode, res = %d\n", res);
	return res;
}
EXPORT_SYMBOL(core_config_read_write_onebyte);

uint32_t core_config_ice_mode_read(uint32_t addr)
{
	int res = 0;
	uint8_t szOutBuf[64] = { 0 };
	uint32_t data = 0;

	szOutBuf[0] = 0x25;
	szOutBuf[1] = (char)((addr & 0x000000FF) >> 0);
	szOutBuf[2] = (char)((addr & 0x0000FF00) >> 8);
	szOutBuf[3] = (char)((addr & 0x00FF0000) >> 16);

	res = core_write(core_config->slave_i2c_addr, szOutBuf, 4);
	if (res < 0)
		goto out;

	mdelay(1);

	res = core_read(core_config->slave_i2c_addr, szOutBuf, 4);
	if (res < 0)
		goto out;

	data = (szOutBuf[0] + szOutBuf[1] * 256 + szOutBuf[2] * 256 * 256 + szOutBuf[3] * 256 * 256 * 256);

	return data;

out:
	ipio_err("Failed to read data in ICE mode, res = %d\n", res);
	return res;
}
EXPORT_SYMBOL(core_config_ice_mode_read);


/*
 * Write commands into firmware in ICE Mode.
 *
 */
int core_config_ice_mode_write(uint32_t addr, uint32_t data, uint32_t size)
{
	int res = 0, i;
	uint8_t szOutBuf[64] = { 0 };

	szOutBuf[0] = 0x25;
	szOutBuf[1] = (char)((addr & 0x000000FF) >> 0);
	szOutBuf[2] = (char)((addr & 0x0000FF00) >> 8);
	szOutBuf[3] = (char)((addr & 0x00FF0000) >> 16);

	for (i = 0; i < size; i++) {
		szOutBuf[i + 4] = (char)(data >> (8 * i));
	}

	res = core_write(core_config->slave_i2c_addr, szOutBuf, size + 4);

	if (res < 0)
		ipio_err("Failed to write data in ICE mode, res = %d\n", res);

	return res;
}
EXPORT_SYMBOL(core_config_ice_mode_write);

/*
 * Doing soft reset on ic.
 *
 * It resets ic's status, moves code and leave ice mode automatically if in
 * that mode.
 */
void core_config_ic_reset(void)
{
#ifdef HOST_DOWNLOAD
	core_config_ice_mode_disable();
#else
	uint32_t key = 0;
	if (core_config->chip_id == CHIP_TYPE_ILI7807) {
		if (core_config->chip_type == ILI7807_TYPE_H)
			key = 0x00117807;
		else
			key = 0x00017807;
	}
	if (core_config->chip_id == CHIP_TYPE_ILI9881) {
		key = 0x00019881;
	}

	ipio_debug(DEBUG_CONFIG, "key = 0x%x\n", key);
	if (key != 0) {
		core_config->do_ic_reset = true;
		core_config_ice_mode_write(core_config->ic_reset_addr, key, 4);
		core_config->do_ic_reset = false;
	}

	msleep(300);
#endif
}
EXPORT_SYMBOL(core_config_ic_reset);

void core_config_sense_ctrl(bool start)
{
	ipio_debug(DEBUG_CONFIG, "sense start = %d\n", start);

	return core_protocol_func_control(0, start);
}
EXPORT_SYMBOL(core_config_sense_ctrl);

void core_config_sleep_ctrl(bool out)
{
	ipio_debug(DEBUG_CONFIG, "Sleep Out = %d\n", out);

	return core_protocol_func_control(1, out);
}
EXPORT_SYMBOL(core_config_sleep_ctrl);

void core_config_glove_ctrl(bool enable, bool seamless)
{
	int cmd = 0x2;		/* default as semaless */

	if (!seamless) {
		if (enable)
			cmd = 0x1;
		else
			cmd = 0x0;
	}

	ipio_debug(DEBUG_CONFIG, "Glove = %d, seamless = %d, cmd = %d\n", enable, seamless, cmd);

	return core_protocol_func_control(2, cmd);
}
EXPORT_SYMBOL(core_config_glove_ctrl);

void core_config_stylus_ctrl(bool enable, bool seamless)
{
	int cmd = 0x2;		/* default as semaless */

	if (!seamless) {
		if (enable)
			cmd = 0x1;
		else
			cmd = 0x0;
	}

	ipio_debug(DEBUG_CONFIG, "stylus = %d, seamless = %d, cmd = %x\n", enable, seamless, cmd);

	return core_protocol_func_control(3, cmd);
}
EXPORT_SYMBOL(core_config_stylus_ctrl);

void core_config_tp_scan_mode(bool mode)
{
	ipio_debug(DEBUG_CONFIG, "TP Scan mode = %d\n", mode);

	return core_protocol_func_control(4, mode);
}
EXPORT_SYMBOL(core_config_tp_scan_mode);

uint32_t core_config_get_reg_data(uint32_t addr)
{
	int res = 0;
	uint32_t reg_data = 0;
	bool ice_enable = core_config->icemodeenable;
	if (!ice_enable) {
		res = core_config_ice_mode_enable();
		if (res < 0) {
			ipio_info("Failed to enter ICE mode, res = %d\n", res);
			goto out;
		}
		mdelay(1);
	}
	reg_data = core_config_ice_mode_read(addr);
	ipio_info("ice_enable = %d addr = 0x%X reg_data = 0x%X\n", ice_enable, addr, reg_data);


out:
	if (!ice_enable) {
		core_config_ice_mode_disable();
	}
	return reg_data;
}

void core_config_wr_pack( int packet )
{
	int retry = 100;
	uint32_t reg_data = 0;
	while(retry--) {
		reg_data = core_config_read_write_onebyte(0x73010);
		if ((reg_data & 0x02) == 0) {
			ipio_info("check ok 0x73010 read 0x%X retry = %d\n", reg_data, retry);
			break;
		}
		mdelay(10);
	}
	if (retry <= 0) {
		ipio_info("check 0x73010 error read 0x%X\n", reg_data);
	}
	core_config_ice_mode_write(0x73000, packet, 4);
}

uint32_t core_config_rd_pack( int packet)
{
	int retry = 100;
	uint32_t reg_data = 0;
	while(retry--) {
		reg_data = core_config_read_write_onebyte(0x73010);
		if ((reg_data & 0x02) == 0) {
			ipio_info("check  ok 0x73010 read 0x%X retry = %d\n", reg_data, retry);
			break;
		}
		mdelay(10);
	}
	if (retry <= 0) {
		ipio_info("check 0x73010 error read 0x%X\n", reg_data);
	}
	core_config_ice_mode_write(0x73000, packet, 4);

	retry = 100;
	while(retry--) {
		reg_data = core_config_read_write_onebyte(0x4800A);
		if ((reg_data & 0x02) == 0x02) {
			ipio_info("check  ok 0x4800A read 0x%X retry = %d\n", reg_data, retry);
			break;
		}
		mdelay(10);
	}
	if (retry <= 0) {
		ipio_info("check 0x4800A error read 0x%X\n", reg_data);
	}
	core_config_ice_mode_write(0x4800A, 0x02, 1);
	reg_data = core_config_ice_mode_read(0x73016);
	return reg_data;
}

void core_get_ddi_register_onlyone(uint8_t page, uint8_t reg)
{
	uint32_t reg_data = 0;
	uint32_t setpage = 0x1FFFFF00 | page;
	uint32_t setreg = 0x2F000100 | (reg << 16);
	ipio_info("setpage =  0x%X setreg = 0x%X\n", setpage, setreg);
	////TDI_RD_KEY
	core_config_wr_pack(0x1FFF9487);
	//( *( __IO uint8 *)	(0x4800A) ) =0x2
	core_config_ice_mode_write(0x4800A, 0x02, 1);

	//// Read Page reg
	core_config_wr_pack(setpage);
	reg_data = core_config_rd_pack(setreg);
	ipio_info("check page = 0x%X reg = 0x%X read 0x%X\n", page, reg, reg_data);

	////TDI_RD_KEY OFF
	core_config_wr_pack(0x1FFF9400);
}

void core_set_ddi_register_onlyone(uint8_t page, uint8_t reg, uint8_t data)
{
	uint32_t setpage = 0x1FFFFF00 | page;
	uint32_t setreg = 0x1F000100 | (reg << 16) | data;
	ipio_info("setpage =  0x%X setreg = 0x%X\n", setpage, setreg);
	////TDI_WR_KEY
	core_config_wr_pack(0x1FFF9527);
	//// Switch to Page
	core_config_wr_pack(setpage);
	//// Page ,
	core_config_wr_pack(setreg);
	////TDI_WR_KEY OFF
	core_config_wr_pack(0x1FFF9500);
}

void core_config_lpwg_ctrl(bool enable)
{
	ipio_debug(DEBUG_CONFIG, "LPWG = %d\n", enable);

	return core_protocol_func_control(5, enable);
}
EXPORT_SYMBOL(core_config_lpwg_ctrl);

void core_config_gesture_ctrl(uint8_t func)
{
	uint8_t max_byte = 0x0, min_byte = 0x0;

	ipio_debug(DEBUG_CONFIG, "Gesture function = 0x%x\n", func);

	max_byte = 0x3F;
	min_byte = 0x20;

	if (func > max_byte || func < min_byte) {
		ipio_err("Gesture ctrl error, 0x%x\n", func);
		return;
	}

	return core_protocol_func_control(6, func);
}
EXPORT_SYMBOL(core_config_gesture_ctrl);

#ifdef ILITEK_EDGE_LIMIT
void core_config_edge_limit_ctrl(bool enable)
{
	ipio_debug(DEBUG_CONFIG, "edge limit = %d\n", enable);

	return core_protocol_func_control(13, enable);
}
EXPORT_SYMBOL(core_config_edge_limit_ctrl);
#endif


void core_config_game_switch_ctrl(bool enable)
{
	ipio_debug(DEBUG_CONFIG, "play_mode = %d\n", enable);

	return core_protocol_func_control(14, enable);
}
EXPORT_SYMBOL(core_config_game_switch_ctrl);

void core_config_phone_cover_ctrl(bool enable)
{
	ipio_debug(DEBUG_CONFIG, "Phone Cover = %d\n", enable);

	return core_protocol_func_control(7, enable);
}
EXPORT_SYMBOL(core_config_phone_cover_ctrl);

void core_config_finger_sense_ctrl(bool enable)
{
	ipio_debug(DEBUG_CONFIG, "Finger sense = %d\n", enable);

	return core_protocol_func_control(0, enable);
}
EXPORT_SYMBOL(core_config_finger_sense_ctrl);

void core_config_proximity_ctrl(bool enable)
{
	ipio_debug(DEBUG_CONFIG, "Proximity = %d\n", enable);

	return core_protocol_func_control(11, enable);
}
EXPORT_SYMBOL(core_config_proximity_ctrl);

void core_config_plug_ctrl(bool out)
{
	ipio_debug(DEBUG_CONFIG, "Plug Out = %d\n", out);

	return core_protocol_func_control(12, out);
}
EXPORT_SYMBOL(core_config_plug_ctrl);

void core_config_set_phone_cover(uint8_t *pattern)
{
	int i;

	if (pattern == NULL) {
		ipio_err("Invaild pattern\n");
		return;
	}

	for(i = 0; i < 8; i++)
		protocol->phone_cover_window[i+1] = pattern[i];

	ipio_debug(DEBUG_CONFIG, "window: cmd = 0x%x\n", protocol->phone_cover_window[0]);
	ipio_debug(DEBUG_CONFIG, "window: ul_x_l = 0x%x, ul_x_h = 0x%x\n", protocol->phone_cover_window[1],
		 protocol->phone_cover_window[2]);
	ipio_debug(DEBUG_CONFIG, "window: ul_y_l = 0x%x, ul_y_l = 0x%x\n", protocol->phone_cover_window[3],
		 protocol->phone_cover_window[4]);
	ipio_debug(DEBUG_CONFIG, "window: br_x_l = 0x%x, br_x_l = 0x%x\n", protocol->phone_cover_window[5],
		 protocol->phone_cover_window[6]);
	ipio_debug(DEBUG_CONFIG, "window: br_y_l = 0x%x, br_y_l = 0x%x\n", protocol->phone_cover_window[7],
		 protocol->phone_cover_window[8]);

	core_protocol_func_control(9, 0);
}
EXPORT_SYMBOL(core_config_set_phone_cover);

void core_config_deep_sleep(void)
{
	   uint8_t cmd[4] = {0x01, 0x02, 0x03};
	   ipio_info("deep sleep write cmd 0x%X 0x%X 0x%X\n", cmd[0], cmd[1], cmd[2]);
	   core_write(core_config->slave_i2c_addr, cmd, 3);
	   return;
}
 
/*
 * ic_suspend: Get IC to suspend called from system.
 *
 * The timing when goes to sense stop or houw much times the command need to be called
 * is depending on customer's system requirement, which might be different due to
 * the DDI design or other conditions.
 */
void core_config_ic_suspend(void)
{
	//uint8_t temp[4] = {0};
	//int res = 0;
	int i = 0;
	ipio_info("Starting to suspend ...\n");

	for (i = 0; i < 40; i++) {
		if (!core_firmware->isUpgrading) {
			break;
		}
		msleep(5);
	}
//#ifdef ILITEK_ESD_CHECK
	//ilitek_cancel_esd_check();
//#endif
	/* sense stop */
	core_config_sense_ctrl(false);

	/* check system busy */
	if (core_config_check_cdc_busy(5, 50) < 0) {
		ipio_err("Check busy is timout !\n");
		core_config_sense_ctrl(false);
		core_config_check_cdc_busy(5, 50);
	}

ipio_debug(DEBUG_CONFIG, "core_config->isEnableGesture = %d before\n", core_config->isEnableGesture);
if(core_config->isEnableGesture)
{
	ipio_debug(DEBUG_CONFIG, "core_config->isEnableGesture = %d after\n", core_config->isEnableGesture);

	enable_irq_wake(ipd->isr_gpio);

	ipio_info("Enabled Gesture = %d\n", core_config->isEnableGesture);

		core_fr->actual_fw_mode = P5_0_FIRMWARE_GESTURE_MODE;
		if (1) {
		#ifdef HOST_DOWNLOAD
		if(core_load_gesture_code() < 0)
			{
			ipio_err("load gesture code fail\n");
			goto out;
			}
		//ilitek_platform_enable_irq();
		#endif
			}
		if(psensor_close == true) {
			core_config->isEnableGesture = false;
			core_config_sleep_ctrl(false);
			ipio_info("suspend set sleep ctrl redo\n");
		}
		gesture_done = true;
	 }else {

		gesture_done = false;
		core_config_deep_sleep();

	}
out:

	ipio_info("Suspend done\n");

}
EXPORT_SYMBOL(core_config_ic_suspend);

/*
 * ic_resume: Get IC to resume called from system.
 *
 * The timing when goes to sense start or houw much times the command need to be called
 * is depending on customer's system requirement, which might be different due to
 * the DDI design or other conditions.
 */
void core_config_ic_resume(void)
{
 //if(core_gesture->suspend)
 {
	//core_gesture->suspend = false;
	//int res = 0;
	ipio_info("Starting to resume ...\n");
	if (core_config->isEnableGesture) {
		#ifdef HOST_DOWNLOAD
		if(core_load_ap_code() < 0)
		{
			ipio_err("load ap code fail\n");
			ilitek_platform_tp_hw_reset(true);
		}
		#endif
	}
	else{
		/*res = ili_power_on(ipd,true);
		if (res) {
			ipio_info("power on fail\n");
		}
		ilitek_platform_tp_hw_reset(true);*/
	}
	/* sleep out */
	core_config_sleep_ctrl(true);

	/* check system busy */
	if (core_config_check_cdc_busy(50, 50) < 0)
		ipio_err("Check busy is timout !\n");

	/* sense start for TP */
	core_config_sense_ctrl(true);
	core_fr_mode_control( &protocol->demo_mode);
	/* Soft reset */
	// core_config_ice_mode_enable();
	// mdelay(10);
	// core_config_ic_reset();
	ipio_info("Resume done\n");
 }
}
EXPORT_SYMBOL(core_config_ic_resume);

int core_config_ice_mode_disable(void)
{
	uint32_t res = 0;
	uint8_t cmd[4];
	cmd[0] = 0x1b;
	cmd[1] = 0x62;
	cmd[2] = 0x10;
	cmd[3] = 0x18;

	ipio_debug(DEBUG_CONFIG, "ICE Mode disabled\n");
	res = core_write(core_config->slave_i2c_addr, cmd, 4);
	core_config->icemodeenable = false;
	return res;
}
EXPORT_SYMBOL(core_config_ice_mode_disable);

int core_config_ice_mode_enable(void)
{
	ipio_debug(DEBUG_CONFIG, "ICE Mode enabled\n");
	core_config->icemodeenable = true;
	if (core_config_ice_mode_write(0x181062, 0x0, 0) < 0)
		return -1;

	return 0;
}
EXPORT_SYMBOL(core_config_ice_mode_enable);

int core_config_set_watch_dog(bool enable)
{
	int timeout = 10, ret = 0;
	uint8_t off_bit = 0x5A, on_bit = 0xA5;
	uint8_t value_low = 0x0, value_high = 0x0;
	uint32_t wdt_addr = core_config->wdt_addr;

	if (wdt_addr <= 0 || core_config->chip_id <= 0) {
		ipio_err("WDT/CHIP ID is invalid\n");
		return -EINVAL;
	}

	/* Config register and values by IC */
	if (core_config->chip_id == CHIP_TYPE_ILI7807) {
		value_low = 0x07;
		value_high = 0x78;
	} else if (core_config->chip_id == CHIP_TYPE_ILI9881 ) {
		value_low = 0x81;
		value_high = 0x98;
	} else {
		ipio_err("Unknown CHIP type (0x%x)\n",core_config->chip_id);
		return -ENODEV;
	}

	if (enable) {
		core_config_ice_mode_write(wdt_addr, 1, 1);
	} else {
		core_config_ice_mode_write(wdt_addr, value_low, 1);
		core_config_ice_mode_write(wdt_addr, value_high, 1);
		/* need to delay 300us after stop mcu to wait fw relaod */
		udelay(300);
	}

	while (timeout > 0) {
		ret = core_config_ice_mode_read(0x51018);
		ipio_debug(DEBUG_CONFIG, "bit = %x\n", ret);

		if (enable) {
			if (CHECK_EQUAL(ret, on_bit) == 0)
				break;
		} else {
			if (CHECK_EQUAL(ret, off_bit) == 0)
				break;
		}

		timeout--;
		mdelay(10);
	}

	if (timeout > 0) {
		if (enable) {
			ipio_debug(DEBUG_CONFIG, "WDT turn on succeed\n");
		} else {
			core_config_ice_mode_write(wdt_addr, 0, 1);
			ipio_debug(DEBUG_CONFIG, "WDT turn off succeed\n");
		}
	} else {
		ipio_err("WDT turn on/off timeout !\n");
		core_config_get_reg_data(ILI9881_PC_COUNTER_ADDR);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(core_config_set_watch_dog);

int core_config_check_cdc_busy(int conut, int delay)
{
	int timer = conut, res = -1;
	uint8_t cmd[2] = { 0 };
	uint8_t busy[2] = { 0 };

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_cdc_busy;

	while (timer > 0) {
		mdelay(delay);
		core_write(core_config->slave_i2c_addr, cmd, 2);
		core_write(core_config->slave_i2c_addr, &cmd[1], 1);
		core_read(core_config->slave_i2c_addr, busy, 2);
		ipio_info("core_config_check_cdc_busy busy[0] = %x,busy[1] = %x\n", busy[0], busy[1]);
		if (core_fr->actual_fw_mode == P5_0_FIRMWARE_DEMO_MODE && busy[0] == 0x41) {
			ipio_debug(DEBUG_CONFIG, "Check busy is free\n");
			res = 0;
			break;
		}
		if (core_fr->actual_fw_mode == P5_0_FIRMWARE_TEST_MODE && busy[0] == 0x51) {
			ipio_debug(DEBUG_CONFIG, "Check busy is free\n");
			res = 0;
			break;
		}
		timer--;
	}

	if (res < -1) {
		ipio_info("%d mode Check busy timeout !!\n",core_fr->actual_fw_mode);
		core_config_get_reg_data(ILI9881_PC_COUNTER_ADDR);
	}
	return res;
}
EXPORT_SYMBOL(core_config_check_cdc_busy);

int core_config_check_int_status(bool high)
{
	int timer = 1000, res = -1;

	/* From FW request, timeout should at least be 5 sec */
	while (timer) {
		if(high) {
			if (gpio_get_value(ipd->int_gpio)) {
				ipio_debug(DEBUG_CONFIG, "Check busy is free\n");
				res = 0;
				break;
			}
		} else {
			if (!gpio_get_value(ipd->int_gpio)) {
				ipio_debug(DEBUG_CONFIG, "Check busy is free\n");
				res = 0;
				break;
			}
		}

		mdelay(3);
		timer--;
	}

	if (res < -1) {
		ipio_info("Check busy timeout !!\n");
		core_config_get_reg_data(ILI9881_PC_COUNTER_ADDR);
	}
	return res;
}
EXPORT_SYMBOL(core_config_check_int_status);

int core_config_get_project_id(uint8_t *pid_data)
{
	int i = 0, res = 0;
	uint32_t pid_addr = 0x1D000, pid_size = 10;

	res = core_config_ice_mode_enable();
	if (res < 0) {
		ipio_err("Failed to enter ICE mode, res = %d\n", res);
		return -1;
	}

	/* Disable watch dog */
	core_config_set_watch_dog(false);

	core_config_ice_mode_write(0x041000, 0x0, 1);   /* CS low */
	core_config_ice_mode_write(0x041004, 0x66aa55, 3);  /* Key */

	core_config_ice_mode_write(0x041008, 0x06, 1);
	core_config_ice_mode_write(0x041000, 0x01, 1);
	core_config_ice_mode_write(0x041000, 0x00, 1);
	core_config_ice_mode_write(0x041004, 0x66aa55, 3);  /* Key */
	core_config_ice_mode_write(0x041008, 0x03, 1);

	core_config_ice_mode_write(0x041008, (pid_addr & 0xFF0000) >> 16, 1);
	core_config_ice_mode_write(0x041008, (pid_addr & 0x00FF00) >> 8, 1);
	core_config_ice_mode_write(0x041008, (pid_addr & 0x0000FF), 1);

	for(i = 0; i < pid_size; i++) {
		core_config_ice_mode_write(0x041008, 0xFF, 1);
		pid_data[i] = core_config_ice_mode_read(0x41010);
		ipio_info("pid_data[%d] = 0x%x\n", i, pid_data[i]);
	}

	core_config_ice_mode_write(0x041010, 0x1, 0);   /* CS high */
	core_config_ic_reset();

	return res;
}
EXPORT_SYMBOL(core_config_get_project_id);
#if 0
int core_config_get_key_info(void)
{
	int res = 0, i;
	uint8_t cmd[2] = { 0 };

	memset(g_read_buf, 0, sizeof(g_read_buf));

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_key_info;

	res = core_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	msleep(1);

	res = core_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	msleep(1);

	res = core_read(core_config->slave_i2c_addr, &g_read_buf[0], protocol->key_info_len);
	if (res < 0) {
		ipio_err("Failed to read data via I2C, %d\n", res);
		goto out;
	}

	if (core_config->tp_info->nKeyCount) {
		/* NOTE: Firmware not ready yet */
		core_config->tp_info->nKeyAreaXLength = (g_read_buf[0] << 8) + g_read_buf[1];
		core_config->tp_info->nKeyAreaYLength = (g_read_buf[2] << 8) + g_read_buf[3];

		ipio_info("key: length of X area = %x\n", core_config->tp_info->nKeyAreaXLength);
		ipio_info("key: length of Y area = %x\n", core_config->tp_info->nKeyAreaYLength);

		for (i = 0; i < core_config->tp_info->nKeyCount; i++) {
			core_config->tp_info->virtual_key[i].nId = g_read_buf[i * 5 + 4];
			core_config->tp_info->virtual_key[i].nX = (g_read_buf[i * 5 + 5] << 8) + g_read_buf[i * 5 + 6];
			core_config->tp_info->virtual_key[i].nY = (g_read_buf[i * 5 + 7] << 8) + g_read_buf[i * 5 + 8];
			core_config->tp_info->virtual_key[i].nStatus = 0;

			ipio_info("key: id = %d, X = %d, Y = %d\n", core_config->tp_info->virtual_key[i].nId,
				 core_config->tp_info->virtual_key[i].nX, core_config->tp_info->virtual_key[i].nY);
		}
	}

out:
	return res;
}
EXPORT_SYMBOL(core_config_get_key_info);

#endif


int core_config_get_tp_info(void)
{
	int res = 0;
	uint8_t cmd[2] = { 0 };

	memset(g_read_buf, 0, sizeof(g_read_buf));

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_tp_info;

	core_config->tp_info->nMinX = minX;
	core_config->tp_info->nMinY = minY;
	core_config->tp_info->nMaxX = maxX;
	core_config->tp_info->nMaxY = maxY;
	core_config->tp_info->nXChannelNum = xchannel;
	core_config->tp_info->nYChannelNum = ychannel;
	core_config->tp_info->self_tx_channel_num = self_tx;
	core_config->tp_info->self_rx_channel_num = self_rx;
	core_config->tp_info->side_touch_type = dside_touch_type;
	core_config->tp_info->nMaxTouchNum = max_touch_num;
	core_config->tp_info->nKeyCount = max_key_num;

	core_config->tp_info->nMaxKeyButtonNum = 5;

	res = core_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}
	msleep(1);
	res = core_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}
	msleep(1);
	res = core_read(core_config->slave_i2c_addr, &g_read_buf[0], protocol->tp_info_len);
	if (res < 0) {
		ipio_err("Failed to read data via I2C, %d\n", res);
		goto out;
	}

	/* in protocol v5, ignore the first btye because of a header. */
/*	core_config->tp_info->nMinX = g_read_buf[1];
	core_config->tp_info->nMinY = g_read_buf[2];
	core_config->tp_info->nMaxX = (g_read_buf[4] << 8) + g_read_buf[3];
	core_config->tp_info->nMaxY = (g_read_buf[6] << 8) + g_read_buf[5];
	core_config->tp_info->nXChannelNum = g_read_buf[7];
	core_config->tp_info->nYChannelNum = g_read_buf[8];
	core_config->tp_info->self_tx_channel_num = g_read_buf[11];
	core_config->tp_info->self_rx_channel_num = g_read_buf[12];
	core_config->tp_info->side_touch_type = g_read_buf[13];
	core_config->tp_info->nMaxTouchNum = g_read_buf[9];
	core_config->tp_info->nKeyCount = g_read_buf[10];
*/

	ipio_info("minX = %d, minY = %d, maxX = %d, maxY = %d\n",
		 core_config->tp_info->nMinX, core_config->tp_info->nMinY,
		 core_config->tp_info->nMaxX, core_config->tp_info->nMaxY);
	ipio_info("xchannel = %d, ychannel = %d, self_tx = %d, self_rx = %d\n",
		 core_config->tp_info->nXChannelNum, core_config->tp_info->nYChannelNum,
		 core_config->tp_info->self_tx_channel_num, core_config->tp_info->self_rx_channel_num);
	ipio_info("side_touch_type = %d, max_touch_num= %d, touch_key_num = %d, max_key_num = %d\n",
		 core_config->tp_info->side_touch_type, core_config->tp_info->nMaxTouchNum,
		 core_config->tp_info->nKeyCount, core_config->tp_info->nMaxKeyButtonNum);

out:
	return res;
}
EXPORT_SYMBOL(core_config_get_tp_info);

int core_config_get_protocol_ver(void)
{
	int res = 0, i = 0;
	int major, mid, minor;
	uint8_t cmd[2] = { 0 };

	memset(g_read_buf, 0, sizeof(g_read_buf));
	memset(core_config->protocol_ver, 0x0, sizeof(core_config->protocol_ver));

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_pro_ver;

	res = core_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	msleep(1);

	res = core_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	msleep(1);

	res = core_read(core_config->slave_i2c_addr, &g_read_buf[0], protocol->pro_ver_len);
	if (res < 0) {
		ipio_err("Failed to read data via I2C, %d\n", res);
		goto out;
	}

	/* ignore the first btye because of a header. */
	for (; i < protocol->pro_ver_len - 1; i++) {
		core_config->protocol_ver[i] = g_read_buf[i + 1];
	}

	ipio_info("Procotol Version = %d.%d.%d\n",
		 core_config->protocol_ver[0], core_config->protocol_ver[1], core_config->protocol_ver[2]);

	major = core_config->protocol_ver[0];
	mid = core_config->protocol_ver[1];
	minor = core_config->protocol_ver[2];

	/* update protocol if they're different with the default ver set by driver */
	if (major != PROTOCOL_MAJOR || mid != PROTOCOL_MID || minor != PROTOCOL_MINOR) {
		res = core_protocol_update_ver(major, mid, minor);
		if (res < 0)
			ipio_err("Protocol version is invalid\n");
	}

out:
	return res;
}
EXPORT_SYMBOL(core_config_get_protocol_ver);

int core_config_get_core_ver(void)
{
	int res = 0, i = 0;
	uint8_t cmd[2] = { 0 };

	memset(g_read_buf, 0, sizeof(g_read_buf));

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_core_ver;

	res = core_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	msleep(1);

	res = core_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	msleep(1);

	res = core_read(core_config->slave_i2c_addr, &g_read_buf[0], protocol->core_ver_len);
	if (res < 0) {
		ipio_err("Failed to read data via I2C, %d\n", res);
		goto out;
	}

	for (; i < protocol->core_ver_len - 1; i++)
		core_config->core_ver[i] = g_read_buf[i + 1];

	/* in protocol v5, ignore the first btye because of a header. */
	ipio_info("Core Version = %d.%d.%d.%d\n",
		 core_config->core_ver[1], core_config->core_ver[2],
		 core_config->core_ver[3], core_config->core_ver[4]);

out:
	return res;
}
EXPORT_SYMBOL(core_config_get_core_ver);

/*
 * Getting the version of firmware used on the current one.
 *
 */
int core_config_get_fw_ver(void)
{
	int res = 0, i = 0;
	uint8_t cmd[2] = { 0 };
	memset(g_read_buf, 0, sizeof(g_read_buf));

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_fw_ver;

	res = core_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	msleep(1);

	res = core_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	msleep(1);

	res = core_read(core_config->slave_i2c_addr, &g_read_buf[0], protocol->fw_ver_len);
	if (res < 0) {
		ipio_err("Failed to read fw version %d\n", res);
		goto out;
	}

	for (; i < protocol->fw_ver_len; i++)
		core_config->firmware_ver[i] = g_read_buf[i];
	if (ili_ctpmodule == TXD_MODEL) {
		sprintf(core_config->ilitek_ts_info.version, "EA6011%02X-2", core_config->firmware_ver[3]);
	} else if (ili_ctpmodule == DJ_MODEL) {
		sprintf(core_config->ilitek_ts_info.version, "EA6013%02X-1", core_config->firmware_ver[3]);
	}
	if (protocol->mid >= 0x3) {
		ipio_info("Firmware Version = %d.%d.%d.%d\n", core_config->firmware_ver[1], core_config->firmware_ver[2], core_config->firmware_ver[3], core_config->firmware_ver[4]);
	} else {
		ipio_info("Firmware Version = %d.%d.%d\n",
			core_config->firmware_ver[1], core_config->firmware_ver[2], core_config->firmware_ver[3]);
	}

	if ( ili_ctpmodule == 0 ){
		sprintf(Ctp_name,"TXD,ILI9881,FW:0x%x\n",core_config->firmware_ver[3]);
	}
	else{
		sprintf(Ctp_name,"DJ,ILI9881,FW:0x%x\n",core_config->firmware_ver[3]);
	}


	if ( ili_ctpmodule == 0 ) {
	    sprintf(ili_version,"Txd_Ili_%x",core_config->firmware_ver[3]);
	    devinfo_info_tp_set(ili_version, "TXD",OPPO_FIRMWARE_NAME_PATH);
	} else {
	    sprintf(ili_version,"DJ_Ili_%x",core_config->firmware_ver[3]);
	    devinfo_info_tp_set(ili_version, "DJ",OPPO_FIRMWARE_NAME_PATH);
	}


out:
	return res;
}
EXPORT_SYMBOL(core_config_get_fw_ver);

int core_config_get_chip_id(void)
{
	int res = 0;
	static int do_once = 0;
	uint32_t RealID = 0, PIDData = 0;

	res = core_config_ice_mode_enable();
	if (res < 0) {
		ipio_err("Failed to enter ICE mode, res = %d\n", res);
		goto out;
	}

	msleep(1);

	PIDData = core_config_ice_mode_read(core_config->pid_addr);
	core_config->chip_pid = PIDData;
	core_config->core_type = PIDData & 0xFF;
	//sprintf(core_config->ilitek_ts_info.manufacture, "0x%08X", core_config->chip_pid);
	ipio_info("PID = 0x%x, Core type = 0x%x\n",
		core_config->chip_pid, core_config->core_type);
	if (PIDData) {
		RealID = check_chip_id(PIDData);
		if (RealID != core_config->chip_id) {
			ipio_err("CHIP ID ERROR: 0x%x, TP_TOUCH_IC = 0x%x\n", RealID, TP_TOUCH_IC);
			res = -ENODEV;
			goto out;
		}
	} else {
		ipio_err("PID DATA error : 0x%x\n", PIDData);
		res = -EINVAL;
		goto out;
	}

	if (do_once == 0) {
		/* reading flash id needs to let ic entry to ICE mode */
		read_flash_info(0x9F, 4);
		do_once = 1;
	}

out:
	core_config_ic_reset();
	//msleep(150);
	return res;
}
EXPORT_SYMBOL(core_config_get_chip_id);

int core_config_init(void)
{
	int i = 0;

	core_config = kzalloc(sizeof(*core_config) * sizeof(uint8_t) * 6, GFP_KERNEL);
	if (ERR_ALLOC_MEM(core_config)) {
		ipio_err("Failed to allocate core_config mem, %ld\n", PTR_ERR(core_config));
		core_config_remove();
		return -ENOMEM;
	}

	core_config->tp_info = kzalloc(sizeof(*core_config->tp_info), GFP_KERNEL);
	if (ERR_ALLOC_MEM(core_config->tp_info)) {
		ipio_err("Failed to allocate core_config->tp_info mem, %ld\n", PTR_ERR(core_config->tp_info));
		core_config_remove();
		return -ENOMEM;
	}
#ifdef ILITEK_GESTURE_WAKEUP
	gesture_report_data = kzalloc(sizeof(struct ili_gesture_info), GFP_KERNEL);
	if (ERR_ALLOC_MEM(gesture_report_data)) {
		ipio_err("Failed to allocate gesture_report_data mem, %ld\n", PTR_ERR(gesture_report_data));
		core_config_remove();
		return -ENOMEM;
	}
#endif
	for (; i < ARRAY_SIZE(ipio_chip_list); i++) {
		if (ipio_chip_list[i] == TP_TOUCH_IC) {
			core_config->chip_id = ipio_chip_list[i];
			core_config->chip_type = 0x0000;

			core_config->do_ic_reset = false;
			core_config->isEnableGesture = false;
			core_config->gesture_backup = false;
			core_config->gameSwitch = false;

			if (core_config->chip_id == CHIP_TYPE_ILI7807) {
				core_config->slave_i2c_addr = ILI7807_SLAVE_ADDR;
				core_config->ice_mode_addr = ILI7807_ICE_MODE_ADDR;
				core_config->pid_addr = ILI7807_PID_ADDR;
				core_config->wdt_addr = ILI7808_WDT_ADDR;
			} else if (core_config->chip_id == CHIP_TYPE_ILI9881) {
				core_config->slave_i2c_addr = ILI9881_SLAVE_ADDR;
				core_config->ice_mode_addr = ILI9881_ICE_MODE_ADDR;
				core_config->pid_addr = ILI9881_PID_ADDR;
				core_config->wdt_addr = ILI9881_WDT_ADDR;
			}
			return 0;
		}
	}

	ipio_err("Can't find this chip in support list\n");
	return 0;
}
EXPORT_SYMBOL(core_config_init);

void core_config_remove(void)
{
	ipio_info("Remove core-config memebers\n");

	if (core_config != NULL) {
		ipio_kfree((void **)&core_config->tp_info);
		ipio_kfree((void **)&core_config);
	}
}
EXPORT_SYMBOL(core_config_remove);
