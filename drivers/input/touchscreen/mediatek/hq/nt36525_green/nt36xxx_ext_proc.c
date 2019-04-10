/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 *
 * $Revision: 32206 $
 * $Date: 2018-08-10 19:23:04 +0800 (ßLÎå, 10 °ËÔÂ 2018) $
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

  
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>

#include "nt36xxx.h"
#include <asm/uaccess.h>

#if NVT_TOUCH_EXT_PROC
#define NVT_FW_VERSION "tp_fw_version"
#define NVT_BASELINE "nvt_baseline"
#define NVT_RAW "nvt_raw"
#define NVT_DIFF "nvt_diff"

#define OPPO_TOUCHPANEL_NAME "touchpanel"
#define OPPO_DATA_LIMIT "data_limit"
#define OPPO_BASELINE_TEST "baseline_test"
#define OPPO_BLACK_SCREEN_TEST "black_screen_test"
#define OPPO_COORDINATE "coordinate"
#define OPPO_DEBUG_INFO "debug_info"
#define OPPO_DELTA "delta"
#define OPPO_BASELINE "baseline"
#define OPPO_MAIN_REGISTER "main_register"
#define OPPO_DEBUG_LEVEL "debug_level"
#define OPPO_GESTURE "double_tap_enable"
#define OPPO_IRQ_DEPATH "irq_depth"
#define OPPO_REGISTER_INFO "oppo_register_info"
#define OPPO_FW_UPDATE "tp_fw_update"
#define OPPO_GAME_SWITCH "game_switch_enable"
#define OPPO_TP_LIMIT_ENABLE "oppo_tp_limit_enable"
#define OPPO_INCELL_PANEL "incell_panel"
#define OPPO_TP_DIRECTION "oppo_tp_direction"

extern struct file_operations nvt_selftest_fops;
extern struct file_operations nvt_lpwg_selftest_fops;

static struct proc_dir_entry *oppo_data_limit;
static struct proc_dir_entry *oppo_baseline_test;
static struct proc_dir_entry *oppo_black_screen_test;
static struct proc_dir_entry *oppo_coordinate;
static struct proc_dir_entry *oppo_delta;
static struct proc_dir_entry *oppo_baseline;
static struct proc_dir_entry *oppo_main_register;
static struct proc_dir_entry *oppo_debug_level;
static struct proc_dir_entry *oppo_gesture;
static struct proc_dir_entry *oppo_irq_depath;
static struct proc_dir_entry *register_info_oppo;
static struct proc_dir_entry *oppo_fw_update;
static struct proc_dir_entry *oppo_game_switch;
static struct proc_dir_entry *oppo_tp_limit_enable;

static struct proc_dir_entry *NVT_proc_diff_entry;
static struct proc_dir_entry *oppo_incell_tp;
static struct proc_dir_entry *oppo_tp_direction;


extern int oppo_nvt_blackscreen_test(void);
extern int g_gesture;
int tp_dflag = 0;

#define SPI_TANSFER_LENGTH  256

#define NORMAL_MODE 0x00
#define TEST_MODE_1 0x21
#define TEST_MODE_2 0x22
#define HANDSHAKING_HOST_READY 0xBB

#define XDATA_SECTOR_SIZE   256

static uint8_t xdata_tmp[2048] = {0};
static int32_t xdata[2048] = {0};
static int32_t xdata_i[2048] = {0};
static int32_t xdata_q[2048] = {0};

static struct proc_dir_entry *NVT_proc_fw_version_entry;
static struct proc_dir_entry *NVT_proc_baseline_entry;
static struct proc_dir_entry *NVT_proc_raw_entry;
static struct proc_dir_entry *NVT_proc_diff_entry;

int tp_limitflag;

/*******************************************************
Description:
	Novatek touchscreen change mode function.

return:
	n.a.
*******************************************************/
void nvt_change_mode(uint8_t mode)
{
	uint8_t buf[8] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	//---set mode---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = mode;
	CTP_SPI_WRITE(ts->client, buf, 2);

	if (mode == NORMAL_MODE) {
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = HANDSHAKING_HOST_READY;
		CTP_SPI_WRITE(ts->client, buf, 2);
		msleep(20);
	}
}

/*******************************************************
Description:
	Novatek touchscreen get firmware pipe function.

return:
	Executive outcomes. 0---pipe 0. 1---pipe 1.
*******************************************************/
uint8_t nvt_get_fw_pipe(void)
{
	uint8_t buf[8]= {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

	//---read fw status---
	buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
	buf[1] = 0x00;
	CTP_SPI_READ(ts->client, buf, 2);

	//NVT_LOG("FW pipe=%d, buf[1]=0x%02X\n", (buf[1]&0x01), buf[1]);

	return (buf[1] & 0x01);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data function.

return:
	n.a.
*******************************************************/
void nvt_read_mdata(uint32_t xdata_addr, uint32_t xdata_btn_addr)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	uint8_t buf[SPI_TANSFER_LENGTH + 1] = {0};
	uint32_t head_addr = 0;
	int32_t dummy_len = 0;
	int32_t data_len = 0;
	int32_t residual_len = 0;

	//---set xdata sector address & length---
	head_addr = xdata_addr - (xdata_addr % XDATA_SECTOR_SIZE);
	dummy_len = xdata_addr - head_addr;
	data_len = ts->x_num * ts->y_num * 2;
	residual_len = (head_addr + dummy_len + data_len) % XDATA_SECTOR_SIZE;

	//printk("head_addr=0x%05X, dummy_len=0x%05X, data_len=0x%05X, residual_len=0x%05X\n", head_addr, dummy_len, data_len, residual_len);

	//read xdata : step 1
	for (i = 0; i < ((dummy_len + data_len) / XDATA_SECTOR_SIZE); i++) {
		//---read xdata by SPI_TANSFER_LENGTH
		for (j = 0; j < (XDATA_SECTOR_SIZE / SPI_TANSFER_LENGTH); j++) {
			//---change xdata index---
			nvt_set_page(head_addr + (XDATA_SECTOR_SIZE * i) + (SPI_TANSFER_LENGTH * j));

			//---read data---
			buf[0] = SPI_TANSFER_LENGTH * j;
			CTP_SPI_READ(ts->client, buf, SPI_TANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < SPI_TANSFER_LENGTH; k++) {
				xdata_tmp[XDATA_SECTOR_SIZE * i + SPI_TANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04X\n", buf[k+1], (XDATA_SECTOR_SIZE*i + SPI_TANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (head_addr+XDATA_SECTOR_SIZE*i));
	}

	//read xdata : step2
	if (residual_len != 0) {
		//---read xdata by SPI_TANSFER_LENGTH
		for (j = 0; j < (residual_len / SPI_TANSFER_LENGTH + 1); j++) {
			//---change xdata index---
			nvt_set_page(xdata_addr + data_len - residual_len + (SPI_TANSFER_LENGTH * j));

			//---read data---
			buf[0] = SPI_TANSFER_LENGTH * j;
			CTP_SPI_READ(ts->client, buf, SPI_TANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < SPI_TANSFER_LENGTH; k++) {
				xdata_tmp[(dummy_len + data_len - residual_len) + SPI_TANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04x\n", buf[k+1], ((dummy_len+data_len-residual_len) + SPI_TANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (xdata_addr+data_len-residual_len));
	}

	//---remove dummy data and 2bytes-to-1data---
	for (i = 0; i < (data_len / 2); i++) {
		xdata[i] = (int16_t)(xdata_tmp[dummy_len + i * 2] + 256 * xdata_tmp[dummy_len + i * 2 + 1]);
	}

#if TOUCH_KEY_NUM > 0
	//read button xdata : step3
	//---change xdata index---
	nvt_set_page(xdata_btn_addr);
	//---read data---
	buf[0] = (xdata_btn_addr & 0xFF);
	CTP_SPI_READ(ts->client, buf, (TOUCH_KEY_NUM * 2 + 1));

	//---2bytes-to-1data---
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		xdata[ts->x_num * ts->y_num + i] = (int16_t)(buf[1 + i * 2] + 256 * buf[1 + i * 2 + 1]);
	}
#endif

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data from IQ to rss function.

return:
	n.a.
*******************************************************/
void nvt_read_mdata_rss(uint32_t xdata_i_addr, uint32_t xdata_q_addr, uint32_t xdata_btn_i_addr, uint32_t xdata_btn_q_addr)
{
	int i = 0;

	nvt_read_mdata(xdata_i_addr, xdata_btn_i_addr);
	memcpy(xdata_i, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));

	nvt_read_mdata(xdata_q_addr, xdata_btn_q_addr);
	memcpy(xdata_q, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));

	for (i = 0; i < (ts->x_num * ts->y_num + TOUCH_KEY_NUM); i++) {
		xdata[i] = (int32_t)int_sqrt((unsigned long)(xdata_i[i] * xdata_i[i]) + (unsigned long)(xdata_q[i] * xdata_q[i]));
	}
}

/*******************************************************
Description:
    Novatek touchscreen get meta data function.

return:
    n.a.
*******************************************************/
void nvt_get_mdata(int32_t *buf, uint8_t *m_x_num, uint8_t *m_y_num)
{
    *m_x_num = ts->x_num;
    *m_y_num = ts->y_num;
    memcpy(buf, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));
}

/*******************************************************
Description:
	Novatek touchscreen firmware version show function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_fw_version_show(struct seq_file *m, void *v)
{
	seq_printf(m, "novatek fw_ver=%d, x_num=%d, y_num=%d, button_num=%d\n", ts->fw_ver, ts->x_num, ts->y_num, ts->max_button_num);
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print show
	function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_show(struct seq_file *m, void *v)
{
	int32_t i = 0;
	int32_t j = 0;

	for (i = 0; i < ts->y_num; i++) {
		for (j = 0; j < ts->x_num; j++) {
			seq_printf(m, "%5d, ", xdata[i * ts->x_num + j]);
		}
		seq_puts(m, "\n");
	}

#if TOUCH_KEY_NUM > 0
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		seq_printf(m, "%5d, ", xdata[ts->x_num * ts->y_num + i]);
	}
	seq_puts(m, "\n");
#endif

	seq_printf(m, "\n\n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print start
	function.

return:
	Executive outcomes. 1---call next function.
	NULL---not call next function and sequence loop
	stop.
*******************************************************/
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print next
	function.

return:
	Executive outcomes. NULL---no next and call sequence
	stop function.
*******************************************************/
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print stop
	function.

return:
	n.a.
*******************************************************/
static void c_stop(struct seq_file *m, void *v)
{
	return;
}

const struct seq_operations nvt_fw_version_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_fw_version_show
};

const struct seq_operations nvt_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_show
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_fw_version open
	function.

return:
	n.a.
*******************************************************/
static int32_t nvt_fw_version_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_fw_version_seq_ops);
}

static const struct file_operations nvt_fw_version_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fw_version_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_baseline open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_baseline_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (ts->carrier_system) {
		nvt_read_mdata_rss(ts->mmap->BASELINE_ADDR, ts->mmap->BASELINE_Q_ADDR,
				ts->mmap->BASELINE_BTN_ADDR, ts->mmap->BASELINE_BTN_Q_ADDR);
	} else {
		nvt_read_mdata(ts->mmap->BASELINE_ADDR, ts->mmap->BASELINE_BTN_ADDR);
	}

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_baseline_fops = {
	.owner = THIS_MODULE,
	.open = nvt_baseline_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_raw open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_raw_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (ts->carrier_system) {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata_rss(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_PIPE0_Q_ADDR,
				ts->mmap->RAW_BTN_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_Q_ADDR);
		else
			nvt_read_mdata_rss(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_PIPE1_Q_ADDR,
				ts->mmap->RAW_BTN_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_Q_ADDR);
	} else {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_ADDR);
	}

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_raw_fops = {
	.owner = THIS_MODULE,
	.open = nvt_raw_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_diff open function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_diff_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (ts->carrier_system) {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata_rss(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_PIPE0_Q_ADDR,
				ts->mmap->DIFF_BTN_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_Q_ADDR);
		else
			nvt_read_mdata_rss(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_PIPE1_Q_ADDR,
				ts->mmap->DIFF_BTN_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_Q_ADDR);
	} else {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_ADDR);
	}

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_diff_fops = {
	.owner = THIS_MODULE,
	.open = nvt_diff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen extra function proc. file node
	initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
/* data_limit */
extern int32_t nvt_mp_parse_dt(struct device_node *root, const char *node_compatible, struct seq_file *m);
extern int32_t nvt_mp_parse_lpwg_dt(struct device_node *root, const char *node_compatible, struct seq_file *m);
extern void nvt_print_criteria(struct seq_file *m);
static int32_t c_data_limit_show(struct seq_file *m, void *v)
{
	struct device_node *np = ts->client->dev.of_node;
	unsigned char mpcriteria[32] = {0}; //novatek-mp-criteria-default

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	/* Parsing criteria from dts */
	if(of_property_read_bool(np, "novatek,mp-support-dt")) {
		snprintf(mpcriteria, PAGE_SIZE, "novatek-mp-criteria-%04X", ts->nvt_pid);
		if (nvt_mp_parse_dt(np, mpcriteria, m)) {
			mutex_unlock(&ts->lock);
			NVT_ERR("mp parse device tree failed!\n");
			return -EINVAL;
		}

		if (nvt_mp_parse_lpwg_dt(np, mpcriteria, m)) {
			mutex_unlock(&ts->lock);
			NVT_ERR("mp parse device tree failed!\n");
			return -EINVAL;
		}
	} else {
		NVT_LOG("Not found novatek,mp-support-dt, use default setting\n");
		//---Print Test Criteria---
		nvt_print_criteria(m);
	}

	mutex_unlock(&ts->lock);
	return 0;
}

const struct seq_operations oppo_data_limit_seq_ops = {
	.start	= c_start,
	.next	= c_next,
	.stop	= c_stop,
	.show = c_data_limit_show
};

static int32_t nvt_data_limit_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &oppo_data_limit_seq_ops);
}

static const struct file_operations oppo_data_limit_fops = {
	.owner = THIS_MODULE,
	.open = nvt_data_limit_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/* coordinate */
static int32_t c_oppo_coordinate_show(struct seq_file *m, void *v)
{
	struct gesture_info *gesture = &ts->gesture;
	char tmp[256] = {0};

	sprintf(tmp, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d",
		gesture->gesture_type,
		gesture->Point_start.x, gesture->Point_start.y,
		gesture->Point_end.x, gesture->Point_end.y,
		gesture->Point_1st.x, gesture->Point_1st.y,
		gesture->Point_2nd.x, gesture->Point_2nd.y,
		gesture->Point_3rd.x, gesture->Point_3rd.y,
		gesture->Point_4th.x, gesture->Point_4th.y,
		gesture->clockwise);

	/* oppo gesture formate */
	seq_printf(m, "%s\n", tmp);

	return 0;
}

const struct seq_operations oppo_coordinate_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_oppo_coordinate_show
};

static int32_t oppo_coordinate_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &oppo_coordinate_seq_ops);
}

static const struct file_operations oppo_coordinate_fops = {
	.owner = THIS_MODULE,
	.open = oppo_coordinate_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/* main_register */
static int32_t c_main_register_show(struct seq_file *m, void *v)
{
	struct irq_desc *desc = irq_to_desc(gpio_to_irq(ts->irq_gpio));
    uint8_t buf[4] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	//---read cmd status---
	buf[0] = 0x5E;
	buf[1] = 0xFF;
	CTP_SPI_READ(ts->client, buf, 2);

	NVT_LOG("PWR_FLAG:%d\n", (buf[1]>> PWR_FLAG) & 0x01);
	seq_printf(m, "PWR_FLAG:%d\n", (buf[1]>> PWR_FLAG) & 0x01);

	NVT_LOG("EDGE_REJECT:%d\n", (buf[1]>> EDGE_REJECT_L) & 0x03);
	seq_printf(m, "EDGE_REJECT:%d\n", (buf[1]>> EDGE_REJECT_L) & 0x03);

	NVT_LOG("JITTER_FLAG:%d\n", (buf[1]>> JITTER_FLAG) & 0x01);
	seq_printf(m, "JITTER_FLAG:%d\n", (buf[1]>> JITTER_FLAG) & 0x01);

	NVT_LOG("HEADSET_FLAG:%d\n", (buf[1]>> HEADSET_FLAG) & 0x01);
	seq_printf(m, "HEADSET_FLAG:%d\n", (buf[1]>> HEADSET_FLAG) & 0x01);

	NVT_ERR("IRQ_DEPTH:%d\n", desc->depth);
	seq_printf(m, "IRQ_DEPTH:%d\n", desc->depth);

	return 0;
}

const struct seq_operations oppo_main_register_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_main_register_show
};

static int32_t nvt_main_register_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &oppo_main_register_seq_ops);
}

static const struct file_operations oppo_main_register_fops = {
	.owner = THIS_MODULE,
	.open = nvt_main_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/* debug_level */
static ssize_t oppo_debug_level_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int tmp = 0;
	char cmd[128] = {0};

	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &tmp)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	ts->debug_level = tmp;

	NVT_LOG("debug_level is %d\n", ts->debug_level);

	if ((ts->debug_level != 0) && (ts->debug_level != 1) && (ts->debug_level != 2)) {
		NVT_ERR("debug level error %d\n", ts->debug_level);
		ts->debug_level = 0;
	}

	return count;
};

static const struct file_operations oppo_debug_level_fops =
{
	.write = oppo_debug_level_write,
	.owner = THIS_MODULE,
};

/* double_tap_enable */
#define GESTURE_ENABLE   (1)
#define GESTURE_DISABLE  (0)
static ssize_t oppo_gesture_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
	//uint8_t buff[4] = {0};
	char *ptr = NULL;
/*	if (ts->sleep_flag !=0) {
		NVT_LOG("%s, is already suspend",__func__);
		return -1;
	}
*/
	ptr = kzalloc(count,GFP_KERNEL);
	if (ptr == NULL){
		NVT_LOG("allocate memory fail\n");
		return -1;
	}
	if (copy_from_user(ptr, buf, count)) {
		NVT_LOG("input value error\n");
		return -EINVAL;
	}
	if (ptr[0] == '1') {
		ts->gesture_flag_back = 1;
		if (ts->sleep_flag == 1) {
			mutex_lock(&ts->lock);
			NVT_LOG("far away psensor\n");
			ts->gesture_enable = 1;
			g_gesture = 1;
			
			
			nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME_SIGNED);
			
			//buff[0] = EVENT_MAP_HOST_CMD;
			//buff[1] = 0x13;
			//CTP_SPI_WRITE(ts->client, buff, 2);
			nvt_mode_change_cmd(0x13);
			NVT_LOG("ts->gesture_enable =%d:need to download fw\n",ts->gesture_enable);
			mutex_unlock(&ts->lock);
		} else {
			ts->gesture_enable = 1;
			g_gesture = 1;
			NVT_LOG("normal to open gesture:Gesture %d\n",ts->gesture_enable);
		}
	} else if (ptr[0] == '0') {
		ts->gesture_flag_back = 0;
		ts->gesture_enable = 0;
		NVT_LOG("normal clsoe gesture:Gesture %d\n",ts->gesture_enable);
		if(ts->sleep_flag == 0)
			{
				g_gesture = 0;
			}
	} else if (ptr[0] == '2') {
		ts->gesture_flag_back = 2;
		if(ts->sleep_flag == 1) {
			ts->gesture_enable = 0;
			NVT_LOG("need psensor close Gesture \n");
			//g_gesture = 0;
			//buff[0] = EVENT_MAP_HOST_CMD;
			//buff[1] = 0x11;
			//CTP_SPI_WRITE(ts->client, buff, 2);
			nvt_mode_change_cmd(0x11);
		}
	}
	else {
		NVT_LOG("can not get the correct gesture control\n");
	}
	/*if (kstrtouint(cmd, 0, &tmp)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	ts->gesture_enable = tmp > 0 ? GESTURE_ENABLE : GESTURE_DISABLE;

	NVT_LOG("Gesture %s\n", ts->gesture_enable ? "enable" : "disable");
	*/
	kfree(ptr);
	return count;

}


static ssize_t oppo_gesture_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	int ret = 0;
	int len;
	uint8_t *ptr = NULL;

    if(*ppos) {
        return 0;
    }

	ptr = kzalloc(sizeof(uint8_t)*8, GFP_KERNEL);
	if (ptr == NULL) {
		NVT_ERR("failed to allocate memory for ptr\n");
		return 0;
	}

	len = sprintf(ptr, "%d\n", ts->gesture_flag_back);
	ret = copy_to_user(buf, ptr, len);

	*ppos += len;

	return len;

}

static const struct file_operations oppo_gesture_fops =
{
	.write = oppo_gesture_write,
	.read = oppo_gesture_read,
	.owner = THIS_MODULE,
};

/* tp_fw_update */
static ssize_t oppo_fw_update_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int tmp = 0;
	uint8_t update_type = 0;
	char cmd[128] = {0};

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if(copy_from_user(cmd, buf, count)) {
		mutex_unlock(&ts->lock);
		NVT_ERR("input value error\n");
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &tmp)) {
		mutex_unlock(&ts->lock);
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	update_type = tmp;

	NVT_LOG("update_type is %d\n", update_type);
	switch (update_type) {
		case 0:	/* noflash: force update. flash: force update */
			NVT_ERR("update_type %d. Do nothing for noflash\n", update_type);
			break;
		case 1: /* noflash: do nothing. flash: check fw version and update */
			force_update_firmware_release();
			nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME);
			break;
		case 2:
			NVT_LOG("update type is 2\n");
			force_update_firmware_release();
			nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME_SIGNED);
			break;
		default:
			NVT_ERR("update_type %d error\n", update_type);
			break;
	}

	NVT_LOG("--\n");
	mutex_unlock(&ts->lock);

	return count;
};

static const struct file_operations oppo_fw_update_fops =
{
	.write = oppo_fw_update_write,
	.owner = THIS_MODULE,
};

/* irq_depth */
static int32_t c_irq_depath_show(struct seq_file *m, void *v)
{
	struct irq_desc *desc = irq_to_desc(gpio_to_irq(ts->irq_gpio));
	NVT_ERR("depth %d\n", desc->depth);

	seq_printf(m, "%d\n", desc->depth);

	return 0;
}

const struct seq_operations oppo_irq_depath_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_irq_depath_show
};

static int32_t nvt_irq_depath_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &oppo_irq_depath_seq_ops);
}

static const struct file_operations oppo_irq_depath_fops = {
	.owner = THIS_MODULE,
	.open = nvt_irq_depath_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/* oppo_register_info */
struct oppo_register_info {
	uint32_t addr;
	uint32_t len;
} oppo_reg;

/*
 * Example data format: echo 11A60,2 > file_node
 */
static ssize_t oppo_register_info_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	uint8_t tmp[5] = {0};
	char cmd[128] = {0};

	/* Error handler */
	if (count != 8) {
		NVT_ERR("count %ld error\n", count);
		return count;
	}

	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}

	/* parsing address (Novatek address length: 5 bit) */
	sprintf(tmp, "%c%c%c%c%c", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);

	if (kstrtouint(tmp, 16, &oppo_reg.addr)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	NVT_LOG("address: 0x%05X\n", oppo_reg.addr);

	/* parsing length */
	sprintf(tmp, "%c", cmd[6]);
	if (kstrtouint(tmp, 10, &oppo_reg.len)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	NVT_LOG("len %d\n", oppo_reg.len);

	return count;
}

static ssize_t oppo_register_info_read(struct file *file, char __user *buff,size_t count, loff_t *ppos)
{
	uint8_t *buf = NULL;
	uint8_t *ptr = NULL;
	uint8_t len = 0;
	uint8_t i = 0;
	int32_t ret = 0;

	if(*ppos) {
		return 0;	/* the end */
	}

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	if (oppo_reg.len == 0) {
		NVT_ERR("len = %d\n", oppo_reg.len);
		goto fail;
	}

	buf = (uint8_t *)kzalloc(sizeof(uint8_t)*(oppo_reg.len), GFP_KERNEL);
	if (buf == NULL) {
		NVT_ERR("failed to allocate memory for buf\n");
		goto fail;
	}

	ptr = (uint8_t *)kzalloc(sizeof(uint8_t)*(oppo_reg.len)*3+1, GFP_KERNEL);
	if (ptr == NULL) {
		NVT_ERR("failed to allocate memory for ptr\n");
		goto fail;
	}

	/* read data */
	nvt_set_page(oppo_reg.addr);
	buf[0] = oppo_reg.addr & 0x7F;
	CTP_SPI_READ(ts->client, buf, oppo_reg.len + 1);

	/* set index to EVENT_BUF_ADDR */
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	/* copy hex data to string */
	for (i=0 ; i<oppo_reg.len ; i++) {
		len += sprintf(ptr+len, "%02X ", buf[i+1]);
		//NVT_ERR("[%d] buf %02X\n", i, buf[i+1]);
	}

	/* new line */
	len += sprintf(ptr+len, "\n");

	ret = copy_to_user(buff,ptr,len);

	*ppos += len;

fail:
	mutex_unlock(&ts->lock);

	return len;
}

static const struct file_operations oppo_register_info_fops =
{
	.write = oppo_register_info_write,
	.read = oppo_register_info_read,
	.owner = THIS_MODULE,
};

/* game_switch_enable */
static ssize_t oppo_game_switch_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int tmp = 0;
	char cmd[128] = {0};

	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &tmp)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	NVT_LOG("game switch enable is %d\n", tmp);
	tmp = !!tmp;

	if(nvt_mode_switch(MODE_GAME, tmp)) {
		NVT_ERR("game switch enable fail!\n");
	}

	return count;
};

static const struct file_operations oppo_game_switch_fops =
{
	.write = oppo_game_switch_write,
	.owner = THIS_MODULE,
};


/* oppo_tp_limit_enable */
static ssize_t oppo_tp_limit_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int tmp = 0;
	char cmd[128] = {0};

	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &tmp)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	NVT_LOG("edge reject enable is %d\n", tmp);
	tp_limitflag = tmp;
	if(nvt_mode_switch(MODE_EDGE, tp_dflag)) {
		NVT_ERR("edge reject enable fail!\n");
	}

	return count;
};
static ssize_t oppo_tp_limit_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char *ptr = NULL;
	int lens = 0;
	if (*ppos){
		printk("is not at the page start \n");
		return 0;
	}

	ptr = kzalloc(32,GFP_KERNEL);
	lens += sprintf(ptr + lens,"limit_control = %d\n",tp_limitflag);
	ret = copy_to_user(buf,ptr,lens);
	if (ret)
		printk("NVT copy_to_user fail \n");
	*ppos +=lens;

	return lens;


}

static const struct file_operations oppo_tp_limit_fops =
{
	.write = oppo_tp_limit_write,
	.read = oppo_tp_limit_read,
	.owner = THIS_MODULE,
};


/* tp_incell */
static int32_t c_tp_incell_show(struct seq_file *m, void *v)
{
	int status = 1;
	NVT_ERR("depth %d\n", status);

	seq_printf(m, "%d\n", status);

	return 0;
}

const struct seq_operations oppo_tp_incell_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_tp_incell_show
};

static int32_t nvt_tp_incell_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &oppo_tp_incell_seq_ops);
}

static const struct file_operations nvt_incell_fops = {
	.owner = THIS_MODULE,
	.open = nvt_tp_incell_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static ssize_t nvt_direction_read(struct file *file, char __user *userbuf, size_t count, loff_t *ppos)
{
	uint8_t buf[2];
	uint8_t len;
	int ret;
	len = 2;
	if(*ppos) {
		return 0;	/* the end */
	}
	sprintf(buf,"%d",tp_dflag);
	NVT_LOG("buf[0]:%d,buf[1]:%d",buf[0],buf[1]);
	len += sprintf(buf+len, "\n");
	ret = copy_to_user(userbuf,buf,len);
	if(ret<0)
		NVT_ERR("copy to user error\n");
	
	*ppos += len;
	return len;
}
static ssize_t nvt_direction_write(struct file *file, const char __user *userbuf, size_t count, loff_t *ppos)
{
	uint8_t cmd[5] = {0};
//	uint8_t tmp[5] = {0};

	if(count == 0) {
		NVT_ERR("count is 0 error\n");
		return -EINVAL;
	}
	NVT_LOG("count:%d",count);
	if(copy_from_user(cmd, userbuf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}
	NVT_LOG("cmd[0]:%d,cmd[1]:%d",cmd[0],cmd[1]);
//	sprintf(tmp, "%d", cmd[0]);
/*	
	if (kstrtouint(tmp, 10, tp_dflag)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}
*/
	tp_dflag = cmd[0] -'0';
	NVT_LOG("tp_dflag:%d",tp_dflag);
	return count;
}


static const struct file_operations nvt_direction_fops = {
	.owner = THIS_MODULE,
	.read = nvt_direction_read,
	.write = nvt_direction_write,
};
int32_t nvt_extra_proc_init(void)
{
    struct proc_dir_entry *oppo_touchpanel_proc = NULL;
	struct proc_dir_entry *debug_info = NULL;
/* chenyunrui@RM.BSP.TP.FUNCTION, 2018/11/8, Add start*/
   //create /proc/devinfo/tp                             
	register_devinfo("tp",&ts->nvt_ts_info);
/*chenyunrui@RM.BSP.TP.FUNCTION, 2018/11/8, Add end*/	

	oppo_touchpanel_proc = proc_mkdir(OPPO_TOUCHPANEL_NAME, NULL);
    if(oppo_touchpanel_proc == NULL) {
        NVT_ERR("create oppo_touchpanel_proc fail\n");
        return -ENOMEM;
    }

	debug_info = proc_mkdir(OPPO_DEBUG_INFO, oppo_touchpanel_proc);
    if(debug_info == NULL) {
        NVT_ERR("create debug_info fail\n");
        return -ENOMEM;
    }

	oppo_baseline_test = proc_create(OPPO_BASELINE_TEST, 0664, oppo_touchpanel_proc, &nvt_selftest_fops);
	if (oppo_baseline_test == NULL) {
        NVT_ERR("create proc/touchpanel/baseline_test Failed!\n");
		return -ENOMEM;
	}

	oppo_black_screen_test = proc_create(OPPO_BLACK_SCREEN_TEST, 0666, oppo_touchpanel_proc, &nvt_lpwg_selftest_fops);
	if (oppo_baseline_test == NULL) {
        NVT_ERR("create proc/touchpanel/black_screen_test Failed!\n");
		return -ENOMEM;
	}

	oppo_coordinate= proc_create(OPPO_COORDINATE, 0664, oppo_touchpanel_proc, &oppo_coordinate_fops);
	if (oppo_coordinate == NULL) {
        NVT_ERR("create proc/touchpanel/coordinate Failed!\n");
		return -ENOMEM;
	}

	oppo_delta= proc_create(OPPO_DELTA, 0664, debug_info, &nvt_diff_fops);
	if (oppo_delta == NULL) {
        NVT_ERR("create proc/touchpanel/debug_info/delta Failed!\n");
		return -ENOMEM;
	}

	oppo_baseline = proc_create(OPPO_BASELINE, 0664, debug_info, &nvt_baseline_fops);
	if (oppo_baseline == NULL) {
        NVT_ERR("create proc/touchpanel/debug_info/baseline Failed!\n");
		return -ENOMEM;
	}

	oppo_main_register= proc_create(OPPO_MAIN_REGISTER, 0664, debug_info, &oppo_main_register_fops);
	if (oppo_main_register == NULL) {
        NVT_ERR("create proc/touchpanel/debug_info/main_register Failed!\n");
		return -ENOMEM;
	}

	oppo_data_limit = proc_create(OPPO_DATA_LIMIT, 0664, debug_info, &oppo_data_limit_fops);
	if (oppo_data_limit == NULL) {
        NVT_ERR("create proc/touchpanel/debug_info/data_limit Failed!\n");
		return -ENOMEM;
	}

	oppo_debug_level= proc_create(OPPO_DEBUG_LEVEL, 0664, oppo_touchpanel_proc, &oppo_debug_level_fops);
	if (oppo_debug_level == NULL) {
        NVT_ERR("create proc/touchpanel/debug_level Failed!\n");
		return -ENOMEM;
	}

	oppo_gesture= proc_create(OPPO_GESTURE,0664, oppo_touchpanel_proc, &oppo_gesture_fops);
	if (oppo_gesture == NULL) {
        NVT_ERR("create proc/touchpanel/double_tap_enable Failed!\n");
		return -ENOMEM;
	}

	oppo_irq_depath = proc_create(OPPO_IRQ_DEPATH, 0664, oppo_touchpanel_proc, &oppo_irq_depath_fops);
	if (oppo_irq_depath == NULL) {
        NVT_ERR("create proc/touchpanel/irq_depath Failed!\n");
		return -ENOMEM;
	}

	register_info_oppo = proc_create(OPPO_REGISTER_INFO,0664, oppo_touchpanel_proc, &oppo_register_info_fops);
	if (register_info_oppo == NULL) {
        NVT_ERR("create proc/touchpanel/oppo_register_info Failed!\n");
		return -ENOMEM;
	}

	oppo_fw_update = proc_create(OPPO_FW_UPDATE,0664, oppo_touchpanel_proc, &oppo_fw_update_fops);
	if (oppo_fw_update == NULL) {
        NVT_ERR("create proc/touchpanel/tp_fw_update Failed!\n");
		return -ENOMEM;
	}

	oppo_game_switch = proc_create(OPPO_GAME_SWITCH,0664, oppo_touchpanel_proc, &oppo_game_switch_fops);
	if (oppo_game_switch == NULL) {
        NVT_ERR("create proc/touchpanel/oppo_game_switch_enable Failed!\n");
		return -ENOMEM;
	}

	oppo_tp_limit_enable = proc_create(OPPO_TP_LIMIT_ENABLE,0664, oppo_touchpanel_proc, &oppo_tp_limit_fops);
	if (oppo_tp_limit_enable == NULL) {
        NVT_ERR("create proc/touchpanel/oppo_tp_limit_enable Failed!\n");
		return -ENOMEM;
	}

	NVT_proc_fw_version_entry = proc_create(NVT_FW_VERSION, 0664, NULL, &nvt_fw_version_fops);
	if (NVT_proc_fw_version_entry == NULL) {
		NVT_ERR("create proc/nvt_fw_version Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_fw_version Succeeded!\n");
	}

	NVT_proc_baseline_entry = proc_create(NVT_BASELINE, 0664, NULL,&nvt_baseline_fops);
	if (NVT_proc_baseline_entry == NULL) {
		NVT_ERR("create proc/nvt_baseline Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_baseline Succeeded!\n");
	}

	NVT_proc_raw_entry = proc_create(NVT_RAW, 0664, NULL,&nvt_raw_fops);
	if (NVT_proc_raw_entry == NULL) {
		NVT_ERR("create proc/nvt_raw Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_raw Succeeded!\n");
	}

	NVT_proc_diff_entry = proc_create(NVT_DIFF, 0664, NULL,&nvt_diff_fops);
	if (NVT_proc_diff_entry == NULL) {
		NVT_ERR("create proc/nvt_diff Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_diff Succeeded!\n");
	}

	oppo_incell_tp = proc_create(OPPO_INCELL_PANEL, 0664, oppo_touchpanel_proc,&nvt_incell_fops);
	if (oppo_incell_tp == NULL) {
		NVT_ERR("create proc/nvt_incell Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_incell Succeeded!\n");
	}

	oppo_tp_direction = proc_create(OPPO_TP_DIRECTION, 0664, oppo_touchpanel_proc,&nvt_direction_fops);
	if (oppo_tp_direction == NULL) {
		NVT_ERR("create proc/nvt_direction Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_direction Succeeded!\n");
	}

	return 0;
}






#endif


