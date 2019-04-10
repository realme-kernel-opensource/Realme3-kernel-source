/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** FILE: - mp_test.c
** Description : This program is for ili9881 driver mp_test.c
** Version: 1.0
** Date : 2018/5/17
** Author: Zhonghua.Hu@ODM_WT.BSP.TP-FP.TP
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/fd.h>
#include <linux/file.h>
#include <linux/version.h>
#include <asm/uaccess.h>

#include "../common.h"
#include "../platform.h"
#include "i2c.h"
#include "config.h"
#include "mp_test.h"
#include "protocol.h"
#include "parser.h"
#include "finger_report.h"

#define EXEC_READ  0
#define EXEC_WRITE 1

#define INT_CHECK 0
#define POLL_CHECK 1
#define DELAY_CHECK 2
#define RETRY_COUNT 1

#define NORMAL_CSV_PASS_NAME		"mp_pass"
#define NORMAL_CSV_FAIL_NAME		"mp_fail"
#define OPPO_CSV_PASS_NAME			"oppo_mp_pass"
#define OPPO_CSV_FAIL_NAME			"oppo_mp_fail"
#define OPPO_CSV_LCMOFF_PASS_NAME		"oppo_mp_lcmoff_pass"
#define OPPO_CSV_LCMOFF_FAIL_NAME		"oppo_mp_lcmoff_fail"

#define CSV_FILE_SIZE   (500 * 1024)
int oppo_get_cdc_data(int index);
int core_mp_init(void);
static void mp_test_init_item(void);
#define Mathabs(x) ({						\
		long ret;							\
		if (sizeof(x) == sizeof(long)) {	\
		long __x = (x);						\
		ret = (__x < 0) ? -__x : __x;		\
		} else {							\
		int __x = (x);						\
		ret = (__x < 0) ? -__x : __x;		\
		}									\
		ret;								\
	})

#define DUMP(level, fmt, arg...)		\
	do {								\
		if(level & ipio_debug_level)	\
		printk(fmt, ##arg);				\
	} while (0)


enum open_test_node_type {
	NO_COMPARE = 0x00,  /*Not A Area, No Compare  */
	AA_Area = 0x01,	    /*AA Area, Compare using Charge_AA  */
	Border_Area = 0x02, /*Border Area, Compare using Charge_Border  */
	Notch = 0x04,       /*Notch Area, Compare using Charge_Notch  */
	Round_Corner = 0x08,/* Round Corner, No Compare */
	Skip_Micro = 0x10   /* Skip_Micro, No Compare */
};

/* You must declare a new test in this struct before running a new process of mp test */
struct mp_test_items tItems[] = {
	{.name = "mutual_dac", .desp = "calibration data(dac)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_bg", .desp = "baseline data(bg)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_signal", .desp = "untouch signal data(bg-raw-4096) - mutual", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_no_bk", .desp = "raw data(no bk)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_no_bk_lcm_off", .desp = "raw data(no bk) (lcm off)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_has_bk", .desp = "raw data(have bk)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_has_bk_lcm_off", .desp = "raw data(have bk) (lcm off)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_bk_dac", .desp = "manual bk data(mutual)", .result = "FAIL", .catalog = MUTUAL_TEST},

	{.name = "self_dac", .desp = "calibration data(dac) - self", .result = "FAIL", .catalog = SELF_TEST},
	{.name = "self_bg", .desp = "baselin data(bg,self_tx,self_rx)", .result = "FAIL", .catalog = SELF_TEST},
	{.name = "self_signal", .desp = "untouch signal data(bg–raw-4096) - self", .result = "FAIL", .catalog = SELF_TEST},
	{.name = "self_no_bk", .desp = "raw data(no bk) - self", .result = "FAIL", .catalog = SELF_TEST},
	{.name = "self_has_bk", .desp = "raw data(have bk) - self", .result = "FAIL", .catalog = SELF_TEST},
	{.name = "self_bk_dac", .desp = "manual bk dac data(self_tx,self_rx)", .result = "FAIL", .catalog = SELF_TEST},

	{.name = "key_dac", .desp = "calibration data(dac/tcon)", .result = "FAIL", .catalog = KEY_TEST},
	{.name = "key_bg", .desp = "key baseline data", .result = "FAIL", .catalog = KEY_TEST},
	{.name = "key_no_bk", .desp = "key raw data", .result = "FAIL", .catalog = KEY_TEST},
	{.name = "key_has_bk", .desp = "key raw bk dac", .result = "FAIL", .catalog = KEY_TEST},
	{.name = "key_open", .desp = "key raw open test", .result = "FAIL", .catalog = KEY_TEST},
	{.name = "key_short", .desp = "key raw short test", .result = "FAIL", .catalog = KEY_TEST},

	{.name = "st_dac", .desp = "st calibration data(dac)", .result = "FAIL", .catalog = ST_TEST},
	{.name = "st_bg", .desp = "st baseline data(bg)", .result = "FAIL", .catalog = ST_TEST},
	{.name = "st_no_bk", .desp = "st raw data(no bk)", .result = "FAIL", .catalog = ST_TEST},
	{.name = "st_has_bk", .desp = "st raw(have bk)", .result = "FAIL", .catalog = ST_TEST},
	{.name = "st_open", .desp = "st open data", .result = "FAIL", .catalog = ST_TEST},

	{.name = "tx_short", .desp = "tx short test", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "rx_short", .desp = "short test", .result = "FAIL", .catalog = SHORT_TEST},
	{.name = "rx_open", .desp = "rx open", .result = "FAIL", .catalog = MUTUAL_TEST},

	{.name = "cm_data", .desp = "untouch cm data", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "cs_data", .desp = "untouch cs data", .result = "FAIL", .catalog = MUTUAL_TEST},

	{.name = "tx_rx_delta", .desp = "tx/rx delta", .result = "FAIL", .catalog = TX_RX_DELTA},

	{.name = "p2p", .desp = "untouch peak to peak", .result = "FAIL", .catalog = UNTOUCH_P2P},

	{.name = "pixel_no_bk", .desp = "pixel raw (no bk)", .result = "FAIL", .catalog = PIXEL},
	{.name = "pixel_has_bk", .desp = "pixel raw (have bk)", .result = "FAIL", .catalog = PIXEL},

	{.name = "open_integration", .desp = "open test(integration)", .result = "FAIL", .catalog = OPEN_TEST},
	{.name = "open_integration_sp", .desp = "open test(integration)_sp", .result = "FAIL", .catalog = OPEN_TEST},
	{.name = "open_cap", .desp = "open test(cap)", .result = "FAIL", .catalog = OPEN_TEST},

	{.name = "noise_peak_to_peak_ic", .desp = "noise peak to peak(ic only)", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},
	{.name = "noise_peak_to_peak_panel", .desp = "noise peak to peak(with panel)", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},
	{.name = "noise_peak_to_peak_ic_lcm_off", .desp = "noise peak to peak(ic only) (lcm off)", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},
	{.name = "noise_peak_to_peak_panel_lcm_off", .desp = "noise peak to peak(with panel) (lcm off)", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},

	{.name = "doze_raw", .desp = "doze raw data", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "doze_p2p", .desp = "doze peak to peak", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},
	{.name = "doze_raw_td_lcm_off", .desp = "raw data_td (lcm off)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "doze_p2p_td_lcm_off", .desp = "peak to peak_td (lcm off)", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},
};

int32_t *frame_buf = NULL;
int32_t *key_buf = NULL;
int32_t *frame1_cbk700 = NULL, *frame1_cbk250 = NULL, *frame1_cbk200 = NULL;
struct core_mp_test_data *core_mp = NULL;
int mp_test_failed_items = 0;

void dump_data(void *data, int type, int len, int row_len, const char *name)
{
	int i, row = 31;
	uint8_t *p8 = NULL;
	int32_t *p32 = NULL;

	if (row_len > 0)
		row = row_len;

	if (ipio_debug_level & DEBUG_MP_TEST) {
		if (data == NULL) {
			ipio_err("The data going to dump is NULL\n");
			return;
		}

		ipio_debug(DEBUG_MP_TEST, "\n\n");
		ipio_debug(DEBUG_MP_TEST, "Dump %s data\n", name);

		if (type == 8)
			p8 = (uint8_t *) data;
		if (type == 32 || type == 10)
			p32 = (int32_t *) data;

		for (i = 0; i < len; i++) {
			if (type == 8)
				ipio_debug(DEBUG_MP_TEST, " %4x ", p8[i]);
			else if (type == 32)
				ipio_debug(DEBUG_MP_TEST, " %4x ", p32[i]);
			else if (type == 10)
				ipio_debug(DEBUG_MP_TEST, " %4d ", p32[i]);
			if ((i % row) == row-1)
				ipio_debug(DEBUG_MP_TEST, "\n");
		}
		ipio_debug(DEBUG_MP_TEST, "\n\n");
	}
}
EXPORT_SYMBOL(dump_data);

static void dump_benchmark_data(int32_t* max_ptr, int32_t* min_ptr)
{
	int i;

	if (ipio_debug_level & DEBUG_MP_TEST) {
		ipio_debug(DEBUG_MP_TEST, "benchmark max\n");

		for(i = 0; i < core_mp->frame_len; i++) {
			ipio_debug(DEBUG_MP_TEST, "%d, ", max_ptr[i]);
			if(i % core_mp->xch_len == core_mp->xch_len - 1)
				ipio_debug(DEBUG_MP_TEST, "\n");
		}

		ipio_debug(DEBUG_MP_TEST, "benchmark min\n");

		for(i = 0; i < core_mp->frame_len; i++) {
			ipio_debug(DEBUG_MP_TEST, "%d, ", min_ptr[i]);
			if(i % core_mp->xch_len == core_mp->xch_len - 1)
				ipio_debug(DEBUG_MP_TEST, "\n");
		}
	}
}

void dump_node_type_buffer(int32_t* node_ptr, uint8_t *name)
{
	int i;

	if (ipio_debug_level & DEBUG_MP_TEST) {
		ipio_debug(DEBUG_MP_TEST, "Node Type buffer: %s\n", name);
		for (i = 0; i < core_mp->frame_len; i++) {
			ipio_debug(DEBUG_MP_TEST, "%d, ",node_ptr[i]);
			if(i % core_mp->xch_len == core_mp->xch_len-1)
				ipio_debug(DEBUG_MP_TEST, "\n");
		}
	}

}

static void mp_compare_cdc_result(int index, int32_t *tmp, int32_t *max_ts, int32_t *min_ts, int* result)
{
	int x, y;

	if (ERR_ALLOC_MEM(tmp)) {
		ipio_err("The data of test item is null (%p)\n", tmp);
		*result = MP_FAIL;
		return;
	}

	for (y = 0; y < core_mp->ych_len; y++) {
		for (x = 0; x < core_mp->xch_len; x++) {
			int shift = y * core_mp->xch_len + x;
			if (tItems[index].catalog == SHORT_TEST) {
				if (tmp[shift] < min_ts[shift]){
					*result = MP_FAIL;
				}
			} else {
				if (tmp[shift] > max_ts[shift] || tmp[shift] < min_ts[shift]){
					*result = MP_FAIL;
				}
			}
		}
	}
}


static void mp_compare_cdc_show_result(int index, int32_t *tmp, char *csv, int *csv_len, int type, int32_t *max_ts, int32_t *min_ts, const char *desp)
{
	int x, y, tmp_len = *csv_len;
	int mp_result = MP_PASS;

	if (ERR_ALLOC_MEM(tmp)) {
		ipio_err("The data of test item is null (%p)\n", tmp);
		mp_result = MP_FAIL;
		goto out;
	}

	/* print X raw only */

	ipio_info("mp_compare_cdc_show_result: print X raw only\n");

	for (x = 0; x < core_mp->xch_len; x++) {
		if (x == 0) {
			DUMP(DEBUG_MP_TEST, "\n %s ", desp);
			tmp_len += sprintf(csv + tmp_len, "\n      %s ,", desp);
		}

		DUMP(DEBUG_MP_TEST, "  X_%d  ,", (x+1));
		tmp_len += sprintf(csv + tmp_len, "  X_%d  ,", (x+1));
	}

	DUMP(DEBUG_MP_TEST, "\n");
	tmp_len += sprintf(csv + tmp_len, "\n");

	ipio_info("mp_compare_cdc_show_result: core_mp->ych_len\n");

	for (y = 0; y < core_mp->ych_len; y++) {
		DUMP(DEBUG_MP_TEST, "  Y_%d  ,", (y+1));
		tmp_len += sprintf(csv + tmp_len, "  Y_%d  ,", (y+1));

		for (x = 0; x < core_mp->xch_len; x++) {
			int shift = y * core_mp->xch_len + x;

			/* In Short teset, we only identify if its value is low than min threshold. */
			if (tItems[index].catalog == SHORT_TEST) {
				if (tmp[shift] < min_ts[shift]) {
					DUMP(DEBUG_MP_TEST, " #%7d ", tmp[shift]);
					tmp_len += sprintf(csv + tmp_len, "#%7d,", tmp[shift]);
					mp_result = MP_FAIL;
				} else {
					DUMP(DEBUG_MP_TEST, " %7d ", tmp[shift]);
					tmp_len += sprintf(csv + tmp_len, " %7d, ", tmp[shift]);
				}
				continue;
			}

			if ((tmp[shift] <= max_ts[shift] && tmp[shift] >= min_ts[shift]) || (type != TYPE_JUGE)){

				if((tmp[shift] == INT_MAX || tmp[shift] == INT_MIN) && (type == TYPE_BENCHMARK)){
					DUMP(DEBUG_MP_TEST, "%s", "BYPASS,");
					tmp_len += sprintf(csv + tmp_len,"BYPASS,");
				}
				else{
					DUMP(DEBUG_MP_TEST, " %7d ", tmp[shift]);
					tmp_len += sprintf(csv + tmp_len, " %7d, ", tmp[shift]);
				}
			} else {
				if (tmp[shift] > max_ts[shift]) {
					DUMP(DEBUG_MP_TEST, " *%7d ", tmp[shift]);
					tmp_len += sprintf(csv + tmp_len, "*%7d,", tmp[shift]);
				} else {
					DUMP(DEBUG_MP_TEST, " #%7d ", tmp[shift]);
					tmp_len += sprintf(csv + tmp_len, "#%7d,", tmp[shift]);
				}
				mp_result = MP_FAIL;
			}
		}

		DUMP(DEBUG_MP_TEST, "\n");
		tmp_len += sprintf(csv + tmp_len, "\n");
	}

out:

	ipio_info("mp_compare_cdc_show_result: begin out\n");
	if (type == TYPE_JUGE) {
		if (mp_result == MP_PASS){
			pr_info("\n Result : PASS\n");
			tmp_len += sprintf(csv + tmp_len, "Result : PASS\n");
		} else {
			pr_info("\n Result : FAIL\n");
			tmp_len += sprintf(csv + tmp_len, "Result : FAIL\n");
		}
	}

	*csv_len = tmp_len;
}

static int create_mp_test_frame_buffer(int index, int frame_count)
{
	//this item aready created buffer
	if(tItems[index].test_count != 0)
		return 0;

	ipio_debug(DEBUG_MP_TEST, "Create MP frame buffers (index = %d), count = %d\n",index, frame_count);

	if (tItems[index].catalog == TX_RX_DELTA) {
		core_mp->tx_delta_buf = vmalloc(core_mp->frame_len * sizeof(int32_t));
		core_mp->rx_delta_buf = vmalloc(core_mp->frame_len * sizeof(int32_t));
		core_mp->tx_max_buf = vmalloc(core_mp->frame_len * sizeof(int32_t));
		core_mp->tx_min_buf = vmalloc(core_mp->frame_len * sizeof(int32_t));
		core_mp->rx_max_buf = vmalloc(core_mp->frame_len * sizeof(int32_t));
		core_mp->rx_min_buf = vmalloc(core_mp->frame_len * sizeof(int32_t));

		if (ERR_ALLOC_MEM(core_mp->tx_delta_buf) || ERR_ALLOC_MEM(core_mp->rx_delta_buf)) {
			ipio_err("Failed to allocate Tx/Rx Delta buffer\n");
			return -ENOMEM;
		}

		if (ERR_ALLOC_MEM(core_mp->tx_max_buf) || ERR_ALLOC_MEM(core_mp->tx_min_buf)) {
			ipio_err("Failed to allocate Tx Max/Min buffer\n");
			return -ENOMEM;
		}

		if (ERR_ALLOC_MEM(core_mp->rx_max_buf) || ERR_ALLOC_MEM(core_mp->rx_min_buf)) {
			ipio_err("Failed to allocate Rx Max/Min buffe\n");
			return -ENOMEM;
		}

	} else {
		if (tItems[index].buf == NULL && tItems[index].max_buf == NULL
				&& tItems[index].min_buf == NULL && tItems[index].result_buf == NULL) {
		tItems[index].buf = vmalloc(frame_count * core_mp->frame_len * sizeof(int32_t));
		tItems[index].result_buf = vmalloc(core_mp->frame_len * sizeof(int32_t));
		tItems[index].max_buf = vmalloc(core_mp->frame_len * sizeof(int32_t));
		tItems[index].min_buf = vmalloc(core_mp->frame_len * sizeof(int32_t));

		}
		if (tItems[index].spec_option == BENCHMARK) {
			if (tItems[index].bench_mark_max == NULL && tItems[index].bench_mark_min == NULL) {
			tItems[index].bench_mark_max = vmalloc(core_mp->frame_len * sizeof(int32_t) * 2);
			tItems[index].bench_mark_min = vmalloc(core_mp->frame_len * sizeof(int32_t) * 2);
			}

			if (ERR_ALLOC_MEM(tItems[index].bench_mark_max) || ERR_ALLOC_MEM(tItems[index].bench_mark_min)){
				ipio_err("Failed to allocate bench_mark FRAME buffer\n");
				return -ENOMEM;
			}
		}
		if (ERR_ALLOC_MEM(tItems[index].buf) || ERR_ALLOC_MEM(tItems[index].max_buf) ||
				ERR_ALLOC_MEM(tItems[index].min_buf) || ERR_ALLOC_MEM(tItems[index].result_buf)) {
			ipio_err("Failed to allocate FRAME buffer\n");
			return -ENOMEM;
		}
	}

	return 0;
}

int core_mp_ctrl_lcm_status(bool on)
{
	int ret = 0, ctrl = 0, delay = 0;
	uint8_t lcd[15] = {0};
	uint8_t header = 0x0F;

	memset(&lcd, 0xFF, ARRAY_SIZE(lcd));

	ctrl = ((on) ? 1 : 2);
	delay = ((on) ? 100 : 10);

	lcd[0] = header;
	lcd[1] = protocol->mutual_bg;
	lcd[2] = 0;
	lcd[3] = ctrl;

	dump_data(lcd, 8, ARRAY_SIZE(lcd), 0, "LCM Command");

	ret = core_write(core_config->slave_i2c_addr, lcd, ARRAY_SIZE(lcd));
	if (ret < 0) {
		ipio_err("Failed to write LCM command\n");
		goto out;
	}

	mdelay(delay);

out:
	return ret;
}
EXPORT_SYMBOL(core_mp_ctrl_lcm_status);

static int allnode_key_cdc_data(int index)
{
	int i, res = 0, len = 0;
	int inDACp = 0, inDACn = 0;
	uint8_t cmd[3] = { 0 };
	uint8_t *ori = NULL;

	len = core_mp->key_len * 2;

	ipio_debug(DEBUG_MP_TEST, "Read key's length = %d\n", len);
	ipio_debug(DEBUG_MP_TEST, "core_mp->key_len = %d\n", core_mp->key_len);

	if (len <= 0) {
		ipio_err("Length is invalid\n");
		res = -1;
		goto out;
	}

	/* CDC init */
	cmd[0] = protocol->cmd_cdc;
	cmd[1] = tItems[index].cmd;
	cmd[2] = 0;

	res = core_write(core_config->slave_i2c_addr, cmd, 3);
	if (res < 0) {
		ipio_err("I2C Write Error while initialising cdc\n");
		goto out;
	}

	msleep(1);

	/* Check busy */
	if (core_config_check_cdc_busy(50, 50) < 0) {
		ipio_err("Check busy is timout !\n");
		res = -1;
		goto out;
	}

	msleep(1);

	/* Prepare to get cdc data */
	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_cdc;

	res = core_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0) {
		ipio_err("I2C Write Error\n");
		goto out;
	}

	msleep(1);

	res = core_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0) {
		ipio_err("I2C Write Error\n");
		goto out;
	}

	/* Allocate a buffer for the original */
	ori = kcalloc(len, sizeof(uint8_t), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ori)) {
		ipio_err("Failed to allocate ori mem (%ld)\n", PTR_ERR(ori));
		goto out;
	}

	msleep(1);

	/* Get original frame(cdc) data */
	res = core_read(core_config->slave_i2c_addr, ori, len);
	if (res < 0) {
		ipio_err("I2C Read Error while getting original cdc data\n");
		goto out;
	}

	dump_data(ori, 8, len, 0, "Key CDC original");

	if (key_buf == NULL) {
		key_buf = kcalloc(core_mp->key_len, sizeof(int32_t), GFP_KERNEL);
		if (ERR_ALLOC_MEM(key_buf)) {
			ipio_err("Failed to allocate FrameBuffer mem (%ld)\n", PTR_ERR(key_buf));
			goto out;
		}
	} else {
		memset(key_buf, 0x0, core_mp->key_len);
	}

	/* Convert original data to the physical one in each node */
	for (i = 0; i < core_mp->frame_len; i++) {
		if (tItems[index].cmd == protocol->key_dac) {
			/* DAC - P */
			if (((ori[(2 * i) + 1] & 0x80) >> 7) == 1) {
				/* Negative */
				inDACp = 0 - (int)(ori[(2 * i) + 1] & 0x7F);
			} else {
				inDACp = ori[(2 * i) + 1] & 0x7F;
			}

			/* DAC - N */
			if (((ori[(1 + (2 * i)) + 1] & 0x80) >> 7) == 1) {
				/* Negative */
				inDACn = 0 - (int)(ori[(1 + (2 * i)) + 1] & 0x7F);
			} else {
				inDACn = ori[(1 + (2 * i)) + 1] & 0x7F;
			}

			key_buf[i] = (inDACp + inDACn) / 2;
		}
	}

	dump_data(key_buf, 32, core_mp->frame_len, core_mp->xch_len, "Key CDC combined data");

out:
	ipio_kfree((void **)&ori);
	return res;
}

int mp_get_timing_info(void)
{
	int slen = 0;
	char str[256] = {0};
	uint8_t info[64] = {0};
	char *key = "timing_info_raw";

	core_mp->isLongV = 0;

	slen = core_parser_get_int_data("pv5_4 command", key, str);
	if (slen < 0)
		return -1;

	if (core_parser_get_u8_array(str, info, 16, slen) < 0)
		return -1;

	core_mp->isLongV = info[6];

	ipio_info("DDI Mode = %s\n", (core_mp->isLongV ? "Long V" : "Long H"));

	return 0;
}

static int mp_cdc_get_pv5_4_command(uint8_t *cmd, int len, int index)
{
	int ret = 0;
	char str[128] = {0};
	char tmp[128] = {0};
	char *key = tItems[index].desp;

	ipio_debug(DEBUG_MP_TEST, "%s gets %s command from INI.\n", tItems[index].desp, key);

	ret = core_parser_get_int_data("pv5_4 command", key, str);
	if (ret < 0) {
		ipio_err("Failed to parse PV54 command, ret = %d\n",ret);
		goto out;
	}

	strncpy(tmp, str, ret);
	core_parser_get_u8_array(tmp, cmd, 16, len);

out:
	return ret;
}

static int mp_cdc_init_cmd_common(uint8_t *cmd, int len, int index)
{
	int ret = 0;

	if (protocol->major >= 5 && protocol->mid >= 4) {
		ipio_debug(DEBUG_MP_TEST, "Get CDC command with protocol v5.4\n");
		protocol->cdc_len = 15;
		return mp_cdc_get_pv5_4_command(cmd, len, index);
	}

	cmd[0] = protocol->cmd_cdc;
	cmd[1] = tItems[index].cmd;
	cmd[2] = 0;

	protocol->cdc_len = 3;

	if (strcmp(tItems[index].name, "open_integration") == 0)
		cmd[2] = 0x2;
	if (strcmp(tItems[index].name, "open_cap") == 0)
		cmd[2] = 0x3;

	if(tItems[index].catalog == PEAK_TO_PEAK_TEST) {
		cmd[2] = ((tItems[index].frame_count & 0xff00) >> 8);
		cmd[3] = tItems[index].frame_count & 0xff;
		cmd[4] = 0;

		protocol->cdc_len = 5;

		if (strcmp(tItems[index].name, "noise_peak_to_peak_cut") == 0)
			cmd[4] = 0x1;

		ipio_debug(DEBUG_MP_TEST, "P2P CMD: %d,%d,%d,%d,%d\n",
				cmd[0],cmd[1],cmd[2],cmd[3],cmd[4]);
	}

	return ret;
}

static int allnode_mutual_cdc_data(int index)
{
	static int i = 0, res = 0, len = 0;
	int inDACp = 0, inDACn = 0;
	static uint8_t cmd[15] = {0};
	uint8_t *ori = NULL;

	/* Multipling by 2 is due to the 16 bit in each node */
	len = (core_mp->xch_len * core_mp->ych_len * 2) + 2;

	ipio_debug(DEBUG_MP_TEST, "Read X/Y Channel length = %d\n", len);
	ipio_debug(DEBUG_MP_TEST, "core_mp->frame_len = %d\n", core_mp->frame_len);

	if (len <= 2) {
		ipio_err("Length is invalid\n");
		res = -1;
		goto out;
	}

	memset(cmd, 0xFF, protocol->cdc_len);

	/* CDC init */
	mp_cdc_init_cmd_common(cmd, protocol->cdc_len, index);

	dump_data(cmd, 8, protocol->cdc_len, 0, "Mutual CDC command");
	ipio_info("CDC CMD 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n",\
		cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7]);
	res = core_write(core_config->slave_i2c_addr, cmd, protocol->cdc_len);
	if (res < 0) {
		ipio_err("I2C Write Error while initialising cdc\n");
		goto out;
	}

	/* Check busy */
	ipio_debug(DEBUG_MP_TEST, "Check busy method = %d\n",core_mp->busy_cdc);
	if (core_mp->busy_cdc == POLL_CHECK) {
		res = core_config_check_cdc_busy(50, 50);
	} else if (core_mp->busy_cdc == INT_CHECK) {
		res = core_config_check_int_status(false);
	} else if (core_mp->busy_cdc == DELAY_CHECK) {
		mdelay(600);
	}

	if (res < 0) {
		ipio_err("Check busy timeout !\n");
		res = -1;
		goto out;
	}

	/* Prepare to get cdc data */
	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_cdc;

	res = core_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0) {
		ipio_err("I2C Write Error\n");
		goto out;
	}

	mdelay(1);

	res = core_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0) {
		ipio_err("I2C Write Error\n");
		goto out;
	}

	mdelay(1);

	/* Allocate a buffer for the original */
	ori = kcalloc(len, sizeof(uint8_t), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ori)) {
		ipio_err("Failed to allocate ori mem (%ld)\n", PTR_ERR(ori));
		goto out;
	}

	/* Get original frame(cdc) data */
	res = core_read(core_config->slave_i2c_addr, ori, len);
	if (res < 0) {
		ipio_err("I2C Read Error while getting original cdc data\n");
		goto out;
	}

	dump_data(ori, 8, len, 0, "Mutual CDC original");

	if (frame_buf == NULL) {
		frame_buf = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		if (ERR_ALLOC_MEM(frame_buf)) {
			ipio_err("Failed to allocate FrameBuffer mem (%ld)\n", PTR_ERR(frame_buf));
			goto out;
		}
	} else {
		memset(frame_buf, 0x0, core_mp->frame_len);
	}

	/* Convert original data to the physical one in each node */
	for (i = 0; i < core_mp->frame_len; i++) {
		if (tItems[index].cmd == protocol->mutual_dac) {
			/* DAC - P */
			if (((ori[(2 * i) + 1] & 0x80) >> 7) == 1) {
				/* Negative */
				inDACp = 0 - (int)(ori[(2 * i) + 1] & 0x7F);
			} else {
				inDACp = ori[(2 * i) + 1] & 0x7F;
			}

			/* DAC - N */
			if (((ori[(1 + (2 * i)) + 1] & 0x80) >> 7) == 1) {
				/* Negative */
				inDACn = 0 - (int)(ori[(1 + (2 * i)) + 1] & 0x7F);
			} else {
				inDACn = ori[(1 + (2 * i)) + 1] & 0x7F;
			}

			frame_buf[i] = (inDACp + inDACn) / 2;
		} else {
			/* H byte + L byte */
			int32_t tmp = (ori[(2 * i) + 1] << 8) + ori[(1 + (2 * i)) + 1];

			if ((tmp & 0x8000) == 0x8000)
				frame_buf[i] = tmp - 65536;
			else
				frame_buf[i] = tmp;

			if(strncmp(tItems[index].name, "mutual_no_bk", strlen("mutual_no_bk")) == 0 ||
				strncmp(tItems[index].name, "mutual_no_bk_lcm_off", strlen("mutual_no_bk_lcm_off")) == 0) {
				if (core_config->chip_id == CHIP_TYPE_ILI9881) {
					if(core_config->chip_type == ILI9881_TYPE_H)
						frame_buf[i] -= RAWDATA_NO_BK_DATA_SHIFT_9881H;
					if(core_config->chip_type == ILI9881_TYPE_F)
						frame_buf[i] -= RAWDATA_NO_BK_DATA_SHIFT_9881F;
				}
			}
		}
	}

	dump_data(frame_buf, 32, core_mp->frame_len,  core_mp->xch_len, "Mutual CDC combined");

out:
	ipio_kfree((void **)&ori);
	return res;
}

static void run_pixel_test(int index)
{
	int i, x, y;
	int32_t *p_comb = frame_buf;

	for (y = 0; y < core_mp->ych_len; y++) {
		for (x = 0; x < core_mp->xch_len; x++) {
			int tmp[4] = { 0 }, max = 0;
			int shift = y * core_mp->xch_len;
			int centre = p_comb[shift + x];

			/* if its position is in corner, the number of point
			   we have to minus is around 2 to 3.  */
			if (y == 0 && x == 0) {
				tmp[0] = Mathabs(centre - p_comb[(shift + 1) + x]);	/* down */
				tmp[1] = Mathabs(centre - p_comb[shift + (x + 1)]);	/* right */
			} else if (y == (core_mp->ych_len - 1) && x == 0) {
				tmp[0] = Mathabs(centre - p_comb[(shift - 1) + x]);	/* up */
				tmp[1] = Mathabs(centre - p_comb[shift + (x + 1)]);	/* right */
			} else if (y == 0 && x == (core_mp->xch_len - 1)) {
				tmp[0] = Mathabs(centre - p_comb[(shift + 1) + x]);	/* down */
				tmp[1] = Mathabs(centre - p_comb[shift + (x - 1)]);	/* left */
			} else if (y == (core_mp->ych_len - 1) && x == (core_mp->xch_len - 1)) {
				tmp[0] = Mathabs(centre - p_comb[(shift - 1) + x]);	/* up */
				tmp[1] = Mathabs(centre - p_comb[shift + (x - 1)]);	/* left */
			} else if (y == 0 && x != 0) {
				tmp[0] = Mathabs(centre - p_comb[(shift + 1) + x]);	/* down */
				tmp[1] = Mathabs(centre - p_comb[shift + (x - 1)]);	/* left */
				tmp[2] = Mathabs(centre - p_comb[shift + (x + 1)]);	/* right */
			} else if (y != 0 && x == 0) {
				tmp[0] = Mathabs(centre - p_comb[(shift - 1) + x]);	/* up */
				tmp[1] = Mathabs(centre - p_comb[shift + (x + 1)]);	/* right */
				tmp[2] = Mathabs(centre - p_comb[(shift + 1) + x]);	/* down */

			} else if (y == (core_mp->ych_len - 1) && x != 0) {
				tmp[0] = Mathabs(centre - p_comb[(shift - 1) + x]);	/* up */
				tmp[1] = Mathabs(centre - p_comb[shift + (x - 1)]);	/* left */
				tmp[2] = Mathabs(centre - p_comb[shift + (x + 1)]);	/* right */
			} else if (y != 0 && x == (core_mp->xch_len - 1)) {
				tmp[0] = Mathabs(centre - p_comb[(shift - 1) + x]);	/* up */
				tmp[1] = Mathabs(centre - p_comb[shift + (x - 1)]);	/* left */
				tmp[2] = Mathabs(centre - p_comb[(shift + 1) + x]);	/* down */
			} else {
				/* middle minus four directions */
				tmp[0] = Mathabs(centre - p_comb[(shift - 1) + x]);	/* up */
				tmp[1] = Mathabs(centre - p_comb[(shift + 1) + x]);	/* down */
				tmp[2] = Mathabs(centre - p_comb[shift + (x - 1)]);	/* left */
				tmp[3] = Mathabs(centre - p_comb[shift + (x + 1)]);	/* right */
			}

			max = tmp[0];

			for (i = 0; i < 4; i++) {
				if (tmp[i] > max)
					max = tmp[i];
			}

			tItems[index].buf[shift + x] = max;
		}
	}
}

static int run_open_test(int index)
{
	int i, x, y, k, res = 0;
	int border_x[] = {-1, 0, 1, 1, 1, 0, -1, -1};
	int border_y[] = {-1, -1, -1, 0, 1, 1, 1, 0};
	int32_t *p_comb = frame_buf;

	if (strcmp(tItems[index].name, "open_integration") == 0) {
		for (i = 0; i < core_mp->frame_len; i++)
			tItems[index].buf[i] = p_comb[i];
	} else if (strcmp(tItems[index].name, "open_cap") == 0) {
		/* Each result is getting from a 3 by 3 grid depending on where the centre location is.
		   So if the centre is at corner, the number of node grabbed from a grid will be different. */
		for (y = 0; y < core_mp->ych_len; y++) {
			for (x = 0; x < core_mp->xch_len; x++) {
				int sum = 0, avg = 0, count = 0;
				int shift = y * core_mp->xch_len;
				int centre = p_comb[shift + x];

				for (k = 0; k < 8; k++) {
					if (((y + border_y[k] >= 0) && (y + border_y[k] < core_mp->ych_len)) &&
								((x + border_x[k] >= 0) && (x + border_x[k] < core_mp->xch_len))) {
						count++;
						sum += p_comb[(y + border_y[k]) * core_mp->xch_len + (x + border_x[k])];
					}
				}

				avg = (sum + centre) / (count + 1);	/* plus 1 because of centre */
				tItems[index].buf[shift + x] = (centre * 100) / avg;
			}
		}
	}
	return res;
}

static void run_tx_rx_delta_test(int index)
{
	int x, y;
	int32_t *p_comb = frame_buf;

	for (y = 0; y < core_mp->ych_len; y++) {
		for (x = 0; x < core_mp->xch_len; x++) {
			int shift = y * core_mp->xch_len;

			/* Tx Delta */
			if (y != (core_mp->ych_len - 1)) {
				core_mp->tx_delta_buf[shift + x] = Mathabs(p_comb[shift + x] - p_comb[(shift + 1) + x]);
			}

			/* Rx Delta */
			if (x != (core_mp->xch_len - 1)) {
				core_mp->rx_delta_buf[shift + x] = Mathabs(p_comb[shift + x] - p_comb[shift + (x + 1)]);
			}
		}
	}
}

static void run_untouch_p2p_test(int index)
{
	int x, y;
	int32_t *p_comb = frame_buf;

	for (y = 0; y < core_mp->ych_len; y++) {
		for (x = 0; x < core_mp->xch_len; x++) {
			int shift = y * core_mp->xch_len;

			if (p_comb[shift + x] > tItems[index].max_buf[shift + x]) {
				tItems[index].max_buf[shift + x] = p_comb[shift + x];
			}

			if (p_comb[shift + x] < tItems[index].min_buf[shift + x]) {
				tItems[index].min_buf[shift + x] = p_comb[shift + x];
			}

			tItems[index].buf[shift + x] =
			    tItems[index].max_buf[shift + x] - tItems[index].min_buf[shift + x];
		}
	}
}

static void compare_MaxMin_result(int index, int32_t *data)
{
	int x, y;

	for (y = 0; y < core_mp->ych_len; y++) {
		for (x = 0; x < core_mp->xch_len; x++) {
			int shift = y * core_mp->xch_len;

			if (tItems[index].catalog == UNTOUCH_P2P)
				return;
			else if (tItems[index].catalog == TX_RX_DELTA) {
				/* Tx max/min comparison */
				if (core_mp->tx_delta_buf[shift + x] < data[shift + x]) {
					core_mp->tx_max_buf[shift + x] = data[shift + x];
				}

				if (core_mp->tx_delta_buf[shift + x] > data[shift + x]) {
					core_mp->tx_min_buf[shift + x] = data[shift + x];
				}

				/* Rx max/min comparison */
				if (core_mp->rx_delta_buf[shift + x] < data[shift + x]) {
					core_mp->rx_max_buf[shift + x] = data[shift + x];
				}

				if (core_mp->rx_delta_buf[shift + x] > data[shift + x]) {
					core_mp->rx_min_buf[shift + x] = data[shift + x];
				}
			} else {
				if (tItems[index].max_buf[shift + x] < data[shift + x]) {
					tItems[index].max_buf[shift + x] = data[shift + x];
				}

				if (tItems[index].min_buf[shift + x] > data[shift + x]) {
					tItems[index].min_buf[shift + x] = data[shift + x];
				}
			}
		}
	}
}


#define ABS(a,b) ( (a>b) ? (a-b) : (b-a) )
#define ADDR(x,y) ((y*core_mp->xch_len)+(x))

int full_open_rate_compare(int32_t* full_open, int32_t* cbk, int x, int y, int32_t inNodeType, int full_open_rate)
{
	int ret = true;

	if ((inNodeType == NO_COMPARE) || ((inNodeType & Round_Corner) == Round_Corner)) {
		return true;
	}

	if(full_open[ADDR(x,y)] < (cbk[ADDR(x,y)] * full_open_rate / 100))
		ret = false;

	return ret;
}


int compare_charge(int32_t* charge_rate, int x, int y, int32_t* inNodeType, int Charge_AA, int Charge_Border, int Charge_Notch)
{
	int OpenThreadhold,tempY, tempX, ret, k;
	int sx[8] = { -1, 0, 1, -1, 1, -1, 0, 1 };
	int sy[8] = { -1, -1, -1, 0, 0, 1, 1, 1 };

	ret = charge_rate[ADDR(x, y)];

	/*Setting Threadhold from node type  */

	if(charge_rate[ADDR(x, y)] == 0)
		return ret;
	else if ((inNodeType[ADDR(x, y)] & AA_Area) == AA_Area)
		OpenThreadhold = Charge_AA;
	else if ((inNodeType[ADDR(x, y)] & Border_Area) == Border_Area)
		OpenThreadhold = Charge_Border;
	else if ((inNodeType[ADDR(x, y)] & Notch) == Notch)
		OpenThreadhold = Charge_Notch;
	else
		return ret;

	/*compare carge rate with 3*3 node*/
	/*by pass => 1.no compare 2.corner 3.Skip_Micro 4.full open fail node*/
	for (k = 0; k < 8; k++) {
		tempX = x + sx[k];
		tempY = y + sy[k];

		if ((tempX < 0) || (tempX >= core_mp->xch_len) || (tempY < 0) || (tempY >= core_mp->ych_len)) /*out of range */
			continue;

		if ((inNodeType[ADDR(tempX, tempY)] == NO_COMPARE) || ((inNodeType[ADDR(tempX, tempY)] & Round_Corner) == Round_Corner) ||
		((inNodeType[ADDR(tempX, tempY)] & Skip_Micro) == Skip_Micro) || charge_rate[ADDR(tempX, tempY)] == 0)
			continue;

		if ((charge_rate[ADDR(tempX, tempY)]-charge_rate[ADDR(x, y)])> OpenThreadhold)
			return OpenThreadhold;
	}
	return ret;
}

/* This will be merged to allnode_mutual_cdc_data in next version */
int allnode_open_cdc_data(int mode, int *buf, int *dac)
{
	int i = 0, res = 0, len = 0;
	int inDACp = 0, inDACn = 0;
	uint8_t cmd[15] = {0};
	uint8_t *ori = NULL;
	char str[128] = {0};
	char tmp[128] = {0};
	char *key[] = {"open dac", "open raw1", "open raw2", "open raw3"};

	/* Multipling by 2 is due to the 16 bit in each node */
	len = (core_mp->xch_len * core_mp->ych_len * 2) + 2;

	ipio_debug(DEBUG_MP_TEST, "Read X/Y Channel length = %d\n", len);
	ipio_debug(DEBUG_MP_TEST, "core_mp->frame_len = %d, mode= %d\n", core_mp->frame_len, mode);

	if (len <= 2) {
		ipio_err("Length is invalid\n");
		res = -1;
		goto out;
	}

	/* CDC init. Read command from ini file */
	res = core_parser_get_int_data("pv5_4 command", key[mode], str);
	if (res < 0) {
		ipio_err("Failed to parse PV54 command, res = %d\n",res);
		goto out;
	}

	strncpy(tmp, str, res);
	core_parser_get_u8_array(tmp, cmd, 16, sizeof(cmd));

	dump_data(cmd, 8, sizeof(cmd), 0, "Open SP command");

	res = core_write(core_config->slave_i2c_addr, cmd, protocol->cdc_len);
	if (res < 0) {
		ipio_err("I2C Write Error while initialising cdc\n");
		goto out;
	}

	/* Check busy */
	ipio_debug(DEBUG_MP_TEST, "Check busy method = %d\n",core_mp->busy_cdc);
	if (core_mp->busy_cdc == POLL_CHECK) {
		res = core_config_check_cdc_busy(50, 50);
	} else if (core_mp->busy_cdc == INT_CHECK) {
		res = core_config_check_int_status(false);
	} else if (core_mp->busy_cdc == DELAY_CHECK) {
		mdelay(600);
	}

	if (res < 0) {
		ipio_err("Check busy timeout !\n");
		res = -1;
		goto out;
	}

	/* Prepare to get cdc data */
	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_cdc;

	res = core_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0) {
		ipio_err("I2C Write Error\n");
		goto out;
	}

	mdelay(1);

	res = core_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0) {
		ipio_err("I2C Write Error\n");
		goto out;
	}

	mdelay(1);

	/* Allocate a buffer for the original */
	ori = kcalloc(len, sizeof(uint8_t), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ori)) {
		ipio_err("Failed to allocate ori mem (%ld)\n", PTR_ERR(ori));
		goto out;
	}

	/* Get original frame(cdc) data */
	res = core_read(core_config->slave_i2c_addr, ori, len);
	if (res < 0) {
		ipio_err("I2C Read Error while getting original cdc data\n");
		goto out;
	}

	dump_data(ori, 8, len, 0, "Open SP CDC original");

	/* Convert original data to the physical one in each node */
	for (i = 0; i < core_mp->frame_len; i++) {
		if (mode == 0) {
			/* DAC - P */
			if (((ori[(2 * i) + 1] & 0x80) >> 7) == 1) {
				/* Negative */
				inDACp = 0 - (int)(ori[(2 * i) + 1] & 0x7F);
			} else {
				inDACp = ori[(2 * i) + 1] & 0x7F;
			}

			/* DAC - N */
			if (((ori[(1 + (2 * i)) + 1] & 0x80) >> 7) == 1) {
				/* Negative */
				inDACn = 0 - (int)(ori[(1 + (2 * i)) + 1] & 0x7F);
			} else {
				inDACn = ori[(1 + (2 * i)) + 1] & 0x7F;
			}

			buf[i] = (inDACp + inDACn) / 2;
		} else {
			/* H byte + L byte */
			int32_t tmp = (ori[(2 * i) + 1] << 8) + ori[(1 + (2 * i)) + 1];
			if ((tmp & 0x8000) == 0x8000)
				buf[i] = tmp - 65536;
			else
				buf[i] = tmp;
			buf[i] = (int)((int)(dac[i] * 2 * 10000 * 161 / 100) - (int)(16384 / 2 - (int)buf[i]) * 20000 * 7 / 16384 * 36 / 10) / 31 / 2;
		}
	}
	dump_data(buf, 10, core_mp->frame_len,  core_mp->xch_len, "Open SP CDC combined");
out:
	ipio_kfree((void **)&ori);

	return res;
}

static int open_test_sp(int index)
{
	struct mp_test_P540_open open[tItems[index].frame_count];
	int i = 0, x = 0, y = 0, res = 0 , addr;
	int Charge_AA = 0, Charge_Border = 0, Charge_Notch =0, full_open_rate = 0;
	char str[512] = { 0 };

	ipio_info( "index = %d, name = %s, CMD = 0x%x, Frame Count = %d\n",
	    index, tItems[index].name, tItems[index].cmd, tItems[index].frame_count);

	/*
	 * We assume that users who are calling the test forget to config frame count
	 * as 1, so we just help them to set it up.
	 */
	if (tItems[index].frame_count <= 0) {
		ipio_err("Frame count is zero, which is at least set as 1\n");
		tItems[index].frame_count = 1;
	}

	if(tItems[index].test_count == 0)
	{
		res = create_mp_test_frame_buffer(index, tItems[index].frame_count);
		if (res < 0)
			goto out;
		frame1_cbk700 = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		frame1_cbk250 = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		frame1_cbk200 = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);

		if (ERR_ALLOC_MEM(frame1_cbk700) || ERR_ALLOC_MEM(frame1_cbk250) || ERR_ALLOC_MEM(frame1_cbk200)){
			ipio_err("Failed to allocate cbk buffer\n");
			return -ENOMEM;
		}
	}

	tItems[index].node_type = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
	if (ERR_ALLOC_MEM(tItems[index].node_type)){
		ipio_err("Failed to allocate node_type FRAME buffer\n");
		return -ENOMEM;
	}

	/* Init Max/Min buffer */
	for (y = 0; y < core_mp->ych_len; y++) {
		for (x = 0; x < core_mp->xch_len; x++) {
				tItems[index].max_buf[y * core_mp->xch_len + x] = INT_MIN;
				tItems[index].min_buf[y * core_mp->xch_len + x] = INT_MAX;
		}
	}

	if (tItems[index].spec_option == BENCHMARK) {
		core_parser_benchmark(tItems[index].bench_mark_max, tItems[index].bench_mark_min, tItems[index].type_option,tItems[index].desp);
		if (ipio_debug_level&&DEBUG_PARSER > 0)
			dump_benchmark_data(tItems[index].bench_mark_max , tItems[index].bench_mark_min);
	}

	core_parser_nodetype(tItems[index].node_type,"node type");
	if (ipio_debug_level&&DEBUG_PARSER > 0)
		dump_node_type_buffer(tItems[index].node_type, "node type");

	res = core_parser_get_int_data(tItems[index].desp, "charge_aa", str);
	if (res || res == 0)
		Charge_AA = katoi(str);

	res = core_parser_get_int_data(tItems[index].desp, "charge_border", str);
	if (res || res == 0)
		Charge_Border = katoi(str);

	res = core_parser_get_int_data(tItems[index].desp, "charge_notch", str);
	if (res || res == 0)
		Charge_Notch = katoi(str);

	res = core_parser_get_int_data(tItems[index].desp, "full open", str);
	if (res || res == 0)
		full_open_rate = katoi(str);

	if (res < 0) {
		ipio_err("Failed to get parameters from ini file\n");
		goto out;
	}

	ipio_info("pen test frame_cont %d, AA %d,Border %d, Notch %d, full_open_rate %d \n",
			tItems[index].frame_count,Charge_AA,Charge_Border,Charge_Notch,full_open_rate);

	for(i = 0; i < tItems[index].frame_count ; i++) {
		open[i].cbk_700 = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		open[i].cbk_250 = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		open[i].cbk_200 = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		open[i].charg_rate = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		open[i].full_Open = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		open[i].dac = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		open[i].cdc = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);

		if (ERR_ALLOC_MEM(open[i].cbk_700 ) || ERR_ALLOC_MEM(open[i].cbk_250 ) ||
			ERR_ALLOC_MEM(open[i].cbk_200 ) || ERR_ALLOC_MEM(open[i].charg_rate ) ||
			ERR_ALLOC_MEM(open[i].full_Open ) || ERR_ALLOC_MEM(open[i].dac ) ||
			ERR_ALLOC_MEM(open[i].cdc )){
				ipio_err("Failed to allocate open test buffer\n");
				goto out;
		}
	}

	for (i = 0; i < tItems[index].frame_count; i++) {
		res = allnode_open_cdc_data(0, open[i].dac, open[i].dac);
		if (res < 0) {
			ipio_err("Failed to get Open SP DAC data, %d\n", res);
			goto out;
		}
		res = allnode_open_cdc_data(1, open[i].cbk_700, open[i].dac);
		if (res < 0) {
			ipio_err("Failed to get Open SP Raw1 data, %d\n", res);
			goto out;
		}
		res = allnode_open_cdc_data(2, open[i].cbk_250, open[i].dac);
		if (res < 0) {
			ipio_err("Failed to get Open SP Raw2 data, %d\n", res);
			goto out;
		}
		res = allnode_open_cdc_data(3, open[i].cbk_200, open[i].dac);
		if (res < 0) {
			ipio_err("Failed to get Open SP Raw3 data, %d\n", res);
			goto out;
		}
        addr = 0;

		/*record fist frame for drbug*/
		if(i == 0)
		{
			memcpy(frame1_cbk700, open[i].cbk_700, core_mp->frame_len * sizeof(int32_t));
			memcpy(frame1_cbk250, open[i].cbk_250, core_mp->frame_len * sizeof(int32_t));
			memcpy(frame1_cbk200, open[i].cbk_200, core_mp->frame_len * sizeof(int32_t));
		}

		dump_data(open[i].cbk_700, 10, core_mp->frame_len, core_mp->xch_len, "cbk 700");
		dump_data(open[i].cbk_250, 10, core_mp->frame_len, core_mp->xch_len, "cbk 250");
		dump_data(open[i].cbk_200, 10, core_mp->frame_len, core_mp->xch_len, "cbk 200");
        for(y = 0; y < core_mp->ych_len; y++){
            for(x = 0; x < core_mp->xch_len; x++){
                open[i].charg_rate[addr] = open[i].cbk_250[addr] * 100 / open[i].cbk_700[addr];
				open[i].full_Open[addr] = open[i].cbk_700[addr] - open[i].cbk_200[addr];
                addr++;
            }
        }
		if (ipio_debug_level & DEBUG_MP_TEST) {
			dump_data(open[i].charg_rate, 10, core_mp->frame_len, core_mp->xch_len, "origin charge rate");
			dump_data(open[i].full_Open, 10, core_mp->frame_len, core_mp->xch_len, "origin full open");
		}

        addr = 0;
        for(y = 0; y < core_mp->ych_len; y++){
            for(x = 0; x < core_mp->xch_len; x++){
				if(full_open_rate_compare(open[i].full_Open, open[i].cbk_700, x, y, tItems[index].node_type[addr], full_open_rate) == false)
				{
                	tItems[index].buf[(i * core_mp->frame_len) + addr] = 0;
					open[i].charg_rate[addr] = 0;
				}
                addr++;
            }
        }
		if (ipio_debug_level & DEBUG_MP_TEST)
			dump_data(&tItems[index].buf[(i * core_mp->frame_len)], 10, core_mp->frame_len, core_mp->xch_len, "after full_open_rate_compare");

        addr = 0;
        for(y = 0; y < core_mp->ych_len; y++){
            for(x = 0; x < core_mp->xch_len; x++){
                tItems[index].buf[(i * core_mp->frame_len) + addr] = compare_charge(open[i].charg_rate, x, y, tItems[index].node_type, Charge_AA, Charge_Border, Charge_Notch);
                addr++;
            }
        }
		if (ipio_debug_level & DEBUG_MP_TEST)
			dump_data(&tItems[index].buf[(i * core_mp->frame_len)], 10, core_mp->frame_len, core_mp->xch_len, "after compare charge rate");

		compare_MaxMin_result(index, &tItems[index].buf[(i * core_mp->frame_len)]);
	}

out:
	ipio_kfree((void **)&tItems[index].node_type);

	for(i = 0; i < tItems[index].frame_count; i++) {
		ipio_kfree((void **)&open[i].cbk_700);
		ipio_kfree((void **)&open[i].cbk_250);
		ipio_kfree((void **)&open[i].cbk_200);
		ipio_kfree((void **)&open[i].charg_rate);
		ipio_kfree((void **)&open[i].full_Open);
		ipio_kfree((void **)&open[i].dac);
	}

	return res;
}

int codeToOhm(int32_t Code, uint16_t *v_tdf, uint16_t *h_tdf)
{
	int douTDF1 = 0;
	int douTDF2 = 0;
	int douTVCH = 24;
	int douTVCL = 8;
	int douCint = 7;
	int douVariation = 64;
	int douRinternal = 930;
	int32_t temp = 0;
	static int test = 0;
	if (core_mp->isLongV) {
		douTDF1 = *v_tdf;
		douTDF2 = *(v_tdf + 1);
	} else {
		douTDF1 = *h_tdf;
		douTDF2 = *(h_tdf + 1);
	}
	if (test == 0) {
		ipio_info("DDI MODE %d TDF1 = %d TDF2 = %d\n", core_mp->isLongV, douTDF1, douTDF2);
		test = 1;
	}
	if (Code == 0) {
		ipio_debug(DEBUG_MP_TEST, "code is invalid\n");
	} else {
		temp = ((douTVCH - douTVCL) * douVariation * (douTDF1 - douTDF2) * (1<<12) / (9 * Code * douCint)) * 100;
		temp = (temp - douRinternal) / 1000;
	}
	/* Unit = M Ohm */
	return temp;
}

static int short_test(int index,int frame_index)
{
	int j = 0, res = 0;
	uint16_t v_tdf[2] = {0};
	uint16_t h_tdf[2] = {0};

	v_tdf[0] = tItems[index].v_tdf_1;
	v_tdf[1] = tItems[index].v_tdf_2;
	h_tdf[0] = tItems[index].h_tdf_1;
	h_tdf[1] = tItems[index].h_tdf_2;
	ipio_info("v_tdf0 = %d v_tdf1 = %d h_tdf0 = %d h_tdf1 = %d\n", v_tdf[0], v_tdf[1], h_tdf[0], h_tdf[1]);
	if(protocol->major >= 5 && protocol->mid >= 4) {
		/* Calculate code to ohm and save to tItems[index].buf */
		for (j = 0; j < core_mp->frame_len; j++)
			tItems[index].buf[frame_index * core_mp->frame_len + j] = codeToOhm(frame_buf[j], v_tdf, h_tdf);
	} else {
		for (j = 0; j < core_mp->frame_len; j++)
			tItems[index].buf[frame_index * core_mp->frame_len + j] = frame_buf[j];
	}

	return res;
}

static int mutual_test(int index)
{
	int i = 0, j = 0, x = 0, y = 0, res = 0 ,get_frame_cont =1 ;

	ipio_debug(DEBUG_MP_TEST, "index = %d, name = %s, CMD = 0x%x, Frame Count = %d\n",
	    index, tItems[index].name, tItems[index].cmd, tItems[index].frame_count);

	/*
	 * We assume that users who are calling the test forget to config frame count
	 * as 1, so we just help them to set it up.
	 */
	if (tItems[index].frame_count <= 0) {
		ipio_err("Frame count is zero, which is at least set as 1\n");
		tItems[index].frame_count = 1;
	}

	//makesure it's first time
	if(tItems[index].test_count == 0)
	{
		res = create_mp_test_frame_buffer(index, tItems[index].frame_count);
		if (res < 0)
			goto out;
	}

	/* Init Max/Min buffer */
	for (y = 0; y < core_mp->ych_len; y++) {
		for (x = 0; x < core_mp->xch_len; x++) {
			if (tItems[i].catalog == TX_RX_DELTA) {
				core_mp->tx_max_buf[y * core_mp->xch_len + x] = INT_MIN;
				core_mp->rx_max_buf[y * core_mp->xch_len + x] = INT_MIN;
				core_mp->tx_min_buf[y * core_mp->xch_len + x] = INT_MAX;
				core_mp->rx_min_buf[y * core_mp->xch_len + x] = INT_MAX;
			} else {
				tItems[index].max_buf[y * core_mp->xch_len + x] = INT_MIN;
				tItems[index].min_buf[y * core_mp->xch_len + x] = INT_MAX;
			}
		}
	}

	if (tItems[index].catalog != PEAK_TO_PEAK_TEST)
		get_frame_cont = tItems[index].frame_count;

	if (tItems[index].spec_option == BENCHMARK) {
		core_parser_benchmark(tItems[index].bench_mark_max, tItems[index].bench_mark_min, tItems[index].type_option,tItems[index].desp);
		if (ipio_debug_level&&DEBUG_PARSER > 0)
			dump_benchmark_data(tItems[index].bench_mark_max , tItems[index].bench_mark_min);
	}

	for (i = 0; i < get_frame_cont; i++) {
		res = allnode_mutual_cdc_data(index);
		if (res < 0) {
			ipio_err("Failed to initialise CDC data, %d\n", res);
			goto out;
		}
		switch (tItems[index].catalog) {
		case PIXEL:
			run_pixel_test(index);
			break;
		case UNTOUCH_P2P:
			run_untouch_p2p_test(index);
			break;
		case OPEN_TEST:
			run_open_test(index);
			break;
		case TX_RX_DELTA:
			run_tx_rx_delta_test(index);
			break;
		case SHORT_TEST:
			short_test(index , i);
			break;
		default:
			for (j = 0; j < core_mp->frame_len; j++)
				tItems[index].buf[i * core_mp->frame_len + j] = frame_buf[j];
			break;
		}
		compare_MaxMin_result(index, &tItems[index].buf[i * core_mp->frame_len]);
	}

out:
	return res;
}

static int key_test(int index)
{
	int i, j = 0, res = 0;

	ipio_debug(DEBUG_MP_TEST, "Item = %s, CMD = 0x%x, Frame Count = %d\n",
	    tItems[index].name, tItems[index].cmd, tItems[index].frame_count);

	if (tItems[index].frame_count == 0) {
		ipio_err("Frame count is zero, which at least sets as 1\n");
		res = -EINVAL;
		goto out;
	}

	if(tItems[index].test_count == 0)
		res = create_mp_test_frame_buffer(index, tItems[index].frame_count);

	if (res < 0)
		goto out;

	for (i = 0; i < tItems[index].frame_count; i++) {
		res = allnode_key_cdc_data(index);
		if (res < 0) {
			ipio_err("Failed to initialise CDC data, %d\n", res);
			goto out;
		}

		for (j = 0; j < core_mp->key_len; j++)
			tItems[index].buf[j] = key_buf[j];
	}

	compare_MaxMin_result(index, tItems[index].buf);

out:
	return res;
}

static int self_test(int index)
{
	ipio_err("TDDI has no self to be tested currently\n");
	return -1;
}

static int st_test(int index)
{
	ipio_err("ST Test is not supported by the driver\n");
	return -1;
}

int mp_test_data_sort_average(int32_t *oringin_data,int index, int32_t *avg_result)
{
	int i,j,k,x,y,len = 5;
	int32_t u32temp;
	int u32up_frame,u32down_frame;
	int32_t *u32sum_raw_data;
	int32_t *u32data_buff;

	if(tItems[index].frame_count <= 1)
		return 0;


	if (ERR_ALLOC_MEM(oringin_data)){
		ipio_err("Input wrong adress\n");
			return -ENOMEM;
	}

	u32data_buff = kcalloc(core_mp->frame_len * tItems[index].frame_count, sizeof(int32_t), GFP_KERNEL);
	u32sum_raw_data = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
	if (ERR_ALLOC_MEM(u32sum_raw_data) || (ERR_ALLOC_MEM(u32data_buff))){
		ipio_err("Failed to allocate u32sum_raw_data FRAME buffer\n");
		ipio_kfree((void **)&u32data_buff);
		ipio_kfree((void **)&u32sum_raw_data);		
		return -ENOMEM;
	}

	for (i = 0; i < core_mp->frame_len * tItems[index].frame_count; i++) {
		u32data_buff[i] = oringin_data[i];
	}

	u32up_frame = tItems[index].frame_count * tItems[index].highest_percentage / 100;
	u32down_frame = tItems[index].frame_count * tItems[index].lowest_percentage / 100;
	ipio_debug(DEBUG_MP_TEST,"Up=%d,Down=%d -%s\n",u32up_frame,u32down_frame,tItems[index].desp);

	if (ipio_debug_level & DEBUG_MP_TEST) {
		printk("\n[Show Original frist%d and last%d node data]\n",len,len);
		for(i = 0 ; i < core_mp->frame_len ; i++){
			for(j = 0 ; j < tItems[index].frame_count ; j++){
				if((i<len) || (i >= (core_mp->frame_len-len)))
					printk("%d,",u32data_buff[j* core_mp->frame_len + i]);
			}
			if((i<len) || (i >=(core_mp->frame_len-len)))
				printk("\n");
		}
	}

	for(i = 0 ; i < core_mp->frame_len ; i++){
		for(j = 0 ; j < tItems[index].frame_count-1 ; j++){
			for(k = 0 ; k < (tItems[index].frame_count-1-j) ; k++){
				x=i+k*core_mp->frame_len;
				y=i+(k+1)*core_mp->frame_len;
				if (*(u32data_buff+x) > *(u32data_buff+y)) {
					u32temp = *(u32data_buff+x);
					*(u32data_buff+x) = *(u32data_buff+y);
					*(u32data_buff+y) = u32temp;
				}
			}
		}
	}

	if (ipio_debug_level & DEBUG_MP_TEST) {
		printk("\n[After sorting frist%d and last%d node data]\n",len,len);
		for(i = 0 ; i < core_mp->frame_len ; i++){
			for(j = u32down_frame ; j < tItems[index].frame_count - u32up_frame ; j++){
				if((i<len) || (i >= (core_mp->frame_len-len)))
					printk("%d,",u32data_buff[i + j * core_mp->frame_len]);
			}
			if((i<len) || (i >= (core_mp->frame_len-len)))
				printk("\n");
		}
	}

	for(i = 0 ; i < core_mp->frame_len ; i++){
		u32sum_raw_data[i]=0;
		for(j = u32down_frame ; j < tItems[index].frame_count - u32up_frame ; j++)
			u32sum_raw_data[i] += u32data_buff[i + j * core_mp->frame_len];

		avg_result[i] = u32sum_raw_data[i] / (tItems[index].frame_count - u32down_frame - u32up_frame);
	}

	if (ipio_debug_level & DEBUG_MP_TEST) {
		printk("\n[Average result frist%d and last%d node data]\n",len,len);
		for(i = 0 ; i < core_mp->frame_len ; i++){
			if((i<len) || (i >= (core_mp->frame_len-len)))
				printk("%d,",avg_result[i]);
		}
		if((i<len) || (i >= (core_mp->frame_len-len)))
			printk("\n");
	}

	ipio_kfree((void **)&u32data_buff);
	ipio_kfree((void **)&u32sum_raw_data);
	return 0;
}


int core_mp_compare_retry_cdc_result(int i)
{
	int j , test_result = MP_PASS;
	int32_t *max_threshold = NULL, *min_threshold = NULL ;

	max_threshold = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
	min_threshold = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
	if (ERR_ALLOC_MEM(max_threshold) || ERR_ALLOC_MEM(min_threshold)) {
		ipio_err("Failed to allocate threshold FRAME buffer\n");
		test_result = MP_FAIL;
		goto fail_open;
	}

	/* Show test result as below */
	if (tItems[i].catalog == TX_RX_DELTA) {

		if (ERR_ALLOC_MEM(core_mp->rx_delta_buf) || ERR_ALLOC_MEM(core_mp->tx_delta_buf)) {
			ipio_err("This test item (%s) has no data inside its buffer\n", tItems[i].desp);
			test_result = MP_FAIL;
			goto fail_open;
		}

		for(j = 0 ;j < core_mp->frame_len ; j++) {
			max_threshold[j] = core_mp->TxDeltaMax;
			min_threshold[j] = core_mp->TxDeltaMin;
		}
		mp_compare_cdc_result(i, core_mp->tx_max_buf, max_threshold, min_threshold, &test_result);
		mp_compare_cdc_result(i, core_mp->tx_min_buf, max_threshold, min_threshold, &test_result);
		for(j = 0 ;j < core_mp->frame_len ; j++) {
			max_threshold[j] = core_mp->RxDeltaMax;
			min_threshold[j] = core_mp->RxDeltaMin;
		}
		mp_compare_cdc_result(i, core_mp->rx_max_buf, max_threshold, min_threshold, &test_result);
		mp_compare_cdc_result(i, core_mp->rx_min_buf, max_threshold, min_threshold, &test_result);
	} else {

		if (ERR_ALLOC_MEM(tItems[i].buf) || ERR_ALLOC_MEM(tItems[i].max_buf) ||
				ERR_ALLOC_MEM(tItems[i].min_buf) || ERR_ALLOC_MEM(tItems[i].result_buf)) {
			ipio_err("This test item (%s) has no data inside its buffer\n", tItems[i].desp);
			test_result = MP_FAIL;
			goto fail_open;
		}

		if ( tItems[i].spec_option == BENCHMARK){
			for(j = 0 ;j < core_mp->frame_len ; j++){
				max_threshold[j] = tItems[i].bench_mark_max[j];
				min_threshold[j] = tItems[i].bench_mark_min[j];
			}
		} else {
			for(j = 0 ;j < core_mp->frame_len ; j++) {
				max_threshold[j] = tItems[i].max;
				min_threshold[j] = tItems[i].min;
			}
		}
		/* general result */
		if(tItems[i].trimmed_mean && tItems[i].catalog != PEAK_TO_PEAK_TEST)
		{
			mp_test_data_sort_average(tItems[i].buf, i, tItems[i].result_buf);
			mp_compare_cdc_result(i, tItems[i].result_buf, max_threshold, min_threshold, &test_result);
		}
		else
		{
			mp_compare_cdc_result(i, tItems[i].max_buf, max_threshold, min_threshold, &test_result);
			mp_compare_cdc_result(i, tItems[i].min_buf, max_threshold, min_threshold, &test_result);
		}
	}

fail_open:
	ipio_kfree((void **)&max_threshold);
	ipio_kfree((void **)&min_threshold);

	return test_result;
}

void core_mp_retry(int index)
{
	core_config_ice_mode_enable();

	if (core_config_set_watch_dog(false) < 0) {
		ipio_err("Failed to disable watch dog\n");
	}

	core_config_ic_reset();

	core_config_ice_mode_disable();

	/* Switch to Demo mode */
	core_fr_mode_control(&protocol->demo_mode);
	ilitek_platform_disable_irq();

	/* Switch to test mode */
	core_fr_mode_control(&protocol->test_mode);
	ilitek_platform_disable_irq();

	tItems[index].do_test(index);
	tItems[index].test_count ++;
}

int ilitek_osc_check(void)
{
	uint8_t test_cmd[8] = { 0 };
	int res = 0;
	int i = 0;
	char *item = "osc check";
	char str[512] = { 0 };

#if (ILITEK_TYPE == ILITEK_TYPE_LV)
		core_mp->osc_test =0;
		return 0;
#endif
	res = core_parser_get_int_data(item, "enable", str);
	if (res <= 0) {
		ipio_info("Failed to get osc_test enable data, %d\n", res);
		core_mp->osc_test = 1;
	}
	else {
		core_mp->osc_test = katoi(str);
	}
	res = core_parser_get_int_data(item, "max", str);
	if (res <= 0) {
		ipio_info("Failed to get osc_threshold_max data, %d\n", res);
		core_mp->osc_threshold_max = 20;
	}
	else {
		core_mp->osc_threshold_max = katoi(str);
	}
	res = core_parser_get_int_data(item, "min", str);
	if (res <= 0) {
		ipio_info("Failed to get osc_threshold_min data, %d\n", res);
		core_mp->osc_threshold_min = -25;
	}
	else {
		core_mp->osc_threshold_min = katoi(str);
	}
	if (!core_mp->osc_test) {
		core_mp->osc_result = MP_PASS;
		return 0;
	}
	for (i = 0; i < 3; i++) {
		test_cmd[0] = 0xF1;
		test_cmd[1] = 0x40;
		test_cmd[2] = 0x00;
		res = core_write(core_config->slave_i2c_addr, test_cmd, 3);
		if (res < 0) {
			ipio_info("Failed to write data, %d\n", res);
			mdelay(20);
			continue;
		}
		res = core_read(core_config->slave_i2c_addr, test_cmd, 5);
		if (res < 0) {
			ipio_info("Failed to read tp set ddi trim code %d\n", res);
			mdelay(20);
			continue;
		}
		else {
			ipio_info("read osc value 0x%X 0x%X 0x%X 0x%X\n", test_cmd[0], test_cmd[1], test_cmd[2], test_cmd[3]);
			core_mp->osc_org_data[0] = test_cmd[0];
			core_mp->osc_org_data[1] = test_cmd[1];
			core_mp->osc_org_data[2] = test_cmd[2];
			core_mp->osc_org_data[3] = test_cmd[3];
			if((uint8_t)(test_cmd[2] & 0x80) == (uint8_t)0x80)
			{
				//core_mp->osc_offset = (0x10000 - ((test_cmd[2] << 8) + test_cmd[3]));
				core_mp->osc_offset = (((test_cmd[2] << 8) + test_cmd[3]) - 0x10000);
			}
			else
			{
				core_mp->osc_offset = ((test_cmd[2] << 8) + test_cmd[3]);
			}
			ipio_info("osc offset org value is %d,  trim count is %d", core_mp->osc_offset, ((test_cmd[0] << 8) + test_cmd[1]));
			break;
		}
	}
	if (i >= 3) {
		ipio_info("Failed to read OSC value, %d\n", res);
		res = -1;
	}
	else {
		if (core_mp->osc_offset > core_mp->osc_threshold_max || core_mp->osc_offset < core_mp->osc_threshold_min) {
			ipio_info("OSC check failed, osc_offset = %d osc_threshold_max = %d osc_threshold_min = %d\n", core_mp->osc_offset, core_mp->osc_threshold_max, core_mp->osc_threshold_min);
			core_mp->osc_result = MP_FAIL;
		}
		else {
			ipio_info("OSC check pass, osc_offset = %d osc_threshold_max = %d osc_threshold_min = %d\n", core_mp->osc_offset, core_mp->osc_threshold_max, core_mp->osc_threshold_min);
			core_mp->osc_result = MP_PASS;
		}
		res = 0;
	}
	return res;
}
EXPORT_SYMBOL(ilitek_osc_check);

void core_mp_show_result(void)
{
	int i, x, y, j, csv_len = 0, pass_item_count = 0 ,line_count = 0 ,get_frame_cont = 1;
	int32_t *max_threshold = NULL, *min_threshold = NULL;
	char *csv = NULL;
	char csv_name[128] = { 0 };
	char *ret_pass_name = NULL, *ret_fail_name = NULL;
	struct file *f = NULL;
	mm_segment_t fs;
	loff_t pos;
	struct timespec now_time;
	struct rtc_time rtc_now_time;
	uint8_t data_buf[128];
	csv = vmalloc(CSV_FILE_SIZE);
	if (ERR_ALLOC_MEM(csv)) {
		ipio_err("Failed to allocate CSV mem\n");
		goto fail_open;
	}

	max_threshold = vmalloc(core_mp->frame_len * sizeof(int32_t));
	min_threshold = vmalloc(core_mp->frame_len * sizeof(int32_t));
	if (ERR_ALLOC_MEM(max_threshold) || ERR_ALLOC_MEM(min_threshold)) {
		ipio_err("Failed to allocate threshold FRAME buffer\n");
		goto fail_open;
	}
	/* header must has 19 line*/
	csv_len += sprintf(csv + csv_len,"==============================================================================\n");
	line_count++;
	csv_len += sprintf(csv + csv_len,"ILITek C-TP Utility V%s  %x : Driver Sensor Test\n", DRIVER_VERSION, core_config->chip_pid);
	line_count++;
	csv_len += sprintf(csv + csv_len,"Confidentiality Notice:\n");
	line_count++;
	csv_len += sprintf(csv + csv_len,"Any information of this tool is confidential and privileged.\n");
	line_count++;
	csv_len += sprintf(csv + csv_len,"@ ILI TECHNOLOGY CORP. All Rights Reserved.\n");
	line_count++;
	csv_len += sprintf(csv + csv_len,"==============================================================================\n");
	line_count++;
	if (protocol->mid >= 0x3) {/*line7*/
		csv_len += sprintf(csv + csv_len,"Firmware Version ,V%d.%d.%d.%d\n", core_config->firmware_ver[1], core_config->firmware_ver[2], core_config->firmware_ver[3], core_config->firmware_ver[4]);
	} else {
		csv_len += sprintf(csv + csv_len,"Firmware Version ,V%d.%d.%d\n",core_config->firmware_ver[1], core_config->firmware_ver[2], core_config->firmware_ver[3]);
	}
	line_count++;
	csv_len += sprintf(csv + csv_len,"Panel information ,XCH=%d, YCH=%d\n", core_mp->xch_len, core_mp->ych_len);
	line_count++;
	csv_len += sprintf(csv + csv_len,"Test Item:\n");
	line_count++;

	ipio_info("core_mp_show_result: finish head\n");

	for (i = 0; i < ARRAY_SIZE(tItems); i++) {
		if (tItems[i].run == 1)
		{
			csv_len += sprintf(csv + csv_len, "	  ---%s\n", tItems[i].desp);
			line_count++;
		}
	}
	while(line_count < 19)
	{
		csv_len += sprintf(csv + csv_len,"\n");
		line_count++;
	}
	csv_len += sprintf(csv + csv_len,"==============================================================================\n");

	for (i = 0; i < ARRAY_SIZE(tItems); i++) {
		if (tItems[i].run != 1)
			continue;

		if (tItems[i].item_result == MP_PASS)
		{
			pr_info("\n\n[%s],OK \n", tItems[i].desp);
			csv_len += sprintf(csv + csv_len, "\n\n[%s],OK\n", tItems[i].desp);
		}else{
			pr_info("\n\n[%s],NG \n", tItems[i].desp);
			csv_len += sprintf(csv + csv_len, "\n\n[%s],NG\n", tItems[i].desp);
		}

		ipio_debug(DEBUG_MP_TEST, "Frame count = %d\n",tItems[i].frame_count);
		csv_len += sprintf(csv + csv_len, "Frame count = %d\n", tItems[i].frame_count);

		if(tItems[i].trimmed_mean && tItems[i].catalog != PEAK_TO_PEAK_TEST) {
			ipio_debug(DEBUG_MP_TEST, "Lowest Percentage = %d\n",tItems[i].lowest_percentage);
			csv_len += sprintf(csv + csv_len, "Lowest Percentage = %d\n", tItems[i].lowest_percentage);

			ipio_debug(DEBUG_MP_TEST, "Highest Percentage = %d\n",tItems[i].highest_percentage);
			csv_len += sprintf(csv + csv_len, "Highest Percentage = %d\n", tItems[i].highest_percentage);
		}

		ipio_info("core_mp_show_result: Show result of benchmark\n");

		/* Show result of benchmark max and min */
		if ( tItems[i].spec_option == BENCHMARK){
			for(j = 0 ;j < core_mp->frame_len ; j++){
				max_threshold[j] = tItems[i].bench_mark_max[j];
				min_threshold[j] = tItems[i].bench_mark_min[j];
			}

			mp_compare_cdc_show_result(i, tItems[i].bench_mark_max, csv, &csv_len, TYPE_BENCHMARK, max_threshold, min_threshold,"Max_Bench");
			mp_compare_cdc_show_result(i, tItems[i].bench_mark_min, csv, &csv_len, TYPE_BENCHMARK, max_threshold, min_threshold,"Min_Bench");
		} else {

			for(j = 0 ;j < core_mp->frame_len ; j++) {
				max_threshold[j] = tItems[i].max;
				min_threshold[j] = tItems[i].min;
			}

			ipio_debug(DEBUG_MP_TEST, "Max = %d\n",tItems[i].max);
			csv_len += sprintf(csv + csv_len, "Max = %d\n", tItems[i].max);

			ipio_debug(DEBUG_MP_TEST, "Min = %d\n",tItems[i].min);
			csv_len += sprintf(csv + csv_len, "Min = %d\n", tItems[i].min);
		}

		//ipio_info("core_mp_show_result: open_integration_sp\n");

		if (strcmp(tItems[i].name, "open_integration_sp") == 0)
		{
			mp_compare_cdc_show_result(i, frame1_cbk700, csv, &csv_len, TYPE_NO_JUGE, max_threshold, min_threshold,"frame1 cbk700");
			mp_compare_cdc_show_result(i, frame1_cbk250, csv, &csv_len, TYPE_NO_JUGE, max_threshold, min_threshold,"frame1 cbk250");
			mp_compare_cdc_show_result(i, frame1_cbk200, csv, &csv_len, TYPE_NO_JUGE, max_threshold, min_threshold,"frame1 cbk200");
			ipio_kfree((void **)&frame1_cbk700);
			ipio_kfree((void **)&frame1_cbk250);
			ipio_kfree((void **)&frame1_cbk200);
		}

		if (tItems[i].catalog == TX_RX_DELTA) {
			if (ERR_ALLOC_MEM(core_mp->rx_delta_buf) || ERR_ALLOC_MEM(core_mp->tx_delta_buf)) {
				ipio_err("This test item (%s) has no data inside its buffer\n", tItems[i].desp);
				continue;
			}
		} else {
			if (ERR_ALLOC_MEM(tItems[i].buf) || ERR_ALLOC_MEM(tItems[i].max_buf) ||
					ERR_ALLOC_MEM(tItems[i].min_buf)) {
				ipio_err("This test item (%s) has no data inside its buffer\n", tItems[i].desp);
				continue;
			}
		}

		ipio_info("core_mp_show_result: show test result as below\n");

		/* Show test result as below */
		if (tItems[i].catalog == KEY_TEST) {
			for (x = 0; x < core_mp->key_len; x++) {
				DUMP(DEBUG_MP_TEST, "KEY_%02d ", x);
				csv_len += sprintf(csv + csv_len, "KEY_%02d,", x);
			}

			DUMP(DEBUG_MP_TEST, "\n");
			csv_len += sprintf(csv + csv_len, "\n");

			for (y = 0; y < core_mp->key_len; y++) {
				DUMP(DEBUG_MP_TEST, " %3d   ", tItems[i].buf[y]);
				csv_len += sprintf(csv + csv_len, " %3d, ", tItems[i].buf[y]);
			}

			DUMP(DEBUG_MP_TEST, "\n");
			csv_len += sprintf(csv + csv_len, "\n");
		} else if (tItems[i].catalog == TX_RX_DELTA) {

			for(j = 0 ;j < core_mp->frame_len ; j++) {
				max_threshold[j] = core_mp->TxDeltaMax;
				min_threshold[j] = core_mp->TxDeltaMin;
			}
			mp_compare_cdc_show_result(i, core_mp->tx_max_buf, csv, &csv_len, TYPE_JUGE, max_threshold, min_threshold,"TX Max Hold");
			mp_compare_cdc_show_result(i, core_mp->tx_min_buf, csv, &csv_len, TYPE_JUGE, max_threshold, min_threshold,"TX Min Hold");

			for(j = 0 ;j < core_mp->frame_len ; j++) {
				max_threshold[j] = core_mp->RxDeltaMax;
				min_threshold[j] = core_mp->RxDeltaMin;
			}
			mp_compare_cdc_show_result(i, core_mp->rx_max_buf, csv, &csv_len, TYPE_JUGE, max_threshold, min_threshold,"RX Max Hold");
			mp_compare_cdc_show_result(i, core_mp->rx_min_buf, csv, &csv_len, TYPE_JUGE, max_threshold, min_threshold,"RX Min Hold");

		} else {
			/* general result */
			if(tItems[i].trimmed_mean && tItems[i].catalog != PEAK_TO_PEAK_TEST){
				mp_compare_cdc_show_result(i, tItems[i].result_buf, csv, &csv_len, TYPE_JUGE, max_threshold, min_threshold,"Mean result");
			} else {
				mp_compare_cdc_show_result(i, tItems[i].max_buf, csv, &csv_len, TYPE_JUGE, max_threshold, min_threshold,"Max Hold");
				mp_compare_cdc_show_result(i, tItems[i].min_buf, csv, &csv_len, TYPE_JUGE, max_threshold, min_threshold,"Min Hold");
			}
			if(tItems[i].catalog != PEAK_TO_PEAK_TEST)
				get_frame_cont = tItems[i].frame_count;
			/* result of each frame */
			for(j = 0; j < get_frame_cont; j++) {
				char frame_name[128] ={ 0 };
				sprintf(frame_name, "Frame %d", (j+1));
				mp_compare_cdc_show_result(i, &tItems[i].buf[(j*core_mp->frame_len)], csv, &csv_len, TYPE_NO_JUGE, max_threshold, min_threshold, frame_name);
			}

		}
	}

	memset(csv_name, 0, 128 * sizeof(char));

	ipio_info("core_mp_show_result: begin Summary\n");

	csv_len += sprintf(csv + csv_len,"==============================================================================\n");
	csv_len += sprintf(csv + csv_len,"Result_Summary           \n");

	for (i = 0; i < ARRAY_SIZE(tItems); i++) {
		if (tItems[i].run) {
			if (tItems[i].item_result == MP_PASS)
				csv_len += sprintf(csv + csv_len,"   {%s}     ,OK\n",tItems[i].desp);
			else
				csv_len += sprintf(csv + csv_len,"   {%s}     ,NG\n",tItems[i].desp);
		}
	}
	if (!(core_mp->oppo_lcm) && core_mp->osc_test) {
		csv_len += sprintf(csv + csv_len, "\n\nOSC read data 0x%02X 0x%02X 0x%02X 0x%02X\n",\
			core_mp->osc_org_data[0], core_mp->osc_org_data[1], core_mp->osc_org_data[2], core_mp->osc_org_data[3]);
		csv_len += sprintf(csv + csv_len, "OSC threshold(%s%d.%d%% ~ +%d.%d%%)\n",\
			(core_mp->osc_threshold_min < 0 && core_mp->osc_threshold_min > -10) ? "-" : " ",\
			core_mp->osc_threshold_min / 10, (int)Mathabs(core_mp->osc_threshold_min % 10),\
			core_mp->osc_threshold_max / 10, core_mp->osc_threshold_max % 10);
		csv_len += sprintf(csv + csv_len, "trim count is %d OSC offset Percent result: %s%d.%d%%  %s \n",\
			((core_mp->osc_org_data[0] << 8) + core_mp->osc_org_data[1]),\
			(core_mp->osc_offset < 0 && core_mp->osc_offset > -10) ? "-" : " ",\
			core_mp->osc_offset / 10, (int)(Mathabs(core_mp->osc_offset % 10)),\
			(core_mp->osc_result == MP_PASS) ? "OSC test pass" : "OSC test NG");
		if (core_mp->osc_result == MP_FAIL) {
			//core_mp->fail_items++;
		}
	}

	for (i = 0; i < ARRAY_SIZE(tItems); i++) {
		if (tItems[i].run) {
			if (tItems[i].item_result == MP_FAIL)
			{
				pass_item_count = 0;
				mp_test_failed_items++;
				//break;
			}
			pass_item_count++;
		}
	}

	/* define csv file name */
	if (core_mp->oppo_run) {
		if (core_mp->oppo_lcm) {
			ret_pass_name = OPPO_CSV_LCMOFF_PASS_NAME;
			ret_fail_name = OPPO_CSV_LCMOFF_FAIL_NAME;
		} else {
			ret_pass_name = OPPO_CSV_PASS_NAME;
			ret_fail_name = OPPO_CSV_FAIL_NAME;
		}
	} else {
		ret_pass_name = NORMAL_CSV_PASS_NAME;
		ret_fail_name = NORMAL_CSV_FAIL_NAME;
	}
#if 0
	if (pass_item_count == 0) {
		core_mp->final_result = MP_FAIL;
		sprintf(csv_name, "%s/%s.csv", CSV_PATH, ret_fail_name);
	} else {
		core_mp->final_result = MP_PASS;
		sprintf(csv_name, "%s/%s.csv", CSV_PATH, ret_pass_name);
	}
#endif
    getnstimeofday(&now_time);
    rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
    snprintf(data_buf, 128, "_%02d%02d%02d-%02d%02d%02d-utc.csv",
            (rtc_now_time.tm_year + 1900) % 100, rtc_now_time.tm_mon + 1, rtc_now_time.tm_mday,
            rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);
	fs = get_fs();
	set_fs(KERNEL_DS);
	sys_mkdir("/sdcard/ILITEK/", 0666);
	set_fs(fs);
	if (mp_test_failed_items) {
		core_mp->final_result = MP_FAIL;
		sprintf(csv_name, "%s/%s%s", CSV_PATH, ret_fail_name, data_buf);
	} else {
		core_mp->final_result = MP_PASS;
		sprintf(csv_name, "%s/%s%s", CSV_PATH, ret_pass_name, data_buf);
	}

	ipio_info("Open CSV : %s\n", csv_name);

	if (f == NULL)
		f = filp_open(csv_name, O_WRONLY | O_CREAT | O_TRUNC, 644);

	if (ERR_ALLOC_MEM(f)) {
		ipio_err("Failed to open CSV file");
		goto fail_open;
	}

	ipio_info("Open CSV succeed, its length = %d\n ", csv_len);

	if (csv_len >= CSV_FILE_SIZE) {
		ipio_err("The length saved to CSV is too long !\n");
		goto fail_open;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_write(f, csv, csv_len, &pos);
	set_fs(fs);
	filp_close(f, NULL);

	ipio_info("Writing Data into CSV succeed\n");

fail_open:
	if(csv != NULL)
		vfree(csv);
	ipio_vfree((void **)&max_threshold);
	ipio_vfree((void **)&min_threshold);
}
EXPORT_SYMBOL(core_mp_show_result);

void core_mp_run_test(char *item, bool ini)
{
	int i = 0 , count ,test_result = MP_PASS;
	char str[512] = { 0 };

	/* update X/Y channel length if they're changed */
	core_mp->xch_len = core_config->tp_info->nXChannelNum;
	core_mp->ych_len = core_config->tp_info->nYChannelNum;

	/* update key's length if they're changed */
	core_mp->key_len = core_config->tp_info->nKeyCount;

	/* compute the total length in one frame */
	core_mp->frame_len = core_mp->xch_len * core_mp->ych_len;

	if (item == NULL || strncmp(item, " ", strlen(item)) == 0 || core_mp->frame_len == 0 ) {
		tItems[i].result = "FAIL";
		core_mp->final_result = MP_FAIL;
		ipio_err("Invaild string or length\n");
		return;
	}

	ipio_debug(DEBUG_MP_TEST, "item = %s, core type = %d\n", item, core_config->core_type);

	for (i = 0; i < core_mp->mp_items; i++) {
		if (strncmp(item, tItems[i].desp, strlen(item)) == 0) {
			if (ini) {
				core_parser_get_int_data(item, "enable", str);
				tItems[i].run = katoi(str);
				core_parser_get_int_data(item, "spec option", str);
				tItems[i].spec_option= katoi(str);
				core_parser_get_int_data(item, "type option", str);
				tItems[i].type_option= katoi(str);
 				core_parser_get_int_data(item, "frame count", str);
				tItems[i].frame_count= katoi(str);
				core_parser_get_int_data(item, "trimmed mean", str);
				tItems[i].trimmed_mean= katoi(str);
				core_parser_get_int_data(item, "lowest percentage", str);
				tItems[i].lowest_percentage= katoi(str);
 				core_parser_get_int_data(item, "highest percentage", str);
				tItems[i].highest_percentage= katoi(str);
				/* Get TDF value from ini */
				if (tItems[i].catalog == SHORT_TEST) {
					core_parser_get_int_data(item, "v_tdf_1", str);
					tItems[i].v_tdf_1 = core_parser_get_tdf_value(i, str);
					core_parser_get_int_data(item, "v_tdf_2", str);
					tItems[i].v_tdf_2 = core_parser_get_tdf_value(i, str);
					core_parser_get_int_data(item, "h_tdf_1", str);
					tItems[i].h_tdf_1 = core_parser_get_tdf_value(i, str);
					core_parser_get_int_data(item, "h_tdf_2", str);
					tItems[i].h_tdf_2 = core_parser_get_tdf_value(i, str);
				} else {
					core_parser_get_int_data(item, "v_tdf", str);
					tItems[i].v_tdf_1 = core_parser_get_tdf_value(i, str);
					core_parser_get_int_data(item, "h_tdf", str);
					tItems[i].h_tdf_1 = core_parser_get_tdf_value(i, str);
				}
				/* Get threshold from ini structure in parser */
				if (strcmp(item, "tx/rx delta") == 0) {
					core_parser_get_int_data(item, "tx max", str);
					core_mp->TxDeltaMax = katoi(str);
					core_parser_get_int_data(item, "tx min", str);
					core_mp->TxDeltaMin = katoi(str);
					core_parser_get_int_data(item, "rx max", str);
					core_mp->RxDeltaMax = katoi(str);
					core_parser_get_int_data(item, "rx min", str);
					core_mp->RxDeltaMin = katoi(str);
					ipio_debug(DEBUG_MP_TEST, "%s: Tx Max = %d, Tx Min = %d, Rx Max = %d,  Rx Min = %d\n",
							tItems[i].desp, core_mp->TxDeltaMax, core_mp->TxDeltaMin,
							core_mp->RxDeltaMax, core_mp->RxDeltaMin);
				} else {
					core_parser_get_int_data(item, "max", str);
					tItems[i].max = katoi(str);
					core_parser_get_int_data(item, "min", str);
					tItems[i].min = katoi(str);
				}

				core_parser_get_int_data(item, "frame count", str);
				tItems[i].frame_count = katoi(str);

				ipio_info("%s: run = %d, max = %d, min = %d, frame_count = %d\n", tItems[i].desp,
						tItems[i].run, tItems[i].max, tItems[i].min, tItems[i].frame_count);
				ipio_info("v_tdf_1 = %d, v_tdf_2 = %d, h_tdf_1 = %d, h_tdf_2 = %d\n", tItems[i].v_tdf_1,
						tItems[i].v_tdf_2, tItems[i].h_tdf_1, tItems[i].h_tdf_2);
			}

			if (tItems[i].run) {
				/* LCM off */
				if (strnstr(tItems[i].desp, "lcm", strlen(tItems[i].desp)) != NULL) {
					if (!core_mp->oppo_run && !core_mp->oppo_lcm)
						core_mp_ctrl_lcm_status(false);
				}

				ipio_info("Running Test Item : %s\n", tItems[i].desp);
				tItems[i].do_test(i);
				tItems[i].test_count ++;
				test_result = core_mp_compare_retry_cdc_result(i);

				/* To see if this item needs to do retry  */
				if (core_mp->retry && (test_result == MP_FAIL)) {
					for(count = 0; count < RETRY_COUNT; count++)
					{
						ipio_info("retry = %d, item = %s\n", (count+1), tItems[i].desp);
						core_mp_retry(i);
						test_result = core_mp_compare_retry_cdc_result(i);
						if(test_result == MP_PASS)
							break;
					}
				}
				tItems[i].item_result = test_result;
				/* LCM on */
				if (strnstr(tItems[i].desp, "lcm", strlen(tItems[i].desp)) != NULL) {
					if (!core_mp->oppo_run && !core_mp->oppo_lcm)
						core_mp_ctrl_lcm_status(true);
				}
			}
			break;
		}
	}
}
EXPORT_SYMBOL(core_mp_run_test);

int core_mp_move_code(void)
{
	int ret = 0;

	ipio_debug(DEBUG_MP_TEST, "Prepaing to enter Test Mode\n");
#ifdef HOST_DOWNLOAD
	ilitek_platform_tp_hw_reset(true);
#else
	if (core_config_check_cdc_busy(50, 50) < 0) {
		ipio_err("%d mode Check busy is timout ! Enter Test Mode failed\n",core_fr->actual_fw_mode);
		return -1;
	}

	if (core_config_ice_mode_enable() < 0) {
		ipio_err("Failed to enter ICE mode\n");
		return -1;
	}

	if (core_config_set_watch_dog(false) < 0) {
		ipio_err("Failed to disable watch dog\n");
	}

	/* DMA Trigger */
	core_config_ice_mode_write(0x41010, 0xFF, 1);

	msleep(30);

	/* CS High */
	core_config_ice_mode_write(0x041000, 0x1, 1);

	msleep(60);

	core_fr->actual_fw_mode = protocol->test_mode;
	/* Code reset */
	core_config_ice_mode_write(0x40040, 0xAE, 1);

	if (core_config_set_watch_dog(true) < 0) {
		ipio_err("Failed to enable watch dog\n");
	}

	core_config_ice_mode_disable();

	if (core_config_check_cdc_busy(300, 50) < 0) {
		ipio_err("Check busy is timout ! Enter Test Mode failed\n");
		return -1;
	}
#endif
	ipio_info("FW Test Mode ready\n");
	return ret;
}
EXPORT_SYMBOL(core_mp_move_code);

void core_mp_test_free(void)
{
	int i;

	ipio_debug(DEBUG_MP_TEST, "Free all allocated mem\n");

	core_mp->final_result = MP_FAIL;

	for (i = 0; i < ARRAY_SIZE(tItems); i++) {
		tItems[i].run = false;
		tItems[i].max_res = MP_FAIL;
		tItems[i].min_res = MP_FAIL;
		tItems[i].item_result = MP_PASS;
		tItems[i].test_count = 0;
		sprintf(tItems[i].result, "%s", "FAIL");

		if (tItems[i].catalog == TX_RX_DELTA) {
			ipio_vfree((void **)&core_mp->rx_delta_buf);
			ipio_vfree((void **)&core_mp->tx_delta_buf);
			ipio_vfree((void **)&core_mp->tx_max_buf);
			ipio_vfree((void **)&core_mp->tx_min_buf);
			ipio_vfree((void **)&core_mp->rx_max_buf);
			ipio_vfree((void **)&core_mp->rx_min_buf);
		} else {
			if (tItems[i].spec_option == BENCHMARK) {
				ipio_vfree((void **)&tItems[i].bench_mark_max);
				ipio_vfree((void **)&tItems[i].bench_mark_min);
			}

			ipio_vfree((void **)&tItems[i].result_buf);
			ipio_vfree((void **)&tItems[i].buf);
			ipio_vfree((void **)&tItems[i].max_buf);
			ipio_vfree((void **)&tItems[i].min_buf);
		}
	}

	ipio_kfree((void **)&frame_buf);
	ipio_kfree((void **)&key_buf);
	ipio_kfree((void **)&core_mp);
}
EXPORT_SYMBOL(core_mp_test_free);

static void mp_test_init_item(void)
{
	int i;

	core_mp->mp_items = ARRAY_SIZE(tItems);

	/* assign test functions run on MP flow according to their catalog */
	for (i = 0; i < ARRAY_SIZE(tItems); i++) {

		tItems[i].spec_option = 0;
		tItems[i].type_option = 0;
		tItems[i].run = false;
		tItems[i].max = 0;
		tItems[i].max_res = MP_FAIL;
		tItems[i].item_result = MP_PASS;
		tItems[i].min = 0;
		tItems[i].min_res = MP_FAIL;
		tItems[i].frame_count = 0;
		tItems[i].trimmed_mean = 0;
		tItems[i].lowest_percentage = 0;
		tItems[i].highest_percentage = 0;
		tItems[i].v_tdf_1 = 0;
		tItems[i].v_tdf_2 = 0;
		tItems[i].h_tdf_1 = 0;
		tItems[i].h_tdf_2 = 0;
		tItems[i].result_buf = NULL;
		tItems[i].buf = NULL;
		tItems[i].max_buf = NULL;
		tItems[i].min_buf = NULL;
		tItems[i].bench_mark_max = NULL;
		tItems[i].bench_mark_min = NULL;
		tItems[i].node_type = NULL;
		tItems[i].test_count = 0;

		if (tItems[i].catalog == MUTUAL_TEST) {
			tItems[i].do_test = mutual_test;
		} else if (tItems[i].catalog == TX_RX_DELTA) {
			tItems[i].do_test = mutual_test;
		} else if (tItems[i].catalog == UNTOUCH_P2P) {
			tItems[i].do_test = mutual_test;
		} else if (tItems[i].catalog == PIXEL) {
			tItems[i].do_test = mutual_test;
		} else if (tItems[i].catalog == OPEN_TEST){
			if (strcmp(tItems[i].name, "open_integration_sp") == 0)
				tItems[i].do_test = open_test_sp;
			else
				tItems[i].do_test = mutual_test;
		} else if (tItems[i].catalog == KEY_TEST) {
			tItems[i].do_test = key_test;
		} else if (tItems[i].catalog == SELF_TEST) {
			tItems[i].do_test = self_test;
		} else if (tItems[i].catalog == ST_TEST) {
			tItems[i].do_test = st_test;
		} else if (tItems[i].catalog == PEAK_TO_PEAK_TEST) {
			tItems[i].do_test = mutual_test;
		} else if (tItems[i].catalog == SHORT_TEST) {
			tItems[i].do_test = mutual_test;
		}

		tItems[i].result = kmalloc(16, GFP_KERNEL);
        sprintf(tItems[i].result, "%s", "FAIL");
	}

	/*
	 * assign protocol command written into firmware via I2C,
	 * which might be differnet if the version of protocol was changed.
	 */
	tItems[0].cmd = protocol->mutual_dac;
	tItems[1].cmd = protocol->mutual_bg;
	tItems[2].cmd = protocol->mutual_signal;
	tItems[3].cmd = protocol->mutual_no_bk;
	tItems[4].cmd = protocol->mutual_has_bk;
	tItems[5].cmd = protocol->mutual_bk_dac;
	tItems[6].cmd = protocol->self_dac;
	tItems[7].cmd = protocol->self_bg;
	tItems[8].cmd = protocol->self_signal;
	tItems[9].cmd = protocol->self_no_bk;
	tItems[10].cmd = protocol->self_has_bk;
	tItems[11].cmd = protocol->self_bk_dac;
	tItems[12].cmd = protocol->key_dac;
	tItems[13].cmd = protocol->key_bg;
	tItems[14].cmd = protocol->key_no_bk;
	tItems[15].cmd = protocol->key_has_bk;
	tItems[16].cmd = protocol->key_open;
	tItems[17].cmd = protocol->key_short;
	tItems[18].cmd = protocol->st_dac;
	tItems[19].cmd = protocol->st_bg;
	tItems[20].cmd = protocol->st_no_bk;
	tItems[21].cmd = protocol->st_has_bk;
	tItems[22].cmd = protocol->st_open;
	tItems[23].cmd = protocol->tx_short;
	tItems[24].cmd = protocol->rx_short;
	tItems[25].cmd = protocol->rx_open;
	tItems[26].cmd = protocol->cm_data;
	tItems[27].cmd = protocol->cs_data;
	tItems[28].cmd = protocol->tx_rx_delta;
	tItems[29].cmd = protocol->mutual_signal;
	tItems[30].cmd = protocol->mutual_no_bk;
	tItems[31].cmd = protocol->mutual_has_bk;
	tItems[32].cmd = protocol->rx_open;
	tItems[33].cmd = protocol->rx_open;
	tItems[34].cmd = protocol->peak_to_peak;
}

int core_mp_init(void)
{
	int res = 0;

	if (!ERR_ALLOC_MEM(core_config->tp_info)) {
		if (core_mp == NULL) {
			core_mp = kzalloc(sizeof(*core_mp), GFP_KERNEL);
			if (ERR_ALLOC_MEM(core_mp)) {
				ipio_err("Failed to init core_mp, %ld\n", PTR_ERR(core_mp));
				res = -ENOMEM;
				goto out;
			}

			core_mp->xch_len = core_config->tp_info->nXChannelNum;
			core_mp->ych_len = core_config->tp_info->nYChannelNum;

			core_mp->stx_len = core_config->tp_info->self_tx_channel_num;
			core_mp->srx_len = core_config->tp_info->self_rx_channel_num;

			core_mp->key_len = core_config->tp_info->nKeyCount;
			core_mp->st_len = core_config->tp_info->side_touch_type;

			core_mp->tdf = 240;
			core_mp->busy_cdc = INT_CHECK;

			core_mp->run = false;
			core_mp->retry = true;
			core_mp->oppo_run = false;
			core_mp->oppo_lcm = false;
			core_mp->final_result = MP_FAIL;

			mp_test_init_item();
		}
	} else {
		ipio_err("Failed to allocate core_mp mem as did not find TP info\n");
		res = -ENOMEM;
	}

out:
	return res;
}
EXPORT_SYMBOL(core_mp_init);
