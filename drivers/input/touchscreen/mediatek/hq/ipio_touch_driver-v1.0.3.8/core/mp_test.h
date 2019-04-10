/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** FILE: - mp_test.h
** Description : This program is for ili9881 driver mp_test.h
** Version: 1.0
** Date : 2018/5/17
** Author: Zhonghua.Hu@ODM_WT.BSP.TP-FP.TP
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/

#ifndef __MP_TEST_H
#define __MP_TEST_H

#define MP_PASS      1
#define MP_FAIL     -1
#define BENCHMARK 1
#define NODETYPE	 	1
#define RAWDATA_NO_BK_DATA_SHIFT_9881H 8192
#define RAWDATA_NO_BK_DATA_SHIFT_9881F 4096
#define TYPE_BENCHMARK 0
#define TYPE_NO_JUGE 1
#define TYPE_JUGE 2

enum mp_test_catalog {
	MUTUAL_TEST = 0,
	SELF_TEST = 1,
	KEY_TEST = 2,
	ST_TEST = 3,
	TX_RX_DELTA = 4,
	UNTOUCH_P2P = 5,
	PIXEL = 6,
	OPEN_TEST = 7,
	PEAK_TO_PEAK_TEST = 8,
	SHORT_TEST = 9,
};

struct mp_test_P540_open {
	int32_t *cbk_700;
	int32_t *cbk_250;
	int32_t *cbk_200;
	int32_t *charg_rate;
	int32_t *full_Open;
	int32_t *dac;
	int32_t *cdc;
};

struct mp_test_items {
	char *name;
	/* The description must be the same as ini's section name */
	char *desp;
	char *result;
	int catalog;
	uint8_t cmd;
	uint8_t spec_option;
	uint8_t type_option;
	bool run;
	int max;
	int max_res;
	int item_result;
	int test_count;
	int min;
	int min_res;
	int frame_count;
	int trimmed_mean;
	int lowest_percentage;
	int highest_percentage;
	int v_tdf_1;
	int v_tdf_2;
	int h_tdf_1;
	int h_tdf_2;
	int32_t *result_buf;
	int32_t *buf;
	int32_t *max_buf;
	int32_t *min_buf;
	int32_t *bench_mark_max;
	int32_t *bench_mark_min;
	int32_t *node_type;
	int (*do_test)(int index);
};


struct core_mp_test_data {
	bool isLongV;
	/* A flag shows a test run in particular */
	bool run;
	bool oppo_run;
	bool oppo_lcm;
	bool m_signal;
	bool m_dac;
	bool s_signal;
	bool s_dac;
	bool key_dac;
	bool st_dac;
	bool p_no_bk;
	bool p_has_bk;
	bool open_integ;
	bool open_cap;

	int xch_len;
	int ych_len;
	int stx_len;
	int srx_len;
	int key_len;
	int st_len;
	int frame_len;
	int mp_items;
	int final_result;

	/* Tx/Rx threshold & buffer */
	int TxDeltaMax;
	int TxDeltaMin;
	int RxDeltaMax;
	int RxDeltaMin;
	int32_t *tx_delta_buf;
	int32_t *rx_delta_buf;
	int32_t *tx_max_buf;
	int32_t *tx_min_buf;
	int32_t *rx_max_buf;
	int32_t *rx_min_buf;

	int tdf;
	bool retry;
	int8_t busy_cdc;
	bool ctrl_lcm;

	bool osc_test;
	int osc_offset;
	int osc_result;
	int osc_threshold_max;
	int osc_threshold_min;
	uint8_t osc_org_data[4];
};

extern struct core_mp_test_data *core_mp;
extern struct mp_test_items tItems[];

extern void dump_data(void *data, int type, int len, int row_len, const char *name);
extern int mp_get_timing_info(void);
extern int core_mp_ctrl_lcm_status(bool on);
extern void core_mp_test_free(void);
extern int ilitek_osc_check(void);
extern void core_mp_show_result(void);
extern void core_mp_run_test(char *item, bool ini);
extern int core_mp_move_code(void);
extern int core_mp_init(void);
#endif
