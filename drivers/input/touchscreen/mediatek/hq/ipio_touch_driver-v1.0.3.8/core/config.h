/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** FILE: - config.c
** Description : This program is for ili9881 driver config.c
** Version: 1.0
** Date : 2018/5/17
** Author: Zhonghua.Hu@ODM_WT.BSP.TP-FP.TP
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/

#ifndef __CONFIG_H
#define __CONFIG_H
#include <soc/oppo/device_info.h>


/* chenyunrui@RM.BSP.TP.FUNCTION, 2018/11/10, Add start */
#define MAX_FW_NAME_LENGTH 60
#define MAX_DEVICE_VERSION_LENGTH 16
#define MAX_DEVICE_MANU_LENGTH 16
/* chenyunrui@RM.BSP.TP.FUNCTION, 2018/11/10, Add end */


typedef struct {
	int nId;
	int nX;
	int nY;
	int nStatus;
	int nFlag;
} VIRTUAL_KEYS;

typedef struct {
	uint16_t nMaxX;
	uint16_t nMaxY;
	uint16_t nMinX;
	uint16_t nMinY;

	uint8_t nMaxTouchNum;
	uint8_t nMaxKeyButtonNum;

	uint8_t nXChannelNum;
	uint8_t nYChannelNum;
	uint8_t nHandleKeyFlag;
	uint8_t nKeyCount;

	uint16_t nKeyAreaXLength;
	uint16_t nKeyAreaYLength;

	VIRTUAL_KEYS virtual_key[10];

	/* added for protocol v5 */
	uint8_t self_tx_channel_num;
	uint8_t self_rx_channel_num;
	uint8_t side_touch_type;

} TP_INFO;

struct core_config_data {
	uint32_t chip_id;
	uint32_t chip_type;
	uint32_t chip_pid;

	uint8_t core_type;
	uint32_t slave_i2c_addr;
	uint32_t ice_mode_addr;
	uint32_t pid_addr;
	uint32_t wdt_addr;
	uint32_t ic_reset_addr;

	uint8_t protocol_ver[4];
	uint8_t firmware_ver[9];
	uint8_t core_ver[5];
#ifdef ILITEK_ESD_CHECK
    uint8_t esd_data;
#endif
	bool do_ic_reset;
	bool gesture_backup;
	bool isEnableGesture;
	bool gameSwitch;
	bool icemodeenable;
	bool spi_pro_9881h11;
	TP_INFO *tp_info;
	int direction;
	uint8_t gesture_flag_bck;
	uint8_t wr_update_type;
	struct manufacture_info ilitek_ts_info;
};

extern struct core_config_data *core_config;

extern int fw_cmd_len;
extern int protocol_cmd_len;
extern int tp_info_len;
extern int key_info_len;
extern int core_cmd_len;

/* R/W with Touch ICs */
extern uint32_t core_config_ice_mode_read(uint32_t addr);
extern int core_config_ice_mode_write(uint32_t addr, uint32_t data, uint32_t size);
extern uint32_t core_config_read_write_onebyte(uint32_t addr);
extern int core_config_ice_mode_disable(void);
extern int core_config_ice_mode_enable(void);

/* Touch IC status */
extern int core_config_set_watch_dog(bool enable);
extern int core_config_check_cdc_busy(int count, int delay);
extern int core_config_check_int_status(bool high);
extern void core_config_ic_suspend(void);
extern void core_config_ic_resume(void);
extern void core_config_ic_reset(void);

/* control features of Touch IC */
extern void core_config_sense_ctrl(bool start);
extern void core_config_sleep_ctrl(bool out);
extern void core_config_glove_ctrl(bool enable, bool seamless);
extern void core_config_stylus_ctrl(bool enable, bool seamless);
extern void core_config_tp_scan_mode(bool mode);
extern void core_config_lpwg_ctrl(bool enable);
extern void core_config_gesture_ctrl(uint8_t func);
extern void core_config_phone_cover_ctrl(bool enable);
extern void core_config_finger_sense_ctrl(bool enable);
extern void core_config_proximity_ctrl(bool enable);
extern void core_config_plug_ctrl(bool out);
extern void core_config_set_phone_cover(uint8_t *pattern);

extern uint32_t core_config_get_reg_data(uint32_t addr);
extern void core_get_ddi_register_onlyone(uint8_t page, uint8_t reg);
extern void core_set_ddi_register_onlyone(uint8_t page, uint8_t reg, uint8_t data);

#ifdef ILITEK_EDGE_LIMIT
extern void core_config_edge_limit_ctrl(bool enable);
#endif
extern void core_config_game_switch_ctrl(bool enable);
/* Touch IC information */
extern int core_config_get_core_ver(void);


//extern int core_config_get_key_info(void);
extern int core_config_get_tp_info(void);
extern int core_config_get_protocol_ver(void);
extern int core_config_get_fw_ver(void);
extern int core_config_get_chip_id(void);

extern int core_config_init(void);
extern void core_config_remove(void);

#endif /* __CONFIG_H */
