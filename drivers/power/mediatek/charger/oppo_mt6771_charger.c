/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/
 
/*
 *
 * Filename:
 * ---------
 *    mtk_charger.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines functions of Battery charging
 *
 * Author:
 * -------
 * Wy Chuang
 *
 */
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/reboot.h>

#include "mtk_charger_intf.h"
#include <mt-plat/charger_type.h>
#include <mt-plat/mtk_battery.h>
#include <mt-plat/mtk_boot.h>
#include <musb_core.h>
#include <pmic.h>
#include <mtk_gauge_time_service.h>

#ifdef VENDOR_EDIT
/* Qiao.Hu@EXP.BSP.CHG.basic, 2017/07/20, Add for charger */
#include <soc/oppo/device_info.h>
#include <soc/oppo/oppo_project.h>
#include <linux/gpio.h>
#include "../../oppo/oppo_gauge.h"
#include "../../oppo/oppo_charger.h"
#include <linux/of_gpio.h>

#endif  /*VENDOR_EDIT*/
#ifdef VENDOR_EDIT
/* Qiao.Hu@EXP.BSP.BaseDrv.CHG.Basic, 2017/08/15, Add for charger full status */
extern bool oppo_chg_check_chip_is_null(void);
#endif /* VENDOR_EDIT */
#ifdef VENDOR_EDIT
/* Qiao.Hu@EXP.BSP.BaseDrv.CHG.Basic, 2017/08/15, Add for charger full status */
extern bool oppo_chg_check_chip_is_null(void);
#endif /* VENDOR_EDIT */

#ifdef VENDOR_EDIT
/************ kpoc_charger *******************/
//huangtongfeng@BSP.CHG.Basic, 2017/12/14, add for kpoc charging param.
extern int oppo_chg_get_ui_soc(void);
extern int oppo_chg_get_notify_flag(void);
extern bool pmic_chrdet_status(void);
extern int oppo_get_prop_status(void);
struct oppo_chg_chip *g_oppo_chip = NULL;
int oppo_usb_switch_gpio_gpio_init(void);
static bool oppo_ship_check_is_gpio(struct oppo_chg_chip *chip);
int oppo_ship_gpio_init(struct oppo_chg_chip *chip);
void smbchg_enter_shipmode(struct oppo_chg_chip *chip);
extern struct oppo_chg_operations  bq24190_chg_ops;
extern struct oppo_chg_operations  bq25890h_chg_ops;
extern struct oppo_chg_operations  bq25601d_chg_ops;
extern struct oppo_chg_operations * oppo_get_chg_ops(void);
// add begin by weijun@
extern struct oppo_chg_operations  oppo_mt6370_chg_ops;
// add end

int oppo_battery_meter_get_battery_voltage(void)
{
	//return battery_get_bat_voltage();
	return 4000;
}
#endif /* VENDOR_EDIT */


#ifdef VENDOR_EDIT
/* Qiao.Hu@BSP.BaseDrv.CHG.Basic, 2017/11/19, Add for charging */
bool meter_fg_30_get_battery_authenticate(void);
#endif /* VENDOR_EDIT */
#ifdef VENDOR_EDIT
/* Qiao.Hu@BSP.BaseDrv.CHG.Basic, 2017/11/19, Add for charging */
int charger_ic_flag = 1;
int oppo_which_charger_ic(void)
{
    return charger_ic_flag;
}
#endif /* VENDOR_EDIT */


#ifdef VENDOR_EDIT
/* Jianchao.Shi@BSP.CHG.Basic, 2017/01/22, sjc Add for charging*/
static int oppo_chg_parse_custom_dt(struct oppo_chg_chip *chip)
{
	int rc = 0;
	struct device_node *node = NULL;

	if (chip)
		node = chip->dev->of_node;
	if (!node) {
			pr_err("device tree node missing\n");
			return -EINVAL;
	}

#ifdef VENDOR_EDIT
/* Jianchao.Shi@BSP.CHG.Basic, 2017/01/22, sjc Add for charging*/
	if (chip) {
		chip->normalchg_gpio.chargerid_switch_gpio =
				of_get_named_gpio(node, "qcom,chargerid_switch-gpio", 0);
		if (chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
			chg_err("Couldn't read chargerid_switch-gpio rc = %d, chargerid_switch_gpio:%d\n",
					rc, chip->normalchg_gpio.chargerid_switch_gpio);
		} else {
			if (gpio_is_valid(chip->normalchg_gpio.chargerid_switch_gpio)) {
				rc = gpio_request(chip->normalchg_gpio.chargerid_switch_gpio, "charging-switch1-gpio");
				if (rc) {
					chg_err("unable to request chargerid_switch_gpio:%d\n", chip->normalchg_gpio.chargerid_switch_gpio);
				} else {
					//smbchg_chargerid_switch_gpio_init(chip);
                    oppo_usb_switch_gpio_gpio_init();
				}
			}
			chg_err("chargerid_switch_gpio:%d\n", chip->normalchg_gpio.chargerid_switch_gpio);
		}
	}
#endif /*VENDOR_EDIT*/

#ifdef VENDOR_EDIT
/* Jianchao.Shi@BSP.CHG.Basic, 2018/03/02, sjc Add for using gpio as shipmode stm6620 */
	if (chip) {
		chip->normalchg_gpio.ship_gpio =
				of_get_named_gpio(node, "qcom,ship-gpio", 0);
		if (chip->normalchg_gpio.ship_gpio <= 0) {
			chg_err("Couldn't read qcom,ship-gpio rc = %d, qcom,ship-gpio:%d\n",
					rc, chip->normalchg_gpio.ship_gpio);
		} else {
			if (oppo_ship_check_is_gpio(chip) == true) {
				rc = gpio_request(chip->normalchg_gpio.ship_gpio, "ship-gpio");
				if (rc) {
					chg_err("unable to request ship-gpio:%d\n", chip->normalchg_gpio.ship_gpio);
				} else {
					oppo_ship_gpio_init(chip);
					if (rc)
						chg_err("unable to init ship-gpio:%d\n", chip->normalchg_gpio.ship_gpio);
				}
			}
			chg_err("ship-gpio:%d\n", chip->normalchg_gpio.ship_gpio);
		}
	}
#endif /*VENDOR_EDIT*/

	return rc;
}
#endif /*VENDOR_EDIT*/

#ifdef VENDOR_EDIT
extern enum charger_type oppo_mt6370_chr_type_check(void); // add by weijun

extern bool upmu_is_chr_det(void);
extern enum charger_type g_chr_type;
extern int otg_is_exist;

#ifdef VENDOR_EDIT//Qiao.Hu@BSP.BaseDrv.CHG.Basic,add 2017/12/09 for shipmode  stm6620


static bool oppo_ship_check_is_gpio(struct oppo_chg_chip *chip)
{
	if (gpio_is_valid(chip->normalchg_gpio.ship_gpio))
		return true;

	return false;
}

int oppo_ship_gpio_init(struct oppo_chg_chip *chip)
{
	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);
	chip->normalchg_gpio.ship_active = 
		pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, 
			"ship_active");

	if (IS_ERR_OR_NULL(chip->normalchg_gpio.ship_active)) {
		chg_err("get ship_active fail\n");
		return -EINVAL;
	}
	chip->normalchg_gpio.ship_sleep = 
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, 
				"ship_sleep");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.ship_sleep)) {
		chg_err("get ship_sleep fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chip->normalchg_gpio.pinctrl,
		chip->normalchg_gpio.ship_sleep);
	return 0;
}

#define SHIP_MODE_CONFIG		0x40
#define SHIP_MODE_MASK			BIT(0)
#define SHIP_MODE_ENABLE		0
#define PWM_COUNT				5
void smbchg_enter_shipmode(struct oppo_chg_chip *chip)
{
	int i = 0;
	chg_err("enter smbchg_enter_shipmode\n");

	if (oppo_ship_check_is_gpio(chip) == true) {
		chg_err("select gpio control\n");
		if (!IS_ERR_OR_NULL(chip->normalchg_gpio.ship_active) && !IS_ERR_OR_NULL(chip->normalchg_gpio.ship_sleep)) {
			pinctrl_select_state(chip->normalchg_gpio.pinctrl,
				chip->normalchg_gpio.ship_sleep);
			for (i = 0; i < PWM_COUNT; i++) {
				//gpio_direction_output(chip->normalchg_gpio.ship_gpio, 1);
				pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.ship_active);
				mdelay(3);
				//gpio_direction_output(chip->normalchg_gpio.ship_gpio, 0);
				pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.ship_sleep);
				mdelay(3);
			}
		}
		chg_err("power off after 15s\n");
	}
}
void enter_ship_mode_function(struct oppo_chg_chip *chip)
{
	if(chip != NULL){
		if (chip->enable_shipmode) {
			printk("enter_ship_mode_function\n");
			smbchg_enter_shipmode(chip);
		}
	}
}

#endif /* VENDOR_EDIT */

int mt_power_supply_type_check(void)
{
	int charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
	int charger_type_final = 0;
	if(otg_is_exist == 1)
		return g_chr_type;
	pr_err("mt_power_supply_type_check-----1---------charger_type = %d\r\n",charger_type);
	if(true == upmu_is_chr_det()) {
		if (charger_ic_flag == 3) { // add for weijun
			charger_type_final = oppo_mt6370_chr_type_check(); // add for weijun
		} // add for weijun
		g_chr_type = charger_type_final;
	}
	else {
		chg_debug(" call first type\n");
		charger_type_final = g_chr_type;
	}

	switch(charger_type_final) {
	case CHARGER_UNKNOWN:
		break;
	case STANDARD_HOST:
	case CHARGING_HOST:
		charger_type = POWER_SUPPLY_TYPE_USB;
		break;
	case NONSTANDARD_CHARGER:
	case APPLE_0_5A_CHARGER:
	case STANDARD_CHARGER:
	case APPLE_2_1A_CHARGER:
	case APPLE_1_0A_CHARGER:
		charger_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		break;
	}
	pr_err("mt_power_supply_type_check-----2---------charger_type = %d,charger_type_final = %d,g_chr_type=%d\r\n",charger_type,charger_type_final,g_chr_type);
	return charger_type;

}


enum {
    Channel_12 = 2,
    Channel_13,
    Channel_14,
};
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern int IMM_IsAdcInitReady(void);
int mt_vadc_read(int times, int Channel)
{
    int ret = 0, data[4], i, ret_value = 0, ret_temp = 0;
    if( IMM_IsAdcInitReady() == 0 )
    {
        return 0;
    }
    i = times ;
    while (i--)
    {
	ret_value = IMM_GetOneChannelValue(Channel, data, &ret_temp);
	if(ret_value != 0)
	{
		i++;
		continue;
	}
	ret += ret_temp;
    }
	ret = ret*1500/4096;
    ret = ret/times;
	chg_debug("[mt_vadc_read] Channel %d: vol_ret=%d\n",Channel,ret);
	return ret;
}
static void set_usbswitch_to_rxtx(struct oppo_chg_chip *chip)
{
	int ret = 0;
	gpio_direction_output(chip->normalchg_gpio.chargerid_switch_gpio, 1);
	ret = pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.charger_gpio_as_output2);
	if (ret < 0) {
		chg_err("failed to set pinctrl int\n");
		return ;
	}
}
static void set_usbswitch_to_dpdm(struct oppo_chg_chip *chip)
{
	int ret = 0;
	gpio_direction_output(chip->normalchg_gpio.chargerid_switch_gpio, 0);
	ret = pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.charger_gpio_as_output1);
	if (ret < 0) {
		chg_err("failed to set pinctrl int\n");
		return ;
	}
	chg_err("set_usbswitch_to_dpdm \n");
}
static bool is_support_chargerid_check(void)
{

#ifdef CONFIG_OPPO_CHECK_CHARGERID_VOLT
	return true;
#else
	return false;
#endif

}
int mt_get_chargerid_volt (void)
{
	int chargerid_volt = 0;
	if(is_support_chargerid_check() == true)
	{
		chargerid_volt = mt_vadc_read(10,Channel_14);//get the charger id volt
		chg_debug("chargerid_volt = %d \n",
					   chargerid_volt);
	}
		else
		{
		chg_debug("is_support_chargerid_check = false !\n");
		return 0;
	}
	return chargerid_volt;
		}


void mt_set_chargerid_switch_val(int value)
{
	chg_debug("set_value= %d\n",value);
	if(NULL == g_oppo_chip)
		return;
	if(is_support_chargerid_check() == false)
		return;
	if(g_oppo_chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("chargerid_switch_gpio not exist, return\n");
		return;
	}
	if(IS_ERR_OR_NULL(g_oppo_chip->normalchg_gpio.pinctrl)
		|| IS_ERR_OR_NULL(g_oppo_chip->normalchg_gpio.charger_gpio_as_output1)
		|| IS_ERR_OR_NULL(g_oppo_chip->normalchg_gpio.charger_gpio_as_output2)) {
		chg_err("pinctrl null, return\n");
		return;
	}
	if(1 == value){
			set_usbswitch_to_rxtx(g_oppo_chip);
	}else if(0 == value){
		set_usbswitch_to_dpdm(g_oppo_chip);
	}else{
		//do nothing
	}
	chg_debug("get_val:%d\n",gpio_get_value(g_oppo_chip->normalchg_gpio.chargerid_switch_gpio));
}

int mt_get_chargerid_switch_val(void)
{
	int gpio_status = 0;
	if(NULL == g_oppo_chip)
		return 0;
	if(is_support_chargerid_check() == false)
		return 0;
	gpio_status = gpio_get_value(g_oppo_chip->normalchg_gpio.chargerid_switch_gpio);

	chg_debug("mt_get_chargerid_switch_val:%d\n",gpio_status);
	return gpio_status;
}


int oppo_usb_switch_gpio_gpio_init(void)
{
	chg_err("---1-----");
	g_oppo_chip->normalchg_gpio.pinctrl = devm_pinctrl_get(g_oppo_chip->dev);
    if (IS_ERR_OR_NULL(g_oppo_chip->normalchg_gpio.pinctrl)) {
       chg_err("get normalchg_gpio.chargerid_switch_gpio pinctrl falil\n");
		return -EINVAL;
    }
    g_oppo_chip->normalchg_gpio.charger_gpio_as_output1 = pinctrl_lookup_state(g_oppo_chip->normalchg_gpio.pinctrl,
								"charger_gpio_as_output_low");
    if (IS_ERR_OR_NULL(g_oppo_chip->normalchg_gpio.charger_gpio_as_output1)) {
       	chg_err("get charger_gpio_as_output_low fail\n");
			return -EINVAL;
    }
	g_oppo_chip->normalchg_gpio.charger_gpio_as_output2 = pinctrl_lookup_state(g_oppo_chip->normalchg_gpio.pinctrl,
								"charger_gpio_as_output_high");
	if (IS_ERR_OR_NULL(g_oppo_chip->normalchg_gpio.charger_gpio_as_output2)) {
		chg_err("get charger_gpio_as_output_high fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(g_oppo_chip->normalchg_gpio.pinctrl,g_oppo_chip->normalchg_gpio.charger_gpio_as_output1);	
	return 0;
}


int charger_pretype_get(void)
{
	int chg_type = STANDARD_HOST;
	//chg_type = hw_charging_get_charger_type();
	return chg_type;
}


bool oppo_pmic_check_chip_is_null(void)
{
    return true;
}

#endif


/************************************************/
/* Power Supply Functions
*************************************************/
static int mt_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
    int rc = 0;
    rc = oppo_ac_get_property(psy, psp, val);

	return 0;
}

static int mt_usb_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int rc = 0;
	rc = oppo_usb_property_is_writeable(psy, psp);
	return rc;
}

static int mt_usb_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
#if 0
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = 500000;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = 5000000;
		return 0;
	default:
		;
	}
#endif
	oppo_usb_get_property(psy, psp, val);
	return 0;
}

static int mt_usb_set_property(struct power_supply *psy,
	enum power_supply_property psp, const union power_supply_propval *val)
{
	oppo_usb_set_property(psy, psp, val);
	return 0;
}

static int battery_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int rc = 0;
	rc = oppo_battery_property_is_writeable(psy, psp);
	return rc;
}

static int battery_set_property(struct power_supply *psy,
	enum power_supply_property psp, const union power_supply_propval *val)
{
	oppo_battery_set_property(psy, psp, val);
	return 0;
}

static int battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	oppo_battery_get_property(psy, psp, val);
	return 0;
}


static enum power_supply_property mt_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property mt_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_OTG_SWITCH,
	POWER_SUPPLY_PROP_OTG_ONLINE,
    
};
static enum power_supply_property battery_properties[] = {
        POWER_SUPPLY_PROP_STATUS,
        POWER_SUPPLY_PROP_HEALTH,
        POWER_SUPPLY_PROP_PRESENT,
        POWER_SUPPLY_PROP_CAPACITY,
        POWER_SUPPLY_PROP_TEMP,
        POWER_SUPPLY_PROP_VOLTAGE_NOW,
        POWER_SUPPLY_PROP_VOLTAGE_MIN,
        POWER_SUPPLY_PROP_CURRENT_NOW,
        POWER_SUPPLY_PROP_CHARGE_NOW,
        POWER_SUPPLY_PROP_AUTHENTICATE,
        POWER_SUPPLY_PROP_CHARGE_TIMEOUT,
        POWER_SUPPLY_PROP_MMI_CHARGING_ENABLE,        /*add for MMI_CHG_TEST*/
#if defined(CONFIG_OPPO_CHARGER_MTK6763) || defined(CONFIG_OPPO_CHARGER_MTK6771)
        POWER_SUPPLY_PROP_STOP_CHARGING_ENABLE,
#endif
#if defined(CONFIG_OPPO_CHARGER_MTK6771)
        POWER_SUPPLY_PROP_CHARGE_FULL,
        POWER_SUPPLY_PROP_CHARGE_COUNTER,
        POWER_SUPPLY_PROP_CURRENT_MAX,
#endif
        POWER_SUPPLY_PROP_BATTERY_FCC,
        POWER_SUPPLY_PROP_BATTERY_SOH,
        POWER_SUPPLY_PROP_BATTERY_CC,
        POWER_SUPPLY_PROP_BATTERY_RM,
        POWER_SUPPLY_PROP_BATTERY_NOTIFY_CODE,
#ifdef CONFIG_OPPO_CHECK_CHARGERID_VOLT
        POWER_SUPPLY_PROP_CHARGERID_VOLT,
#endif
#ifdef CONFIG_OPPO_SHIP_MODE_SUPPORT
        POWER_SUPPLY_PROP_SHIP_MODE,
#endif
#ifdef CONFIG_OPPO_CALL_MODE_SUPPORT
        POWER_SUPPLY_PROP_CALL_MODE,
#endif
#ifdef CONFIG_OPPO_SHORT_C_BATT_CHECK
#ifdef CONFIG_OPPO_SHORT_USERSPACE
        POWER_SUPPLY_PROP_SHORT_C_LIMIT_CHG,
        POWER_SUPPLY_PROP_SHORT_C_LIMIT_RECHG,
        POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
        POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
#else
        POWER_SUPPLY_PROP_SHORT_C_BATT_UPDATE_CHANGE,
        POWER_SUPPLY_PROP_SHORT_C_BATT_IN_IDLE,
        POWER_SUPPLY_PROP_SHORT_C_BATT_CV_STATUS,
#endif /*CONFIG_OPPO_SHORT_USERSPACE*/
#endif
#ifdef CONFIG_OPPO_SHORT_HW_CHECK
        POWER_SUPPLY_PROP_SHORT_C_HW_FEATURE,
        POWER_SUPPLY_PROP_SHORT_C_HW_STATUS,
#endif
};


static int oppo_power_supply_init(struct oppo_chg_chip *chip)
{
    int ret = 0;
    struct oppo_chg_chip *mt_chg = NULL;
    mt_chg = chip;
    mt_chg->ac_psd.name = "ac";
    mt_chg->ac_psd.type = POWER_SUPPLY_TYPE_MAINS;
    mt_chg->ac_psd.properties = mt_ac_properties;
    mt_chg->ac_psd.num_properties = ARRAY_SIZE(mt_ac_properties);
    mt_chg->ac_psd.get_property = mt_ac_get_property;
    mt_chg->ac_cfg.drv_data = mt_chg;

    mt_chg->usb_psd.name = "usb";
    mt_chg->usb_psd.type = POWER_SUPPLY_TYPE_USB;
    mt_chg->usb_psd.properties = mt_usb_properties;
    mt_chg->usb_psd.num_properties = ARRAY_SIZE(mt_usb_properties);
    mt_chg->usb_psd.get_property = mt_usb_get_property;
    mt_chg->usb_psd.set_property = mt_usb_set_property;
    mt_chg->usb_psd.property_is_writeable = mt_usb_prop_is_writeable;
    mt_chg->usb_cfg.drv_data = mt_chg;
    
    mt_chg->battery_psd.name = "battery";
    mt_chg->battery_psd.type = POWER_SUPPLY_TYPE_BATTERY;
    mt_chg->battery_psd.properties = battery_properties;
    mt_chg->battery_psd.num_properties = ARRAY_SIZE(battery_properties);
    mt_chg->battery_psd.get_property = battery_get_property;
    mt_chg->battery_psd.set_property = battery_set_property;
    mt_chg->battery_psd.property_is_writeable = battery_prop_is_writeable,


    mt_chg->ac_psy = power_supply_register(mt_chg->dev, &mt_chg->ac_psd,
        &mt_chg->ac_cfg);
    if (IS_ERR(mt_chg->ac_psy)) {
        dev_err(mt_chg->dev, "Failed to register power supply: %ld\n",
            PTR_ERR(mt_chg->ac_psy));
        ret = PTR_ERR(mt_chg->ac_psy);
        goto err_ac_psy;
    }
    mt_chg->usb_psy = power_supply_register(mt_chg->dev, &mt_chg->usb_psd,
        &mt_chg->usb_cfg);
    if (IS_ERR(mt_chg->usb_psy)) {
        dev_err(mt_chg->dev, "Failed to register power supply: %ld\n",
            PTR_ERR(mt_chg->usb_psy));
        ret = PTR_ERR(mt_chg->usb_psy);
        goto err_usb_psy;
    }
    mt_chg->batt_psy = power_supply_register(mt_chg->dev, &mt_chg->battery_psd,
        NULL);
    if (IS_ERR(mt_chg->batt_psy)) {
        dev_err(mt_chg->dev, "Failed to register power supply: %ld\n",
            PTR_ERR(mt_chg->batt_psy));
        ret = PTR_ERR(mt_chg->batt_psy);
        goto err_battery_psy;
    }
    pr_info("%s\n", __func__);
    return 0;

err_usb_psy:
    power_supply_unregister(mt_chg->ac_psy);
err_ac_psy:
    power_supply_unregister(mt_chg->usb_psy);
err_battery_psy:
    power_supply_unregister(mt_chg->batt_psy);

    return ret;
}


static int oppo_charger_probe(struct platform_device *pdev)
{
#ifdef VENDOR_EDIT
       /* Jianchao.Shi@BSP.CHG.Basic, 2016/12/26, sjc Add for charging*/
        int ret = 0;       
        struct oppo_chg_chip *oppo_chip = NULL;
        //struct mt_charger *mt_chg = NULL;
        printk("oppo_charger_probe\n");
        oppo_chip = devm_kzalloc(&pdev->dev,sizeof(struct oppo_chg_chip), GFP_KERNEL);
        if (!oppo_chip) {
            chg_err(" kzalloc() failed\n");
            return -ENOMEM;
        }
        oppo_chip->dev = &pdev->dev;
        
        ret = oppo_power_supply_init(oppo_chip);

        if (oppo_gauge_check_chip_is_null()) {
            chg_err("gauge chip null, will do after bettery init.\n");
            return -EPROBE_DEFER;
        }
        if (oppo_which_charger_ic() == -1) {
            chg_err("charger IC is null, will do after bettery init.\n");
            return -EPROBE_DEFER;
        } else if (oppo_which_charger_ic() == 3) { // add by weijun
            oppo_chip->chg_ops = &oppo_mt6370_chg_ops; // add by weijun
        }

        printk("oppo_charger_probe end %p, prev %p, next %p\n",&oppo_chip->batt_psy->dev.power.wakeup->entry,
            oppo_chip->batt_psy->dev.power.wakeup->entry.prev,oppo_chip->batt_psy->dev.power.wakeup->entry.next);
        g_oppo_chip = oppo_chip;
        oppo_chip->chg_ops->hardware_init();
        oppo_chip->authenticate = oppo_gauge_get_batt_authenticate();
        oppo_chg_parse_custom_dt(oppo_chip);
        oppo_chg_parse_charger_dt(oppo_chip);
        oppo_chg_init(oppo_chip);
        platform_set_drvdata(pdev, oppo_chip);
	    device_init_wakeup(&pdev->dev, 1);
        oppo_chg_wake_update_work();

        return 0;
#endif

}

static int oppo_charger_remove(struct platform_device *dev)
{
	return 0;
}

static void oppo_charger_shutdown(struct platform_device *dev)
{
	if (g_oppo_chip != NULL)
		enter_ship_mode_function(g_oppo_chip);
}

static const struct of_device_id oppo_charger_of_match[] = {
	{.compatible = "mediatek,oppo-charger",},
	{},
};

MODULE_DEVICE_TABLE(of, oppo_charger_of_match);

struct platform_device oppo_charger_device = {
	.name = "oppo_charger",
	.id = -1,
};

static struct platform_driver oppo_charger_driver = {
	.probe = oppo_charger_probe,
	.remove = oppo_charger_remove,
	.shutdown = oppo_charger_shutdown,
	.driver = {
		   .name = "oppo_charger",
		   .of_match_table = oppo_charger_of_match,
		   },
};


static int __init oppo_charger_init(void)
{
	return platform_driver_register(&oppo_charger_driver);
}
late_initcall(oppo_charger_init);


static void __exit mtk_charger_exit(void)
{
	platform_driver_unregister(&oppo_charger_driver);
}
module_exit(mtk_charger_exit);


MODULE_AUTHOR("wy.chuang <wy.chuang@mediatek.com>");
MODULE_DESCRIPTION("MTK Charger Driver");
MODULE_LICENSE("GPL");
