/* Huaqin  Inc. (C) 2011. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("HUAQIN SOFTWARE")
 * RECEIVED FROM HUAQIN AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. HUAQIN EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES HUAQIN PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE HUAQIN SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN HUAQIN SOFTWARE. HUAQIN SHALL ALSO NOT BE RESPONSIBLE FOR ANY HUAQIN
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND HUAQIN'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE HUAQIN SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT HUAQIN'S OPTION, TO REVISE OR REPLACE THE HUAQIN SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * HUAQIN FOR SUCH HUAQIN SOFTWARE AT ISSUE.
 *
 */

/*******************************************************************************
* Dependency
*******************************************************************************/
#ifdef ODM_HQ_EDIT //2018/10/13,add hq hardware info
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/hardware_info.h>
#if defined(CONFIG_NANOHUB) && defined(CONFIG_CUSTOM_KERNEL_SENSORHUB)
#include "../sensors-1.0/sensorHub/inc_v1/SCP_sensorHub.h"
#include "../sensors-1.0/hwmon/include/hwmsensor.h"
#endif

#define HARDWARE_INFO_VERSION   "MT6771"
#define HARDWARE_INFO_WCN           "MT6631"

#ifdef CONFIG_HQ_BOARDINFO_BY_HW
#define AUX_ADC_EXTERNAL_CHANNEL   3
#define AUX_ADC_VALUE_TOLERANCE   100
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static BOARD_VERSION_TABLE revision_table[] =
{
        { .adc_vol = 1200, .revision = "PDP", },

        { .adc_vol = 1000, .revision = "DP", },

        { .adc_vol = 800, .revision = "SP1", },

        { .adc_vol = 600, .revision = "SP2", },

        { .adc_vol = 400, .revision = "AP", },

        { .adc_vol = 200, .revision = "PQ", },

        { .adc_vol = 000, .revision = "RU", },
};
#endif

/******************************************************************************
 * EMMC Configuration
*******************************************************************************/
//adb shell cat /sys/devices/mtk-msdc.0/11230000.msdc0/mmc_host/mmc0/mmc0:0001/cid------>/sys/devices/platform/bootdevice/mmc_host/mmc0/mmc0:0001

static EMMC_TABLE emmc_table[] =
{
	//zuoqiquan@ODM.Factory.Emmc.info id.2018/11/27 add for hardinfo Emmc info.
	// KMGD6001BM_B421 + Hynix  3 + 32  1st-1
	{
		.cid = "150100474436424d42015cf6cec6752f",
		.name = "Samsun 3GB+32GB KMGD6001BM_B421-DDR3",
	},
	// H9TQ27ADFTMCUR_KUM + Hynix  3 + 32  1st-2
	{
		.cid = "90014a68423861503e031758b052850d",
		.name = "Hynix 3GB+32GB H9TQ27ADFTMCUR_KUM-DDR3",
	},
	// H9TQ52ACLTMCUR_KUM + Hynix  4 + 64  2st-1
	{
		.cid = "90014a484347386134a249051a368541",
		.name = "Hynix 4GB+64B H9TQ52ACLTMCUR_KUM-DDR3",
	},
	// MT29TZZZAD8DKKBT_107W_9F8 + Hynix  4 + 64  2st-2
	{
		.cid = "150100474436424d42015cf0ce7e75bb",
		.name = "Micron 4GB+64B MT29TZZZAD8DKKBT_107W_9F8-DDR3",
	},

};

/******************************************************************************
 * Hardware Info Driver
*************************`*****************************************************/
struct global_otp_struct hw_info_main_otp;
struct global_otp_struct hw_info_sub_otp;
struct global_otp_struct hw_info_main2_otp;
static HARDWARE_INFO hwinfo_data;
void get_hardware_info_data(enum hardware_id id, const void *data)
{
        if (NULL == data) {
                printk("%s the data of hwid %d is NULL\n", __func__, id);
        } else {
                switch (id) {
                case HWID_LCM:
                        hwinfo_data.lcm = data;
                        break;
                case HWID_CTP_DRIVER:
                        hwinfo_data.ctp_driver = data;
                        break;
                case HWID_CTP_MODULE:
                        hwinfo_data.ctp_module = data;
                        break;
                case HWID_CTP_FW_VER:
                        strcpy(hwinfo_data.ctp_fw_version,data);
                        break;
                case HWID_CTP_FW_INFO:
                        hwinfo_data.ctp_fw_info = data;
                        break;
                case HWID_MAIN_CAM:
                        hwinfo_data.main_camera = data;
                        break;
                case HWID_SUB_CAM:
                        hwinfo_data.sub_camera = data;
                        break;
                case HWID_FLASH:
                        hwinfo_data.flash = data;
                        break;
                case HWID_ALSPS:
                        hwinfo_data.alsps = data;
                        break;
                case HWID_GSENSOR:
                        hwinfo_data.gsensor = data;
                        break;
                case HWID_GYROSCOPE:
                        hwinfo_data.gyroscope = data;
                        break;
                case HWID_MSENSOR:
                        hwinfo_data.msensor = data;
                        break;
                case HWID_SAR_SENSOR_1:
                        hwinfo_data.sar_sensor_1 = data;
                        break;
                case HWID_SAR_SENSOR_2:
                        hwinfo_data.sar_sensor_2 = data;
                        break;
                case HWID_BATERY_ID:
                        hwinfo_data.bat_id = data;
                        break;
                case HWID_NFC:
                        hwinfo_data.nfc = data;
                        break;
                case HWID_FINGERPRINT:
                        hwinfo_data.fingerprint = data;
                        break;
                default:
                        printk("%s Invalid HWID\n", __func__);
                        break;
                }
        }
}

static ssize_t show_lcm(struct device *dev, struct device_attribute *attr, char *buf)
{
        if (NULL != hwinfo_data.lcm) {
                return sprintf(buf, "lcd name :%s\n", hwinfo_data.lcm);
        } else {
           return sprintf(buf, "lcd name :Not Found\n");
        }
}

static ssize_t show_ctp(struct device *dev, struct device_attribute *attr, char *buf)
{
        if ((NULL != hwinfo_data.ctp_driver) || (NULL != hwinfo_data.ctp_module) || (NULL != hwinfo_data.ctp_fw_version)) {
        return sprintf(buf, "ctp name :%s FW_VER_%s\n", hwinfo_data.ctp_driver, hwinfo_data.ctp_fw_version);
        } else {
                return sprintf(buf, "ctp name :Not Found\n");
        }
}

static ssize_t show_fw_info(struct device *dev, struct device_attribute *attr, char *buf)
{
        if (NULL != hwinfo_data.ctp_fw_info) {
                return sprintf(buf, "ctp fw :%s\n", hwinfo_data.ctp_fw_info);
        } else {
                return sprintf(buf, "ctp fw :Invalid\n");
        }
}

static ssize_t show_main_camera(struct device *dev, struct device_attribute *attr, char *buf)
{
        if (NULL != hw_info_main_otp.sensor_name)
                hwinfo_data.main_camera = hw_info_main_otp.sensor_name;

        if (NULL != hwinfo_data.main_camera) {
                return sprintf(buf , "main camera :%s\n", hwinfo_data.main_camera);
        } else {
                return sprintf(buf , "main camera :Not Found\n");
        }
}

static ssize_t show_sub_camera(struct device *dev, struct device_attribute *attr, char *buf)
{
        if (NULL != hw_info_sub_otp.sensor_name)
                hwinfo_data.sub_camera = hw_info_sub_otp.sensor_name;

        if (NULL != hwinfo_data.sub_camera) {
                return sprintf(buf , "sub camera :%s\n", hwinfo_data.sub_camera);
        } else {
                return sprintf(buf , "sub camera :Not Found\n");
        }
}

static ssize_t show_aux_camera(struct device *dev, struct device_attribute *attr, char *buf)
{
        if (NULL != hw_info_main2_otp.sensor_name)
                hwinfo_data.aux_camera = hw_info_main2_otp.sensor_name;

        if (NULL != hwinfo_data.aux_camera) {
                return sprintf(buf , "auxiliary camera :%s\n", hwinfo_data.aux_camera);
        } else {
                return sprintf(buf , "auxiliary camera :Not Found\n");
        }
}

static ssize_t show_main_otp(struct device *dev, struct device_attribute *attr, char *buf)
{
        if (hw_info_main_otp.otp_valid) {
                return sprintf(buf, "main otp :Vendor 0x%0x, ModuleCode 0x%x, ModuleVer 0x%x, SW_ver 0x%x, Date %d-%d-%d, VcmVendor 0x%0x, VcmModule 0x%x \n",
                                                        hw_info_main_otp.vendor_id, hw_info_main_otp.module_code, hw_info_main_otp.module_ver, hw_info_main_otp.sw_ver, hw_info_main_otp.year,
                                                        hw_info_main_otp.month,        hw_info_main_otp.day, hw_info_main_otp.vcm_vendorid, hw_info_main_otp.vcm_moduleid);
        } else {
                return sprintf(buf, "main otp :No Valid OTP\n");
        }
}

static ssize_t show_sub_otp(struct device *dev, struct device_attribute *attr, char *buf)
{
        if (hw_info_sub_otp.otp_valid) {
                return sprintf(buf, "sub otp :Vendor 0x%0x, ModuleCode 0x%x, ModuleVer 0x%x, SW_ver 0x%x, Date %d-%d-%d, VcmVendor 0x%0x, VcmModule 0x%0x \n",
                                                        hw_info_sub_otp.vendor_id, hw_info_sub_otp.module_code, hw_info_sub_otp.module_ver, hw_info_sub_otp.sw_ver, hw_info_sub_otp.year,
                                                        hw_info_sub_otp.month, hw_info_sub_otp.day, hw_info_sub_otp.vcm_vendorid, hw_info_sub_otp.vcm_moduleid);
        } else {
                return sprintf(buf, "sub otp :No Valid OTP\n");
        }
}

static ssize_t show_main2_otp(struct device *dev, struct device_attribute *attr, char *buf)
{
        if (hw_info_main2_otp.otp_valid) {
                return sprintf(buf, "main2 otp :Vendor 0x%0x, ModuleCode 0x%x, ModuleVer 0x%x, SW_ver 0x%x, Date %d-%d-%d, VcmVendor 0x%0x, VcmModule 0x%x \n",
                                                        hw_info_main_otp.vendor_id, hw_info_main_otp.module_code, hw_info_main_otp.module_ver, hw_info_main_otp.sw_ver, hw_info_main_otp.year,
                                                        hw_info_main_otp.month,        hw_info_main_otp.day, hw_info_main_otp.vcm_vendorid, hw_info_main_otp.vcm_moduleid);
        } else {
                return sprintf(buf, "main2 otp :No Valid OTP\n");
        }
}

static ssize_t show_flash(struct device *dev, struct device_attribute *attr, char *buf)
{
        unsigned int i = 0;
        unsigned int raw_cid[4] = {0};
        char emmc_cid[32] = {0};
        static unsigned int flash_found_id = 0;

        if (flash_found_id) {
                return sprintf(buf, "flash name :%s\n", emmc_table[flash_found_id - 1].name);
        } else {
                if (NULL == hwinfo_data.flash) {
                        return sprintf(buf, "flash name :Can Not Detect EMMC\n");
                }

                memcpy(raw_cid, hwinfo_data.flash, sizeof(raw_cid));
                sprintf(emmc_cid, "%08x%08x%08x", raw_cid[0], raw_cid[1], raw_cid[2]);

                for (i = 0; i < ARRAY_SIZE(emmc_table); i++) {
                        if (memcmp(emmc_cid, emmc_table[i].cid, EMMC_CMP_SIZE) == 0) {
                                flash_found_id = i + 1;
                                return sprintf(buf, "flash name :%s\n", emmc_table[i].name);
                        }
                }
        }

        return sprintf(buf, "flash name :Not Found\n");
}

static ssize_t show_wifi(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "wifi name :%s\n", HARDWARE_INFO_WCN);
}

static ssize_t show_bt(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "bt name :%s\n", HARDWARE_INFO_WCN);
}

static ssize_t show_gps(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "gps name :%s\n", HARDWARE_INFO_WCN);
}

static ssize_t show_fm(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "fm name :%s\n", HARDWARE_INFO_WCN);
}

static ssize_t show_alsps(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_CUSTOM_KERNEL_ALSPS
#if defined(CONFIG_NANOHUB) && defined(CONFIG_CUSTOM_KERNEL_SENSORHUB)
        struct sensorInfo_t devinfo;
        int err = 0;
        err = sensor_set_cmd_to_hub(ID_PROXIMITY,
                CUST_ACTION_GET_SENSOR_INFO, &devinfo);

        if( err == 0)
                return sprintf(buf, "alsps name :%s\n",devinfo.name);
        else
                return sprintf(buf, "alsps name :Not Found\n");
#else
        if (NULL != hwinfo_data.alsps) {
                return sprintf(buf, "alsps name :%s\n", hwinfo_data.alsps);
        } else {
                return sprintf(buf, "alsps name :Not Found\n");
        }
#endif
#else
        return sprintf(buf, "alsps name :Not Support ALSPS\n");
#endif
}

static ssize_t show_gsensor(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_CUSTOM_KERNEL_ACCELEROMETER
#if defined(CONFIG_NANOHUB) && defined(CONFIG_CUSTOM_KERNEL_SENSORHUB)
        struct sensorInfo_t devinfo;
        int err = 0;
        err = sensor_set_cmd_to_hub(ID_ACCELEROMETER,
                CUST_ACTION_GET_SENSOR_INFO, &devinfo);

        if( err == 0)
                return sprintf(buf, "gsensor name :%s\n",devinfo.name);
        else
                return sprintf(buf, "gsensor name :Not Found\n");
#else
        if (NULL != hwinfo_data.gsensor) {
                return sprintf(buf, "gsensor name :%s\n", hwinfo_data.gsensor);
        } else {
                return sprintf(buf, "gsensor name :Not Found\n");
        }
#endif
#else
        return sprintf(buf, "gsensor name :Not support GSensor\n");
#endif
}

static ssize_t show_msensor(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_CUSTOM_KERNEL_MAGNETOMETER
#if defined(CONFIG_NANOHUB) && defined(CONFIG_CUSTOM_KERNEL_SENSORHUB)
        struct sensorInfo_t devinfo;
        int err = 0;
        err = sensor_set_cmd_to_hub(ID_MAGNETIC,
                CUST_ACTION_GET_SENSOR_INFO, &devinfo);

        if( err == 0)
                return sprintf(buf, "msensor name :%s\n",devinfo.mag_dev_info.libname);
        else
                return sprintf(buf, "msensor name :Not Found\n");
#else
        if (NULL != hwinfo_data.msensor) {
                return sprintf(buf, "msensor name :%s\n", hwinfo_data.msensor);
        } else {
                return sprintf(buf, "msensor name :Not Found\n");
        }
#endif
#else
        return sprintf(buf, "msensor name :Not support MSensor\n");
#endif
}

static ssize_t show_gyro(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_CUSTOM_KERNEL_GYROSCOPE
        if (NULL != hwinfo_data.gyroscope) {
                return sprintf(buf, "gyro name :%s\n", hwinfo_data.gyroscope);
        } else {
                return sprintf(buf, "gyro name :Not Found\n");
        }
#else
        return sprintf(buf, "gyro name :Not support Gyro\n");
#endif
}

static ssize_t show_sar_sensor_1(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_SAR_SENSOR_SX9310_1
        if (NULL != hwinfo_data.sar_sensor_1) {
                return sprintf(buf, "sar_sensor_1 name :%s\n", hwinfo_data.sar_sensor_1);
        } else {
                return sprintf(buf, "sar_sensor_1 name :Not Found\n");
        }
#else
        return sprintf(buf, "sar_sensor_1 name :Not support sar_sensor_1\n");
#endif
}
static ssize_t show_sar_sensor_2(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_SAR_SENSOR_SX9310_2
        if (NULL != hwinfo_data.sar_sensor_2) {
                return sprintf(buf, "sar_sensor_2 name :%s\n", hwinfo_data.sar_sensor_2);
        } else {
                return sprintf(buf, "sar_sensor_2 name :Not Found\n");
        }
#else
        return sprintf(buf, "sar_sensor_2 name :Not support sar_sensor_2\n");
#endif
}
static ssize_t show_version(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_HQ_BOARDINFO_BY_HW
        int len = 0;
        int rf_gpios[3] = {-1,-1,-1};
        int rf_val = 0;
        int sim_gpio = -1;
        int sim_val = 1;
        struct device_node *node = NULL;
        unsigned int i, vol;
        int res, min_vol, max_vol;
        unsigned int data[4] = {0};
        unsigned int rawdata = 0;
        static unsigned int revision_found_id = 0;

        node = of_find_compatible_node(NULL, NULL, "mediatek,pcb-gpio");
        if (node) {
                for(i=0; i<ARRAY_SIZE(rf_gpios); i++){
                        rf_gpios[i] = of_get_named_gpio(node, "rf-gpios", i);
                        rf_val |= (__gpio_get_value(rf_gpios[i]) << i);
                        printk("pcb-gpios rf gpio:%d,val:%d,rf_val:%d\n",rf_gpios[i],__gpio_get_value(rf_gpios[i]),rf_val);
                }
                sim_gpio = of_get_named_gpio(node, "sim-gpio", 0);
                sim_val = __gpio_get_value(sim_gpio);
                printk("pcb-gpios sim gpio:%d,val:%d,sim:%d\n",sim_gpio,__gpio_get_value(sim_gpio),sim_val);
        }

        len += scnprintf(buf+len, PAGE_SIZE-len, "BoardInfo:");
        switch(rf_val)
        {
                case 1:
                        len += scnprintf(buf+len, PAGE_SIZE-len, "ZAL1680A_");
                        break;
                case 2:
                        len += scnprintf(buf+len, PAGE_SIZE-len, "ZAL1680B_");
                        break;
                case 4:
                        len += scnprintf(buf+len, PAGE_SIZE-len, "ZAL1680C_");
                        break;
                default:
                        len += scnprintf(buf+len, PAGE_SIZE-len, "UNKNOWN_");
                        break;
        }

        if (sim_val) {
                len += scnprintf(buf+len, PAGE_SIZE-len, "DUAL_");
        }
        else {
                len += scnprintf(buf+len, PAGE_SIZE-len, "SINGLE_");
        }

        if (revision_found_id) {
                len += scnprintf(buf+len, PAGE_SIZE-len, "%s\n", revision_table[revision_found_id - 1].revision);
                return len;
        } else {
                res = IMM_GetOneChannelValue(AUX_ADC_EXTERNAL_CHANNEL, data, &rawdata);
                if (res < 0) {
                        len += scnprintf(buf+len, PAGE_SIZE-len, "\nrevision name :get voltage of aux_adc fail\n");
                        return len;
                }
                vol = data[0] * 1000 + data[1] * 10;
                printk("pcb-gpios revision vol:%d\n", vol);
                for (i = 0; i < ARRAY_SIZE(revision_table); i++) {
                        max_vol = revision_table[i].adc_vol + AUX_ADC_VALUE_TOLERANCE;
                        min_vol = max_vol - AUX_ADC_VALUE_TOLERANCE * 2;
                        if (min_vol < 0)
                                min_vol = 0;
                        if ((vol >= min_vol) && (vol < max_vol)) {
                                revision_found_id = i + 1;
                                len += scnprintf(buf+len, PAGE_SIZE-len, "%s\n", revision_table[i].revision);
                                return len;
                        }
                }
        }

        len += scnprintf(buf+len, PAGE_SIZE-len, "\nrevision name :Not Found\n");
        return len;
#else
        return sprintf(buf, "version :%s\n", HARDWARE_INFO_VERSION);
#endif
}

static ssize_t show_bat_id(struct device *dev, struct device_attribute *attr, char *buf)
{
#if 1//def CONFIG_HQ_BATTERY_ID
        if (NULL != hwinfo_data.bat_id) {
                return sprintf(buf, "bat_id name :%s\n", hwinfo_data.bat_id);
        } else {
                return sprintf(buf, "bat_id name :Not found\n");
        }
#else
        return sprintf(buf, "bat_id name :Not support Bat_id\n");
#endif
}
static ssize_t show_nfc(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_NFC_CHIP_PN553
        if (NULL != hwinfo_data.nfc) {
                return sprintf(buf, "nfc name :%s\n", hwinfo_data.nfc);
        } else {
                return sprintf(buf, "nfc name :Not found\n");
        }
#else
        return sprintf(buf, "nfc name :Not found\n");
#endif
}
static ssize_t show_fingerprint(struct device *dev, struct device_attribute *attr, char *buf)
{
        if (NULL != hwinfo_data.fingerprint) {
                return sprintf(buf, "fingerprint:%s\n", hwinfo_data.fingerprint);
        } else {
                return sprintf(buf, "fingerprint:Not found\n");
        }
}

static DEVICE_ATTR(version, 0444, show_version, NULL);
static DEVICE_ATTR(lcm, 0444, show_lcm, NULL);
static DEVICE_ATTR(ctp, 0444, show_ctp, NULL);
static DEVICE_ATTR(ctp_fw, 0444, show_fw_info, NULL);
static DEVICE_ATTR(main_camera, 0444, show_main_camera, NULL);
static DEVICE_ATTR(sub_camera, 0444, show_sub_camera, NULL);
static DEVICE_ATTR(main2_camera, 0444, show_aux_camera, NULL);
static DEVICE_ATTR(flash, 0444, show_flash, NULL);
static DEVICE_ATTR(gsensor, 0444, show_gsensor, NULL);
static DEVICE_ATTR(msensor, 0444, show_msensor, NULL);
static DEVICE_ATTR(alsps, 0444, show_alsps, NULL);
static DEVICE_ATTR(gyro, 0444, show_gyro, NULL);
static DEVICE_ATTR(wifi, 0444, show_wifi, NULL);
static DEVICE_ATTR(bt, 0444, show_bt, NULL);
static DEVICE_ATTR(gps, 0444, show_gps, NULL);
static DEVICE_ATTR(fm, 0444, show_fm, NULL);
static DEVICE_ATTR(main_otp, 0444, show_main_otp, NULL);
static DEVICE_ATTR(sub_otp, 0444, show_sub_otp, NULL);
static DEVICE_ATTR(main2_otp, 0444, show_main2_otp, NULL);
static DEVICE_ATTR(sar_sensor_1, 0444, show_sar_sensor_1, NULL);
static DEVICE_ATTR(sar_sensor_2, 0444, show_sar_sensor_2, NULL);
static DEVICE_ATTR(bat_id, 0444, show_bat_id,NULL );
static DEVICE_ATTR(nfc, 0444, show_nfc,NULL );
static DEVICE_ATTR(fingerprint, 0444, show_fingerprint,NULL );


static struct attribute *hdinfo_attributes[] = {
        &dev_attr_version.attr,
        &dev_attr_lcm.attr,
        &dev_attr_ctp.attr,
        &dev_attr_ctp_fw.attr,
        &dev_attr_main_camera.attr,
        &dev_attr_sub_camera.attr,
        &dev_attr_main2_camera.attr,
        &dev_attr_flash.attr,
        &dev_attr_gsensor.attr,
        &dev_attr_msensor.attr,
        &dev_attr_alsps.attr,
        &dev_attr_gyro.attr,
        &dev_attr_wifi.attr,
        &dev_attr_bt.attr,
        &dev_attr_gps.attr,
        &dev_attr_fm.attr,
        &dev_attr_main_otp.attr,
        &dev_attr_sub_otp.attr,
        &dev_attr_main2_otp.attr,
        &dev_attr_sar_sensor_1.attr,
        &dev_attr_sar_sensor_2.attr,
        &dev_attr_bat_id.attr,
        &dev_attr_nfc.attr,
        &dev_attr_fingerprint.attr,
        NULL
};

static struct attribute_group hdinfo_attribute_group = {
        .attrs = hdinfo_attributes
};

static int HardwareInfo_driver_probe(struct platform_device *dev)
{
        int err = -1;

        memset(&hwinfo_data, 0, sizeof(hwinfo_data));
        memset(&hw_info_main_otp, 0, sizeof(hw_info_main_otp));
        memset(&hw_info_main_otp, 0, sizeof(hw_info_main_otp));

        err = sysfs_create_group(&(dev->dev.kobj), &hdinfo_attribute_group);
        if (err < 0) {
                printk("** sysfs_create_group failed!\n");
                return err;
        }

        return err;
}

static int HardwareInfo_driver_remove(struct platform_device *dev)
{
        sysfs_remove_group(&(dev->dev.kobj), &hdinfo_attribute_group);

        return 0;
}

static struct platform_driver HardwareInfo_driver = {
        .probe = HardwareInfo_driver_probe,
        .remove = HardwareInfo_driver_remove,
        .driver = {
                .name = "HardwareInfo",
        },
};

static struct platform_device HardwareInfo_device = {
        .name = "HardwareInfo",
        .id = -1,
};

static int __init HardwareInfo_mod_init(void)
{
        int ret = -1;

        ret = platform_device_register(&HardwareInfo_device);
        if (ret) {
                printk("** platform_device_register failed!(%d)\n", ret);
                goto  err;
        }

        ret = platform_driver_register(&HardwareInfo_driver);
        if (ret) {
                printk("** platform_driver_register failed!(%d)\n", ret);
                goto  err2;
        }

        return ret;

err2:
        platform_device_unregister(&HardwareInfo_device);
err:
        return ret;
}


static void __exit HardwareInfo_mod_exit(void)
{
        platform_driver_unregister(&HardwareInfo_driver);
        platform_device_unregister(&HardwareInfo_device);
}


fs_initcall(HardwareInfo_mod_init);
module_exit(HardwareInfo_mod_exit);

MODULE_AUTHOR("Kaka Ni <nigang@huaqin.com>");
MODULE_DESCRIPTION("Huaqin Hareware Info driver");
MODULE_LICENSE("GPL");
#endif /* ODM_HQ_EDIT */
