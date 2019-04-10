/*
 * @file   silead_fp_mtk.c
 * @brief  Contains silead_fp device implements for Mediatek platform.
 *
 *
 * Copyright 2016-2017 Slead Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * ------------------- Revision History ------------------------------
 * <author>    <date>   <version>     <desc>
 * Bill Yu    2018/5/2    0.1.0      Init version
 * Bill Yu    2018/5/20   0.1.1      Default wait 3ms after reset
 * Bill Yu    2018/6/5    0.1.2      Support chip enter power down
 *
 */

#ifdef BSP_SIL_PLAT_MTK

#ifndef __SILEAD_FP_MTK__
#define __SILEAD_FP_MTK__

#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <mt-plat/upmu_common.h>

#if !defined(CONFIG_MTK_CLKMGR)
#include <linux/clk.h>
#endif

#if (!defined(CONFIG_SILEAD_FP_PLATFORM))
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

struct mt_spi_t {
    struct platform_device *pdev;
    void __iomem *regs;
    int irq;
    int running;
    u32 pad_macro;
    struct wake_lock wk_lock;
    struct mt_chip_conf *config;
    struct spi_master *master;

    struct spi_transfer *cur_transfer;
    struct spi_transfer *next_transfer;

    spinlock_t lock;
    struct list_head queue;
#if !defined(CONFIG_MTK_CLKMGR)
    struct clk *clk_main;
#endif
};

extern void mt_spi_enable_master_clk(struct spi_device *spidev);
extern void mt_spi_disable_master_clk(struct spi_device *spidev);
#endif

#define FP_IRQ_OF  "mediatek,finger-fp"
#define FP_PINS_OF "mediatek,finger-fp"

const static uint8_t TANAME[] = { 0x51, 0x1E, 0xAD, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static irqreturn_t silfp_irq_handler(int irq, void *dev_id);
static void silfp_work_func(struct work_struct *work);
static int silfp_input_init(struct silfp_data *fp_dev);

/* -------------------------------------------------------------------- */
/*                            power supply                              */
/* -------------------------------------------------------------------- */
static void silfp_hw_poweron(struct silfp_data *fp_dev)
{
    int err = 0;
    LOG_MSG_DEBUG(INFO_LOG, "[%s] enter.\n", __func__);

#ifdef BSP_SIL_POWER_SUPPLY_REGULATOR
       /* NOTE: the LDO is 200MV Step */
    if ( fp_dev->avdd_ldo&& ( 0 == regulator_is_enabled(fp_dev->avdd_ldo))) {
        err = regulator_set_voltage(fp_dev->avdd_ldo, AVDD_MIN, AVDD_MAX);
        LOG_MSG_DEBUG(ERR_LOG, "%s: silead set 3.2V err = %d.\n", __func__, err);
        err = regulator_enable(fp_dev->avdd_ldo);
        LOG_MSG_DEBUG(INFO_LOG, "%s: regulator_enable = %d\n", __func__, err);
    }

#endif

#ifdef BSP_SIL_POWER_SUPPLY_PINCTRL

    if ( fp_dev->pin.pins_avdd_h ) {
        err = pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_avdd_h);
    }
    if ( fp_dev->pin.pins_vddio_h ) {
        err = pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_vddio_h);
    }
#endif

#ifdef BSP_SIL_POWER_SUPPLY_GPIO
    if ( fp_dev->avdd_port > 0 ) {
        err = gpio_direction_output(fp_dev->avdd_port, 1);
    }
    if ( fp_dev->vddio_port > 0 ) {
        err = gpio_direction_output(fp_dev->vddio_port, 1);
    }
#endif
    LOG_MSG_DEBUG(INFO_LOG, "%s: power supply ret:%d \n", __func__, err);
}


static void silfp_hw_poweroff(struct silfp_data *fp_dev)
{
    int err = 0;
    LOG_MSG_DEBUG(INFO_LOG, "[%s] enter.\n", __func__);
#ifdef BSP_SIL_POWER_SUPPLY_REGULATOR
    if ( fp_dev->avdd_ldo && (regulator_is_enabled(fp_dev->avdd_ldo) > 0)) {
        err = regulator_disable(fp_dev->avdd_ldo);
		LOG_MSG_DEBUG(INFO_LOG, "%s: regulator_disable = %d\n", __func__, err);
    }
#endif
}

static void silfp_power_deinit(struct silfp_data *fp_dev)
{
    LOG_MSG_DEBUG(INFO_LOG, "[%s] enter.\n", __func__);
#ifdef BSP_SIL_POWER_SUPPLY_REGULATOR
    if ( fp_dev->avdd_ldo ) {
        regulator_disable(fp_dev->avdd_ldo);
        regulator_put(fp_dev->avdd_ldo);
        fp_dev->avdd_ldo = NULL;
    }
#endif

#ifdef BSP_SIL_POWER_SUPPLY_PINCTRL
    fp_dev->pin.pins_avdd_h = NULL;
    fp_dev->pin.pins_vddio_h = NULL;
#endif

#ifdef BSP_SIL_POWER_SUPPLY_GPIO
    if ( fp_dev->avdd_port > 0 ) {
        gpio_direction_output(fp_dev->avdd_port, 0);
        gpio_free(fp_dev->avdd_port);
        fp_dev->avdd_port = 0;
    }
    if ( fp_dev->vddio_port > 0 ) {
        gpio_direction_output(fp_dev->vddio_port, 0);
        gpio_free(fp_dev->vddio_port);
        fp_dev->vddio_port = 0;
    }
#endif
}

/* -------------------------------------------------------------------- */
/*                            hardware reset                            */
/* -------------------------------------------------------------------- */
static void silfp_hw_reset(struct silfp_data *fp_dev, u8 delay)
{
    LOG_MSG_DEBUG(INFO_LOG, "[%s] enter, port=%d\n", __func__, fp_dev->rst_port);
    pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_rst_l);
    mdelay((delay?delay:5)*RESET_TIME_MULTIPLE);
    pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_rst_h);
    mdelay((delay?delay:3)*RESET_TIME_MULTIPLE);
}

/* -------------------------------------------------------------------- */
/*                            power  down                               */
/* -------------------------------------------------------------------- */
static void silfp_pwdn(struct silfp_data *fp_dev, u8 flag_avdd)
{
    LOG_MSG_DEBUG(INFO_LOG, "[%s] enter, port=%d flag_avdd=%d\n", __func__, fp_dev->rst_port,flag_avdd);

    pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_rst_l);

    if (SIFP_PWDN_FLASH == flag_avdd) {
        silfp_hw_poweroff(fp_dev);
        msleep(200*RESET_TIME_MULTIPLE);
        silfp_hw_poweron(fp_dev);
    }

    if (SIFP_PWDN_POWEROFF == flag_avdd) {
        silfp_hw_poweroff(fp_dev);
    }

}

/* -------------------------------------------------------------------- */
/*                         init/deinit functions                        */
/* -------------------------------------------------------------------- */
static int silfp_parse_dts(struct silfp_data* fp_dev)
{
#ifdef CONFIG_OF
    struct device_node *node = NULL;
    struct platform_device *pdev = NULL;
    int  ret;

    LOG_MSG_DEBUG(INFO_LOG, "%s, enter silfp_parse_dts\n", __func__);
    node = of_find_compatible_node(NULL, NULL, FP_IRQ_OF);
    if (node) {
        fp_dev->int_port = irq_of_parse_and_map(node, 0);
        LOG_MSG_DEBUG(INFO_LOG, "%s, irq = %d\n", __func__, fp_dev->int_port);
    } else {
        LOG_MSG_DEBUG(ERR_LOG, "%s %s compatible device node is null\n", __func__,FP_IRQ_OF);
    }

    node = of_find_compatible_node(NULL, NULL, FP_PINS_OF);
    if (node) {
        LOG_MSG_DEBUG(INFO_LOG, "%s, irq = %d\n", __func__, fp_dev->int_port);
        pdev = of_find_device_by_node(node);
        if (pdev) {
            fp_dev->pin.pinctrl = devm_pinctrl_get(&pdev->dev);
            if (IS_ERR(fp_dev->pin.pinctrl)) {
                ret = PTR_ERR(fp_dev->pin.pinctrl);
                LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp pinctrl\n", __func__);
                return ret;
            }
        } else {
            LOG_MSG_DEBUG(ERR_LOG, "%s platform device is null\n", __func__);
        }
    } else {
        LOG_MSG_DEBUG(ERR_LOG, "%s %s compatible device node is null\n", __func__,FP_PINS_OF);
    }

    fp_dev->pin.pins_irq = pinctrl_lookup_state(fp_dev->pin.pinctrl, "irq-init");
    if (IS_ERR(fp_dev->pin.pins_irq)) {
        ret = PTR_ERR(fp_dev->pin.pins_irq);
        LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp irq-init\n", __func__);
        return ret;
    }

    fp_dev->pin.pins_rst_h = pinctrl_lookup_state(fp_dev->pin.pinctrl, "rst-high");
    if (IS_ERR(fp_dev->pin.pins_rst_h)) {
        ret = PTR_ERR(fp_dev->pin.pins_rst_h);
        LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp rst-high\n", __func__);
        return ret;
    }
    fp_dev->pin.pins_rst_l = pinctrl_lookup_state(fp_dev->pin.pinctrl, "rst-low");
    if (IS_ERR(fp_dev->pin.pins_rst_l)) {
        ret = PTR_ERR(fp_dev->pin.pins_rst_l);
        LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp rst-high\n", __func__);
        return ret;
    }
    LOG_MSG_DEBUG(ERR_LOG, "%s, parse the rst pin successed\n", __func__);
#if 0
    fp_dev->pin.spi_default = pinctrl_lookup_state(fp_dev->pin.pinctrl, "spi-default");
    if (IS_ERR(fp_dev->pin.spi_default)) {
        ret = PTR_ERR(fp_dev->pin.spi_default);
        pr_info("%s can't find silfp spi-default\n", __func__);
        return ret;
    }
    pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.spi_default);
#endif

#ifdef BSP_SIL_POWER_SUPPLY_PINCTRL
    fp_dev->pin.pins_avdd_h = pinctrl_lookup_state(fp_dev->pin.pinctrl, "avdd-enable");
    if (IS_ERR_OR_NULL(fp_dev->pin.pins_avdd_h)) {
        fp_dev->pin.pins_avdd_h = NULL;
        LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp avdd-enable\n", __func__);
    }

    fp_dev->pin.pins_vddio_h = pinctrl_lookup_state(fp_dev->pin.pinctrl, "vddio-enable");
    if (IS_ERR_OR_NULL(fp_dev->pin.pins_vddio_h)) {
        fp_dev->pin.pins_vddio_h = NULL;
        LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp vddio-enable\n", __func__);
    }
#endif

#ifdef BSP_SIL_POWER_SUPPLY_REGULATOR
    fp_dev->avdd_ldo = regulator_get(&fp_dev->spi->dev, "irtx_ldo");
    LOG_MSG_DEBUG(ERR_LOG, "%s regulator_get successed\n", __func__);
#endif

#if (!defined(CONFIG_SILEAD_FP_PLATFORM))
    if ( fp_dev->spi->dev.of_node ) {
        ret = of_property_read_u32(fp_dev->spi->dev.of_node,"spi-id", &fp_dev->pin.spi_id);
        LOG_MSG_DEBUG(ERR_LOG, "%s spi-id = %d\n", __func__, fp_dev->pin.spi_id);
        if (ret) {
            fp_dev->pin.spi_id = 0;
            pr_info("Error getting spi_id\n");
        }
        ret = of_property_read_u32(fp_dev->spi->dev.of_node,"spi-irq", &fp_dev->pin.spi_irq);
        if (ret) {
            fp_dev->pin.spi_irq = 0;
            pr_info("Error getting spi_irq\n");
        }
        ret = of_property_read_u32(fp_dev->spi->dev.of_node,"spi-reg", &fp_dev->pin.spi_reg);
        if (ret) {
            fp_dev->pin.spi_reg = 0;
            pr_info("Error getting spi_reg\n");
        }
    }
#endif
#endif
    return 0;
}

static int silfp_set_spi(struct silfp_data *fp_dev, bool enable)
{

#if (!defined(CONFIG_SILEAD_FP_PLATFORM))
    int ret = -ENOENT;
    struct mt_spi_t *ms = NULL;
    ms = spi_master_get_devdata(fp_dev->spi->master);


    if ( !ms ) {
        LOG_MSG_DEBUG(ERR_LOG, "%s: not support\n", __func__);
        return ret;
    }

    if ( enable && !atomic_read(&fp_dev->spionoff_count) ) {
        atomic_inc(&fp_dev->spionoff_count);
        mt_spi_enable_master_clk(fp_dev->spi);
       //  clk_prepare_enable(ms->clk_main); 
       //   ret = clk_enable(ms->clk_main);
    } else if (atomic_read(&fp_dev->spionoff_count)) {
        atomic_dec(&fp_dev->spionoff_count);
        mt_spi_disable_master_clk(fp_dev->spi);
       // clk_disable_unprepare(ms->clk_main); 
       //  clk_disable(ms->clk_main);
        ret = 0;
    }
    LOG_MSG_DEBUG(DBG_LOG, "[%s] done (%d).\n",__func__,ret);
#endif
    return 0;
}

static int silfp_resource_init(struct silfp_data *fp_dev, struct fp_dev_init_t *dev_info)
{
    int status = 0;
    int ret;

    LOG_MSG_DEBUG(ERR_LOG, "[%s] enter silfp_resource_init",__func__);
    if (atomic_read(&fp_dev->init)) {
        atomic_inc(&fp_dev->init);
        LOG_MSG_DEBUG(DBG_LOG, "[%s] dev already init(%d).\n",__func__,atomic_read(&fp_dev->init));
        return status;
    }

    silfp_parse_dts(fp_dev);
    silfp_hw_poweron(fp_dev);

    LOG_MSG_DEBUG(INFO_LOG, "[%s] int_port %d, rst_port %d.\n",__func__,fp_dev->int_port,fp_dev->rst_port);

    pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_irq);
    fp_dev->irq = fp_dev->int_port;
    fp_dev->irq_is_disable = 0;

    ret  = request_irq(fp_dev->irq,
                       silfp_irq_handler,
                       IRQ_TYPE_EDGE_RISING,
                       "silfp",
                       fp_dev);
    if ( ret < 0 ) {
        LOG_MSG_DEBUG(ERR_LOG, "[%s] Filed to request_irq (%d), ert=%d",__func__,fp_dev->irq, ret);
        status = -ENODEV;
        goto err_irq;
    } else {
        LOG_MSG_DEBUG(INFO_LOG,"[%s] Enable_irq_wake.\n",__func__);
        enable_irq_wake(fp_dev->irq);
        silfp_irq_disable(fp_dev);
    }

    if (fp_dev->rst_port > 0 ) {
        ret = gpio_request(fp_dev->rst_port, "SILFP_RST_PIN");
        if (ret < 0) {
            LOG_MSG_DEBUG(ERR_LOG, "[%s] Failed to request GPIO=%d, ret=%d",__func__,(s32)fp_dev->rst_port, ret);
            status = -ENODEV;
            goto err_rst;
        } else {
            gpio_direction_output(fp_dev->rst_port, 1);
        }
    }

    if (!ret) {
        if (silfp_input_init(fp_dev)) {
            goto err_input;
        }
        atomic_set(&fp_dev->init,1);
    }

    if (dev_info) {
    dev_info->reserve = PKG_SIZE;
    dev_info->reserve <<= 12;

        if(fp_dev->pin.spi_id){
        dev_info->dev_id = (uint8_t)fp_dev->pin.spi_id;
        dev_info->reserve |= fp_dev->pin.spi_irq & 0x0FFF;
        dev_info->reg = fp_dev->pin.spi_reg;
        memcpy(dev_info->ta,TANAME,sizeof(dev_info->ta));
        }
    }
    LOG_MSG_DEBUG(ERR_LOG, "[%s] end silfp_resource_init",__func__);

    return status;

err_input:
    if (fp_dev->rst_port > 0 ) {
    }

err_rst:
    free_irq(fp_dev->irq, fp_dev);
    gpio_direction_input(fp_dev->int_port);

err_irq:
    fp_dev->int_port = 0;
    fp_dev->rst_port = 0;

    return status;
}

#endif /* __SILEAD_FP_MTK__ */

#endif /* BSP_SIL_PLAT_MTK */

/* End of file spilead_fp_mtk.c */
