/************************************************************************************
** File: - vendor\oppo_app\securebsp\Fingerprint\drivers\et512\et512.c
** VENDOR_EDIT
** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd
**
** Description:
**      egistec fingerprint kernel device driver
**
** Version: 1.0
** Date created: 15:03:11,18/10/2018
** Author: liuqingwen@RM.BSP.Fingerprint.Basic
** TAG: RM.BSP.Fingerprint.Basic
**
** --------------------------- Revision History: --------------------------------
**    <author>     <data>        <desc>
**    liuqingwen   2018/10/18    create the file
**    guomingzhi   2018/11/02    modified to get screen state and power-sequence controled by hal
************************************************************************************/


#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif

#include <linux/notifier.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/list.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>  
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/fb.h>
#include <linux/wakelock.h>
#include "../include/oppo_fp_common.h"
#include "et512.h"

#define GPIO_DTS 0
#define DUAL_FP 0
#define EGIS_NAVI_INPUT 1 // 1:open ; 0:close
#if EGIS_NAVI_INPUT
#include "ets_navi_input.h"
#endif

#define GPIO_PIN_IRQ  126
#define GPIO_PIN_RESET 93
#define GPIO_PIN_33V 94
struct wake_lock et512_wake_lock;
extern void mt_spi_enable_master_clk(struct spi_device *spidev);
extern void mt_spi_disable_master_clk(struct spi_device *spidev);
 
struct interrupt_desc fps_ints = {0 , 0, "BUT0" , 0};
unsigned int bufsiz = 4096;
int gpio_irq;
int request_irq_done = 0;
int egistec_platformInit_done = 0;
static struct regulator *buck;

#define EDGE_TRIGGER_FALLING    0x0
#define EDGE_TRIGGER_RISING    0x1
#define LEVEL_TRIGGER_LOW       0x2
#define LEVEL_TRIGGER_HIGH      0x3

int egistec_platformInit(struct egistec_data *egistec);
int egistec_platformFree(struct egistec_data *egistec);

static int egistec_regulator(void);


struct ioctl_cmd {
int int_mode;
int detect_period;
int detect_threshold;
};

int g_screen_onoff = 1;

static void delete_device_node(void);
static struct egistec_data *g_data;

DECLARE_BITMAP(minors, N_SPI_MINORS);
LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static struct of_device_id egistec_match_table[] = {
	{ .compatible = "mediatek,finger-fp",},
	{},
};

static struct of_device_id et512_spi_of_match[] = {
	{ .compatible = "mediatek,fingerspi-fp", },
	{}
};

MODULE_DEVICE_TABLE(of, et512_spi_of_match);

#if DUAL_FP
static struct of_device_id fpswitch_match_table[] = {
	{ .compatible = "fp_id,fp_id",},
	{},
};
#endif

MODULE_DEVICE_TABLE(of, egistec_match_table);

static void spi_clk_enable(u8 bonoff)
{
	if (bonoff) {
		pr_err("EGISTEC %s line:%d enable spi clk\n", __func__,__LINE__);
		mt_spi_enable_master_clk(g_data->spi);
	} else 	{
		pr_err("EGISTEC %s line:%d disable spi clk\n", __func__,__LINE__);
		mt_spi_disable_master_clk(g_data->spi);
	}
}

#define FP_INT_DETECTION_PERIOD  10
#define FP_DETECTION_THRESHOLD	10

static DECLARE_WAIT_QUEUE_HEAD(interrupt_waitq);

void interrupt_timer_routine(unsigned long _data)
{
	struct interrupt_desc *bdata = (struct interrupt_desc *)_data;
	DEBUG_PRINT("EGISTEC FPS interrupt count = %d \n", bdata->int_count);

	if (bdata->int_count >= bdata->detect_threshold) {
		bdata->finger_on = 1;
		DEBUG_PRINT("EGISTEC FPS triggered !!!!!!!\n");
	} else {
		DEBUG_PRINT("EGISTEC FPS not triggered !!!!!!!\n");
	}
	bdata->int_count = 0;
	wake_up_interruptible(&interrupt_waitq);
}

static irqreturn_t fp_eint_func(int irq, void *dev_id)
{
	if (!fps_ints.int_count) {
		mod_timer(&fps_ints.timer,jiffies + msecs_to_jiffies(fps_ints.detect_period));
	}
	fps_ints.int_count++;
	return IRQ_HANDLED;
}

static irqreturn_t fp_eint_func_ll(int irq , void *dev_id)
{
	DEBUG_PRINT("[EGISTEC]fp_eint_func_ll\n");
	fps_ints.finger_on = 1;
	disable_irq_nosync(gpio_irq);
	fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;
	wake_up_interruptible(&interrupt_waitq);
	return IRQ_RETVAL(IRQ_HANDLED);
}

int Interrupt_Init(struct egistec_data *egistec,int int_mode,int detect_period,int detect_threshold)
{

	int err = 0;
	int status = 0;
    struct device_node *node = NULL;
	DEBUG_PRINT("EGISTEC   %s mode = %d period = %d threshold = %d\n",__func__,int_mode,detect_period,detect_threshold);
	DEBUG_PRINT("EGISTEC   %s request_irq_done = %d gpio_irq = %d  pin = %d  \n",__func__,request_irq_done,gpio_irq, egistec->irqPin);

	fps_ints.detect_period = detect_period;
	fps_ints.detect_threshold = detect_threshold;
	fps_ints.int_count = 0;
	fps_ints.finger_on = 0;

	if (request_irq_done == 0) {
		node = of_find_matching_node(node, egistec_match_table);
		if (node){
			gpio_irq = irq_of_parse_and_map(node, 0);
			printk("EGISTEC fp_irq number %d\n", gpio_irq);
		}else{
			printk("node = of_find_matching_node fail error  \n");
		}
		if (gpio_irq < 0) {
			DEBUG_PRINT("EGISTEC %s gpio_to_irq failed\n", __func__);
			status = gpio_irq;
			goto done;
		}

		DEBUG_PRINT("[EGISTEC Interrupt_Init] flag current: %d disable: %d enable: %d\n",
		fps_ints.drdy_irq_flag, DRDY_IRQ_DISABLE, DRDY_IRQ_ENABLE);

		if (int_mode == EDGE_TRIGGER_RISING) {
			DEBUG_PRINT("EGISTEC %s EDGE_TRIGGER_RISING\n", __func__);
			err = request_irq(gpio_irq, fp_eint_func,IRQ_TYPE_EDGE_RISING,"fp_detect-eint", egistec);
			if (err) {
				pr_err("request_irq failed==========%s,%d\n", __func__,__LINE__);
			}
		}
		else if (int_mode == EDGE_TRIGGER_FALLING) {
			DEBUG_PRINT("EGISTEC %s EDGE_TRIGGER_FALLING\n", __func__);
			err = request_irq(gpio_irq, fp_eint_func,IRQ_TYPE_EDGE_FALLING,"fp_detect-eint", egistec);
			if (err) {
				pr_err("request_irq failed==========%s,%d\n", __func__,__LINE__);
			}
		}
		else if (int_mode == LEVEL_TRIGGER_LOW) {
			DEBUG_PRINT("EGISTEC %s LEVEL_TRIGGER_LOW\n", __func__);
			err = request_irq(gpio_irq, fp_eint_func_ll,IRQ_TYPE_LEVEL_LOW,"fp_detect-eint", egistec);
			if (err){
				pr_err("request_irq failed==========%s,%d\n", __func__,__LINE__);
				}
		}
		else if (int_mode == LEVEL_TRIGGER_HIGH) {
			DEBUG_PRINT("EGISTEC %s LEVEL_TRIGGER_HIGH\n", __func__);
			err = request_irq(gpio_irq, fp_eint_func_ll,IRQ_TYPE_LEVEL_HIGH,"fp_detect-eint", egistec);
			if (err) {
				pr_err("request_irq failed==========%s,%d\n", __func__,__LINE__);
			}
		}
		DEBUG_PRINT("[EGISTEC Interrupt_Init]:gpio_to_irq return: %d\n", gpio_irq);
		DEBUG_PRINT("[EGISTEC Interrupt_Init]:request_irq return: %d\n", err);

		fps_ints.drdy_irq_flag = DRDY_IRQ_ENABLE;
		enable_irq_wake(gpio_irq);
		request_irq_done = 1;
	}
	if (fps_ints.drdy_irq_flag == DRDY_IRQ_DISABLE) {
		fps_ints.drdy_irq_flag = DRDY_IRQ_ENABLE;
		enable_irq_wake(gpio_irq);
		enable_irq(gpio_irq);
	}

done:
	return 0;
}

int Interrupt_Free(struct egistec_data *egistec)
{
	DEBUG_PRINT("EGISTEC %s\n", __func__);
	fps_ints.finger_on = 0;

	if (fps_ints.drdy_irq_flag == DRDY_IRQ_ENABLE) {
		DEBUG_PRINT("EGISTEC %s (DISABLE IRQ)\n", __func__);
		disable_irq_nosync(gpio_irq);
		del_timer_sync(&fps_ints.timer);
		fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;
	}

	return 0;
}

unsigned int fps_interrupt_poll(struct file *file, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	poll_wait(file, &interrupt_waitq, wait);
	if (fps_ints.finger_on) {
		mask |= POLLIN | POLLRDNORM;
	}
	return mask;
}

void fps_interrupt_abort(void)
{
	DEBUG_PRINT("EGISTEC %s\n", __func__);
	fps_ints.finger_on = 0;
	wake_up_interruptible(&interrupt_waitq);
}

/*---------Power Control-----------------------------------------------------*/
static int egistec_powersetup(struct egistec_data *egistec)
{
	int ret = 0;
	DEBUG_PRINT("EGISTEC %s\n", __func__);
	ret = egistec_regulator();
	if (ret < 0){
		printk("egitec regulator_setup fail \n");
	}
	mdelay(3);
	pinctrl_select_state(egistec->pinctrl_gpios, egistec->pins_reset_high);
	mdelay(5);
	return ret;
}

static int egistec_poweronoff(struct egistec_data *egistec)
{
	int ret = 0;
	DEBUG_PRINT(" %s\n", __func__);
	ret = regulator_set_voltage(buck, 0, 0);
	ret = regulator_disable(buck);
	if (ret < 0) {
		printk("egitec enter cdfinger probe regulator_set_voltage=0 fail \n");
    }
	msleep(1);
	ret = regulator_set_voltage(buck, 3400000, 3400000);
	ret = regulator_enable(buck);
	if (ret < 0) {
		printk("egitec enter cdfinger probe regulator_set_voltage=3.3V fail \n");
    }
	msleep(1);
	return ret;
}

/*-------------------------------------------------------------------------*/
static void egistec_reset(struct egistec_data *egistec)
{
	DEBUG_PRINT(" %s\n", __func__);
	pinctrl_select_state(egistec->pinctrl_gpios, egistec->pins_reset_low);
	mdelay(10);
	pinctrl_select_state(egistec->pinctrl_gpios, egistec->pins_reset_high);
	mdelay(10);
}

static ssize_t egistec_read(struct file *filp,
	char __user *buf, size_t count, loff_t *f_pos)
{
	/*Implement by vendor if needed*/
	return 0;
}

static ssize_t egistec_write(struct file *filp,
	const char __user *buf, size_t count, loff_t *f_pos)
{
	/*Implement by vendor if needed*/
	return 0;
}

static int egistec_fb_notifier_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	unsigned int blank;
	int retval = 0;

	/* If we aren't interested in this event, skip it immediately ... */
	if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */)
		return 0;

	blank = *(int *)evdata->data;

	DEBUG_PRINT("EGISTEC [%s] : enter, blank=0x%x\n", __func__, blank);

	switch (blank) {
	case FB_BLANK_UNBLANK:
		g_screen_onoff = 1;
		DEBUG_PRINT("EGISTEC %s lcd on notify  ------\n", __func__);
		break;

	case FB_BLANK_POWERDOWN:
		g_screen_onoff = 0;
		DEBUG_PRINT("EGISTEC %s lcd off notify  ------\n", __func__);
		break;

	default:
		DEBUG_PRINT("EGISTEC %s other notifier, ignore\n", __func__);
		break;
	}
	return retval;
}


static long egistec_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	struct egistec_data *egistec;
	struct ioctl_cmd data;
	int status = 0;

	memset(&data, 0, sizeof(data));
	printk(" %s  cmd = 0x%X \n", __func__, cmd);
	egistec = filp->private_data;

	if(!egistec_platformInit_done)
	/* platform init */
	status = egistec_platformInit(egistec);
	if (status != 0) {
		pr_err(" %s platforminit failed\n", __func__);
	}

	switch (cmd) {
		case INT_TRIGGER_INIT:
			if (copy_from_user(&data, (int __user *)arg, sizeof(data))) {
				retval = -EFAULT;
				goto done;
			}
			DEBUG_PRINT("EGISTEC fp_ioctl >>> fp Trigger function init\n");
			retval = Interrupt_Init(egistec, data.int_mode,data.detect_period,data.detect_threshold);
			DEBUG_PRINT("EGISTEC fp_ioctl trigger init = %x\n", retval);
			goto done;

		case FP_SENSOR_RESET:
				DEBUG_PRINT("EGISTEC fp_ioctl ioc->opcode == FP_SENSOR_RESET \n");
				egistec_reset(egistec);
			goto done;
		case FP_POWERSETUP:
				DEBUG_PRINT("EGISTEC fp_ioctl ioc->opcode == FP_POWER_SETUP \n");
				egistec_powersetup(egistec);
			goto done;
		case FP_POWER_ONOFF:
				DEBUG_PRINT("EGISTEC fp_ioctl ioc->opcode == FP_POWER_ONOFF \n");
				egistec_poweronoff(egistec);
			goto done;
		case INT_TRIGGER_CLOSE:
				DEBUG_PRINT("EGISTEC fp_ioctl <<< fp Trigger function close\n");
				retval = Interrupt_Free(egistec);
				DEBUG_PRINT("EGISTEC fp_ioctl trigger close = %x\n", retval);
			goto done;
		case INT_TRIGGER_ABORT:
				DEBUG_PRINT("EGISTEC fp_ioctl <<< fp Trigger function close\n");
				fps_interrupt_abort();
			goto done;
		case FP_FREE_GPIO:
				DEBUG_PRINT("EGISTEC fp_ioctl <<< FP_FREE_GPIO  \n");
				egistec_platformFree(egistec);
			goto done;
		case FP_WAKELOCK_TIMEOUT_ENABLE: //0Xb1
				DEBUG_PRINT("EGISTEC fp_ioctl <<< FP_WAKELOCK_TIMEOUT_ENABLE  \n");
				wake_lock_timeout(&et512_wake_lock, msecs_to_jiffies(1000));
			goto done;
		case FP_WAKELOCK_TIMEOUT_DISABLE: //0Xb2
				DEBUG_PRINT("EGISTEC fp_ioctl <<< FP_WAKELOCK_TIMEOUT_DISABLE  \n");
				wake_unlock(&et512_wake_lock);
			goto done;
		case FP_SPICLK_ENABLE:
				DEBUG_PRINT("EGISTEC fp_ioctl <<< FP_SPICLK_ENABLE  \n");
				spi_clk_enable(1);
			goto done;
		case FP_SPICLK_DISABLE:
				DEBUG_PRINT("EGISTEC fp_ioctl <<< FP_SPICLK_DISABLE  \n");
				spi_clk_enable(0);
			goto done;
		case DELETE_DEVICE_NODE:
				DEBUG_PRINT("EGISTEC fp_ioctl <<< DELETE_DEVICE_NODE  \n");
				delete_device_node();
			goto done;
		case GET_SCREEN_ONOFF:
				DEBUG_PRINT("EGISTEC fp_ioctl <<< GET_SCREEN_ONOFF  \n");
				data.int_mode = g_screen_onoff;
			if (copy_to_user((int __user *)arg, &data, sizeof(data))) {
				retval = -EFAULT;
				goto done;
			}

		goto done;
		default:
		retval = -ENOTTY;
		break;
	}

done:
	DEBUG_PRINT(" %s done  \n", __func__);
	return (retval);
}

#ifdef CONFIG_COMPAT
static long egistec_compat_ioctl(struct file *filp,
	unsigned int cmd,unsigned long arg)
{
	return egistec_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define egistec_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int egistec_open(struct inode *inode, struct file *filp)
{
	struct egistec_data *egistec;
	int	status = -ENXIO;

	DEBUG_PRINT("%s\n", __func__);
	mutex_lock(&device_list_lock);
	list_for_each_entry(egistec, &device_list, device_entry)
	{
		if (egistec->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (egistec->buffer == NULL) {
			egistec->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (egistec->buffer == NULL) {
				pr_debug("open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			egistec->users++;
			filp->private_data = egistec;
			nonseekable_open(inode, filp);
		}
	} else {
		pr_debug("%s nothing for minor %d\n", __func__, iminor(inode));
	}
	mutex_unlock(&device_list_lock);
	return status;
}

static int egistec_release(struct inode *inode, struct file *filp)
{
	struct egistec_data *egistec;

	DEBUG_PRINT("%s\n", __func__);
	mutex_lock(&device_list_lock);
	egistec = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	egistec->users--;
	if (egistec->users == 0) {
		int	dofree;

		kfree(egistec->buffer);
		egistec->buffer = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&egistec->spi_lock);
		dofree = (egistec->pd == NULL);
		spin_unlock_irq(&egistec->spi_lock);

		if (dofree)
			kfree(egistec);
	}
	mutex_unlock(&device_list_lock);
	return 0;
}

int egistec_platformFree(struct egistec_data *egistec)
{
	int status = 0;
	DEBUG_PRINT("%s\n", __func__);
	if (egistec_platformInit_done != 1)
	return status;

	if (egistec != NULL) {
		if (request_irq_done==1)
		{
			free_irq(gpio_irq, NULL);
			request_irq_done = 0;
		}
		gpio_free(egistec->irqPin);
		gpio_free(GPIO_PIN_RESET);
	}

	egistec_platformInit_done = 0;
	DEBUG_PRINT("%s successful status=%d\n", __func__, status);
	return status;
}

int egistec_platformInit(struct egistec_data *egistec)
{
	int status = 0;

	DEBUG_PRINT("%s\n", __func__);

	if (egistec != NULL) {
#if GPIO_DTS
		/* Initial Reset Pin */
		status = gpio_request(egistec->rstPin, "reset-gpio");
		if (status < 0) {
			pr_err("%s gpio_requset egistec_Reset failed\n",__func__);
			goto egistec_platformInit_rst_failed;
		}

		gpio_direction_output(egistec->rstPin, 1);
		if (status < 0) {
			pr_err("%s gpio_direction_output Reset failed\n",__func__);
			status = -EBUSY;
			goto egistec_platformInit_rst_failed;
		}

		//added to initialize it as high
		gpio_set_value(GPIO_PIN_RESET, 1);
		msleep(30);

		/* initial 33V power pin */
		gpio_direction_output(egistec->vcc_33v_Pin, 1);
		gpio_set_value(egistec->vcc_33v_Pin, 1);

		status = gpio_request(egistec->vcc_33v_Pin, "33v-gpio");
		if (status < 0) {
			pr_err("%s gpio_requset egistec_Reset failed\n",__func__);
			goto egistec_platformInit_rst_failed;
		}
		gpio_direction_output(egistec->vcc_33v_Pin, 1);
		if (status < 0) {
			pr_err("%s gpio_direction_output Reset failed\n",__func__);
			status = -EBUSY;
			goto egistec_platformInit_rst_failed;
		}
		gpio_set_value(egistec->vcc_33v_Pin, 1);

		/* Initial IRQ Pin */
		status = gpio_request(egistec->irqPin, "irq-gpio");
		if (status < 0) {
			pr_err("%s gpio_request egistec_irq failed\n",__func__);
			goto egistec_platformInit_irq_failed;
		}

		status = gpio_direction_input(egistec->irqPin);
		if (status < 0) {
			pr_err("%s gpio_direction_input IRQ failed\n",__func__);
			goto egistec_platformInit_gpio_init_failed;
		}
#endif

	}

	egistec_platformInit_done = 1;
	DEBUG_PRINT("%s successful status=%d\n", __func__, status);
	return status;

#if GPIO_DTS
egistec_platformInit_gpio_init_failed:
	gpio_free(egistec->irqPin);
	gpio_free(egistec->vcc_33v_Pin);

egistec_platformInit_irq_failed:
	gpio_free(egistec->rstPin);

egistec_platformInit_rst_failed:

	pr_err("%s is failed\n", __func__);
	return status;
#endif
}

static int egistec_parse_dt(struct device *dev,
	struct egistec_data *data)
{

#ifdef CONFIG_OF
	int ret;
	struct device_node *node = NULL;
	struct platform_device *pdev = NULL;

	printk(KERN_ERR "%s, from dts pinctrl\n", __func__);

	node = of_find_compatible_node(NULL, NULL, "mediatek,finger-fp");

	if (node) {
		pdev = of_find_device_by_node(node);
		printk(KERN_ERR "%s egistec find node enter \n", __func__);
		if (pdev) {
			data->pinctrl_gpios = devm_pinctrl_get(&pdev->dev);
			if (IS_ERR(data->pinctrl_gpios)) {
				ret = PTR_ERR(data->pinctrl_gpios);
				printk(KERN_ERR "%s can't find fingerprint pinctrl\n", __func__);
				return ret;
			}
			data->pins_reset_high = pinctrl_lookup_state(data->pinctrl_gpios, "rst-high");
			if (IS_ERR(data->pins_reset_high)) {
				ret = PTR_ERR(data->pins_reset_high);
				printk(KERN_ERR "%s can't find fingerprint pinctrl reset_high\n", __func__);
				return ret;
			}
			data->pins_reset_low = pinctrl_lookup_state(data->pinctrl_gpios, "rst-low");
			if (IS_ERR(data->pins_reset_low)) {
				ret = PTR_ERR(data->pins_reset_low);
				printk(KERN_ERR "%s can't find fingerprint pinctrl reset_low\n", __func__);
				return ret;
			}
			printk(KERN_ERR "%s, get pinctrl success!\n", __func__);
		} else {
			printk(KERN_ERR "%s platform device is null\n", __func__);
		}
	} else {
		printk(KERN_ERR "%s device node is null\n", __func__);
	}

#endif
	DEBUG_PRINT("%s is successful\n", __func__);
	return 0;
}

static const struct file_operations egistec_fops = {
	.owner = THIS_MODULE,
	.write = egistec_write,
	.read = egistec_read,
	.unlocked_ioctl = egistec_ioctl,
	.compat_ioctl = egistec_compat_ioctl,
	.open = egistec_open,
	.release = egistec_release,
	.llseek = no_llseek,
	.poll = fps_interrupt_poll
};

static struct class *egistec_class;

static int egistec_probe(struct platform_device *pdev);
static int egistec_remove(struct platform_device *pdev);

typedef struct {
	struct spi_device      *spi;
	struct class           *class;
	struct device          *device;
	dev_t                  devno;
	u8                     *huge_buffer;
	size_t                 huge_buffer_size;
	struct input_dev       *input_dev;
} et512_data_t;

/* -------------------------------------------------------------------- */

static int et512_spi_probe(struct spi_device *spi)
{
	int error = 0;
	et512_data_t *et512 = NULL;
	/* size_t buffer_size; */
	DEBUG_PRINT("EGISTEC %s enter\n", __func__);

	et512 = kzalloc(sizeof(*et512), GFP_KERNEL);
	if (!et512) {
		return -ENOMEM;
	}
	printk(KERN_INFO"%s\n", __func__);
	spi_set_drvdata(spi, et512);
	g_data->spi = spi;
	DEBUG_PRINT("EGISTEC %s is successful\n", __func__);
	return error;
}

/* -------------------------------------------------------------------- */
static int et512_spi_remove(struct spi_device *spi)
{
	et512_data_t *et512 = spi_get_drvdata(spi);
	spi_clk_enable(0);

	DEBUG_PRINT("%s\n", __func__);

	kfree(et512);

	return 0;
}

static struct spi_driver spi_driver = {
	.driver = {
		.name	= "et512",
		.owner	= THIS_MODULE,
		.of_match_table = et512_spi_of_match,
        .bus	= &spi_bus_type,
	},
	.probe	= et512_spi_probe,
	.remove	= et512_spi_remove
};

struct spi_board_info spi_board_devs[] __initdata = {
    [0] = {
        .modalias="et512",
        .bus_num = 0,
        .chip_select=1,
        .mode = SPI_MODE_0,
    },
};

static struct platform_driver egistec_driver = {
	.driver = {
		.name		= "et512",
		.owner		= THIS_MODULE,
		.of_match_table = egistec_match_table,
	},
    .probe =    egistec_probe,
    .remove =   egistec_remove,
};

static int egistec_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct egistec_data *egistec = dev_get_drvdata(dev);

    DEBUG_PRINT("%s(#%d)\n", __func__, __LINE__);
	free_irq(gpio_irq, NULL);

	#if EGIS_NAVI_INPUT
	uinput_egis_destroy(egistec);
	sysfs_egis_destroy(egistec);
	#endif
	wake_lock_destroy(&et512_wake_lock);
	del_timer_sync(&fps_ints.timer);
	request_irq_done = 0;
	fb_unregister_client(&egistec->notifier);
	kfree(egistec);
    return 0;
}

static int egistec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct egistec_data *egistec;
	int status = 0;
	unsigned long minor;

	DEBUG_PRINT("%s initial\n", __func__);
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(ET512_MAJOR, "et512", &egistec_fops);
	if (status < 0) {
		pr_err("%s register_chrdev error.\n", __func__);
		return status;
	}

	egistec_class = class_create(THIS_MODULE, "et512");
	if (IS_ERR(egistec_class)) {
		pr_err("%s class_create error.\n", __func__);
		unregister_chrdev(ET512_MAJOR, egistec_driver.driver.name);
		return PTR_ERR(egistec_class);
	}

	/* Allocate driver data */
	egistec = kzalloc(sizeof(*egistec), GFP_KERNEL);
	if (egistec== NULL) {
		pr_err("%s - Failed to kzalloc\n", __func__);
		return -ENOMEM;
	}

	egistec->notifier.notifier_call = egistec_fb_notifier_callback;
	status = fb_register_client(&egistec->notifier);
	if (status) {
		DEBUG_PRINT("%s register fb failed, retval=%d\n", __func__, status);
	}

	/* device tree call */
	if (pdev->dev.of_node) {
		status = egistec_parse_dt(&pdev->dev, egistec);
		if (status) {
			pr_err("%s - Failed to parse DT\n", __func__);
			goto egistec_probe_parse_dt_failed;
		}
	}
#if GPIO_DTS
	egistec->rstPin = GPIO_PIN_RESET;
	egistec->irqPin = GPIO_PIN_IRQ;
	egistec->vcc_33v_Pin = GPIO_PIN_33V;
#endif

	/* Initialize the driver data */
	egistec->pd = pdev;
	g_data = egistec;
	wake_lock_init(&et512_wake_lock, WAKE_LOCK_SUSPEND, "et512_wake_lock");
	pr_err("egistec_module_probe\n");
	spin_lock_init(&egistec->spi_lock);
	mutex_init(&egistec->buf_lock);
	mutex_init(&device_list_lock);

	INIT_LIST_HEAD(&egistec->device_entry);

	/* platform init */
	status = egistec_platformInit(egistec);
	if (status != 0) {
		pr_err("%s platforminit failed \n", __func__);
		goto egistec_probe_platformInit_failed;
	}

#if GPIO_DTS
	/* platform init */
	status = egistec_platformInit(egistec);
	if (status != 0) {
		pr_err("%s platforminit failed\n", __func__);
		goto egistec_probe_platformInit_failed;
	}
#endif

	fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *fdev;
		egistec->devt = MKDEV(ET512_MAJOR, minor);
		fdev = device_create(egistec_class, &pdev->dev, egistec->devt,
				egistec, "esfp0");
		status = IS_ERR(fdev) ? PTR_ERR(fdev) : 0;
	} else {
		dev_dbg(&pdev->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&egistec->device_entry, &device_list);
	}

	mutex_unlock(&device_list_lock);

	if (status == 0){
		dev_set_drvdata(dev, egistec);
	}
	else {
		goto egistec_probe_failed;
	}

	fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;

	setup_timer(&fps_ints.timer, interrupt_timer_routine,(unsigned long)&fps_ints);
	add_timer(&fps_ints.timer);

#if EGIS_NAVI_INPUT
	sysfs_egis_init(egistec);
	uinput_egis_init(egistec);
#endif

	DEBUG_PRINT("%s : initialize success %d\n", __func__, status);

	request_irq_done = 0;
	return status;

egistec_probe_failed:
	device_destroy(egistec_class, egistec->devt);
	class_destroy(egistec_class);

egistec_probe_platformInit_failed:
egistec_probe_parse_dt_failed:
	kfree(egistec);
	pr_err("%s is failed\n", __func__);
	return status;
}

static void delete_device_node(void)
{
	DEBUG_PRINT("%s\n", __func__);
	device_destroy(egistec_class, g_data->devt);
	DEBUG_PRINT("device_destroy \n");
	list_del(&g_data->device_entry);
	DEBUG_PRINT("list_del \n");
	class_destroy(egistec_class);
	DEBUG_PRINT("class_destroy\n");
	unregister_chrdev(ET512_MAJOR, egistec_driver.driver.name);
	DEBUG_PRINT("unregister_chrdev\n");
	g_data = NULL;
}

static int __init egis512_init(void)
{
	int status = 0;
	int rc = 0;

	pr_err("egistec_module_init");

	/*Qingwen.Liu@BSP.Fingerprint.Basic, 20181024, modify for compatible silead fp*/
	if (FP_EGIS_520 != get_fpsensor_type()) {
		status = -EINVAL;
		printk("%s line=%d,found not et512 fp sensor status = %d\n", __func__, __LINE__, status);
		return status;
	 }
#if DUAL_FP
    int sim_gpio = -1;
    int sim_val = 1;
	struct device_node *node = NULL;

	fp_id_gpio = 174;
    gpio_direction_input(fp_id_gpio);
    fp_id_gpio_value = gpio_get_value(fp_id_gpio);

	node = of_find_compatible_node(NULL, NULL, "mediatek,fp_id");
	if(node < 0){
		pr_err("egistec get node fail \n");
	}
	sim_gpio = of_get_named_gpio(node, "fp-id-gpios", 0);
	if(sim_gpio < 0){
		pr_err("egistec get sim_gpio fail \n");
	}
	pr_err("egistec sim_gpio get done \n");
	sim_val = __gpio_get_value(sim_gpio);
	pr_err("%s, egistec_Get FP_ID from GPIO_PIN is / FP_ID = %d.\n", __func__,sim_val);

    if(sim_val==1){
		pr_info("%s, Need to register egistec FP driver\n", __func__);
    }else{
        pr_info("%s, Don't need to register egistec FP driver\n", __func__);
        return status;
    }

#endif
	printk(KERN_INFO "%s! \n", __func__);

    status = platform_driver_register(&egistec_driver);
	if(status < 0) {
		pr_err("register spi driver fail%s\n", __func__);
		return -EINVAL;
	}

	rc = spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));

	printk(KERN_ERR "%s  02  \n", __func__);

	if (rc)
	{
		printk(KERN_ERR "!!!!!spi register board info%s\n", __func__);
	    pr_err("egistec_module_init6");
	    return -EINVAL;
	}

	if (spi_register_driver(&spi_driver))
	{
		printk(KERN_ERR "register spi driver fail%s\n", __func__);
		return -EINVAL;
	}

	DEBUG_PRINT("EGISTEC spi_register_driver init OK! \n");
    return status;
}

static void __exit egis512_exit(void)
{
    platform_driver_unregister(&egistec_driver);
    spi_unregister_driver(&spi_driver);
}
/*johnson add start*/

static int egistec_regulator(void)
{
     int  ldo_vout_status;

    buck = regulator_get(NULL, "irtx_ldo");
    if (buck ==NULL) {
		printk("egitec enter cdfinger probe regulator_get r \n");
		return -1;
    }
	printk("egitec enter cdfinger probe regulator_get! \n");
    /* NOTE: the LDO is 200MV Step */
    ldo_vout_status = regulator_set_voltage(buck, 3200000, 3200000);
    if (ldo_vout_status < 0) {
		printk("egitec set 3.2V enter cdfinger probe regulator_set_voltage fail \n");
		return -1;
    }
    printk("egitec set 3.2V enter cdfinger probe regulator_set_voltage success \n");

    ldo_vout_status = regulator_enable(buck);
    if (ldo_vout_status < 0) {
		printk("egitec enter cdfinger probe regulator_enable r \n");
		return -1;
    }
	printk("egistec enter cdfinger probe regulator_enable r1111 \n");

	return 0 ;
}

module_init(egis512_init);
module_exit(egis512_exit);

MODULE_AUTHOR("Wang YuWei, <robert.wang@egistec.com>");
MODULE_DESCRIPTION("SPI Interface for ET512");
MODULE_LICENSE("GPL");
