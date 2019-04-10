/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 IMX214mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

//#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"
#include "s5kgm1spmipiraw_Sensor.h"
#ifndef VENDOR_EDIT
#define VENDOR_EDIT
#endif
#ifdef VENDOR_EDIT
/*Feng.Hu@Camera.Driver 20170815 add for multi project using one build*/
#include <soc/oppo/oppo_project.h>
#endif

#define PFX "S5KGM1SP_camera_sensor"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __func__, ##args)

#ifdef VENDOR_EDIT
/*zhaozhengtao 2016/02/19,modify for different module*/
#define MODULE_ID_OFFSET 0x0000
static kal_uint32 streaming_control(kal_bool enable);
#endif

#ifdef VENDOR_EDIT
/* Add by LiuBin for register device info at 20160616 */
#define DEVICE_VERSION_S5KGM1SP    "s5kgm1sp"
extern void register_imgsensor_deviceinfo(char *name, char *version, u8 module_id);
static kal_uint8 deviceInfo_register_value = 0x00;
#endif
static bool bNeedSetNormalMode = KAL_FALSE;
static DEFINE_SPINLOCK(imgsensor_drv_lock);


static imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5KGM1SP_SENSOR_ID,

	.checksum_value = 0xB1F1B3CC,

	.pre = {
		.pclk = 482000000,              //record different mode's pclk
		.linelength = 5024, /* record different mode's linelength */
		.framelength = 3194,         //record different mode's framelength
		.startx = 0,    //record different mode's startx of grabwindow
		.starty = 0,    //record different mode's starty of grabwindow
		.grabwindow_width = 2000, /* record different mode's width of grabwindow */
		.grabwindow_height = 1500, /* record different mode's height of grabwindow */
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.mipi_pixel_rate = 480000000,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 482000000,
		.linelength = 5024,
		.framelength = 3194,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 460800000,
		.max_framerate = 300,
	},
	#if 0
	.cap1 = {
		.pclk = 482000000,
		.linelength = 5024,
		.framelength = 3992,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 460800000,
		.max_framerate = 240,
	},
	.cap2 = {
		.pclk =560000000,
		.linelength = 10240,
		.framelength = 3643,
		.startx = 0,
		.starty =0,
		.grabwindow_width = 4640,
		.grabwindow_height = 3488,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 460800000,
		.max_framerate = 150,
	},
	#endif
	.normal_video = {
		.pclk = 482000000,
		.linelength = 5024,
		.framelength = 3194,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 2256,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 480000000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 482000000,
		.linelength = 4896,
		.framelength = 820,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 145600000,
		.max_framerate = 1200,
	},
	.custom1 = {
		.pclk = 482000000,
		.linelength = 5024,
		.framelength = 3992,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,
		.mipi_pixel_rate = 460800000,
	},
	.margin = 4,
	.min_shutter = 4,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame =0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num

	.cap_delay_frame = 3,
	.pre_delay_frame = 3,
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.custom1_delay_frame = 3,   /* enter custom1 delay frame num */
	.frame_time_delay_frame = 1,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = 0, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x21,0x5B,0xff},
};

static SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3]=
{  // Preview mode setting
   {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
    0x00, 0x2B, 0x0910, 0x06D0, 0x01, 0x00, 0x0000, 0x0000,
    0x01, 0x30, 0x0136, 0x0170, 0x03, 0x00, 0x0000, 0x0000},
   // Capture mode setting
   {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
    0x00, 0x2B, 0x1220, 0x0DA0, 0x01, 0x00, 0x0000, 0x0000,
    0x01, 0x30, 0x026C, 0x02E0, 0x03, 0x00, 0x0000, 0x0000},
   // Video mode setting
   {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
    0x00, 0x2B, 0x1220, 0x0DA0, 0x01, 0x00, 0x0000, 0x0000,
    0x01, 0x30, 0x026C, 0x0228, 0x03, 0x00, 0x0000, 0x0000}};

static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,    //mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,    //current shutter
	.gain = 0x100,    //current gain
	.dummy_pixel = 0,    //current dummypixel
	.dummy_line = 0,    //current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,    //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_mode = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x20,
#ifdef VENDOR_EDIT
	/*Chengtian.Ding@Camera, 2018-12-28 add for 18531 n+1 long exposure*/
	.current_ae_effective_frame = 1,
#endif
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
	{4000, 3000,  0,   0, 4000, 3000, 2000, 1500, 0000, 0000, 2000, 1500, 0, 0, 2000, 1500},	/* Preview */
	{4000, 3000,  0,   0, 4000, 3000, 4000, 3000, 0000, 0000, 4000, 3000, 0, 0, 4000, 3000},	/* capture */
	{4000, 3000,  0, 380, 4000, 2256, 4000, 2256, 0000, 0000, 4000, 2256, 0, 0, 4000, 2256},	/* video */
	{4000, 3000, 80, 420, 3840, 2160, 1280,  720, 0000, 0000, 1280,  720, 0, 0, 1280,  720},	/* hight speed video */
	{4000, 3000,  0,   0, 4000, 3000, 4000, 3000, 0000, 0000, 4000, 3000, 0, 0, 4000, 3000},	/* custom1 */
};


//no mirror flip
static SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
	.i4OffsetX = 16,
	.i4OffsetY = 28,
	.i4PitchX  = 32,
	.i4PitchY  = 32,
	.i4PairNum  =16,
	.i4SubBlkW  =8,
	.i4SubBlkH  =8,
	.i4PosL = {{18,29},{26,29},{34,29},{42,29},{22,41},{30,41},{38,41},{46,41},{18,49},{26,49},{34,49},{42,49},{22,53},{30,53},{38,53},{46,53}},
	.i4PosR = {{18,33},{26,33},{34,33},{42,33},{22,37},{30,37},{38,37},{46,37},{18,45},{26,45},{34,45},{42,45},{22,57},{30,57},{38,57},{46,57}},
	.i4BlockNumX = 124,
	.i4BlockNumY = 92,
	.iMirrorFlip = 0,
	.i4Crop = { {0, 0}, {0, 0}, {0, 380}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
};
static SET_PD_BLOCK_INFO_T imgsensor_pd_info_16_9 =
{
	.i4OffsetX = 16,
	.i4OffsetY = 28,
	.i4PitchX  = 32,
	.i4PitchY  = 32,
	.i4PairNum  =16,
	.i4SubBlkW  =8,
	.i4SubBlkH  =8,
	.i4PosL = {{18,29},{26,29},{34,29},{42,29},{22,41},{30,41},{38,41},{46,41},{18,49},{26,49},{34,49},{42,49},{22,53},{30,53},{38,53},{46,53}},
	.i4PosR = {{18,33},{26,33},{34,33},{42,33},{22,37},{30,37},{38,37},{46,37},{18,45},{26,45},{34,45},{42,45},{22,57},{30,57},{38,57},{46,57}},
	.i4BlockNumX = 124,
	.i4BlockNumY = 69,
	.iMirrorFlip = 0,
	.i4Crop = { {0, 0}, {0, 0}, {0, 380}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
};
static SET_PD_BLOCK_INFO_T imgsensor_pd_info_binning =
{
	.i4OffsetX = 8,
	.i4OffsetY = 14,
	.i4PitchX  = 16,
	.i4PitchY  = 16,
	.i4PairNum  =4,
	.i4SubBlkW  =8,
	.i4SubBlkH  =8,
	.i4PosL = {{8,15},{16,15},{8,25},{16,25}},
	.i4PosR = {{8,17},{16,17},{8,23},{16,23}},
	.i4BlockNumX = 124,
	.i4BlockNumY = 69,
	.iMirrorFlip = 0,
	.i4Crop = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
};

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);

#ifdef VENDOR_EDIT
/*Henry.Chang@Camera.Driver add for 18531 ModuleSN*/
static kal_uint8 gS5kgm1_SN[16];
static void read_eeprom_SN(void)
{
	kal_uint16 idx = 0;
	kal_uint8 *get_byte= &gS5kgm1_SN[0];
	for (idx = 0; idx <16; idx++) {
		char pusendcmd[2] = {0x00 , (char)((0xB0 + idx) & 0xFF) };
		iReadRegI2C(pusendcmd , 2, (u8*)&get_byte[idx],1, 0xA0);
		LOG_INF("s5kgm1_SN[%d]: 0x%x  0x%x\n", idx, get_byte[idx], gS5kgm1_SN[idx]);
	}
}

/*Henry.Chang@camera.driver 20181129, add for sensor Module SET*/
static void table_write_eeprom_8Bytes(kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	char pusendcmd[10];
	pusendcmd[0] = (char)(addr >> 8);
	pusendcmd[1] = (char)(addr & 0xFF);

	memcpy(&pusendcmd[2], para, len);

	iBurstWriteReg((kal_uint8 *)pusendcmd , (len + 2), 0xA0);
}

static kal_uint16 read_cmos_eeprom_8(kal_uint16 addr)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,0xA0);
	return get_byte;
}

/*Henry.Chang@camera.driver 20181129, add for sensor Module SET*/
static void write_Module_data(ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereodata)
{
	kal_uint16 data_base, data_length;
	kal_uint32 idx, idy;
	kal_uint8 *pData;

	if(pStereodata != NULL) {
		pr_debug("write_Module SET_SENSOR_OTP: 0x%x 0x%x %d %d\n",
                        pStereodata->baseAddr,
                        pStereodata->uSensorId,
                        pStereodata->dataLength,
                        pStereodata->uDeviceId);

		data_base = pStereodata->baseAddr;
		data_length = pStereodata->dataLength;
		pData = pStereodata->uData;
		if ((pStereodata->uSensorId == S5KGM1SP_SENSOR_ID)
				&& (data_base == 0x1640)
				&& (data_length == 1561)) {
			pr_debug("s5kgm1write_Module_data Write start: %x %x %x %x\n", pData[0], pData[1], pData[2], pData[3]);
			idx = data_length/8;
			idy = data_length%8;
			for (UINT32 i = 0; i < idx; i++ ) {
				table_write_eeprom_8Bytes((data_base+8*i), &pData[8*i], 8);
				msleep(6);
			}
			table_write_eeprom_8Bytes((data_base+8*i), &pData[8*idx], idy);
			msleep(6);
			pr_debug("s5kgm1 read 0x1640:0x%x\n", read_cmos_eeprom_8(0x1640));
			msleep(6);
			pr_debug("s5kgm1 read 0x1641:0x%x\n", read_cmos_eeprom_8(0x1641));
			msleep(6);
			pr_debug("s5kgm1 read 0x1C56:0x%x\n", read_cmos_eeprom_8(0x1C56));
			msleep(6);
			pr_debug("s5kgm1 read 0x1C57:0x%x\n", read_cmos_eeprom_8(0x1C57));
			msleep(6);
			pr_debug("s5kgm1write_Module_data Write end\n");
		}
	} else {
		pr_err("s5kgm1write_Module_data pStereodata is null\n");
	}
}
#endif
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
	iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	//return; //for test
	write_cmos_sensor(0x0340, imgsensor.frame_length);
	write_cmos_sensor(0x0342, imgsensor.line_length);
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	//kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d\n", framerate,
		min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length) {
		imgsensor.frame_length = frame_length;
	} else {
		imgsensor.frame_length = imgsensor.min_frame_length;
	}
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en) {
		imgsensor.min_frame_length = imgsensor.frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static kal_uint32 streaming_control(kal_bool enable)
{
	int timeout = 100;
	int i = 0;
	int framecnt = 0;

	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor(0x6028, 0x4000);
		write_cmos_sensor_8(0x0100, 0X01);
		mdelay(10);
	} else {
		write_cmos_sensor(0x6028, 0x4000);
		write_cmos_sensor_8(0x0100, 0x00);
		for (i = 0; i < timeout; i++) {
			mdelay(10);
			framecnt = read_cmos_sensor_8(0x0005);
			if (framecnt == 0xFF) {
				LOG_INF(" Stream Off OK at i=%d.\n", i);
				return ERROR_NONE;
			}
		}
		LOG_INF("Stream Off Fail! framecnt=%d.\n", framecnt);
	}
	return ERROR_NONE;
}


static void write_shutter(kal_uint32 shutter)
{

	kal_uint16 realtime_fps = 0;
	#ifdef VENDOR_EDIT
	kal_uint64 CintR = 0;
	/*Add by Chengtian.Ding@Camera 2018-12-28 for n+1 long exposure*/
	kal_uint64 Time_Farme = 0;

	#endif
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin) {
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	} else {
		imgsensor.frame_length = imgsensor.min_frame_length;
	}
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter) {
		shutter = imgsensor_info.min_shutter;
	}

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296,0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146,0);
		} else {
			// Extend frame length
			write_cmos_sensor(0x0340, imgsensor.frame_length);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length);
	}
	// Update Shutter
	#ifdef VENDOR_EDIT
	LOG_INF("2 shutter = %d\n", shutter);
	// 1H=5024/4820000000 =10.4 us
	// 16s=16000000/10.4 =1538462
	//1s=1000000/10.4=96153
	if (shutter >= 0xFFF0) {  // need to modify line_length & PCLK
		bNeedSetNormalMode = KAL_TRUE;


		if (shutter >= 1538000) {  //>16s
			shutter = 1538000;
		}
		// 1s > 0x05DB (1499)
		// 2s > 0x0BB6
		// 3s > 0x1191
		// 4s > 0x176C
		// 5s > 0x1D47
		// 6s > 0x2322
		// 7s > 0x28FD
		// 8s > 0x2ED8
		// 9s > 0x34B3
		//10s > 0x3A8E  959324(14956)
		//11s > 0x4069
		//12s > 0x4644
		//13s > 0x4C1F  1247122(19443)
		//14s > 0x51FA
		//15s > 0x57D5
		//16s > 0xBB61  1534919(23930)
		//0x0E14 value = shutter * 482000000 / (5024 * 2^1536)
		//0x0E14 value = shutter * 1499;
		//CintR = (482000000*shutter*0.0000104)/(5024*64);
		CintR = (5013 * (unsigned long long)shutter) / 321536;
		Time_Farme = CintR + 0x0002;  // 1st framelength
		LOG_INF("CintR =%d \n", CintR);
		//write_cmos_sensor(0x6028, 0x4000);
		//write_cmos_sensor(0x0100, 0x0000);
		streaming_control(KAL_FALSE); // check stream off
		write_cmos_sensor(0x0340, Time_Farme & 0xFFFF);  // Framelength
		write_cmos_sensor(0x0202, CintR & 0xFFFF);  //shutter
		write_cmos_sensor(0x0702, 0x0600);
		write_cmos_sensor(0x0704, 0x0600);
		//write_cmos_sensor(0x0100, 0x0100);
		streaming_control(KAL_TRUE);

		/*Chengtian.Ding@Camera, 2018-12-28 add for n+1 long exposure*/
		/* Frame exposure mode customization for LE*/
		imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		imgsensor.current_ae_effective_frame = 1;
	} else {
		if (bNeedSetNormalMode) {
			LOG_INF("exit long shutter\n");
			//write_cmos_sensor(0x6028, 0x4000);
			//write_cmos_sensor(0x0100, 0x0000);
			//remove stream off to fix long shutter issue
			streaming_control(KAL_FALSE); // check stream off
			write_cmos_sensor(0x0702, 0x0000);
			write_cmos_sensor(0x0704, 0x0000);
			//write_cmos_sensor(0x0100, 0x0100);
			streaming_control(KAL_TRUE);
			bNeedSetNormalMode = KAL_FALSE;
		}

		write_cmos_sensor(0x0340, imgsensor.frame_length);
		write_cmos_sensor(0x0202, imgsensor.shutter);

		/*Chengtian.Ding@Camera, 2018-12-27 add for 18531 n+1 long exposure*/
		imgsensor.current_ae_effective_frame = 1;
	}
	#endif
	LOG_INF("shutter =%d, framelength =%d \n", shutter,imgsensor.frame_length);

}	/*	write_shutter  */


/*************************************************************************
* FUNCTION
*	set_shutter_frame_length
*
* DESCRIPTION
*	for frame & 3A sync
*
*************************************************************************/

static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	/* OV Recommend Solution */
	/* if shutter bigger than frame_length, should extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;
	imgsensor.min_frame_length = imgsensor.frame_length;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin) {
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	}
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;


	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			/* Extend frame length */
			write_cmos_sensor(0x0340, imgsensor.frame_length);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x0340, imgsensor.frame_length);
	}

	/* Update Shutter */
	write_cmos_sensor(0x0202, imgsensor.shutter);
	LOG_INF("Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
		shutter, imgsensor.frame_length, frame_length, dummy_line, read_cmos_sensor(0x0350));


}	/* set_shutter_frame_length */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	 kal_uint16 reg_gain = 0x0;

    reg_gain = gain/2;
    return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN) {
			gain = BASEGAIN;
		} else if (gain > 32 * BASEGAIN) {
			gain = 32 * BASEGAIN;
		}
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	//write_cmos_sensor(0x0204,reg_gain);
	write_cmos_sensor_8(0x0204, (reg_gain >> 8));
	write_cmos_sensor_8(0x0205, (reg_gain & 0xff));

    return gain;
}	/*	set_gain  */
#if 1
static void set_mirror_flip(kal_uint8 image_mirror)
{
	kal_uint8 itemp;
	LOG_INF("image_mirror = %d\n", image_mirror);
	itemp=read_cmos_sensor(0x0101);
	itemp &= ~0x03;

	switch(image_mirror)
		{

			case IMAGE_NORMAL:
				write_cmos_sensor(0x0101, itemp);
				break;

			case IMAGE_V_MIRROR:
				write_cmos_sensor(0x0101, itemp | 0x02);
				break;

			case IMAGE_H_MIRROR:
				write_cmos_sensor(0x0101, itemp | 0x01);
				break;

			case IMAGE_HV_MIRROR:
				write_cmos_sensor(0x0101, itemp | 0x03);
				break;
		}
}
#endif
/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
#if 0
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/
#endif

static void sensor_init(void)
{
	LOG_INF("E\n");
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0000, 0x0006);
	write_cmos_sensor(0x0000, 0x08D1);
	write_cmos_sensor(0x6010, 0x0001);
	mdelay(3);
	write_cmos_sensor(0x6214, 0x7971);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x0A02, 0x0074);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x3F5C);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0549);
	write_cmos_sensor(0x6F12, 0x0448);
	write_cmos_sensor(0x6F12, 0x054A);
	write_cmos_sensor(0x6F12, 0xC1F8);
	write_cmos_sensor(0x6F12, 0x5005);
	write_cmos_sensor(0x6F12, 0x101A);
	write_cmos_sensor(0x6F12, 0xA1F8);
	write_cmos_sensor(0x6F12, 0x5405);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB6B9);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x441C);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2E30);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x6E00);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xFF5F);
	write_cmos_sensor(0x6F12, 0xE948);
	write_cmos_sensor(0x6F12, 0x8B46);
	write_cmos_sensor(0x6F12, 0x1746);
	write_cmos_sensor(0x6F12, 0x0068);
	write_cmos_sensor(0x6F12, 0x9A46);
	write_cmos_sensor(0x6F12, 0x4FEA);
	write_cmos_sensor(0x6F12, 0x1049);
	write_cmos_sensor(0x6F12, 0x80B2);
	write_cmos_sensor(0x6F12, 0x8046);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x4846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xE2F9);
	write_cmos_sensor(0x6F12, 0xE34D);
	write_cmos_sensor(0x6F12, 0x95F8);
	write_cmos_sensor(0x6F12, 0x6D00);
	write_cmos_sensor(0x6F12, 0x0228);
	write_cmos_sensor(0x6F12, 0x35D0);
	write_cmos_sensor(0x6F12, 0x0224);
	write_cmos_sensor(0x6F12, 0xE14E);
	write_cmos_sensor(0x6F12, 0x5346);
	write_cmos_sensor(0x6F12, 0xB6F8);
	write_cmos_sensor(0x6F12, 0xB802);
	write_cmos_sensor(0x6F12, 0xB0FB);
	write_cmos_sensor(0x6F12, 0xF4F0);
	write_cmos_sensor(0x6F12, 0xA6F8);
	write_cmos_sensor(0x6F12, 0xB802);
	write_cmos_sensor(0x6F12, 0xD5F8);
	write_cmos_sensor(0x6F12, 0x1411);
	write_cmos_sensor(0x6F12, 0x06F5);
	write_cmos_sensor(0x6F12, 0x2E76);
	write_cmos_sensor(0x6F12, 0x6143);
	write_cmos_sensor(0x6F12, 0xC5F8);
	write_cmos_sensor(0x6F12, 0x1411);
	write_cmos_sensor(0x6F12, 0xB5F8);
	write_cmos_sensor(0x6F12, 0x8C11);
	write_cmos_sensor(0x6F12, 0x411A);
	write_cmos_sensor(0x6F12, 0x89B2);
	write_cmos_sensor(0x6F12, 0x25F8);
	write_cmos_sensor(0x6F12, 0x981B);
	write_cmos_sensor(0x6F12, 0x35F8);
	write_cmos_sensor(0x6F12, 0x142C);
	write_cmos_sensor(0x6F12, 0x6243);
	write_cmos_sensor(0x6F12, 0x521E);
	write_cmos_sensor(0x6F12, 0x00FB);
	write_cmos_sensor(0x6F12, 0x0210);
	write_cmos_sensor(0x6F12, 0xB5F8);
	write_cmos_sensor(0x6F12, 0xF210);
	write_cmos_sensor(0x6F12, 0x07FB);
	write_cmos_sensor(0x6F12, 0x04F2);
	write_cmos_sensor(0x6F12, 0x0844);
	write_cmos_sensor(0x6F12, 0xC5F8);
	write_cmos_sensor(0x6F12, 0xF800);
	write_cmos_sensor(0x6F12, 0x5946);
	write_cmos_sensor(0x6F12, 0x0098);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xBBF9);
	write_cmos_sensor(0x6F12, 0x3088);
	write_cmos_sensor(0x6F12, 0x4146);
	write_cmos_sensor(0x6F12, 0x6043);
	write_cmos_sensor(0x6F12, 0x3080);
	write_cmos_sensor(0x6F12, 0xE86F);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0xB0FB);
	write_cmos_sensor(0x6F12, 0xF4F0);
	write_cmos_sensor(0x6F12, 0xE867);
	write_cmos_sensor(0x6F12, 0x04B0);
	write_cmos_sensor(0x6F12, 0x4846);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF05F);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xA7B9);
	write_cmos_sensor(0x6F12, 0x0124);
	write_cmos_sensor(0x6F12, 0xC8E7);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x8046);
	write_cmos_sensor(0x6F12, 0xC248);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x4168);
	write_cmos_sensor(0x6F12, 0x0D0C);
	write_cmos_sensor(0x6F12, 0x8EB2);
	write_cmos_sensor(0x6F12, 0x3146);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x99F9);
	write_cmos_sensor(0x6F12, 0xC14C);
	write_cmos_sensor(0x6F12, 0xBF4F);
	write_cmos_sensor(0x6F12, 0x2078);
	write_cmos_sensor(0x6F12, 0x97F8);
	write_cmos_sensor(0x6F12, 0x8B12);
	write_cmos_sensor(0x6F12, 0x10FB);
	write_cmos_sensor(0x6F12, 0x01F0);
	write_cmos_sensor(0x6F12, 0x2070);
	write_cmos_sensor(0x6F12, 0x4046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x98F9);
	write_cmos_sensor(0x6F12, 0x2078);
	write_cmos_sensor(0x6F12, 0x97F8);
	write_cmos_sensor(0x6F12, 0x8B12);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0xB0FB);
	write_cmos_sensor(0x6F12, 0xF1F0);
	write_cmos_sensor(0x6F12, 0x2070);
	write_cmos_sensor(0x6F12, 0x3146);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x81B9);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xFF47);
	write_cmos_sensor(0x6F12, 0x8146);
	write_cmos_sensor(0x6F12, 0xB048);
	write_cmos_sensor(0x6F12, 0x1746);
	write_cmos_sensor(0x6F12, 0x8846);
	write_cmos_sensor(0x6F12, 0x8068);
	write_cmos_sensor(0x6F12, 0x1C46);
	write_cmos_sensor(0x6F12, 0x85B2);
	write_cmos_sensor(0x6F12, 0x060C);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x3046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x72F9);
	write_cmos_sensor(0x6F12, 0x2346);
	write_cmos_sensor(0x6F12, 0x3A46);
	write_cmos_sensor(0x6F12, 0x4146);
	write_cmos_sensor(0x6F12, 0x4846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x7BF9);
	write_cmos_sensor(0x6F12, 0xAB4A);
	write_cmos_sensor(0x6F12, 0x9088);
	write_cmos_sensor(0x6F12, 0xF0B3);
	write_cmos_sensor(0x6F12, 0xA848);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0xBA10);
	write_cmos_sensor(0x6F12, 0xD1B3);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x2801);
	write_cmos_sensor(0x6F12, 0x1168);
	write_cmos_sensor(0x6F12, 0x8842);
	write_cmos_sensor(0x6F12, 0x00D3);
	write_cmos_sensor(0x6F12, 0x0846);
	write_cmos_sensor(0x6F12, 0x010A);
	write_cmos_sensor(0x6F12, 0xB1FA);
	write_cmos_sensor(0x6F12, 0x81F0);
	write_cmos_sensor(0x6F12, 0xC0F1);
	write_cmos_sensor(0x6F12, 0x1700);
	write_cmos_sensor(0x6F12, 0xC140);
	write_cmos_sensor(0x6F12, 0x02EB);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0xC9B2);
	write_cmos_sensor(0x6F12, 0x0389);
	write_cmos_sensor(0x6F12, 0xC288);
	write_cmos_sensor(0x6F12, 0x9B1A);
	write_cmos_sensor(0x6F12, 0x4B43);
	write_cmos_sensor(0x6F12, 0x8033);
	write_cmos_sensor(0x6F12, 0x02EB);
	write_cmos_sensor(0x6F12, 0x2322);
	write_cmos_sensor(0x6F12, 0x0092);
	write_cmos_sensor(0x6F12, 0x438A);
	write_cmos_sensor(0x6F12, 0x028A);
	write_cmos_sensor(0x6F12, 0x9B1A);
	write_cmos_sensor(0x6F12, 0x4B43);
	write_cmos_sensor(0x6F12, 0x8033);
	write_cmos_sensor(0x6F12, 0x02EB);
	write_cmos_sensor(0x6F12, 0x2322);
	write_cmos_sensor(0x6F12, 0x0192);
	write_cmos_sensor(0x6F12, 0x838B);
	write_cmos_sensor(0x6F12, 0x428B);
	write_cmos_sensor(0x6F12, 0x9B1A);
	write_cmos_sensor(0x6F12, 0x4B43);
	write_cmos_sensor(0x6F12, 0x8033);
	write_cmos_sensor(0x6F12, 0x02EB);
	write_cmos_sensor(0x6F12, 0x2322);
	write_cmos_sensor(0x6F12, 0x0292);
	write_cmos_sensor(0x6F12, 0xC28C);
	write_cmos_sensor(0x6F12, 0x808C);
	write_cmos_sensor(0x6F12, 0x121A);
	write_cmos_sensor(0x6F12, 0x4A43);
	write_cmos_sensor(0x6F12, 0x8032);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x2220);
	write_cmos_sensor(0x6F12, 0x0390);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x6846);
	write_cmos_sensor(0x6F12, 0x54F8);
	write_cmos_sensor(0x6F12, 0x2210);
	write_cmos_sensor(0x6F12, 0x50F8);
	write_cmos_sensor(0x6F12, 0x2230);
	write_cmos_sensor(0x6F12, 0x5943);
	write_cmos_sensor(0x6F12, 0x090B);
	write_cmos_sensor(0x6F12, 0x44F8);
	write_cmos_sensor(0x6F12, 0x2210);
	write_cmos_sensor(0x6F12, 0x521C);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x01E0);
	write_cmos_sensor(0x6F12, 0x042A);
	write_cmos_sensor(0x6F12, 0xF2D3);
	write_cmos_sensor(0x6F12, 0x04B0);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x3046);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF047);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x1FB9);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x864C);
	write_cmos_sensor(0x6F12, 0x8449);
	write_cmos_sensor(0x6F12, 0x0646);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0x6970);
	write_cmos_sensor(0x6F12, 0x8988);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0x8120);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0xC1B1);
	write_cmos_sensor(0x6F12, 0x2146);
	write_cmos_sensor(0x6F12, 0xD1F8);
	write_cmos_sensor(0x6F12, 0x9410);
	write_cmos_sensor(0x6F12, 0x72B1);
	write_cmos_sensor(0x6F12, 0x8FB1);
	write_cmos_sensor(0x6F12, 0x0846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x1FF9);
	write_cmos_sensor(0x6F12, 0x0546);
	write_cmos_sensor(0x6F12, 0xE06F);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x1BF9);
	write_cmos_sensor(0x6F12, 0x8542);
	write_cmos_sensor(0x6F12, 0x02D2);
	write_cmos_sensor(0x6F12, 0xD4F8);
	write_cmos_sensor(0x6F12, 0x9400);
	write_cmos_sensor(0x6F12, 0x26E0);
	write_cmos_sensor(0x6F12, 0xE06F);
	write_cmos_sensor(0x6F12, 0x24E0);
	write_cmos_sensor(0x6F12, 0x002F);
	write_cmos_sensor(0x6F12, 0xFBD1);
	write_cmos_sensor(0x6F12, 0x002A);
	write_cmos_sensor(0x6F12, 0x24D0);
	write_cmos_sensor(0x6F12, 0x0846);
	write_cmos_sensor(0x6F12, 0x1EE0);
	write_cmos_sensor(0x6F12, 0x7249);
	write_cmos_sensor(0x6F12, 0x0D8E);
	write_cmos_sensor(0x6F12, 0x496B);
	write_cmos_sensor(0x6F12, 0x4B42);
	write_cmos_sensor(0x6F12, 0x77B1);
	write_cmos_sensor(0x6F12, 0x7248);
	write_cmos_sensor(0x6F12, 0x806F);
	write_cmos_sensor(0x6F12, 0x10E0);
	write_cmos_sensor(0x6F12, 0x4242);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x0246);
	write_cmos_sensor(0x6F12, 0x0029);
	write_cmos_sensor(0x6F12, 0x0FDB);
	write_cmos_sensor(0x6F12, 0x8A42);
	write_cmos_sensor(0x6F12, 0x0FDD);
	write_cmos_sensor(0x6F12, 0x3046);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xFFB8);
	write_cmos_sensor(0x6F12, 0x002A);
	write_cmos_sensor(0x6F12, 0x0CD0);
	write_cmos_sensor(0x6F12, 0x6948);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x8C00);
	write_cmos_sensor(0x6F12, 0x25B1);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0xEDDA);
	write_cmos_sensor(0x6F12, 0xEAE7);
	write_cmos_sensor(0x6F12, 0x1946);
	write_cmos_sensor(0x6F12, 0xEDE7);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xF7F8);
	write_cmos_sensor(0x6F12, 0xE060);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF081);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF35F);
	write_cmos_sensor(0x6F12, 0xDFF8);
	write_cmos_sensor(0x6F12, 0x74A1);
	write_cmos_sensor(0x6F12, 0x0C46);
	write_cmos_sensor(0x6F12, 0xBAF8);
	write_cmos_sensor(0x6F12, 0xBE04);
	write_cmos_sensor(0x6F12, 0x08B1);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xEEF8);
	write_cmos_sensor(0x6F12, 0x5D4E);
	write_cmos_sensor(0x6F12, 0x3088);
	write_cmos_sensor(0x6F12, 0x0128);
	write_cmos_sensor(0x6F12, 0x06D1);
	write_cmos_sensor(0x6F12, 0x002C);
	write_cmos_sensor(0x6F12, 0x04D1);
	write_cmos_sensor(0x6F12, 0x594D);
	write_cmos_sensor(0x6F12, 0x2889);
	write_cmos_sensor(0x6F12, 0x18B1);
	write_cmos_sensor(0x6F12, 0x401E);
	write_cmos_sensor(0x6F12, 0x2881);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xFC9F);
	write_cmos_sensor(0x6F12, 0xDFF8);
	write_cmos_sensor(0x6F12, 0x5C91);
	write_cmos_sensor(0x6F12, 0xD9F8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0xD602);
	write_cmos_sensor(0x6F12, 0x38B1);
	write_cmos_sensor(0x6F12, 0x3089);
	write_cmos_sensor(0x6F12, 0x401C);
	write_cmos_sensor(0x6F12, 0x80B2);
	write_cmos_sensor(0x6F12, 0x3081);
	write_cmos_sensor(0x6F12, 0xFF28);
	write_cmos_sensor(0x6F12, 0x01D9);
	write_cmos_sensor(0x6F12, 0xE889);
	write_cmos_sensor(0x6F12, 0x3081);
	write_cmos_sensor(0x6F12, 0x5148);
	write_cmos_sensor(0x6F12, 0x4FF0);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x6F12, 0xC6F8);
	write_cmos_sensor(0x6F12, 0x0C80);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x5EB0);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0xFF31);
	write_cmos_sensor(0x6F12, 0x0B20);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xCBF8);
	write_cmos_sensor(0x6F12, 0xD9F8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0027);
	write_cmos_sensor(0x6F12, 0x3C46);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0xD412);
	write_cmos_sensor(0x6F12, 0x21B1);
	write_cmos_sensor(0x6F12, 0x0098);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB2F8);
	write_cmos_sensor(0x6F12, 0x0746);
	write_cmos_sensor(0x6F12, 0x0BE0);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0xD602);
	write_cmos_sensor(0x6F12, 0x40B1);
	write_cmos_sensor(0x6F12, 0x3089);
	write_cmos_sensor(0x6F12, 0xE989);
	write_cmos_sensor(0x6F12, 0x8842);
	write_cmos_sensor(0x6F12, 0x04D3);
	write_cmos_sensor(0x6F12, 0x0098);
	write_cmos_sensor(0x6F12, 0xFFF7);
	write_cmos_sensor(0x6F12, 0x6EFF);
	write_cmos_sensor(0x6F12, 0x0746);
	write_cmos_sensor(0x6F12, 0x0124);
	write_cmos_sensor(0x6F12, 0x3846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB5F8);
	write_cmos_sensor(0x6F12, 0xD9F8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0xD602);
	write_cmos_sensor(0x6F12, 0x08B9);
	write_cmos_sensor(0x6F12, 0xA6F8);
	write_cmos_sensor(0x6F12, 0x0280);
	write_cmos_sensor(0x6F12, 0xC7B3);
	write_cmos_sensor(0x6F12, 0x4746);
	write_cmos_sensor(0x6F12, 0xA6F8);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xADF8);
	write_cmos_sensor(0x6F12, 0xF068);
	write_cmos_sensor(0x6F12, 0x3061);
	write_cmos_sensor(0x6F12, 0x688D);
	write_cmos_sensor(0x6F12, 0x50B3);
	write_cmos_sensor(0x6F12, 0xA88D);
	write_cmos_sensor(0x6F12, 0x50BB);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xAAF8);
	write_cmos_sensor(0x6F12, 0xA889);
	write_cmos_sensor(0x6F12, 0x20B3);
	write_cmos_sensor(0x6F12, 0x1CB3);
	write_cmos_sensor(0x6F12, 0x706B);
	write_cmos_sensor(0x6F12, 0xAA88);
	write_cmos_sensor(0x6F12, 0xDAF8);
	write_cmos_sensor(0x6F12, 0x0815);
	write_cmos_sensor(0x6F12, 0xCAB1);
	write_cmos_sensor(0x6F12, 0x8842);
	write_cmos_sensor(0x6F12, 0x0CDB);
	write_cmos_sensor(0x6F12, 0x90FB);
	write_cmos_sensor(0x6F12, 0xF1F3);
	write_cmos_sensor(0x6F12, 0x90FB);
	write_cmos_sensor(0x6F12, 0xF1F2);
	write_cmos_sensor(0x6F12, 0x01FB);
	write_cmos_sensor(0x6F12, 0x1303);
	write_cmos_sensor(0x6F12, 0xB3EB);
	write_cmos_sensor(0x6F12, 0x610F);
	write_cmos_sensor(0x6F12, 0x00DD);
	write_cmos_sensor(0x6F12, 0x521C);
	write_cmos_sensor(0x6F12, 0x01FB);
	write_cmos_sensor(0x6F12, 0x1200);
	write_cmos_sensor(0x6F12, 0x0BE0);
	write_cmos_sensor(0x6F12, 0x91FB);
	write_cmos_sensor(0x6F12, 0xF0F3);
	write_cmos_sensor(0x6F12, 0x91FB);
	write_cmos_sensor(0x6F12, 0xF0F2);
	write_cmos_sensor(0x6F12, 0x00FB);
	write_cmos_sensor(0x6F12, 0x1313);
	write_cmos_sensor(0x6F12, 0xB3EB);
	write_cmos_sensor(0x6F12, 0x600F);
	write_cmos_sensor(0x6F12, 0x00DD);
	write_cmos_sensor(0x6F12, 0x521C);
	write_cmos_sensor(0x6F12, 0x5043);
	write_cmos_sensor(0x6F12, 0x401A);
	write_cmos_sensor(0x6F12, 0xF168);
	write_cmos_sensor(0x6F12, 0x01EB);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0xF060);
	write_cmos_sensor(0x6F12, 0xA88D);
	write_cmos_sensor(0x6F12, 0x10B1);
	write_cmos_sensor(0x6F12, 0xF089);
	write_cmos_sensor(0x6F12, 0x3087);
	write_cmos_sensor(0x6F12, 0xAF85);
	write_cmos_sensor(0x6F12, 0x5846);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xFC5F);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x7EB8);
	write_cmos_sensor(0x6F12, 0x10B5);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x6731);
	write_cmos_sensor(0x6F12, 0x1948);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x7CF8);
	write_cmos_sensor(0x6F12, 0x0F4C);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xD921);
	write_cmos_sensor(0x6F12, 0x2060);
	write_cmos_sensor(0x6F12, 0x1648);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x74F8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xA121);
	write_cmos_sensor(0x6F12, 0x6060);
	write_cmos_sensor(0x6F12, 0x1448);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x6DF8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xE911);
	write_cmos_sensor(0x6F12, 0xA060);
	write_cmos_sensor(0x6F12, 0x1148);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x66F8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x6511);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0x1040);
	write_cmos_sensor(0x6F12, 0x0E48);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x5EB8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x4410);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2C30);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2E30);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2580);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x6000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2BA0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x3600);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x0890);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x7000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x24A7);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x1AF3);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x09BD);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x576B);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x57ED);
	write_cmos_sensor(0x6F12, 0x4AF6);
	write_cmos_sensor(0x6F12, 0x293C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x42F2);
	write_cmos_sensor(0x6F12, 0xA74C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x41F6);
	write_cmos_sensor(0x6F12, 0xF32C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x010C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x40F6);
	write_cmos_sensor(0x6F12, 0xBD1C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x010C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4AF6);
	write_cmos_sensor(0x6F12, 0x532C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x45F2);
	write_cmos_sensor(0x6F12, 0x377C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x45F2);
	write_cmos_sensor(0x6F12, 0xD56C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x45F2);
	write_cmos_sensor(0x6F12, 0xC91C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0xAB2C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x44F6);
	write_cmos_sensor(0x6F12, 0x897C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x45F2);
	write_cmos_sensor(0x6F12, 0xA56C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x45F2);
	write_cmos_sensor(0x6F12, 0xEF6C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0x6D7C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4BF2);
	write_cmos_sensor(0x6F12, 0xAB4C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x08D1);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0027);
}	/*      sensor_init  */


static void preview_setting(void)
{
	//Preview 2320*1744 30fps 24M MCLK 4lane 1200Mbps/lane
	// preview 30.01fps
	LOG_INF("preview_setting\n");
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0x7971);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x0344, 0x0008);
	write_cmos_sensor(0x0346, 0x0008);
	write_cmos_sensor(0x0348, 0x0FA7);
	write_cmos_sensor(0x034A, 0x0BBF);
	write_cmos_sensor(0x034C, 0x07D0);
	write_cmos_sensor(0x034E, 0x05DC);
	write_cmos_sensor(0x0350, 0x0000);
	write_cmos_sensor(0x0352, 0x0000);
	write_cmos_sensor(0x0340, 0x0C78);
	write_cmos_sensor(0x0342, 0x13A0);
	write_cmos_sensor(0x0900, 0x0112);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0003);
	write_cmos_sensor(0x0404, 0x2000);
	write_cmos_sensor(0x0402, 0x1010);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x0306, 0x00F1);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0300, 0x0008);
	write_cmos_sensor(0x030E, 0x0004);
	write_cmos_sensor(0x0312, 0x0000);
	write_cmos_sensor(0x0310, 0x0064);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1492);
	write_cmos_sensor(0x6F12, 0x0078);
	write_cmos_sensor(0x602A, 0x0E4E);
	write_cmos_sensor(0x6F12, 0x006F);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0118, 0x0004);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x2126);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1168);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x602A, 0x2DB6);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1668);
	write_cmos_sensor(0x6F12, 0xFF00);
	write_cmos_sensor(0x602A, 0x166A);
	write_cmos_sensor(0x6F12, 0xFF00);
	write_cmos_sensor(0x602A, 0x118A);
	write_cmos_sensor(0x6F12, 0x0802);
	write_cmos_sensor(0x602A, 0x151E);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x602A, 0x217E);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1520);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x2522);
	write_cmos_sensor(0x6F12, 0x1004);
	write_cmos_sensor(0x602A, 0x2524);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x2568);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x2588);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x258C);
	write_cmos_sensor(0x6F12, 0x1111);
	write_cmos_sensor(0x602A, 0x25A6);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x252C);
	write_cmos_sensor(0x6F12, 0x7801);
	write_cmos_sensor(0x602A, 0x252E);
	write_cmos_sensor(0x6F12, 0x7805);
	write_cmos_sensor(0x602A, 0x25A8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x25AC);
	write_cmos_sensor(0x6F12, 0x1111);
	write_cmos_sensor(0x602A, 0x25B0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x25B4);
	write_cmos_sensor(0x6F12, 0x1111);
	write_cmos_sensor(0x602A, 0x15A4);
	write_cmos_sensor(0x6F12, 0x0641);
	write_cmos_sensor(0x602A, 0x15A6);
	write_cmos_sensor(0x6F12, 0x0145);
	write_cmos_sensor(0x602A, 0x15A8);
	write_cmos_sensor(0x6F12, 0x0149);
	write_cmos_sensor(0x602A, 0x15AA);
	write_cmos_sensor(0x6F12, 0x064D);
	write_cmos_sensor(0x602A, 0x15AC);
	write_cmos_sensor(0x6F12, 0x0651);
	write_cmos_sensor(0x602A, 0x15AE);
	write_cmos_sensor(0x6F12, 0x0155);
	write_cmos_sensor(0x602A, 0x15B0);
	write_cmos_sensor(0x6F12, 0x0159);
	write_cmos_sensor(0x602A, 0x15B2);
	write_cmos_sensor(0x6F12, 0x065D);
	write_cmos_sensor(0x602A, 0x15B4);
	write_cmos_sensor(0x6F12, 0x0661);
	write_cmos_sensor(0x602A, 0x15B6);
	write_cmos_sensor(0x6F12, 0x0165);
	write_cmos_sensor(0x602A, 0x15B8);
	write_cmos_sensor(0x6F12, 0x0169);
	write_cmos_sensor(0x602A, 0x15BA);
	write_cmos_sensor(0x6F12, 0x066D);
	write_cmos_sensor(0x602A, 0x15BC);
	write_cmos_sensor(0x6F12, 0x0671);
	write_cmos_sensor(0x602A, 0x15BE);
	write_cmos_sensor(0x6F12, 0x0175);
	write_cmos_sensor(0x602A, 0x15C0);
	write_cmos_sensor(0x6F12, 0x0179);
	write_cmos_sensor(0x602A, 0x15C2);
	write_cmos_sensor(0x6F12, 0x067D);
	write_cmos_sensor(0x602A, 0x15C4);
	write_cmos_sensor(0x6F12, 0x0641);
	write_cmos_sensor(0x602A, 0x15C6);
	write_cmos_sensor(0x6F12, 0x0145);
	write_cmos_sensor(0x602A, 0x15C8);
	write_cmos_sensor(0x6F12, 0x0149);
	write_cmos_sensor(0x602A, 0x15CA);
	write_cmos_sensor(0x6F12, 0x064D);
	write_cmos_sensor(0x602A, 0x15CC);
	write_cmos_sensor(0x6F12, 0x0651);
	write_cmos_sensor(0x602A, 0x15CE);
	write_cmos_sensor(0x6F12, 0x0155);
	write_cmos_sensor(0x602A, 0x15D0);
	write_cmos_sensor(0x6F12, 0x0159);
	write_cmos_sensor(0x602A, 0x15D2);
	write_cmos_sensor(0x6F12, 0x065D);
	write_cmos_sensor(0x602A, 0x15D4);
	write_cmos_sensor(0x6F12, 0x0661);
	write_cmos_sensor(0x602A, 0x15D6);
	write_cmos_sensor(0x6F12, 0x0165);
	write_cmos_sensor(0x602A, 0x15D8);
	write_cmos_sensor(0x6F12, 0x0169);
	write_cmos_sensor(0x602A, 0x15DA);
	write_cmos_sensor(0x6F12, 0x066D);
	write_cmos_sensor(0x602A, 0x15DC);
	write_cmos_sensor(0x6F12, 0x0671);
	write_cmos_sensor(0x602A, 0x15DE);
	write_cmos_sensor(0x6F12, 0x0175);
	write_cmos_sensor(0x602A, 0x15E0);
	write_cmos_sensor(0x6F12, 0x0179);
	write_cmos_sensor(0x602A, 0x15E2);
	write_cmos_sensor(0x6F12, 0x067D);
	write_cmos_sensor(0x602A, 0x1A50);
	write_cmos_sensor(0x6F12, 0x0004);
	write_cmos_sensor(0x602A, 0x1A54);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0D00, 0x0101);
	write_cmos_sensor(0x0D02, 0x0101);
	write_cmos_sensor(0x0114, 0x0301);
	write_cmos_sensor(0x0202, 0x0010);
	write_cmos_sensor(0x0226, 0x0010);
	write_cmos_sensor(0x0204, 0x0020);
	write_cmos_sensor(0x0B06, 0x0101);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x107A);
	write_cmos_sensor(0x6F12, 0x1D00);
	write_cmos_sensor(0x602A, 0x1074);
	write_cmos_sensor(0x6F12, 0x1D00);
	write_cmos_sensor(0x602A, 0x0E7C);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1120);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x1122);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x602A, 0x1128);
	write_cmos_sensor(0x6F12, 0x0604);
	write_cmos_sensor(0x602A, 0x1AC0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x1AC2);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x602A, 0x1494);
	write_cmos_sensor(0x6F12, 0x3D68);
	write_cmos_sensor(0x602A, 0x1498);
	write_cmos_sensor(0x6F12, 0xF10D);
	write_cmos_sensor(0x602A, 0x1488);
	write_cmos_sensor(0x6F12, 0x0F0F);
	write_cmos_sensor(0x602A, 0x148A);
	write_cmos_sensor(0x6F12, 0x0F0F);
	write_cmos_sensor(0x602A, 0x150E);
	write_cmos_sensor(0x6F12, 0x00C2);
	write_cmos_sensor(0x602A, 0x1510);
	write_cmos_sensor(0x6F12, 0xC0AF);
	write_cmos_sensor(0x602A, 0x1512);
	write_cmos_sensor(0x6F12, 0x0080);
	write_cmos_sensor(0x602A, 0x1486);
	write_cmos_sensor(0x6F12, 0x1430);
	write_cmos_sensor(0x602A, 0x1490);
	write_cmos_sensor(0x6F12, 0x4D09);
	write_cmos_sensor(0x602A, 0x149E);
	write_cmos_sensor(0x6F12, 0x01C4);
	write_cmos_sensor(0x602A, 0x11CC);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x602A, 0x11CE);
	write_cmos_sensor(0x6F12, 0x000B);
	write_cmos_sensor(0x602A, 0x11D0);
	write_cmos_sensor(0x6F12, 0x0003);
	write_cmos_sensor(0x602A, 0x11DA);
	write_cmos_sensor(0x6F12, 0x0012);
	write_cmos_sensor(0x602A, 0x11E6);
	write_cmos_sensor(0x6F12, 0x002A);
	write_cmos_sensor(0x602A, 0x125E);
	write_cmos_sensor(0x6F12, 0x0048);
	write_cmos_sensor(0x602A, 0x11F4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x11F8);
	write_cmos_sensor(0x6F12, 0x0016);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0xF444, 0x05BF);
	write_cmos_sensor(0xF44A, 0x0008);
	write_cmos_sensor(0xF44E, 0x0012);
	write_cmos_sensor(0xF46E, 0x74C0);
	write_cmos_sensor(0xF470, 0x2809);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1CAA);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CAC);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CAE);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB2);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBA);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBC);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBE);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC2);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x6000);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x6002);
	write_cmos_sensor(0x6F12, 0xFFFF);
	write_cmos_sensor(0x602A, 0x6004);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x6006);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6008);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600E);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6010);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6012);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6014);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6016);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6018);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601E);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6020);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6022);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6024);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6026);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6028);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x602A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x602C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x1144);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1146);
	write_cmos_sensor(0x6F12, 0x1B00);
	write_cmos_sensor(0x602A, 0x1080);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1084);
	write_cmos_sensor(0x6F12, 0x00C0);
	write_cmos_sensor(0x602A, 0x108A);
	write_cmos_sensor(0x6F12, 0x00C0);
	write_cmos_sensor(0x602A, 0x1090);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1092);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1094);
	write_cmos_sensor(0x6F12, 0xA32E);
}	/* preview_setting  */

static void capture_setting(void)
{
	LOG_INF("capture_setting enter\n");
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0x7971);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x0344, 0x0008);
	write_cmos_sensor(0x0346, 0x0008);
	write_cmos_sensor(0x0348, 0x0FA7);
	write_cmos_sensor(0x034A, 0x0BBF);
	write_cmos_sensor(0x034C, 0x0FA0);
	write_cmos_sensor(0x034E, 0x0BB8);
	write_cmos_sensor(0x0350, 0x0000);
	write_cmos_sensor(0x0352, 0x0000);
	write_cmos_sensor(0x0340, 0x0C7A);
	write_cmos_sensor(0x0342, 0x13A0);
	write_cmos_sensor(0x0900, 0x0111);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0001);
	write_cmos_sensor(0x0404, 0x1000);
	write_cmos_sensor(0x0402, 0x1010);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x0306, 0x00F1);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0300, 0x0008);
	write_cmos_sensor(0x030E, 0x0003);
	write_cmos_sensor(0x0312, 0x0001);
	write_cmos_sensor(0x0310, 0x0090);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1492);
	write_cmos_sensor(0x6F12, 0x0078);
	write_cmos_sensor(0x602A, 0x0E4E);
	write_cmos_sensor(0x6F12, 0x007A);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0118, 0x0004);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x2126);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1168);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x602A, 0x2DB6);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1668);
	write_cmos_sensor(0x6F12, 0xF0F0);
	write_cmos_sensor(0x602A, 0x166A);
	write_cmos_sensor(0x6F12, 0xF0F0);
	write_cmos_sensor(0x602A, 0x118A);
	write_cmos_sensor(0x6F12, 0x0802);
	write_cmos_sensor(0x602A, 0x151E);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x217E);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1520);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x602A, 0x2522);
	write_cmos_sensor(0x6F12, 0x0804);
	write_cmos_sensor(0x602A, 0x2524);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x602A, 0x2568);
	write_cmos_sensor(0x6F12, 0x5500);
	write_cmos_sensor(0x602A, 0x2588);
	write_cmos_sensor(0x6F12, 0x1111);
	write_cmos_sensor(0x602A, 0x258C);
	write_cmos_sensor(0x6F12, 0x1111);
	write_cmos_sensor(0x602A, 0x25A6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x252C);
	write_cmos_sensor(0x6F12, 0x0601);
	write_cmos_sensor(0x602A, 0x252E);
	write_cmos_sensor(0x6F12, 0x0605);
	write_cmos_sensor(0x602A, 0x25A8);
	write_cmos_sensor(0x6F12, 0x1100);
	write_cmos_sensor(0x602A, 0x25AC);
	write_cmos_sensor(0x6F12, 0x0011);
	write_cmos_sensor(0x602A, 0x25B0);
	write_cmos_sensor(0x6F12, 0x1100);
	write_cmos_sensor(0x602A, 0x25B4);
	write_cmos_sensor(0x6F12, 0x0011);
	write_cmos_sensor(0x602A, 0x15A4);
	write_cmos_sensor(0x6F12, 0x0141);
	write_cmos_sensor(0x602A, 0x15A6);
	write_cmos_sensor(0x6F12, 0x0545);
	write_cmos_sensor(0x602A, 0x15A8);
	write_cmos_sensor(0x6F12, 0x0649);
	write_cmos_sensor(0x602A, 0x15AA);
	write_cmos_sensor(0x6F12, 0x024D);
	write_cmos_sensor(0x602A, 0x15AC);
	write_cmos_sensor(0x6F12, 0x0151);
	write_cmos_sensor(0x602A, 0x15AE);
	write_cmos_sensor(0x6F12, 0x0555);
	write_cmos_sensor(0x602A, 0x15B0);
	write_cmos_sensor(0x6F12, 0x0659);
	write_cmos_sensor(0x602A, 0x15B2);
	write_cmos_sensor(0x6F12, 0x025D);
	write_cmos_sensor(0x602A, 0x15B4);
	write_cmos_sensor(0x6F12, 0x0161);
	write_cmos_sensor(0x602A, 0x15B6);
	write_cmos_sensor(0x6F12, 0x0565);
	write_cmos_sensor(0x602A, 0x15B8);
	write_cmos_sensor(0x6F12, 0x0669);
	write_cmos_sensor(0x602A, 0x15BA);
	write_cmos_sensor(0x6F12, 0x026D);
	write_cmos_sensor(0x602A, 0x15BC);
	write_cmos_sensor(0x6F12, 0x0171);
	write_cmos_sensor(0x602A, 0x15BE);
	write_cmos_sensor(0x6F12, 0x0575);
	write_cmos_sensor(0x602A, 0x15C0);
	write_cmos_sensor(0x6F12, 0x0679);
	write_cmos_sensor(0x602A, 0x15C2);
	write_cmos_sensor(0x6F12, 0x027D);
	write_cmos_sensor(0x602A, 0x15C4);
	write_cmos_sensor(0x6F12, 0x0141);
	write_cmos_sensor(0x602A, 0x15C6);
	write_cmos_sensor(0x6F12, 0x0545);
	write_cmos_sensor(0x602A, 0x15C8);
	write_cmos_sensor(0x6F12, 0x0649);
	write_cmos_sensor(0x602A, 0x15CA);
	write_cmos_sensor(0x6F12, 0x024D);
	write_cmos_sensor(0x602A, 0x15CC);
	write_cmos_sensor(0x6F12, 0x0151);
	write_cmos_sensor(0x602A, 0x15CE);
	write_cmos_sensor(0x6F12, 0x0555);
	write_cmos_sensor(0x602A, 0x15D0);
	write_cmos_sensor(0x6F12, 0x0659);
	write_cmos_sensor(0x602A, 0x15D2);
	write_cmos_sensor(0x6F12, 0x025D);
	write_cmos_sensor(0x602A, 0x15D4);
	write_cmos_sensor(0x6F12, 0x0161);
	write_cmos_sensor(0x602A, 0x15D6);
	write_cmos_sensor(0x6F12, 0x0565);
	write_cmos_sensor(0x602A, 0x15D8);
	write_cmos_sensor(0x6F12, 0x0669);
	write_cmos_sensor(0x602A, 0x15DA);
	write_cmos_sensor(0x6F12, 0x026D);
	write_cmos_sensor(0x602A, 0x15DC);
	write_cmos_sensor(0x6F12, 0x0171);
	write_cmos_sensor(0x602A, 0x15DE);
	write_cmos_sensor(0x6F12, 0x0575);
	write_cmos_sensor(0x602A, 0x15E0);
	write_cmos_sensor(0x6F12, 0x0679);
	write_cmos_sensor(0x602A, 0x15E2);
	write_cmos_sensor(0x6F12, 0x027D);
	write_cmos_sensor(0x602A, 0x1A50);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1A54);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0D00, 0x0101);
	write_cmos_sensor(0x0D02, 0x0101);
	write_cmos_sensor(0x0114, 0x0301);
	write_cmos_sensor(0x0202, 0x0010);
	write_cmos_sensor(0x0226, 0x0010);
	write_cmos_sensor(0x0204, 0x0020);
	write_cmos_sensor(0x0B06, 0x0101);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x107A);
	write_cmos_sensor(0x6F12, 0x1D00);
	write_cmos_sensor(0x602A, 0x1074);
	write_cmos_sensor(0x6F12, 0x1D00);
	write_cmos_sensor(0x602A, 0x0E7C);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1120);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x1122);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x602A, 0x1128);
	write_cmos_sensor(0x6F12, 0x0604);
	write_cmos_sensor(0x602A, 0x1AC0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x1AC2);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x602A, 0x1494);
	write_cmos_sensor(0x6F12, 0x3D68);
	write_cmos_sensor(0x602A, 0x1498);
	write_cmos_sensor(0x6F12, 0xF10D);
	write_cmos_sensor(0x602A, 0x1488);
	write_cmos_sensor(0x6F12, 0x0F0F);
	write_cmos_sensor(0x602A, 0x148A);
	write_cmos_sensor(0x6F12, 0x170F);
	write_cmos_sensor(0x602A, 0x150E);
	write_cmos_sensor(0x6F12, 0x00C2);
	write_cmos_sensor(0x602A, 0x1510);
	write_cmos_sensor(0x6F12, 0xC0AF);
	write_cmos_sensor(0x602A, 0x1512);
	write_cmos_sensor(0x6F12, 0x00A0);
	write_cmos_sensor(0x602A, 0x1486);
	write_cmos_sensor(0x6F12, 0x1430);
	write_cmos_sensor(0x602A, 0x1490);
	write_cmos_sensor(0x6F12, 0x4D09);
	write_cmos_sensor(0x602A, 0x149E);
	write_cmos_sensor(0x6F12, 0x01C4);
	write_cmos_sensor(0x602A, 0x11CC);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x602A, 0x11CE);
	write_cmos_sensor(0x6F12, 0x000B);
	write_cmos_sensor(0x602A, 0x11D0);
	write_cmos_sensor(0x6F12, 0x0003);
	write_cmos_sensor(0x602A, 0x11DA);
	write_cmos_sensor(0x6F12, 0x0012);
	write_cmos_sensor(0x602A, 0x11E6);
	write_cmos_sensor(0x6F12, 0x002A);
	write_cmos_sensor(0x602A, 0x125E);
	write_cmos_sensor(0x6F12, 0x0048);
	write_cmos_sensor(0x602A, 0x11F4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x11F8);
	write_cmos_sensor(0x6F12, 0x0016);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0xF444, 0x05BF);
	write_cmos_sensor(0xF44A, 0x0008);
	write_cmos_sensor(0xF44E, 0x0012);
	write_cmos_sensor(0xF46E, 0x40C0);
	write_cmos_sensor(0xF470, 0x7809);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1CAA);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CAC);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CAE);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB2);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBA);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBC);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBE);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC2);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x6000);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x6002);
	write_cmos_sensor(0x6F12, 0xFFFF);
	write_cmos_sensor(0x602A, 0x6004);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x6006);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6008);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600E);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6010);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6012);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6014);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6016);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6018);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601E);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6020);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6022);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6024);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6026);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6028);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x602A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x602C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x1144);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1146);
	write_cmos_sensor(0x6F12, 0x1B00);
	write_cmos_sensor(0x602A, 0x1080);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1084);
	write_cmos_sensor(0x6F12, 0x00C0);
	write_cmos_sensor(0x602A, 0x108A);
	write_cmos_sensor(0x6F12, 0x00C0);
	write_cmos_sensor(0x602A, 0x1090);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1092);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1094);
	write_cmos_sensor(0x6F12, 0xA32E);
}

static void normal_video_setting(void)
{
	LOG_INF("normal_video_setting!\n");
	// full size 30fps
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0x7971);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x0344, 0x0008);
	write_cmos_sensor(0x0346, 0x0184);
	write_cmos_sensor(0x0348, 0x0FA7);
	write_cmos_sensor(0x034A, 0x0A53);
	write_cmos_sensor(0x034C, 0x0FA0);
	write_cmos_sensor(0x034E, 0x08D0);
	write_cmos_sensor(0x0350, 0x0000);
	write_cmos_sensor(0x0352, 0x0000);
	write_cmos_sensor(0x0340, 0x0C7A);
	write_cmos_sensor(0x0342, 0x13A0);
	write_cmos_sensor(0x0900, 0x0111);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0001);
	write_cmos_sensor(0x0404, 0x1000);
	write_cmos_sensor(0x0402, 0x1010);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x0306, 0x00F1);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0300, 0x0008);
	write_cmos_sensor(0x030E, 0x0004);
	write_cmos_sensor(0x0312, 0x0000);
	write_cmos_sensor(0x0310, 0x0064);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1492);
	write_cmos_sensor(0x6F12, 0x0078);
	write_cmos_sensor(0x602A, 0x0E4E);
	write_cmos_sensor(0x6F12, 0x007A);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0118, 0x0004);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x2126);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1168);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x602A, 0x2DB6);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1668);
	write_cmos_sensor(0x6F12, 0xF0F0);
	write_cmos_sensor(0x602A, 0x166A);
	write_cmos_sensor(0x6F12, 0xF0F0);
	write_cmos_sensor(0x602A, 0x118A);
	write_cmos_sensor(0x6F12, 0x0802);
	write_cmos_sensor(0x602A, 0x151E);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x217E);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1520);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x602A, 0x2522);
	write_cmos_sensor(0x6F12, 0x0804);
	write_cmos_sensor(0x602A, 0x2524);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x602A, 0x2568);
	write_cmos_sensor(0x6F12, 0x5500);
	write_cmos_sensor(0x602A, 0x2588);
	write_cmos_sensor(0x6F12, 0x1111);
	write_cmos_sensor(0x602A, 0x258C);
	write_cmos_sensor(0x6F12, 0x1111);
	write_cmos_sensor(0x602A, 0x25A6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x252C);
	write_cmos_sensor(0x6F12, 0x0601);
	write_cmos_sensor(0x602A, 0x252E);
	write_cmos_sensor(0x6F12, 0x0605);
	write_cmos_sensor(0x602A, 0x25A8);
	write_cmos_sensor(0x6F12, 0x1100);
	write_cmos_sensor(0x602A, 0x25AC);
	write_cmos_sensor(0x6F12, 0x0011);
	write_cmos_sensor(0x602A, 0x25B0);
	write_cmos_sensor(0x6F12, 0x1100);
	write_cmos_sensor(0x602A, 0x25B4);
	write_cmos_sensor(0x6F12, 0x0011);
	write_cmos_sensor(0x602A, 0x15A4);
	write_cmos_sensor(0x6F12, 0x0141);
	write_cmos_sensor(0x602A, 0x15A6);
	write_cmos_sensor(0x6F12, 0x0545);
	write_cmos_sensor(0x602A, 0x15A8);
	write_cmos_sensor(0x6F12, 0x0649);
	write_cmos_sensor(0x602A, 0x15AA);
	write_cmos_sensor(0x6F12, 0x024D);
	write_cmos_sensor(0x602A, 0x15AC);
	write_cmos_sensor(0x6F12, 0x0151);
	write_cmos_sensor(0x602A, 0x15AE);
	write_cmos_sensor(0x6F12, 0x0555);
	write_cmos_sensor(0x602A, 0x15B0);
	write_cmos_sensor(0x6F12, 0x0659);
	write_cmos_sensor(0x602A, 0x15B2);
	write_cmos_sensor(0x6F12, 0x025D);
	write_cmos_sensor(0x602A, 0x15B4);
	write_cmos_sensor(0x6F12, 0x0161);
	write_cmos_sensor(0x602A, 0x15B6);
	write_cmos_sensor(0x6F12, 0x0565);
	write_cmos_sensor(0x602A, 0x15B8);
	write_cmos_sensor(0x6F12, 0x0669);
	write_cmos_sensor(0x602A, 0x15BA);
	write_cmos_sensor(0x6F12, 0x026D);
	write_cmos_sensor(0x602A, 0x15BC);
	write_cmos_sensor(0x6F12, 0x0171);
	write_cmos_sensor(0x602A, 0x15BE);
	write_cmos_sensor(0x6F12, 0x0575);
	write_cmos_sensor(0x602A, 0x15C0);
	write_cmos_sensor(0x6F12, 0x0679);
	write_cmos_sensor(0x602A, 0x15C2);
	write_cmos_sensor(0x6F12, 0x027D);
	write_cmos_sensor(0x602A, 0x15C4);
	write_cmos_sensor(0x6F12, 0x0141);
	write_cmos_sensor(0x602A, 0x15C6);
	write_cmos_sensor(0x6F12, 0x0545);
	write_cmos_sensor(0x602A, 0x15C8);
	write_cmos_sensor(0x6F12, 0x0649);
	write_cmos_sensor(0x602A, 0x15CA);
	write_cmos_sensor(0x6F12, 0x024D);
	write_cmos_sensor(0x602A, 0x15CC);
	write_cmos_sensor(0x6F12, 0x0151);
	write_cmos_sensor(0x602A, 0x15CE);
	write_cmos_sensor(0x6F12, 0x0555);
	write_cmos_sensor(0x602A, 0x15D0);
	write_cmos_sensor(0x6F12, 0x0659);
	write_cmos_sensor(0x602A, 0x15D2);
	write_cmos_sensor(0x6F12, 0x025D);
	write_cmos_sensor(0x602A, 0x15D4);
	write_cmos_sensor(0x6F12, 0x0161);
	write_cmos_sensor(0x602A, 0x15D6);
	write_cmos_sensor(0x6F12, 0x0565);
	write_cmos_sensor(0x602A, 0x15D8);
	write_cmos_sensor(0x6F12, 0x0669);
	write_cmos_sensor(0x602A, 0x15DA);
	write_cmos_sensor(0x6F12, 0x026D);
	write_cmos_sensor(0x602A, 0x15DC);
	write_cmos_sensor(0x6F12, 0x0171);
	write_cmos_sensor(0x602A, 0x15DE);
	write_cmos_sensor(0x6F12, 0x0575);
	write_cmos_sensor(0x602A, 0x15E0);
	write_cmos_sensor(0x6F12, 0x0679);
	write_cmos_sensor(0x602A, 0x15E2);
	write_cmos_sensor(0x6F12, 0x027D);
	write_cmos_sensor(0x602A, 0x1A50);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1A54);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0D00, 0x0101);
	write_cmos_sensor(0x0D02, 0x0101);
	write_cmos_sensor(0x0114, 0x0301);
	write_cmos_sensor(0x0202, 0x0010);
	write_cmos_sensor(0x0226, 0x0010);
	write_cmos_sensor(0x0204, 0x0020);
	write_cmos_sensor(0x0B06, 0x0101);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x107A);
	write_cmos_sensor(0x6F12, 0x1D00);
	write_cmos_sensor(0x602A, 0x1074);
	write_cmos_sensor(0x6F12, 0x1D00);
	write_cmos_sensor(0x602A, 0x0E7C);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1120);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x1122);
	write_cmos_sensor(0x6F12, 0x0078);
	write_cmos_sensor(0x602A, 0x1128);
	write_cmos_sensor(0x6F12, 0x0604);
	write_cmos_sensor(0x602A, 0x1AC0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x1AC2);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x602A, 0x1494);
	write_cmos_sensor(0x6F12, 0x3D68);
	write_cmos_sensor(0x602A, 0x1498);
	write_cmos_sensor(0x6F12, 0xF10D);
	write_cmos_sensor(0x602A, 0x1488);
	write_cmos_sensor(0x6F12, 0x0F0F);
	write_cmos_sensor(0x602A, 0x148A);
	write_cmos_sensor(0x6F12, 0x170F);
	write_cmos_sensor(0x602A, 0x150E);
	write_cmos_sensor(0x6F12, 0x00C2);
	write_cmos_sensor(0x602A, 0x1510);
	write_cmos_sensor(0x6F12, 0xC0AF);
	write_cmos_sensor(0x602A, 0x1512);
	write_cmos_sensor(0x6F12, 0x00A0);
	write_cmos_sensor(0x602A, 0x1486);
	write_cmos_sensor(0x6F12, 0x1430);
	write_cmos_sensor(0x602A, 0x1490);
	write_cmos_sensor(0x6F12, 0x4D09);
	write_cmos_sensor(0x602A, 0x149E);
	write_cmos_sensor(0x6F12, 0x01C4);
	write_cmos_sensor(0x602A, 0x11CC);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x602A, 0x11CE);
	write_cmos_sensor(0x6F12, 0x000B);
	write_cmos_sensor(0x602A, 0x11D0);
	write_cmos_sensor(0x6F12, 0x0003);
	write_cmos_sensor(0x602A, 0x11DA);
	write_cmos_sensor(0x6F12, 0x0012);
	write_cmos_sensor(0x602A, 0x11E6);
	write_cmos_sensor(0x6F12, 0x002A);
	write_cmos_sensor(0x602A, 0x125E);
	write_cmos_sensor(0x6F12, 0x0048);
	write_cmos_sensor(0x602A, 0x11F4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x11F8);
	write_cmos_sensor(0x6F12, 0x0016);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0xF444, 0x05BF);
	write_cmos_sensor(0xF44A, 0x0008);
	write_cmos_sensor(0xF44E, 0x0012);
	write_cmos_sensor(0xF46E, 0x40C0);
	write_cmos_sensor(0xF470, 0x7809);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1CAA);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CAC);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CAE);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB2);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBA);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBC);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBE);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC2);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x6000);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x6002);
	write_cmos_sensor(0x6F12, 0xFFFF);
	write_cmos_sensor(0x602A, 0x6004);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x6006);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6008);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600E);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6010);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6012);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6014);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6016);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6018);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601E);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6020);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6022);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6024);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6026);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6028);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x602A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x602C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x1144);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1146);
	write_cmos_sensor(0x6F12, 0x1B00);
	write_cmos_sensor(0x602A, 0x1080);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1084);
	write_cmos_sensor(0x6F12, 0x00C0);
	write_cmos_sensor(0x602A, 0x108A);
	write_cmos_sensor(0x6F12, 0x00C0);
	write_cmos_sensor(0x602A, 0x1090);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1092);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1094);
	write_cmos_sensor(0x6F12, 0xA32E);
}

static void hs_video_setting(void)
{
	LOG_INF("hs_video_setting E\n");
	//720p 120fps
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0x7971);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x0344, 0x0058);
	write_cmos_sensor(0x0346, 0x01AC);
	write_cmos_sensor(0x0348, 0x0F57);
	write_cmos_sensor(0x034A, 0x0A1B);
	write_cmos_sensor(0x034C, 0x0500);
	write_cmos_sensor(0x034E, 0x02D0);
	write_cmos_sensor(0x0350, 0x0000);
	write_cmos_sensor(0x0352, 0x0000);
	write_cmos_sensor(0x0340, 0x0334);
	write_cmos_sensor(0x0342, 0x1320);
	write_cmos_sensor(0x0900, 0x0123);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0002);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0005);
	write_cmos_sensor(0x0404, 0x1000);
	write_cmos_sensor(0x0402, 0x1810);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x0306, 0x00F1);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0300, 0x0008);
	write_cmos_sensor(0x030E, 0x0003);
	write_cmos_sensor(0x0312, 0x0002);
	write_cmos_sensor(0x0310, 0x005B);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1492);
	write_cmos_sensor(0x6F12, 0x0078);
	write_cmos_sensor(0x602A, 0x0E4E);
	write_cmos_sensor(0x6F12, 0xFFFF);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0118, 0x0104);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x2126);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1168);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x602A, 0x2DB6);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1668);
	write_cmos_sensor(0x6F12, 0xF0F0);
	write_cmos_sensor(0x602A, 0x166A);
	write_cmos_sensor(0x6F12, 0xF0F0);
	write_cmos_sensor(0x602A, 0x118A);
	write_cmos_sensor(0x6F12, 0x0802);
	write_cmos_sensor(0x602A, 0x151E);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x217E);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1520);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x2522);
	write_cmos_sensor(0x6F12, 0x0804);
	write_cmos_sensor(0x602A, 0x2524);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x602A, 0x2568);
	write_cmos_sensor(0x6F12, 0x5500);
	write_cmos_sensor(0x602A, 0x2588);
	write_cmos_sensor(0x6F12, 0x1111);
	write_cmos_sensor(0x602A, 0x258C);
	write_cmos_sensor(0x6F12, 0x1111);
	write_cmos_sensor(0x602A, 0x25A6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x252C);
	write_cmos_sensor(0x6F12, 0x0601);
	write_cmos_sensor(0x602A, 0x252E);
	write_cmos_sensor(0x6F12, 0x0605);
	write_cmos_sensor(0x602A, 0x25A8);
	write_cmos_sensor(0x6F12, 0x1100);
	write_cmos_sensor(0x602A, 0x25AC);
	write_cmos_sensor(0x6F12, 0x0011);
	write_cmos_sensor(0x602A, 0x25B0);
	write_cmos_sensor(0x6F12, 0x1100);
	write_cmos_sensor(0x602A, 0x25B4);
	write_cmos_sensor(0x6F12, 0x0011);
	write_cmos_sensor(0x602A, 0x15A4);
	write_cmos_sensor(0x6F12, 0x0141);
	write_cmos_sensor(0x602A, 0x15A6);
	write_cmos_sensor(0x6F12, 0x0545);
	write_cmos_sensor(0x602A, 0x15A8);
	write_cmos_sensor(0x6F12, 0x0649);
	write_cmos_sensor(0x602A, 0x15AA);
	write_cmos_sensor(0x6F12, 0x024D);
	write_cmos_sensor(0x602A, 0x15AC);
	write_cmos_sensor(0x6F12, 0x0151);
	write_cmos_sensor(0x602A, 0x15AE);
	write_cmos_sensor(0x6F12, 0x0555);
	write_cmos_sensor(0x602A, 0x15B0);
	write_cmos_sensor(0x6F12, 0x0659);
	write_cmos_sensor(0x602A, 0x15B2);
	write_cmos_sensor(0x6F12, 0x025D);
	write_cmos_sensor(0x602A, 0x15B4);
	write_cmos_sensor(0x6F12, 0x0161);
	write_cmos_sensor(0x602A, 0x15B6);
	write_cmos_sensor(0x6F12, 0x0565);
	write_cmos_sensor(0x602A, 0x15B8);
	write_cmos_sensor(0x6F12, 0x0669);
	write_cmos_sensor(0x602A, 0x15BA);
	write_cmos_sensor(0x6F12, 0x026D);
	write_cmos_sensor(0x602A, 0x15BC);
	write_cmos_sensor(0x6F12, 0x0171);
	write_cmos_sensor(0x602A, 0x15BE);
	write_cmos_sensor(0x6F12, 0x0575);
	write_cmos_sensor(0x602A, 0x15C0);
	write_cmos_sensor(0x6F12, 0x0679);
	write_cmos_sensor(0x602A, 0x15C2);
	write_cmos_sensor(0x6F12, 0x027D);
	write_cmos_sensor(0x602A, 0x15C4);
	write_cmos_sensor(0x6F12, 0x0141);
	write_cmos_sensor(0x602A, 0x15C6);
	write_cmos_sensor(0x6F12, 0x0545);
	write_cmos_sensor(0x602A, 0x15C8);
	write_cmos_sensor(0x6F12, 0x0649);
	write_cmos_sensor(0x602A, 0x15CA);
	write_cmos_sensor(0x6F12, 0x024D);
	write_cmos_sensor(0x602A, 0x15CC);
	write_cmos_sensor(0x6F12, 0x0151);
	write_cmos_sensor(0x602A, 0x15CE);
	write_cmos_sensor(0x6F12, 0x0555);
	write_cmos_sensor(0x602A, 0x15D0);
	write_cmos_sensor(0x6F12, 0x0659);
	write_cmos_sensor(0x602A, 0x15D2);
	write_cmos_sensor(0x6F12, 0x025D);
	write_cmos_sensor(0x602A, 0x15D4);
	write_cmos_sensor(0x6F12, 0x0161);
	write_cmos_sensor(0x602A, 0x15D6);
	write_cmos_sensor(0x6F12, 0x0565);
	write_cmos_sensor(0x602A, 0x15D8);
	write_cmos_sensor(0x6F12, 0x0669);
	write_cmos_sensor(0x602A, 0x15DA);
	write_cmos_sensor(0x6F12, 0x026D);
	write_cmos_sensor(0x602A, 0x15DC);
	write_cmos_sensor(0x6F12, 0x0171);
	write_cmos_sensor(0x602A, 0x15DE);
	write_cmos_sensor(0x6F12, 0x0575);
	write_cmos_sensor(0x602A, 0x15E0);
	write_cmos_sensor(0x6F12, 0x0679);
	write_cmos_sensor(0x602A, 0x15E2);
	write_cmos_sensor(0x6F12, 0x027D);
	write_cmos_sensor(0x602A, 0x1A50);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1A54);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0D00, 0x0100);
	write_cmos_sensor(0x0D02, 0x0001);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0202, 0x0010);
	write_cmos_sensor(0x0226, 0x0010);
	write_cmos_sensor(0x0204, 0x0020);
	write_cmos_sensor(0x0B06, 0x0101);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x107A);
	write_cmos_sensor(0x6F12, 0x1D00);
	write_cmos_sensor(0x602A, 0x1074);
	write_cmos_sensor(0x6F12, 0x1D00);
	write_cmos_sensor(0x602A, 0x0E7C);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1120);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x1122);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x602A, 0x1128);
	write_cmos_sensor(0x6F12, 0x0604);
	write_cmos_sensor(0x602A, 0x1AC0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x1AC2);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x602A, 0x1494);
	write_cmos_sensor(0x6F12, 0x3D68);
	write_cmos_sensor(0x602A, 0x1498);
	write_cmos_sensor(0x6F12, 0xF10D);
	write_cmos_sensor(0x602A, 0x1488);
	write_cmos_sensor(0x6F12, 0x090F);
	write_cmos_sensor(0x602A, 0x148A);
	write_cmos_sensor(0x6F12, 0x170F);
	write_cmos_sensor(0x602A, 0x150E);
	write_cmos_sensor(0x6F12, 0x00C2);
	write_cmos_sensor(0x602A, 0x1510);
	write_cmos_sensor(0x6F12, 0xC0AF);
	write_cmos_sensor(0x602A, 0x1512);
	write_cmos_sensor(0x6F12, 0x0080);
	write_cmos_sensor(0x602A, 0x1486);
	write_cmos_sensor(0x6F12, 0x1430);
	write_cmos_sensor(0x602A, 0x1490);
	write_cmos_sensor(0x6F12, 0x4D09);
	write_cmos_sensor(0x602A, 0x149E);
	write_cmos_sensor(0x6F12, 0x01C4);
	write_cmos_sensor(0x602A, 0x11CC);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x602A, 0x11CE);
	write_cmos_sensor(0x6F12, 0x000B);
	write_cmos_sensor(0x602A, 0x11D0);
	write_cmos_sensor(0x6F12, 0x0003);
	write_cmos_sensor(0x602A, 0x11DA);
	write_cmos_sensor(0x6F12, 0x0012);
	write_cmos_sensor(0x602A, 0x11E6);
	write_cmos_sensor(0x6F12, 0x002A);
	write_cmos_sensor(0x602A, 0x125E);
	write_cmos_sensor(0x6F12, 0x0048);
	write_cmos_sensor(0x602A, 0x11F4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x11F8);
	write_cmos_sensor(0x6F12, 0x0016);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0xF444, 0x05BF);
	write_cmos_sensor(0xF44A, 0x0008);
	write_cmos_sensor(0xF44E, 0x0012);
	write_cmos_sensor(0xF46E, 0x6CC0);
	write_cmos_sensor(0xF470, 0x7809);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1CAA);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CAC);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CAE);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB2);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBA);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBC);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBE);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC2);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x6000);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x6002);
	write_cmos_sensor(0x6F12, 0xFFFF);
	write_cmos_sensor(0x602A, 0x6004);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x6006);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6008);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600E);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6010);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6012);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6014);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6016);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6018);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601E);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6020);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6022);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6024);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6026);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6028);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x602A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x602C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x1144);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1146);
	write_cmos_sensor(0x6F12, 0x1B00);
	write_cmos_sensor(0x602A, 0x1080);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1084);
	write_cmos_sensor(0x6F12, 0x00C0);
	write_cmos_sensor(0x602A, 0x108A);
	write_cmos_sensor(0x6F12, 0x00C0);
	write_cmos_sensor(0x602A, 0x1090);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1092);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1094);
	write_cmos_sensor(0x6F12, 0xA32E);
}

static void custom1_setting(void)
{
	LOG_INF(" E!\n");
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0x7971);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x0344, 0x0008);
	write_cmos_sensor(0x0346, 0x0008);
	write_cmos_sensor(0x0348, 0x0FA7);
	write_cmos_sensor(0x034A, 0x0BBF);
	write_cmos_sensor(0x034C, 0x0FA0);
	write_cmos_sensor(0x034E, 0x0BB8);
	write_cmos_sensor(0x0350, 0x0000);
	write_cmos_sensor(0x0352, 0x0000);
	write_cmos_sensor(0x0340, 0x0F98);
	write_cmos_sensor(0x0342, 0x13A0);
	write_cmos_sensor(0x0900, 0x0111);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0001);
	write_cmos_sensor(0x0404, 0x1000);
	write_cmos_sensor(0x0402, 0x1010);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x0306, 0x00F1);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0300, 0x0008);
	write_cmos_sensor(0x030E, 0x0003);
	write_cmos_sensor(0x0312, 0x0001);
	write_cmos_sensor(0x0310, 0x0090);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1492);
	write_cmos_sensor(0x6F12, 0x0078);
	write_cmos_sensor(0x602A, 0x0E4E);
	write_cmos_sensor(0x6F12, 0x007A);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0118, 0x0004);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x2126);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1168);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x602A, 0x2DB6);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1668);
	write_cmos_sensor(0x6F12, 0xF0F0);
	write_cmos_sensor(0x602A, 0x166A);
	write_cmos_sensor(0x6F12, 0xF0F0);
	write_cmos_sensor(0x602A, 0x118A);
	write_cmos_sensor(0x6F12, 0x0802);
	write_cmos_sensor(0x602A, 0x151E);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x217E);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1520);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x602A, 0x2522);
	write_cmos_sensor(0x6F12, 0x0804);
	write_cmos_sensor(0x602A, 0x2524);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x602A, 0x2568);
	write_cmos_sensor(0x6F12, 0x5500);
	write_cmos_sensor(0x602A, 0x2588);
	write_cmos_sensor(0x6F12, 0x1111);
	write_cmos_sensor(0x602A, 0x258C);
	write_cmos_sensor(0x6F12, 0x1111);
	write_cmos_sensor(0x602A, 0x25A6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x252C);
	write_cmos_sensor(0x6F12, 0x0601);
	write_cmos_sensor(0x602A, 0x252E);
	write_cmos_sensor(0x6F12, 0x0605);
	write_cmos_sensor(0x602A, 0x25A8);
	write_cmos_sensor(0x6F12, 0x1100);
	write_cmos_sensor(0x602A, 0x25AC);
	write_cmos_sensor(0x6F12, 0x0011);
	write_cmos_sensor(0x602A, 0x25B0);
	write_cmos_sensor(0x6F12, 0x1100);
	write_cmos_sensor(0x602A, 0x25B4);
	write_cmos_sensor(0x6F12, 0x0011);
	write_cmos_sensor(0x602A, 0x15A4);
	write_cmos_sensor(0x6F12, 0x0141);
	write_cmos_sensor(0x602A, 0x15A6);
	write_cmos_sensor(0x6F12, 0x0545);
	write_cmos_sensor(0x602A, 0x15A8);
	write_cmos_sensor(0x6F12, 0x0649);
	write_cmos_sensor(0x602A, 0x15AA);
	write_cmos_sensor(0x6F12, 0x024D);
	write_cmos_sensor(0x602A, 0x15AC);
	write_cmos_sensor(0x6F12, 0x0151);
	write_cmos_sensor(0x602A, 0x15AE);
	write_cmos_sensor(0x6F12, 0x0555);
	write_cmos_sensor(0x602A, 0x15B0);
	write_cmos_sensor(0x6F12, 0x0659);
	write_cmos_sensor(0x602A, 0x15B2);
	write_cmos_sensor(0x6F12, 0x025D);
	write_cmos_sensor(0x602A, 0x15B4);
	write_cmos_sensor(0x6F12, 0x0161);
	write_cmos_sensor(0x602A, 0x15B6);
	write_cmos_sensor(0x6F12, 0x0565);
	write_cmos_sensor(0x602A, 0x15B8);
	write_cmos_sensor(0x6F12, 0x0669);
	write_cmos_sensor(0x602A, 0x15BA);
	write_cmos_sensor(0x6F12, 0x026D);
	write_cmos_sensor(0x602A, 0x15BC);
	write_cmos_sensor(0x6F12, 0x0171);
	write_cmos_sensor(0x602A, 0x15BE);
	write_cmos_sensor(0x6F12, 0x0575);
	write_cmos_sensor(0x602A, 0x15C0);
	write_cmos_sensor(0x6F12, 0x0679);
	write_cmos_sensor(0x602A, 0x15C2);
	write_cmos_sensor(0x6F12, 0x027D);
	write_cmos_sensor(0x602A, 0x15C4);
	write_cmos_sensor(0x6F12, 0x0141);
	write_cmos_sensor(0x602A, 0x15C6);
	write_cmos_sensor(0x6F12, 0x0545);
	write_cmos_sensor(0x602A, 0x15C8);
	write_cmos_sensor(0x6F12, 0x0649);
	write_cmos_sensor(0x602A, 0x15CA);
	write_cmos_sensor(0x6F12, 0x024D);
	write_cmos_sensor(0x602A, 0x15CC);
	write_cmos_sensor(0x6F12, 0x0151);
	write_cmos_sensor(0x602A, 0x15CE);
	write_cmos_sensor(0x6F12, 0x0555);
	write_cmos_sensor(0x602A, 0x15D0);
	write_cmos_sensor(0x6F12, 0x0659);
	write_cmos_sensor(0x602A, 0x15D2);
	write_cmos_sensor(0x6F12, 0x025D);
	write_cmos_sensor(0x602A, 0x15D4);
	write_cmos_sensor(0x6F12, 0x0161);
	write_cmos_sensor(0x602A, 0x15D6);
	write_cmos_sensor(0x6F12, 0x0565);
	write_cmos_sensor(0x602A, 0x15D8);
	write_cmos_sensor(0x6F12, 0x0669);
	write_cmos_sensor(0x602A, 0x15DA);
	write_cmos_sensor(0x6F12, 0x026D);
	write_cmos_sensor(0x602A, 0x15DC);
	write_cmos_sensor(0x6F12, 0x0171);
	write_cmos_sensor(0x602A, 0x15DE);
	write_cmos_sensor(0x6F12, 0x0575);
	write_cmos_sensor(0x602A, 0x15E0);
	write_cmos_sensor(0x6F12, 0x0679);
	write_cmos_sensor(0x602A, 0x15E2);
	write_cmos_sensor(0x6F12, 0x027D);
	write_cmos_sensor(0x602A, 0x1A50);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1A54);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0D00, 0x0101);
	write_cmos_sensor(0x0D02, 0x0101);
	write_cmos_sensor(0x0114, 0x0301);
	write_cmos_sensor(0x0202, 0x0010);
	write_cmos_sensor(0x0226, 0x0010);
	write_cmos_sensor(0x0204, 0x0020);
	write_cmos_sensor(0x0B06, 0x0101);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x107A);
	write_cmos_sensor(0x6F12, 0x1D00);
	write_cmos_sensor(0x602A, 0x1074);
	write_cmos_sensor(0x6F12, 0x1D00);
	write_cmos_sensor(0x602A, 0x0E7C);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1120);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x1122);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x602A, 0x1128);
	write_cmos_sensor(0x6F12, 0x0604);
	write_cmos_sensor(0x602A, 0x1AC0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x1AC2);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x602A, 0x1494);
	write_cmos_sensor(0x6F12, 0x3D68);
	write_cmos_sensor(0x602A, 0x1498);
	write_cmos_sensor(0x6F12, 0xF10D);
	write_cmos_sensor(0x602A, 0x1488);
	write_cmos_sensor(0x6F12, 0x0F0F);
	write_cmos_sensor(0x602A, 0x148A);
	write_cmos_sensor(0x6F12, 0x170F);
	write_cmos_sensor(0x602A, 0x150E);
	write_cmos_sensor(0x6F12, 0x00C2);
	write_cmos_sensor(0x602A, 0x1510);
	write_cmos_sensor(0x6F12, 0xC0AF);
	write_cmos_sensor(0x602A, 0x1512);
	write_cmos_sensor(0x6F12, 0x00A0);
	write_cmos_sensor(0x602A, 0x1486);
	write_cmos_sensor(0x6F12, 0x1430);
	write_cmos_sensor(0x602A, 0x1490);
	write_cmos_sensor(0x6F12, 0x4D09);
	write_cmos_sensor(0x602A, 0x149E);
	write_cmos_sensor(0x6F12, 0x01C4);
	write_cmos_sensor(0x602A, 0x11CC);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x602A, 0x11CE);
	write_cmos_sensor(0x6F12, 0x000B);
	write_cmos_sensor(0x602A, 0x11D0);
	write_cmos_sensor(0x6F12, 0x0003);
	write_cmos_sensor(0x602A, 0x11DA);
	write_cmos_sensor(0x6F12, 0x0012);
	write_cmos_sensor(0x602A, 0x11E6);
	write_cmos_sensor(0x6F12, 0x002A);
	write_cmos_sensor(0x602A, 0x125E);
	write_cmos_sensor(0x6F12, 0x0048);
	write_cmos_sensor(0x602A, 0x11F4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x11F8);
	write_cmos_sensor(0x6F12, 0x0016);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0xF444, 0x05BF);
	write_cmos_sensor(0xF44A, 0x0008);
	write_cmos_sensor(0xF44E, 0x0012);
	write_cmos_sensor(0xF46E, 0x40C0);
	write_cmos_sensor(0xF470, 0x7809);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1CAA);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CAC);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CAE);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB2);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBA);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBC);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBE);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC2);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x6000);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x6002);
	write_cmos_sensor(0x6F12, 0xFFFF);
	write_cmos_sensor(0x602A, 0x6004);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x6006);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6008);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600E);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6010);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6012);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6014);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6016);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6018);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601E);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6020);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6022);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6024);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6026);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6028);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x602A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x602C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x1144);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1146);
	write_cmos_sensor(0x6F12, 0x1B00);
	write_cmos_sensor(0x602A, 0x1080);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1084);
	write_cmos_sensor(0x6F12, 0x00C0);
	write_cmos_sensor(0x602A, 0x108A);
	write_cmos_sensor(0x6F12, 0x00C0);
	write_cmos_sensor(0x602A, 0x1090);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1092);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1094);
	write_cmos_sensor(0x6F12, 0xA32E);
}

#ifdef VENDOR_EDIT
/*zhaozhengtao 2016/02/19,modify for different module*/
static kal_uint16 read_module_id(void)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(MODULE_ID_OFFSET >> 8), (char)(MODULE_ID_OFFSET & 0xFF)};

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, 0xA0/*EEPROM_READ_ID*/);
	pr_err("the module id is %d\n", get_byte);
	return get_byte;
}
#endif
/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 module_id = 0;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor_8(0x0000) << 8) | read_cmos_sensor_8(0x0001));
			pr_err("read_0x0000=0x%x, 0x0001=0x%x,0x0000_0001=0x%x\n",read_cmos_sensor_8(0x0000),read_cmos_sensor_8(0x0001),read_cmos_sensor(0x0000));
			if (*sensor_id == imgsensor_info.sensor_id) {
				#ifdef VENDOR_EDIT
				module_id = read_module_id();
				read_eeprom_SN();
				if(deviceInfo_register_value == 0x00){
					register_imgsensor_deviceinfo("Cam_b", DEVICE_VERSION_S5KGM1SP, module_id);
					deviceInfo_register_value = 0x01;
				}
				#endif
				pr_err("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				return ERROR_NONE;
			}
			pr_err("Read sensor id fail, id: 0x%x ensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;
	LOG_INF("PLATFORM:MT6595,MIPI 2LANE\n");
	LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");

	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor_8(0x0000) << 8) | read_cmos_sensor_8(0x0001));
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id) {
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/
	streaming_control(KAL_FALSE);
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
		imgsensor.pclk = imgsensor_info.cap2.pclk;
		imgsensor.line_length = imgsensor_info.cap2.linelength;
		imgsensor.frame_length = imgsensor_info.cap2.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate) {
			LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				imgsensor.current_fps, imgsensor_info.cap.max_framerate / 10);
		}
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);

	capture_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
} /* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	hs_video   */


static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom1_setting();
	return ERROR_NONE;
}   /*  Custom1   */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame; /* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = 2;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0; // 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;

	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

			break;
		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			Custom1(image_window, sensor_config_data);  // Custom1
			break;
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE)) {
		imgsensor.current_fps = 296;
	} else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE)) {
		imgsensor.current_fps = 146;
	} else {
		imgsensor.current_fps = framerate;
	}
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) {//enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	} else {//Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) {
				set_dummy();
			}
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0) {
				return ERROR_NONE;
			}
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) {
				set_dummy();
			}
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
				frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			} else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
				frame_length = imgsensor_info.cap2.pclk / framerate * 10 / imgsensor_info.cap2.linelength;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.cap2.framelength) ? (frame_length - imgsensor_info.cap2.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.cap2.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			} else {
				if (imgsensor.current_fps != imgsensor_info.cap.max_framerate) {
					LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate / 10);
				}
				frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			}
			if (imgsensor.frame_length > imgsensor.shutter) {
				set_dummy();
			}
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) {
				set_dummy();
			}
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
			if (imgsensor.dummy_line < 0) {
				imgsensor.dummy_line = 0;
			}
			imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) {
				set_dummy();
			}
			break;
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) {
				set_dummy();
			}
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*framerate = imgsensor_info.custom1.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0600, 0x0002);
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0600,0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
	unsigned long long *feature_data=(unsigned long long *) feature_para;
	//unsigned long long *feature_return_para=(unsigned long long *) feature_para;
	SET_PD_BLOCK_INFO_T *PDAFinfo;
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	SENSOR_VC_INFO_STRUCT *pvcinfo;
	//SET_SENSOR_AWB_GAIN *pSetSensorAWB=(SET_SENSOR_AWB_GAIN *)feature_para;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*LOG_INF("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
		#ifdef VENDOR_EDIT
		/*Henry.Chang@Camera.Driver add for 18531 ModuleSN*/
		case SENSOR_FEATURE_GET_MODULE_SN:
			LOG_INF("s5kgm1 GET_MODULE_SN:%d %d\n", *feature_para_len, *feature_data_32);
			if (*feature_data_32 < 4)
				*(feature_data_32 + 1) = (gS5kgm1_SN[4*(*feature_data_32) + 3] << 24)
							| (gS5kgm1_SN[4*(*feature_data_32) + 2] << 16)
							| (gS5kgm1_SN[4*(*feature_data_32) + 1] << 8)
							| (gS5kgm1_SN[4*(*feature_data_32)] & 0xFF);
			break;
		/*Henry.Chang@camera.driver 20181129, add for sensor Module SET*/
		case SENSOR_FEATURE_SET_SENSOR_OTP:
			LOG_INF("SENSOR_FEATURE_SET_SENSOR_OTP length :%d\n", (UINT32)*feature_para_len);
			write_Module_data((ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(feature_para));
			break;
		#endif
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			//night_mode((BOOL) *feature_data);
			break;
		#ifdef VENDOR_EDIT
		/*zhengjiang.zhu@Camera.driver, 2017/10/17	add  for  module id*/
		case SENSOR_FEATURE_CHECK_MODULE_ID:
			*feature_return_para_32 = imgsensor_info.module_id;
			break;
		#endif
		case SENSOR_FEATURE_SET_GAIN:
			set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
			set_shutter_frame_length((UINT16) (*feature_data), (UINT16) (*(feature_data + 1)));
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			set_video_mode(*feature_data);
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32);
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data + 1)));
			break;
		case SENSOR_FEATURE_GET_PDAF_DATA:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
			//read_3P8_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_FRAMERATE:
			LOG_INF("current fps :%d\n", (UINT32)*feature_data_32);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.current_fps = *feature_data_32;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data_32);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.ihdr_mode = *feature_data_32;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
			LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
			wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));

			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CUSTOM1:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[5], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
						break;
		case SENSOR_FEATURE_GET_PDAF_INFO:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n", (UINT16) *feature_data);
			PDAFinfo= (SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CUSTOM1:
					memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(SET_PD_BLOCK_INFO_T));
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info_16_9,sizeof(SET_PD_BLOCK_INFO_T));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info_binning, sizeof(SET_PD_BLOCK_INFO_T));
					break;
				default:
					break;
			}
			break;
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", (UINT16) *feature_data);
			//PDAF capacity enable or not, 2p8 only full size support PDAF
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				case MSDK_SCENARIO_ID_CUSTOM1:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; // video & capture &custom1 use same setting
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
			}
			break;
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
			LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			break;
		case SENSOR_FEATURE_SET_AWB_GAIN:
			break;
		case SENSOR_FEATURE_SET_HDR_SHUTTER:
			LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1));
			//ihdr_write_shutter((UINT16)*feature_data,(UINT16)*(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		{
			kal_uint32 rate;
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					rate = imgsensor_info.cap.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					rate = imgsensor_info.normal_video.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					rate = imgsensor_info.hs_video.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_CUSTOM1:
					rate = imgsensor_info.custom1.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					rate = imgsensor_info.pre.mipi_pixel_rate;
					break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
		}
		break;
		case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
			LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
			streaming_control(KAL_FALSE);
			break;
		case SENSOR_FEATURE_SET_STREAMING_RESUME:
			LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
			if (*feature_data != 0)
				set_shutter(*feature_data);
			streaming_control(KAL_TRUE);
			break;
		case SENSOR_FEATURE_GET_VC_INFO:
			LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
			pvcinfo = (SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
			switch (*feature_data_32) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[1],sizeof(SENSOR_VC_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[2],sizeof(SENSOR_VC_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			case MSDK_SCENARIO_ID_CUSTOM1:
			default:
				memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],sizeof(SENSOR_VC_INFO_STRUCT));
				break;
			}
		/*Chengtian.Ding@Camera, 2018-12-27 add for 18531 n+1 long exposure*/
		case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
			*feature_return_para_32 = imgsensor.current_ae_effective_frame;
			break;
		case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
			memcpy(feature_return_para_32, &imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
			break;

		default:
			break;
	}

	return ERROR_NONE;
}	/*	feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5KGM1SP_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
//UINT32 IMX214_MIPI_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL) {
		*pfFunc=&sensor_func;
	}
	return ERROR_NONE;
}	/*	S5KGM1SP_MIPI_RAW_SensorInit	*/
