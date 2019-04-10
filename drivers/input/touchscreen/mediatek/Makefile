subdir-ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
# In case the platform does NOT support this type of sensors

#Qiuyu.Fan@ODM_HQ.MM.Tp, 2018/10/02, add tp driver
ifeq ($(ODM_HQ_EDIT),yes)
obj-n   +=  mtk_tpd.o
obj-n   +=  tpd_button.o
obj-n   +=  tpd_calibrate.o
obj-n   +=  tpd_debug.o
obj-n   +=  tpd_default.o
obj-n   +=  tpd_init.o
obj-n   +=  tpd_misc.o
obj-n   +=  tpd_setting.o
else
obj-y   +=  mtk_tpd.o
obj-y   +=  tpd_button.o
obj-y   +=  tpd_calibrate.o
obj-y   +=  tpd_debug.o
obj-y   +=  tpd_default.o
obj-y   +=  tpd_init.o
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o
endif

obj-$(CONFIG_TOUCHSCREEN_MTK_GT1151)	+=  GT1151/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT5688)	+=  GT5688/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT911)	+=  GT911/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_LM)	+=  GT9XX_LM/
obj-$(CONFIG_TOUCHSCREEN_MTK_GSLX680)	+=  gslX680/
obj-$(CONFIG_TOUCHSCREEN_FT5X26)	+=  FT5X26/
obj-$(CONFIG_TOUCHSCREEN_MTK_NT36xxx)	+=  NT36xxx/

#Qiuyu.Fan@ODM_HQ.MM.Tp, 2018/10/03, add tp driver
ifeq ($(ODM_HQ_EDIT),yes)
obj-$(CONFIG_TOUCHPANEL_HQ) += hq/
endif
