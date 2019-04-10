ifeq ($(CONFIG_OPPO_CHARGER_MTK),y)
ifeq ($(CONFIG_OPPO_CHARGER_6750T),y)
else ifeq ($(CONFIG_OPPO_CHARGER_MTK6763),y)
else ifeq ($(CONFIG_OPPO_CHARGER_MTK6771),y)
else
include $(srctree)/drivers/misc/mediatek/Makefile.custom
endif

subdir-ccflags-y += -Werror
subdir-ccflags-y += -Wno-unused-variable
endif

obj-y	+= oppo_charger.o
obj-y	+= oppo_gauge.o
