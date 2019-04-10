# include header files for MTK

ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
#ccflags-y += -Wall

# Build method

	obj-y += platform.o userspace.o
	obj-y += core/config.o \
		core/finger_report.o \
		core/firmware.o \
		core/flash.o \
		core/i2c.o \
		core/spi.o \
		core/mp_test.o \
		core/protocol.o \
		core/parser.o \
		core/gesture.o


