ARCH := arm
CROSS_COMPILE := ${HOME}/luckfox/pico/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-
KERN_DIR := ${HOME}/luckfox/pico/sysdrv/source/objs_kernel

all:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERN_DIR) M=`pwd` modules
clean:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERN_DIR) M=`pwd` modules clean

clena: clean
#CFLAGS_$(MODULE_NAME).o := -DDEBUG
obj-m += st7586.o
obj-m += st7567_tinydrm.o
st7567_tinydrm-objs := st7567.o drm_mipi_dbi.o
