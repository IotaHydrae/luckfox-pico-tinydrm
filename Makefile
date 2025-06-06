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
obj-m += st7576_tinydrm.o
st7576_tinydrm-objs := st7576.o drm_mipi_dbi.o drm_fb_cma_helper.o

obj-m += st7735r_tinydrm.o
st7735r_tinydrm-objs := st7735r.o drm_mipi_dbi.o drm_fb_cma_helper.o
