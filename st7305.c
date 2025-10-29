// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for display panels connected to a Sitronix st7305
 * display controller in SPI mode.
 *
 * This driver is inspired by:
 *   https://github.com/DuRuofu/esp-idf-st7305-Ink-screen
 *
 * Copyright (c) 2025 DuRuofu <duruofu@qq.com>
 * Copyright (c) 2025 Wooden Chair <hua.zheng@embeddedboys.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <video/mipi_display.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_rect.h>

#define DRV_NAME "st7305"

#define ST7305_MADCTL_MY BIT(7) // Page Address Order
#define ST7305_MADCTL_MX BIT(6) // Column Address Order
#define ST7305_MADCTL_MV BIT(5) // Page/Column Order
#define ST7305_MADCTL_DO BIT(4) // Data Order, using in MX=1
#define ST7305_MADCTL_GS BIT(3) // Data refresh Bottom to Top

struct st7305 {
	struct device *dev;
	struct mipi_dbi_dev *dbidev;
	struct mipi_dbi *dbi;
	struct drm_device *drm;

	/* TODO: support TE */
	struct gpio_desc *te;

	const struct st7305_panel_desc *desc;
};

struct st7305_panel_desc {
	const struct drm_display_mode *mode;

	u8 caset[2];
	u8 raset[2];

	u8 page_size; // each page contains two rows

	int (*init_seq)(struct st7305 *st7305);
};

static inline struct st7305 *dbi_to_st7305(struct mipi_dbi *dbi)
{
	return spi_get_drvdata(dbi->spi);
}

static inline struct st7305 *dbidev_to_st7305(struct mipi_dbi_dev *dbidev)
{
	return dbi_to_st7305(&dbidev->dbi);
}

/*
 * The device tree node may specify the wrong GPIO
 * active behavior, hard-coded as low active here
 */
static inline void st7305_reset(struct mipi_dbi *dbi)
{
	gpiod_set_raw_value(dbi->reset, 0);
	msleep(10);
	gpiod_set_raw_value(dbi->reset, 1);
	msleep(10);
}

static void st7305_pipe_enable(struct drm_simple_display_pipe *pipe,
			       struct drm_crtc_state *crtc_state,
			       struct drm_plane_state *plane_state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	struct st7305 *st7305 = dbi_to_st7305(dbi);
	int idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	DRM_DEBUG_KMS("\n");

	st7305_reset(dbi);

	st7305->desc->init_seq(st7305);

	drm_dev_exit(idx);
}

static void st7305_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	DRM_DEBUG_KMS("\n");
	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_OFF);
}

static inline void st7305_draw_pixel(u8 *dst, uint x, uint y, u8 page_size,
				     u8 gray)
{
	u32 byte_index = ((y >> 1) * page_size) + (x >> 2);
	u32 bit_index = ((x & 3) << 1) | (y & 1);
	u8 mask = 1u << (7 - bit_index);
	u8 set = (gray >> 7) * mask;

	dst[byte_index] = (dst[byte_index] & ~mask) | set;
}

static void st7305_xrgb8888_to_monochrome(u8 *dst, void *vaddr,
					  struct drm_framebuffer *fb,
					  struct drm_rect *clip)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(fb->dev);
	size_t len = (clip->x2 - clip->x1) * (clip->y2 - clip->y1);
	struct st7305 *st7305 = dbidev_to_st7305(dbidev);
	unsigned int x, y;
	u8 *src, *buf;
	u8 page_size;

	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		return;

	page_size = st7305->desc->page_size;

	drm_fb_xrgb8888_to_gray8(buf, vaddr, fb, clip);
	src = buf;

	for (y = clip->y1; y < clip->y2; y++)
		for (x = clip->x1; x < clip->x2; x++)
			st7305_draw_pixel(dst, x, y, page_size, *src++);

	kfree(buf);
}

static int st7305_buf_copy(void *dst, struct drm_framebuffer *fb,
			   struct drm_rect *clip)
{
	struct drm_gem_cma_object *cma_obj = drm_fb_cma_get_gem_obj(fb, 0);
	struct dma_buf_attachment *import_attach = cma_obj->base.import_attach;
	void *src = cma_obj->vaddr;
	int ret = 0;

	if (import_attach) {
		ret = dma_buf_begin_cpu_access(import_attach->dmabuf,
					       DMA_FROM_DEVICE);
		if (ret)
			return ret;
	}

	st7305_xrgb8888_to_monochrome(dst, src, fb, clip);

	if (import_attach)
		ret = dma_buf_end_cpu_access(import_attach->dmabuf,
					     DMA_FROM_DEVICE);

	return ret;
}

static void st7305_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(fb->dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	struct st7305 *st7305 = dbi_to_st7305(dbi);
	const u8 *caset, *raset;
	int ret = 0;
	int idx;

	if (!drm_dev_enter(fb->dev, &idx))
		return;

	DRM_DEBUG_KMS("Flushing [FB:%d] " DRM_RECT_FMT "\n", fb->base.id,
		      DRM_RECT_ARG(rect));

	caset = st7305->desc->caset;
	raset = st7305->desc->raset;

	ret = st7305_buf_copy(dbidev->tx_buf, fb, rect);
	if (ret)
		goto err_msg;

	mipi_dbi_command(dbi, MIPI_DCS_SET_COLUMN_ADDRESS, caset[0], caset[1]);

	mipi_dbi_command(dbi, MIPI_DCS_SET_PAGE_ADDRESS, raset[0], raset[1]);

	ret = mipi_dbi_command_buf(dbi, MIPI_DCS_WRITE_MEMORY_START,
				   (u8 *)dbidev->tx_buf,
				   (fb->width / 4) * (fb->height / 2));
err_msg:
	if (ret)
		dev_err_once(fb->dev->dev, "Failed to update display %d\n",
			     ret);

	drm_dev_exit(idx);
}

static void st7305_pipe_update(struct drm_simple_display_pipe *pipe,
			       struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_rect rect;

	if (!pipe->crtc.state->active)
		return;

	if (drm_atomic_helper_damage_merged(old_state, state, &rect))
		st7305_fb_dirty(state->fb, &rect);
}

static const u32 st7305_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static const struct drm_simple_display_pipe_funcs st7305_pipe_funcs = {
	.enable = st7305_pipe_enable,
	.disable = st7305_pipe_disable,
	.update = st7305_pipe_update,
	.prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

static int fp_290h07b_init_seq(struct st7305 *st7305)
{
	struct mipi_dbi_dev *dbidev = st7305->dbidev;
	struct mipi_dbi *dbi = st7305->dbi;
	u8 addr_mode;

	mipi_dbi_command(dbi, 0xD6, 0x13, 0x02); // NVM Load Control
	mipi_dbi_command(dbi, 0xD1, 0x01); // Booster Enable
	mipi_dbi_command(dbi, 0xC0, 0x08, 0x06); // Gate Voltage Setting

	mipi_dbi_command(dbi, 0xC1, 0x3C, 0x3E, 0x3C,
			 0x3C); // VSHP Setting (4.8V)
	mipi_dbi_command(dbi, 0xC2, 0x23, 0x21, 0x23,
			 0x23); // VSLP Setting (0.98V)
	mipi_dbi_command(dbi, 0xC4, 0x5A, 0x5C, 0x5A,
			 0x5A); // VSHN Setting (-3.6V)
	mipi_dbi_command(dbi, 0xC5, 0x37, 0x35, 0x37,
			 0x37); // VSLN Setting (0.22V)

	mipi_dbi_command(dbi, 0xD8, 0x80, 0xE9);

	mipi_dbi_command(dbi, 0xB2, 0x02); // Frame Rate Control

	// Update Period Gate EQ Control in HPM
	mipi_dbi_command(dbi, 0xB3, 0xE5, 0xF6, 0x17, 0x77, 0x77, 0x77, 0x77,
			 0x77, 0x77, 0x71);
	// Update Period Gate EQ Control in LPM
	mipi_dbi_command(dbi, 0xB4, 0x05, 0x46, 0x77, 0x77, 0x77, 0x77, 0x76,
			 0x45);
	mipi_dbi_command(dbi, 0x62, 0x32, 0x03, 0x1F); // Gate Timing Control

	mipi_dbi_command(dbi, 0xB7, 0x13); // Source EQ Enable
	mipi_dbi_command(dbi, 0xB0, 0x60); // Gate Line Setting: 384 line

	mipi_dbi_command(dbi, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(120);

	mipi_dbi_command(dbi, 0xC9, 0x00); // Source Voltage Select

	switch (dbidev->rotation) {
	default:
		addr_mode = ST7305_MADCTL_MX | ST7305_MADCTL_GS;
		break;
	case 90:
	case 180:
	case 270:
		addr_mode = ST7305_MADCTL_MX | ST7305_MADCTL_GS;
		printk("%s, Rotation is not supported yet.", __func__);
		break;
	}
	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);

	mipi_dbi_command(dbi, MIPI_DCS_SET_PIXEL_FORMAT,
			 0x11); // 3 write for 24bit
	mipi_dbi_command(dbi, 0xB9, 0x20); // Gamma Mode Setting
	mipi_dbi_command(dbi, 0xB8, 0x29); // Panel Setting
	// mipi_dbi_command(dbi, MIPI_DCS_SET_COLUMN_ADDRESS, 0x17, 0x24);
	// mipi_dbi_command(dbi, MIPI_DCS_SET_PAGE_ADDRESS, 0x00, 0xBF);
	mipi_dbi_command(dbi, 0xD0, 0xFF); // Auto power down
	mipi_dbi_command(dbi, 0x38); // High Power Mode on
	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_ON);

	return 0;
}

static const struct drm_display_mode fp_290h07b_mode = {
	DRM_SIMPLE_MODE(168, 384, 29, 67),
};

static const struct st7305_panel_desc fp_290h07b_desc = {
	.mode = &fp_290h07b_mode,

	.caset[0] = 0x17,
	.caset[1] = 0x24, // 0X24-0X17=14 // 14*4*3=168

	.raset[0] = 0x00,
	.raset[1] = 0xBF, // 192*2=384

	.page_size = 42, // 168/8*2=42

	.init_seq = fp_290h07b_init_seq,
};

DEFINE_DRM_GEM_CMA_FOPS(st7305_fops);

static struct drm_driver st7305_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &st7305_fops,
	DRM_GEM_CMA_DRIVER_OPS_VMAP,
	.debugfs_init = mipi_dbi_debugfs_init,
	.name = "st7305",
	.desc = "Sitronix ST7305",
	.date = "20251022",
	.major = 1,
	.minor = 0,
};

static const struct of_device_id st7305_of_match[] = {
	{ .compatible = "sitronix,st7567", .data = &fp_290h07b_desc },
	{ .compatible = "opstek,fp-290h07b", .data = &fp_290h07b_desc },
	{},
};
MODULE_DEVICE_TABLE(of, st7305_of_match);

static const struct spi_device_id st7305_id[] = {
	{ "st7305" },
	{ "fp-290h07b" },
	{},
};
MODULE_DEVICE_TABLE(spi, st7305_id);

static int st7305_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct st7305 *st7305;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	u16 width, height;
	u32 rotation = 0;
	size_t bufsize;
	int ret;

	st7305 = devm_kzalloc(dev, sizeof(*st7305), GFP_KERNEL);
	if (IS_ERR(st7305))
		return -ENOMEM;

	dbidev = devm_drm_dev_alloc(dev, &st7305_driver, struct mipi_dbi_dev,
				    drm);
	if (IS_ERR(dbidev))
		return PTR_ERR(dbidev);

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

	st7305->dev = dev;
	st7305->dbidev = dbidev;
	st7305->drm = drm;
	st7305->dbi = dbi;
	st7305->desc = device_get_match_data(dev);
	if (!st7305->desc)
		return -ENODEV;

	width = st7305->desc->mode->hdisplay;
	height = st7305->desc->mode->vdisplay;
	// bufsize = (width * height * sizeof(u16));
	bufsize = (width / 4) * (height / 2);

	dbi->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(dbi->reset)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'reset'\n");
		return PTR_ERR(dbi->reset);
	}

	/*
	 * we are using 8-bit data, so we are not actually swapping anything,
	 * but setting mipi->swap_bytes makes mipi_dbi_typec3_command() do the
	 * right thing and not use 16-bit transfers (which results in swapped
	 * bytes on little-endian systems and causes out of order data to be
	 * sent to the display).
	 */
	dbi->swap_bytes = true;

	dc = devm_gpiod_get(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'dc'\n");
		return PTR_ERR(dc);
	}

	device_property_read_u32(dev, "rotation", &rotation);
	dev_info(dev, "rotation: %d\n", rotation);

	ret = mipi_dbi_spi_init(spi, dbi, dc);
	if (ret)
		return ret;

	/* SDO signal is not available on this panel. */
	dbi->read_commands = NULL;

	ret = mipi_dbi_dev_init_with_formats(dbidev, &st7305_pipe_funcs,
					     st7305_formats,
					     ARRAY_SIZE(st7305_formats),
					     &fp_290h07b_mode, rotation,
					     bufsize);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, st7305);

	drm_fbdev_generic_setup(drm, 0);

	dev_info(dev, "%ux%u mipi-dbi@%uMHz - ready\n", width, height,
		 spi->max_speed_hz / 1000000);

	return 0;
}

static int st7305_remove(struct spi_device *spi)
{
	struct st7305 *st7305 = spi_get_drvdata(spi);
	struct drm_device *drm = st7305->drm;

	DRM_DEBUG_KMS("\n");

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);

	return 0;
}

static void st7305_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver st7305_spi_driver = {
	.driver =
	{
		.name = DRV_NAME,
		.of_match_table = st7305_of_match,
	},
	.id_table = st7305_id,
	.probe = st7305_probe,
	.remove = st7305_remove,
	.shutdown = st7305_shutdown,
};
module_spi_driver(st7305_spi_driver);

MODULE_DESCRIPTION("Sitronix ST7305 DRM driver");
MODULE_AUTHOR("Wooden Chair <hua.zheng@embeddedboys.com>");
MODULE_LICENSE("GPL");
